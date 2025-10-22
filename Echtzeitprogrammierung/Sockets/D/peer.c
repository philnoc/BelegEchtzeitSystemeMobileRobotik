// peer_local.c
// Peer = Server (Listendienst) + REPL mit permanentem Remote-Empfangs-Thread.
// - Lokal: 127.0.0.1 only
// - Standard: Port 5000. Falls bind() scheitert -> Client zu 127.0.0.1:<port>
// - Handover: alter Leader -> @handover; neuer Leader importiert & uebernimmt Port (Retry bis 5s) -> @ready
//             alter Leader schliesst Listener, broadcastet @newleader 127.0.0.1:<port>, trennt alle Clients.
// - Befehle: /read | /clear | /delete N | /quit

#define _XOPEN_SOURCE 700
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef DEFAULT_PORT
#define DEFAULT_PORT 5000
#endif

// feste Groessen und Puffer
#define BACKLOG      64
#define IN_BUFSZ     4096
#define RESP_BUFSZ   8192
#define MSG_LEN      256
#define MAX_MSG      256
#define MAX_TRACKED  1024

// globaler Port und Protokoll-Endmarke fuer Antworten
static int  g_port = DEFAULT_PORT;
static const char *ENDMARK = "--END--\n";

// Forward-Declaration
static int send_all(int fd, const char *buf, size_t len);

// -------------------- Datenmodell Liste --------------------
// Eintrag: Text + Timestamp
typedef struct { char text[MSG_LEN]; time_t ts; } ListEntry;
// Liste: Anzahl + Array von Eintraegen
typedef struct { int count; ListEntry entries[MAX_MSG]; } SharedList;

// globale Liste (nur im Serverprozess gueltig), mit Mutex
static SharedList g_list = {0};
static pthread_mutex_t g_list_lock = PTHREAD_MUTEX_INITIALIZER;

// -------------------- Utils --------------------
// TCP Keepalive aktivieren, damit tote Verbindungen erkannt werden
static void enable_keepalive(int fd) {
    int yes=1; setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
#ifdef TCP_KEEPIDLE
    int idle=30; setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#endif
#ifdef TCP_KEEPINTVL
    int intv=10; setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intv, sizeof(intv));
#endif
#ifdef TCP_KEEPCNT
    int cnt=3; setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
}

// send() bis alle Bytes raus sind oder Fehler
static int send_all(int fd, const char *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = send(fd, buf + off, len - off, 0);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        off += (size_t)n;
    }
    return 0;
}

// Zeilenende entfernen (\n, \r)
static void trim_eol(char *s){
    size_t n=strlen(s);
    while(n && (s[n-1]=='\n'||s[n-1]=='\r')) s[--n]='\0';
}

// -------------------- Verbindungs-Tracker --------------------
// Merkt sich offene Client-FDs, um Handover/Broadcasts zu senden
typedef struct { int fd; time_t since; } TrackConn;
static TrackConn g_conns[MAX_TRACKED];
static pthread_mutex_t g_conn_lock = PTHREAD_MUTEX_INITIALIZER;

// FD in Tracking aufnehmen
static void track_add(int fd){
    pthread_mutex_lock(&g_conn_lock);
    for(int i=0;i<MAX_TRACKED;i++)
        if(!g_conns[i].fd){ g_conns[i].fd=fd; g_conns[i].since=time(NULL); break; }
    pthread_mutex_unlock(&g_conn_lock);
}

// FD aus Tracking entfernen
static void track_del(int fd){
    pthread_mutex_lock(&g_conn_lock);
    for(int i=0;i<MAX_TRACKED;i++)
        if(g_conns[i].fd==fd){ g_conns[i].fd=0; g_conns[i].since=0; break; }
    pthread_mutex_unlock(&g_conn_lock);
}

// Aeltesten Client-FD finden (fuer Handover-Ziel)
static int pick_oldest_fd(void){
    pthread_mutex_lock(&g_conn_lock);
    int best=-1; time_t t=0;
    for(int i=0;i<MAX_TRACKED;i++)
        if(g_conns[i].fd>0)
            if(best==-1 || g_conns[i].since<t){ best=g_conns[i].fd; t=g_conns[i].since; }
    pthread_mutex_unlock(&g_conn_lock);
    return best;
}

// Alle Client-FDs schliessen (zB beim Handover-Ende des alten Leaders)
static void track_close_all(void){
    pthread_mutex_lock(&g_conn_lock);
    for(int i=0;i<MAX_TRACKED;i++)
        if(g_conns[i].fd>0){ close(g_conns[i].fd); g_conns[i].fd=0; }
    pthread_mutex_unlock(&g_conn_lock);
}

// @newleader an alle Clients senden (lokal)
static void broadcast_newleader_local(int port){
    char msg[128];
    int n=snprintf(msg,sizeof(msg),"@newleader 127.0.0.1:%d\n%s",port,ENDMARK);
    pthread_mutex_lock(&g_conn_lock);
    for(int i=0;i<MAX_TRACKED;i++){
        int fd=g_conns[i].fd;
        if(fd>0) (void)send_all(fd,msg,(size_t)n);
    }
    pthread_mutex_unlock(&g_conn_lock);
}

// -------------------- Handover-Status --------------------
// Synchronisiert @ready-Antwort vom neuen Leader
static pthread_mutex_t g_handover_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_handover_fd    = -1; // FD, auf dem @ready erwartet wird
static int g_handover_ready =  0; // Flag: @ready empfangen?

// -------------------- Listening-Socket global --------------------
// Nimmt eingehende Verbindungen an; bei Handover schliessen
static pthread_mutex_t g_listen_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_listen_fd = -1;

static void stop_listening_socket(void) {
    pthread_mutex_lock(&g_listen_lock);
    if (g_listen_fd != -1) { close(g_listen_fd); g_listen_fd = -1; }
    pthread_mutex_unlock(&g_listen_lock);
}

// -------------------- Export/Import Liste --------------------
// Liste zu Text (index|timestamp|text pro Zeile)
static void export_list(char *out, size_t outsz){
    out[0]='\0';
    pthread_mutex_lock(&g_list_lock);
    int n=g_list.count;
    for(int i=0;i<n;i++){
        char ln[MSG_LEN+64];
        snprintf(ln,sizeof(ln),"%d|%ld|%s\n",i,(long)g_list.entries[i].ts,g_list.entries[i].text);
        if(strlen(out)+strlen(ln)+1>=outsz) break;
        strcat(out,ln);
    }
    pthread_mutex_unlock(&g_list_lock);
}

// Text zu Liste (Zeilen parsen, Felder per | trennen)
static void import_list(const char *in){
    pthread_mutex_lock(&g_list_lock);
    memset(&g_list,0,sizeof(g_list));
    const char *p=in;
    while(*p && g_list.count<MAX_MSG){
        const char *nl=strchr(p,'\n');
        size_t len=nl?(size_t)(nl-p):strlen(p);
        char row[MSG_LEN+64]; if(len>=sizeof(row)) len=sizeof(row)-1;
        memcpy(row,p,len); row[len]='\0';
        char *s1=strchr(row,'|'); char *s2=s1?strchr(s1+1,'|'):NULL;
        if(s1&&s2){
            *s1=*s2='\0';
            long ts=atol(s1+1);
            snprintf(g_list.entries[g_list.count].text,MSG_LEN,"%s",s2+1);
            g_list.entries[g_list.count].ts=(time_t)ts;
            g_list.count++;
        }
        p=nl?nl+1:p+len;
    }
    pthread_mutex_unlock(&g_list_lock);
}

// -------------------- Listen-Operationen --------------------
// Text anhaengen
static void handle_append(const char* payload,char* resp,size_t rsz){
    if(!payload||!payload[0]){ snprintf(resp,rsz,"Fehler: leerer Text\n"); return; }
    pthread_mutex_lock(&g_list_lock);
    if(g_list.count>=MAX_MSG){
        pthread_mutex_unlock(&g_list_lock);
        snprintf(resp,rsz,"Fehler: Liste voll (%d)\n",MAX_MSG);
        return;
    }
    int idx=g_list.count++;
    snprintf(g_list.entries[idx].text,MSG_LEN,"%s",payload);
    g_list.entries[idx].ts=time(NULL);
    pthread_mutex_unlock(&g_list_lock);
    snprintf(resp,rsz,"OK: Eintrag (%d) hinzugefuegt\n",idx);
}

// Liste lesen (Snapshot bilden, dann formatieren)
static void handle_read(char* resp,size_t rsz){
    ListEntry snap[MAX_MSG]; int n=0;
    pthread_mutex_lock(&g_list_lock);
    n=g_list.count; if(n>MAX_MSG) n=MAX_MSG;
    for(int i=0;i<n;i++) snap[i]=g_list.entries[i];
    pthread_mutex_unlock(&g_list_lock);
    if(n==0){ snprintf(resp,rsz,"(leer)\n"); return; }
    resp[0]='\0';
    for (int i = 0; i < n; i++) {
        char tbuf[32] = {0};
        struct tm tm; localtime_r(&snap[i].ts, &tm);
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm);
        char ln[MSG_LEN + 64];
        snprintf(ln, sizeof(ln), "(%d) [%s] %s\n", i, tbuf, snap[i].text);
        if (strlen(resp) + strlen(ln) + 1 >= rsz) { break; }
        strcat(resp, ln);
    }
}

// Liste loeschen
static void handle_clear(char* resp,size_t rsz){
    pthread_mutex_lock(&g_list_lock);
    memset(&g_list,0,sizeof(g_list));
    pthread_mutex_unlock(&g_list_lock);
    snprintf(resp,rsz,"Liste geloescht\n");
}

// Eintrag id entfernen und nachruecken
static void handle_delete(const char* payload,char* resp,size_t rsz){
    char *endp=NULL; long id=strtol(payload?payload:"",&endp,10);
    if(!payload || endp==payload || *endp!='\0' || id<0){
        snprintf(resp,rsz,"Fehler: ungueltige ID '%s'\n",payload?payload:"");
        return;
    }
    pthread_mutex_lock(&g_list_lock);
    if(id>=g_list.count){
        pthread_mutex_unlock(&g_list_lock);
        snprintf(resp,rsz,"Fehler: ID (%ld) existiert nicht (Anzahl=%d)\n",id,g_list.count);
        return;
    }
    for(int i=(int)id;i<g_list.count-1;i++) g_list.entries[i]=g_list.entries[i+1];
    g_list.count--;
    pthread_mutex_unlock(&g_list_lock);
    snprintf(resp,rsz,"OK: Eintrag (%ld) geloescht\n",id);
}

// -------------------- Befehlsverarbeitung --------------------
static void process_line(const char* line_in,char* resp,size_t rsz,int* should_close){
    char line[IN_BUFSZ]; snprintf(line,sizeof(line),"%s",line_in); trim_eol(line);

    if(strcmp(line,"/quit")==0){ snprintf(resp,rsz,"bye\n"); *should_close=1; }
    else if(strcmp(line,"/read")==0){ handle_read(resp,rsz); }
    else if(strcmp(line,"/clear")==0){ handle_clear(resp,rsz); }
    else if(strncmp(line,"/delete ",8)==0){ handle_delete(line+8,resp,rsz); }
    else if(line[0]=='/'){ snprintf(resp,rsz,"Unbekannter Befehl: %s\n",line); }
    else { handle_append(line,resp,rsz); }
}

// -------------------- Server: pro Verbindung ein Thread --------------------
typedef struct { int cfd; } ConnArg;

static void *conn_thread(void *arg){
    ConnArg *ca=(ConnArg*)arg; int fd=ca->cfd; free(ca);
    enable_keepalive(fd);
    track_add(fd);

    char acc[IN_BUFSZ]={0}; size_t acc_len=0;
    for(;;){
        // Daten vom Client lesen
        char buf[1024]; ssize_t n=recv(fd,buf,sizeof(buf),0);
        if(n<=0) break; // Verbindung beendet oder Fehler

        // Puffer zusammenbauen
        if(acc_len+(size_t)n>=sizeof(acc)) acc_len=0;
        memcpy(acc+acc_len,buf,(size_t)n);
        acc_len+=(size_t)n;

        // zeilenweise verarbeiten
        size_t start=0;
        for(;;){
            char *nl=memchr(acc+start,'\n',acc_len-start);
            if(!nl) break;
            size_t linelen=(size_t)(nl-(acc+start))+1;
            char line[IN_BUFSZ]; if(linelen>=sizeof(line)) linelen=sizeof(line)-1;
            memcpy(line,acc+start,linelen); line[linelen]='\0'; start+=linelen;

            // @ready aus Handover-Handshake erkennen
            if (strcmp(line, "@ready\n") == 0 || strcmp(line, "@ready") == 0) {
                pthread_mutex_lock(&g_handover_lock);
                if (g_handover_fd == fd) { g_handover_ready = 1; }
                pthread_mutex_unlock(&g_handover_lock);
                continue; // keine Antwort senden
            }

            // Befehl ausfuehren und Antwort schicken
            char resp[RESP_BUFSZ]; int close_it=0;
            process_line(line,resp,sizeof(resp),&close_it);
            if(send_all(fd,resp,strlen(resp))<0 || send_all(fd,ENDMARK,strlen(ENDMARK))<0) close_it=1;
            if(close_it) goto done;
        }

        // Restfragment nach vorn schieben
        if(start){
            size_t remain=acc_len-start;
            memmove(acc,acc+start,remain);
            acc_len=remain;
        }
    }
done:
    track_del(fd);
    close(fd);
    return NULL;
}

// -------------------- Server-Thread: Listen + Accept --------------------
typedef struct { int port; } ServerArg;

static void *server_thread(void *arg){
    int port=((ServerArg*)arg)->port; free(arg);

    // Socket anlegen
    int srv=socket(AF_INET,SOCK_STREAM,0); if(srv<0){ perror("socket"); return NULL; }
    int opt=1; setsockopt(srv,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(srv,SOL_SOCKET,SO_REUSEPORT,&opt,sizeof(opt));
#endif

    // lokal an 127.0.0.1:port binden
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family=AF_INET; addr.sin_port=htons((unsigned short)port);
    inet_pton(AF_INET,"127.0.0.1",&addr.sin_addr);
    if(bind(srv,(struct sockaddr*)&addr,sizeof(addr))<0){ perror("bind"); close(srv); return NULL; }
    if(listen(srv,BACKLOG)<0){ perror("listen"); close(srv); return NULL; }

    // Listener global merken (fuer Handover-Schliessung)
    pthread_mutex_lock(&g_listen_lock);
    g_listen_fd = srv;
    pthread_mutex_unlock(&g_listen_lock);

    printf("[peer] Server-Modus auf 127.0.0.1:%d\n", port);

    // neue Clients annehmen
    for(;;){
        int cfd=accept(srv,NULL,NULL);
        if(cfd<0){ if(errno==EINTR) continue; perror("accept"); break; }
        enable_keepalive(cfd);
        ConnArg *ca=malloc(sizeof(*ca)); if(!ca){ close(cfd); continue; }
        ca->cfd=cfd;
        pthread_t th; pthread_create(&th,NULL,conn_thread,ca); pthread_detach(th);
    }

    // Listener austragen und schliessen
    pthread_mutex_lock(&g_listen_lock);
    if (g_listen_fd == srv) g_listen_fd = -1;
    pthread_mutex_unlock(&g_listen_lock);

    close(srv);
    return NULL;
}

// Startet Server-Thread, aber prueft vorher mit Test-Bind ob Port frei ist
static int start_server_on_port(int port){
    int test=socket(AF_INET,SOCK_STREAM,0); if(test<0) return -1;
    int opt=1; setsockopt(test,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(test,SOL_SOCKET,SO_REUSEPORT,&opt,sizeof(opt));
#endif
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family=AF_INET; addr.sin_port=htons((unsigned short)port);
    inet_pton(AF_INET,"127.0.0.1",&addr.sin_addr);
    if(bind(test,(struct sockaddr*)&addr,sizeof(addr))<0){ close(test); return -1; }
    close(test);

    ServerArg *sa=malloc(sizeof(*sa)); if(!sa) return -1; sa->port=port;
    pthread_t th; pthread_create(&th,NULL,server_thread,sa); pthread_detach(th);
    return 0;
}

// -------------------- Client + Async-Receiver --------------------
// Verbindung zu lokalem Server aufbauen
static int connect_remote_local(int port){
    int fd=socket(AF_INET,SOCK_STREAM,0); if(fd<0){ perror("socket"); return -1; }
    enable_keepalive(fd);
    struct sockaddr_in srv; memset(&srv,0,sizeof(srv));
    srv.sin_family=AF_INET; srv.sin_port=htons((unsigned short)port);
    inet_pton(AF_INET,"127.0.0.1",&srv.sin_addr);
    if(connect(fd,(struct sockaddr*)&srv,sizeof(srv))<0){ perror("connect"); close(fd); return -1; }
    return fd;
}

// Remote-Reader: liest Antworten und Steuerframes asynchron
static pthread_t g_rx_thread;
static pthread_mutex_t g_rfd_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_rfd = -1;
static volatile int g_rx_run = 0;

static void *remote_rx(void *arg){
    (void)arg;
    char acc[RESP_BUFSZ]; size_t used=0; acc[0]='\0';

    for(;;){
        if(!g_rx_run) break;

        // aktuellen FD lesen
        pthread_mutex_lock(&g_rfd_lock);
        int fd = g_rfd;
        pthread_mutex_unlock(&g_rfd_lock);

        if(fd<0){
            struct timespec ts={0,100*1000*1000}; nanosleep(&ts,NULL);
            continue;
        }

        // Daten vom Server
        char buf[1024+1];
        ssize_t n=recv(fd,buf,1024,0);
        if(n<=0){
            if(n<0 && errno==EINTR) continue;
            pthread_mutex_lock(&g_rfd_lock);
            if(g_rfd>=0){ close(g_rfd); g_rfd=-1; }
            pthread_mutex_unlock(&g_rfd_lock);
            continue;
        }
        buf[n]='\0';

        // Antwort in Akkumulator
        if(used+(size_t)n+1>=sizeof(acc)){ used=0; acc[0]='\0'; }
        memcpy(acc+used,buf,(size_t)n); used+=(size_t)n; acc[used]='\0';

        // kompletten Block bis ENDMARK?
        char *mark=strstr(acc,ENDMARK);
        if(!mark) continue;

        *mark='\0'; // Block isolieren

        // Steuerframes behandeln
        if(strncmp(acc,"@handover",9)==0){
            // Port extrahieren, Liste importieren
            int port_to_take=g_port;
            const char *eol=strchr(acc,'\n');
            if(eol){
                const char *p=strstr(acc,"port=");
                if(p && p<eol){ int pn=atoi(p+5); if(pn>0 && pn<65536) port_to_take=pn; }
            }
            const char *payload=eol?eol+1:acc+10;
            import_list(payload);
            printf("[peer] Handover empfangen\n");

            // ACK an alten Leader
            const char *ack="@ready\n";
            (void)send_all(fd, ack, strlen(ack));

            // Listener auf gleichem Port uebernehmen (retry bis 5s)
            int ok = -1;
            for (int attempt = 0; attempt < 50; ++attempt) {
                ok = start_server_on_port(port_to_take);
                if (ok == 0) break;
                struct timespec ts={0,100*1000*1000}; nanosleep(&ts,NULL);
            }
            if (ok == 0) printf("[peer] Port %d uebernommen\n", port_to_take);
            else         printf("[peer] Port %d nicht uebernommen\n", port_to_take);

            // Verbindung schliessen: wir sind jetzt Leader
            pthread_mutex_lock(&g_rfd_lock);
            if(g_rfd>=0){ close(g_rfd); g_rfd=-1; }
            pthread_mutex_unlock(&g_rfd_lock);
        }
        else if(strncmp(acc,"@newleader ",11)==0){
            // immer lokal verbinden
            int port=0; char host_ignore[64];
            if(sscanf(acc+11,"%63[^:]:%d",host_ignore,&port)==2 && port>0 && port<65536){
                printf("[peer] neuer Leader %d\n",port);
                int okc=-1;
                for(int attempt=0; attempt<20; ++attempt){
                    int fd2=connect_remote_local(port);
                    if(fd2>=0){
                        pthread_mutex_lock(&g_rfd_lock);
                        if(g_rfd>=0) close(g_rfd);
                        g_rfd=fd2;
                        pthread_mutex_unlock(&g_rfd_lock);
                        okc=0; break;
                    }
                    struct timespec ts={0,100*1000*1000}; nanosleep(&ts,NULL);
                }
                if(okc==0) puts("[peer] verbunden");
                else       puts("[peer] reconnect fehlgeschlagen");
            }
        }
        else {
            // normale Server-Antwort
            printf("%s", acc);
        }

        // Rest hinter dem verarbeiteten Block nach vorn ziehen
        size_t processed = (size_t)((mark - acc) + strlen(ENDMARK));
        if (processed > used) { used = 0; acc[0] = '\0'; }
        else {
            size_t remain = used - processed;
            if (remain > 0) { memmove(acc, acc + processed, remain); used = remain; acc[used] = '\0'; }
            else { used = 0; acc[0] = '\0'; }
        }
    }
    return NULL;
}

// Remote-Verbindung starten und RX-Thread sicherstellen
static void remote_connect_start_rx_local(int port){
    int fd=connect_remote_local(port);
    if(fd<0){ printf("[peer] Client-Verbindung fehlgeschlagen\n"); return; }
    pthread_mutex_lock(&g_rfd_lock);
    if(g_rfd>=0) close(g_rfd);
    g_rfd=fd;
    pthread_mutex_unlock(&g_rfd_lock);
    if(!g_rx_run){ g_rx_run=1; pthread_create(&g_rx_thread,NULL,remote_rx,NULL); pthread_detach(g_rx_thread); }
    printf("[peer] Client-Modus > 127.0.0.1:%d\n",port);
}

// Remote-Verbindung stoppen
static void remote_stop_rx(void){
    pthread_mutex_lock(&g_rfd_lock);
    if(g_rfd>=0){ close(g_rfd); g_rfd=-1; }
    pthread_mutex_unlock(&g_rfd_lock);
}

// -------------------- Shutdown / Handover --------------------
static volatile sig_atomic_t g_shutdown = 0;
static void on_sigint(int sig){ (void)sig; g_shutdown=1; }

// Rollenuebergabe an aeltesten Client
static void try_handover(void){
    int fd=pick_oldest_fd();
    if(fd<0){ fprintf(stderr,"[peer] kein Client fuer Handover\n"); return; }

    // Merken, auf welchem FD wir @ready erwarten
    pthread_mutex_lock(&g_handover_lock);
    g_handover_fd = fd;
    g_handover_ready = 0;
    pthread_mutex_unlock(&g_handover_lock);

    // Liste exportieren und senden
    char snap[RESP_BUFSZ]; export_list(snap,sizeof(snap));
    char hdr[64]; snprintf(hdr,sizeof(hdr),"@handover port=%d\n",g_port);

    if(send_all(fd,hdr,strlen(hdr))==0 &&
       send_all(fd,snap,strlen(snap))==0 &&
       send_all(fd,ENDMARK,strlen(ENDMARK))==0) {
        fprintf(stderr,"[peer] Handover gesendet, warte auf @ready...\n");
    } else {
        fprintf(stderr,"[peer] Handover fehlgeschlagen\n");
    }

    // bis 2s auf @ready warten
    int ready=0;
    for(int attempt=0; attempt<20; ++attempt){
        pthread_mutex_lock(&g_handover_lock);
        ready = g_handover_ready;
        pthread_mutex_unlock(&g_handover_lock);
        if(ready) break;
        struct timespec ts={0,100*1000*1000}; nanosleep(&ts,NULL);
    }
    if(!ready) fprintf(stderr,"[peer] kein @ready, fahre fort\n");
    else       fprintf(stderr,"[peer] @ready empfangen\n");

    // Listener schliessen (keine neuen Verbindungen mehr)
    stop_listening_socket();

    // WICHTIG: erst Broadcast, dann Verbindungen trennen
    broadcast_newleader_local(g_port);
    track_close_all();
}


// -------------------- Main / REPL --------------------
int main(int argc,char**argv){
    // optionaler Port als Argument
    if (argc >= 2) {
        int p = atoi(argv[1]);
        if (p > 0 && p < 65536) g_port = p;
    }

    signal(SIGINT,on_sigint);
    signal(SIGTERM,on_sigint);

    // Versuche Server auf g_port, sonst Client
    if(start_server_on_port(g_port)==0){
        printf("[peer] Server-Modus aktiv (127.0.0.1:%d)\n",g_port);
    } else {
        printf("[peer] Port %d belegt > Client-Modus\n",g_port);
        remote_connect_start_rx_local(g_port);
    }

    printf("[peer] Befehle: /read | /clear | /delete N | /quit\n");

    char line[IN_BUFSZ];
    for(;;){
        // Strg+C loest geordneten Handover aus
        if(g_shutdown){ try_handover(); break; }

        // nicht blockieren, damit wir Signale zeitnah sehen
        fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO,&rfds);
        struct timeval tv={0,200000}; // 200 ms Polling
        int rc=select(STDIN_FILENO+1,&rfds,NULL,NULL,&tv);
        if(rc<0){ if(errno==EINTR) continue; perror("select"); break; }
        if(rc==0) continue;

        // Eingabe lesen
        if(!fgets(line,sizeof(line),stdin)) break;
        trim_eol(line); if(!line[0]) continue;

        // /quit: remote benachrichtigen (falls Client), dann Handover anstossen
        if(strcmp(line,"/quit")==0){
            pthread_mutex_lock(&g_rfd_lock);
            int fd=g_rfd;
            pthread_mutex_unlock(&g_rfd_lock);
            if(fd>=0){ const char *q="/quit\n"; (void)send_all(fd,q,strlen(q)); }
            try_handover();
            break;
        }

        // lokal (Server) oder remote (Client) ausfuehren
        pthread_mutex_lock(&g_rfd_lock);
        int fd=g_rfd;
        pthread_mutex_unlock(&g_rfd_lock);

        if(fd<0){
            // lokal: direkt Serverlogik rufen
            char resp[RESP_BUFSZ]; int dummy=0;
            char withnl[IN_BUFSZ]; snprintf(withnl,sizeof(withnl),"%s\n",line);
            process_line(withnl,resp,sizeof(resp),&dummy);
            printf("%s",resp);
        } else {
            // remote: senden, Antwort kommt asynchron im RX-Thread
            char snd[IN_BUFSZ]; snprintf(snd,sizeof(snd),"%s\n",line);
            if(send_all(fd,snd,strlen(snd))<0){
                perror("send");
                remote_stop_rx();
                puts("[peer] remote getrennt");
            }
        }
    }

    // Aufraeumen
    remote_stop_rx();
    return 0;
}
