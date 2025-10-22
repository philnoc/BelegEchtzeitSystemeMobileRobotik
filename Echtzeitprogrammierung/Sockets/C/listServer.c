// list_server_persist.c
// TCP-Server mit select(): mehrere Befehle pro Verbindung, --END-- als Antwort-Delimiter.
// Befehle: /read | /clear | /delete N | (sonst: append) | /quit (schließt nur diese Verbindung)

#define _XOPEN_SOURCE 700
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PORT      5000
#define BACKLOG   64
#define MAXFD     FD_SETSIZE
#define IN_BUFSZ  4096
#define RESP_BUFSZ 8192
#define MSG_LEN   256
#define MAX_MSG   256

typedef struct {
    char   text[MSG_LEN];
    time_t ts;
} ListEntry;

typedef struct {
    int count;
    ListEntry entries[MAX_MSG];
} SharedList;

static SharedList g_list = {0};
static const char *ENDMARK = "--END--\n";

static int send_all(int fd, const char *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = send(fd, buf + off, len - off, 0);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        off += (size_t)n;
    }
    return 0;
}
static void trim_newline(char *s) {
    size_t n = strlen(s);
    while (n && (s[n-1] == '\n' || s[n-1] == '\r')) s[--n] = '\0';
}

static void handle_append(const char *payload, char *resp, size_t rsz) {
    if (g_list.count >= MAX_MSG) { snprintf(resp, rsz, "Fehler: Liste voll (%d)\n", MAX_MSG); return; }
    int idx = g_list.count++;
    snprintf(g_list.entries[idx].text, MSG_LEN, "%s", payload);
    g_list.entries[idx].ts = time(NULL);
    snprintf(resp, rsz, "OK: Eintrag (%d) hinzugefügt\n", idx);
}
static void handle_read(char *resp, size_t rsz) {
    if (g_list.count == 0) { snprintf(resp, rsz, "(leer)\n"); return; }
    resp[0] = '\0';
    char line[MSG_LEN + 64];
    for (int i = 0; i < g_list.count; i++) {
        char tbuf[32] = {0};
        struct tm tm; localtime_r(&g_list.entries[i].ts, &tm);
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm);
        snprintf(line, sizeof(line), "(%d) [%s] %s\n", i, tbuf, g_list.entries[i].text);
        if (strlen(resp) + strlen(line) + 1 >= rsz) break;
        strcat(resp, line);
    }
}
static void handle_clear(char *resp, size_t rsz) {
    memset(&g_list, 0, sizeof(g_list));
    snprintf(resp, rsz, "Liste wurde gelöscht\n");
}
static void handle_delete(const char *payload, char *resp, size_t rsz) {
    char *endp = NULL; long id = strtol(payload, &endp, 10);
    if (endp == payload || *endp != '\0' || id < 0 || id >= g_list.count) {
        snprintf(resp, rsz, "Fehler: ungültige ID '%s'\n", payload); return;
    }
    for (int i = (int)id; i < g_list.count - 1; i++) g_list.entries[i] = g_list.entries[i+1];
    g_list.count--;
    snprintf(resp, rsz, "OK: Eintrag (%ld) gelöscht\n", id);
}

typedef struct {
    char inbuf[IN_BUFSZ];
    size_t inlen;
} ConnState;

int main(void) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return EXIT_FAILURE; }
    int opt = 1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr; memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET; addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(srv); return EXIT_FAILURE; }
    if (listen(srv, BACKLOG) < 0) { perror("listen"); close(srv); return EXIT_FAILURE; }

    fd_set master, readfds; FD_ZERO(&master); FD_SET(srv, &master);
    int fdmax = srv;

    ConnState state[MAXFD]; memset(state, 0, sizeof(state));

    for (;;) {
        readfds = master;
        int rc = select(fdmax + 1, &readfds, NULL, NULL, NULL);
        if (rc < 0) { if (errno == EINTR) continue; perror("select"); break; }

        for (int fd = 0; fd <= fdmax; fd++) {
            if (!FD_ISSET(fd, &readfds)) continue;

            if (fd == srv) {
                // neue Verbindung
                int cfd = accept(srv, NULL, NULL);
                if (cfd < 0) { perror("accept"); continue; }
                if (cfd >= MAXFD) { fprintf(stderr, "FD too large\n"); close(cfd); continue; }
                FD_SET(cfd, &master);
                if (cfd > fdmax) fdmax = cfd;
                state[cfd].inlen = 0;
            } else {
                char buf[1024];
                ssize_t n = recv(fd, buf, sizeof(buf), 0);
                if (n <= 0) {
                    close(fd); FD_CLR(fd, &master); state[fd].inlen = 0;
                    continue;
                }
                // Anhängen an inbuf
                ConnState *st = &state[fd];
                if (st->inlen + (size_t)n > sizeof(st->inbuf)) {
                    // zu groß -> zurücksetzen
                    st->inlen = 0;
                } else {
                    memcpy(st->inbuf + st->inlen, buf, (size_t)n);
                    st->inlen += (size_t)n;
                }
                // Zeilen weise verarbeiten
                size_t start = 0;
                for (;;) {
                    // nach \n suchen
                    char *nl = memchr(st->inbuf + start, '\n', st->inlen - start);
                    if (!nl) break;
                    size_t linelen = (size_t)(nl - (st->inbuf + start)) + 1; // inkl. \n
                    char line[IN_BUFSZ];
                    if (linelen >= sizeof(line)) linelen = sizeof(line) - 1;
                    memcpy(line, st->inbuf + start, linelen);
                    line[linelen] = '\0';
                    start += linelen;

                    // Befehl verarbeiten
                    char resp[RESP_BUFSZ];
                    char work[IN_BUFSZ]; snprintf(work, sizeof(work), "%s", line);
                    trim_newline(work);

                    int should_close = 0;
                    if (strcmp(work, "/quit") == 0) {
                        snprintf(resp, sizeof(resp), "bye\n");
                        should_close = 1;
                    } else if (strcmp(work, "/read") == 0) {
                        handle_read(resp, sizeof(resp));
                    } else if (strcmp(work, "/clear") == 0) {
                        handle_clear(resp, sizeof(resp));
                    } else if (strncmp(work, "/delete ", 8) == 0) {
                        handle_delete(work + 8, resp, sizeof(resp));
                    } else {
                        handle_append(work, resp, sizeof(resp));
                    }

                    // Antwort + Delimiter senden
                    if (send_all(fd, resp, strlen(resp)) < 0 || send_all(fd, ENDMARK, strlen(ENDMARK)) < 0) {
                        should_close = 1;
                    }

                    if (should_close) {
                        close(fd); FD_CLR(fd, &master); st->inlen = 0;
                        break; // fd ist zu, raus
                    }
                }
                // unverbrauchte Bytes nach vorne schieben
                if (FD_ISSET(fd, &master)) {
                    size_t remain = st->inlen - start;
                    memmove(st->inbuf, st->inbuf + start, remain);
                    st->inlen = remain;
                }
            }
        }
    }

    close(srv);
    return EXIT_SUCCESS;
}
