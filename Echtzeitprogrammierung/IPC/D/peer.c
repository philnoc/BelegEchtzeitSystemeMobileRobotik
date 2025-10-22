// peer.c — Gleichberechtigter Peer (Teilnehmer) für Shared-Memory-Liste (Teilaufgabe d)
// Nutzung:      Tippe Text = append
//               /read      = Liste anzeigen
//               /delete N  = Eintrag N löschen (IDs beginnen bei 1)
//               /clear     = Liste leeren
//               /quit      = beenden

#define _XOPEN_SOURCE 700
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>

// Konfiguration
#define FTOK_PATH "/tmp/ipc_shm_demo_key"   // Datei für ftok (wird angelegt)
#define FTOK_PROJ 'S'

#define MSG_LEN    256     // max. Text pro Listeneintrag
#define REQ_SIZE   256     // max. Eingabetext
#define RESP_SIZE  4096    // Puffer zum formatierten Anzeigen der Liste
#define MAX_MSG    256     // Anzahl Einträge in Liste (256*256 ≈ 64 KiB plus Overhead)

// Datenstrukturen im Shared Memory
typedef struct {
    char   text[MSG_LEN];
    time_t ts;
} ListEntry;

typedef struct {
    int       count;                      // 0..MAX_MSG
    ListEntry entries[MAX_MSG];
} SharedList;

typedef struct {
    int    refcount;                      // Anzahl verbundener Peers
    pid_t  creator;                       // PID des Peers, der initial erstellt hat (nur Info)
    SharedList list;                      // gemeinsame Liste
} ShmState;

// SysV semctl union
union semun { int val; struct semid_ds *buf; unsigned short *array; };

// Globale Handles
static int shmid = -1, semid = -1;
static ShmState *state = NULL;

// Utils/ Hilfsfunktionen
static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

static void ensure_ftok_file(void){
    FILE *f = fopen(FTOK_PATH, "a");
    if (f) fclose(f);
}

// Binärer Mutex via Semaphor[0]
static void sem_lock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = -1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(-1)");
}
static void sem_unlock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = +1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(+1)");
}

static void cleanup_and_exit(int sig){
    (void)sig;

    // refcount-Logik: nur letzter Peer räumt auf
    if (semid != -1 && state) {
        sem_lock(semid);
        if (state->refcount > 0) state->refcount--;
        int last = (state->refcount == 0);
        sem_unlock(semid);

        if (last) {
            // letzter Peer: SHM & SEM entfernen
            if (shmid != -1) shmctl(shmid, IPC_RMID, NULL);
            if (semid != -1) semctl(semid, 0, IPC_RMID, (union semun){0});
        }
    }

    if (state && state != (void*)-1) shmdt(state);

    fprintf(stderr, "\n[peer] bye.\n");
    _exit(0);
}

// Operationen

// Append: neuer Listeneintrag
static void op_append(const char *text, char *resp, size_t rsz){
    // Eingabe validieren (kein Text oder zu lang)
    if (!text || text[0] == '\0') {
        snprintf(resp, rsz, "Fehler: leerer Text");
        return;
    }
    if (strlen(text) >= MSG_LEN) {
        snprintf(resp, rsz, "Fehler: Eintrag zu lang (max %d Zeichen)", MSG_LEN-1);
        return;
    }

    sem_lock(semid);
    if (state->list.count >= MAX_MSG) {
        sem_unlock(semid);
        snprintf(resp, rsz, "Fehler: Liste voll (%d)", MAX_MSG);
        return;
    }
    int idx = state->list.count++;
    snprintf(state->list.entries[idx].text, MSG_LEN, "%s", text);
    state->list.entries[idx].ts = time(NULL);
    sem_unlock(semid);

    snprintf(resp, rsz, "OK: Eintrag (%d) hinzugefügt", idx + 1);
}

// Read: Liste formatiert ausgeben (IDs ab 0, Zeitstempel)
static void op_read(char *resp, size_t rsz){
    
    ListEntry snap[MAX_MSG];
    int n;
    sem_lock(semid);
    n = state->list.count;
    if (n > MAX_MSG) n = MAX_MSG;
    for (int i = 0; i < n; i++) snap[i] = state->list.entries[i];
    sem_unlock(semid);

    // Formatierung außerhalb des Locks
    char buf[RESP_SIZE]; buf[0] = '\0';
    for (int i = 0; i < n; i++) {
        char tbuf[32] = {0};
        struct tm tm; localtime_r(&snap[i].ts, &tm);
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm);
        char line[MSG_LEN + 64];
        snprintf(line, sizeof(line), "(%d) [%s] %s\n", i, tbuf, snap[i].text);
        if (strlen(buf) + strlen(line) + 1 < sizeof(buf)) strcat(buf, line);
        else break;
    }
    snprintf(resp, rsz, "%s", buf[0] ? buf : "(leer)\n");
}

// Delete: Eintrag N löschen, Nachrücken
static void op_delete(int id, char *resp, size_t rsz){
    if (id < 0) {
        snprintf(resp, rsz, "Fehler: ungültige ID (%d)", id);
        return;
    }

    sem_lock(semid);
    int n = state->list.count;
    if (id >= n) {
        sem_unlock(semid);
        snprintf(resp, rsz, "Fehler: ID (%d) existiert nicht (Anzahl=%d)", id, n);
        return;
    }
    for (int i = id; i < n - 1; i++) {
        state->list.entries[i] = state->list.entries[i + 1];
    }
    state->list.count = n - 1;
    sem_unlock(semid);

    snprintf(resp, rsz, "OK: Eintrag (%d) gelöscht", id);
}

// Clear: Liste leeren
static void op_clear(char *resp, size_t rsz){
    sem_lock(semid);
    state->list.count = 0;
    sem_unlock(semid);
    snprintf(resp, rsz, "Liste wurde gelöscht");
}

int main(void){
    signal(SIGINT,  cleanup_and_exit);
    signal(SIGTERM, cleanup_and_exit);

    // Key & Ressourcen
    ensure_ftok_file();
    key_t key = ftok(FTOK_PATH, FTOK_PROJ);
    if (key == -1) die("ftok");

    int created = 0;
    shmid = shmget(key, sizeof(ShmState), IPC_CREAT | IPC_EXCL | 0666);
    if (shmid == -1) {
        if (errno != EEXIST) die("shmget(create)");
        shmid = shmget(key, sizeof(ShmState), 0666);
        if (shmid == -1) die("shmget(open)");
    } else {
        created = 1;
    }

    state = (ShmState*) shmat(shmid, NULL, 0);
    if (state == (void*)-1) die("shmat");
    if (created) {
        memset(state, 0, sizeof(*state));
        state->creator = getpid();
    }

    int sem_created = 0;
    semid = semget(key, 1, IPC_CREAT | IPC_EXCL | 0666);
    if (semid == -1) {
        if (errno != EEXIST) die("semget(create)");
        semid = semget(key, 1, 0666);
        if (semid == -1) die("semget(open)");
    } else {
        sem_created = 1;
    }
    if (sem_created) {
        union semun u; u.val = 1;                         // Mutex frei
        if (semctl(semid, 0, SETVAL, u) == -1) die("semctl(SETVAL)");
    }

    // Refcount inkrementieren
    sem_lock(semid);
    state->refcount++;
    int current = state->refcount;
    sem_unlock(semid);

    printf("[peer] ready (shmid=%d, semid=%d, refcount=%d)\n", shmid, semid, current);
    printf("Befehle: /read | /clear | /delete N | /quit\n");
    printf("Textzeile ohne Slash = append\n");

    // Interaktive Schleife
    char line[REQ_SIZE * 2];
    for (;;) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break; // EOF/Ctrl+D

        size_t len = strlen(line);
        if (len && line[len-1] == '\n') line[len-1] = '\0';
        if (strcmp(line, "/quit") == 0) break;

        char resp[RESP_SIZE];
        if (strcmp(line, "/read") == 0) {
            // Formatwunsch: Leerzeile nach "PEER:" für einheitliche Optik
            op_read(resp, sizeof(resp));
            printf("PEER:\n\n%s", resp);
        } else if (strcmp(line, "/clear") == 0) {
            op_clear(resp, sizeof(resp));
            printf("PEER: %s\n", resp);
        } else if (strncmp(line, "/delete ", 8) == 0) {
            char *p = line + 8;
            char *endp = NULL;
            long id = strtol(p, &endp, 10);
            if (endp == p) {
                printf("PEER: Fehler: Bitte /delete <ID>\n");
                continue;
            }
            op_delete((int)id, resp, sizeof(resp));
            printf("PEER: %s\n", resp);
        } else if (line[0] == '/') {
            printf("PEER: Unbekannter Befehl: %s\n", line);
        } else {
            // Append normaler Text
            // falls zu lang wird abgeschnitten
            if (strlen(line) >= MSG_LEN) line[MSG_LEN-1] = '\0';
            op_append(line, resp, sizeof(resp));
            printf("PEER: %s\n", resp);
        }
    }

    cleanup_and_exit(0);
    return 0; // nie erreicht
}
