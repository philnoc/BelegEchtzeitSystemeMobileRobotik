// Server (Teilaufgabe a): Nimmt Anfragen entgegen und beantwortet sie.
// Beispiel: Client sendet eine Zahl, Server antwortet mit der Quersumme.
//
// Kompilieren:
//   gcc -D_XOPEN_SOURCE=700 -O2 -Wall -Wextra -o server_quersumme server_quersumme.c
//
// Start:
//   ./server_quersumme
//
// Manuelle Aufräum-Hinweise (nur falls der Server hart beendet wurde):
//   ipcs -m -s
//   ipcrm -m <SHMID> ; ipcrm -s <SEMID>

#include "simpleHeader.h"
#include <signal.h>

static int shmid = -1, semid = -1;
static ShmBlock *shm = NULL;

// Quersumme bilden (Ziffernsumme einer ganzen Zahl, Vorzeichen ignorieren)
static int quersumme(int64_t x) {
    if (x < 0) x = -x;
    int sum = 0;
    while (x > 0) {
        sum += (int)(x % 10);
        x /= 10;
    }
    return sum;
}

// sauberes Aufräumen bei SIGINT/SIGTERM
static void cleanup(int sig){
    (void)sig;
    if (shm && shm != (void*)-1) shmdt(shm);
    if (shmid != -1) shmctl(shmid, IPC_RMID, NULL);
    if (semid != -1) semctl(semid, 0, IPC_RMID, (union semun){0});
    fprintf(stderr, "\n[server] cleaned up.\n");
    _exit(0);
}

int main(void){
    signal(SIGINT,  cleanup);
    signal(SIGTERM, cleanup);

    // --- Key erzeugen (ftok) ---
    ensure_ftok_file();
    key_t key = ftok(FTOK_PATH, FTOK_PROJ);
    if (key == -1) die("ftok");

    // --- Shared Memory anlegen/öffnen ---
    int created = 0;
    shmid = shmget(key, sizeof(ShmBlock), IPC_CREAT | IPC_EXCL | 0666);
    if (shmid == -1) {
        if (errno != EEXIST) die("shmget(create)");
        shmid = shmget(key, sizeof(ShmBlock), 0666);
        if (shmid == -1) die("shmget(open)");
    } else {
        created = 1;
    }

    // --- SHM anhängen ---
    shm = (ShmBlock*) shmat(shmid, NULL, 0);
    if (shm == (void*)-1) die("shmat");
    if (created) memset(shm, 0, sizeof(*shm)); // Flags/Buffer nullen

    // --- Semaphor (1 Stück) anlegen/öffnen, als Mutex ---
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
        union semun u; u.val = 1; // frei = 1
        if (semctl(semid, 0, SETVAL, u) == -1) die("semctl(SETVAL)");
    }

    printf("[server] ready  shmid=%d  semid=%d\n", shmid, semid);
    printf("  ipcs -m -s\n");
    printf("  ipcrm -m %d ; ipcrm -s %d\n", shmid, semid);
    printf("  Warte auf Anfragen: Client setzt req_ready=1 und füllt op/value.\n");

    // --- Hauptloop: auf Anfragen reagieren ---
    struct timespec ts = {0, 50 * 1000 * 1000}; // 50 ms „sanftes Polling“
    for (;;) {
        // Wenn keine Anfrage anliegt, kurz schlafen (CPU schonen)
        if (!shm->req_ready) { nanosleep(&ts, NULL); continue; }

        // Kritischen Bereich betreten (alle SHM-Zugriffe sind nun exklusiv)
        sem_lock(semid);

        if (shm->req_ready) {
            // Anfrage „schnappen“
            int      op = shm->op;
            int64_t  val = shm->value;
            pid_t    who = shm->sender;

            // Anfrage als „verbraucht“ markieren
            shm->req_ready = 0;

            // Verarbeiten
            if (op == OP_QSUM) {
                int s = quersumme(val);
                shm->result = s;
                snprintf(shm->info, sizeof(shm->info),
                         "PID=%d: Quersumme(%lld) = %d",
                         (int)who, (long long)val, s);
            } else {
                shm->result = 0;
                snprintf(shm->info, sizeof(shm->info),
                         "Unbekannte Operation: %d", op);
            }

            // Antwort liegt vor
            shm->resp_ready = 1;
        }

        // Kritischen Bereich verlassen
        sem_unlock(semid);
    }

    // (nie erreicht)
    return 0;
}
