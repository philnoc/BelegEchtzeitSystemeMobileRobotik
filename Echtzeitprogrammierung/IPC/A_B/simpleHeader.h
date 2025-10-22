#ifndef SIMPLE_HEADER_H
#define SIMPLE_HEADER_H


#define _XOPEN_SOURCE 700

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

// ---- Gemeinsamer ftok()-Schlüssel ----
#define FTOK_PATH "/tmp/ipc_shm_calc_key"   // wird bei Bedarf erzeugt
#define FTOK_PROJ 'Q'                        // Projekt-ID (beliebig, aber stabil)

// ---- Protokoll-Flags und Größen ----
// Hier sind keine großen Textpuffer nötig; wir senden nur Zahlen + Flags.
enum OpCode {
    OP_NONE    = 0,
    OP_QSUM    = 1,   // Quersumme berechnen
};

// ---- Shared-Memory-Datenblock ----
// Sehr kompakt: Flags + Operation + Request/Response-Werte.
typedef struct {
    volatile int req_ready;   // Client: 1 = Anfrage liegt an
    volatile int resp_ready;  // Server: 1 = Antwort liegt an

    int   op;                 // OP_QSUM (kann später erweitert werden)
    pid_t sender;             // PID des Clients (nur Info/Debug)

    int64_t  value;           // Anfragewert (vom Client gesetzt)
    int      result;          // Ergebnis (vom Server gesetzt)

    char     info[128];       // kurze Info-/Fehlermeldung
} ShmBlock;

// ---- union semun (wird von glibc nicht global definiert) ----
union semun { int val; struct semid_ds *buf; unsigned short *array; };

// ---- kleine Hilfsfunktionen: Fehler & Semaphor-Mutex ----
static inline void die(const char *m) {
    perror(m);
    exit(EXIT_FAILURE);
}

// Binärer Mutex Semaphor[0]: -1 = Lock, +1 = Unlock (mit SEM_UNDO für Robustheit)
static inline void sem_lock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = -1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(-1)");
}
static inline void sem_unlock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = +1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(+1)");
}

// ftok()-Datei sicherstellen (wird erzeugt, falls nicht da)
static inline void ensure_ftok_file(void){
    FILE *f = fopen(FTOK_PATH, "a");
    if (f) fclose(f);
}

#endif
