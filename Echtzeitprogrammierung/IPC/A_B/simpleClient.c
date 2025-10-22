// Client: sendet eine Zahl an den Server; Server liefert Quersumme zurück.
//
// Kompilieren:
//   gcc -D_XOPEN_SOURCE=700 -O2 -Wall -Wextra -o client_quersumme client_quersumme.c
//
// Start:
//   ./client_quersumme
//   (danach Zahlen eingeben, z. B. 12345 ; Ende mit /quit oder Strg+D)

#include "simpleHeader.h"

int main(void){
    // --- Key wie beim Server ---
    key_t key = ftok(FTOK_PATH, FTOK_PROJ);
    if (key == -1) die("ftok");

    // --- SHM & Semaphor öffnen (Server muss laufen) ---
    int shmid = shmget(key, sizeof(ShmBlock), 0666);
    if (shmid == -1) die("shmget(open)");
    ShmBlock *shm = (ShmBlock*) shmat(shmid, NULL, 0);
    if (shm == (void*)-1) die("shmat");

    int semid = semget(key, 1, 0666);
    if (semid == -1) die("semget(open)");

    printf("Quersummen-Client. Zahl eingeben und Enter drücken.\n");
    printf("Befehle: /quit beendet.\n");

    char line[256];
    struct timespec ts = {0, 20 * 1000 * 1000}; // 20 ms warten beim Polling

    for (;;) {
        printf("> ");
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) break;      // EOF/Strg+D
        size_t len = strlen(line);
        if (len && line[len-1] == '\n') line[len-1] = '\0';

        if (strcmp(line, "/quit") == 0) break;
        if (line[0] == '\0') continue;                     // leere Eingabe ignorieren

        // Zahl parsen (einfach gehalten): strtoll akzeptiert +/- und prüft Fehler robust
        char *endp = NULL;
        long long v = strtoll(line, &endp, 10);
        if (endp == line || *endp != '\0') {
            printf("CLIENT: Bitte eine ganze Zahl eingeben (oder /quit).\n");
            continue;
        }

        // --- Anfrage senden (geschützt durch Mutex) ---
        sem_lock(semid);

        shm->op        = OP_QSUM;
        shm->sender    = getpid();
        shm->value     = (int64_t)v;
        shm->resp_ready= 0;       // alte Antwort ignorieren
        shm->req_ready = 1;       // Anfrage liegt an

        sem_unlock(semid);

        // --- Auf Antwort warten ---
        while (!shm->resp_ready) nanosleep(&ts, NULL);

        // --- Antwort lesen ---
        printf("SERVER:\n\n");
        printf("  %s\n", shm->info);
        printf("  Ergebnis: %d\n\n", shm->result);

        // optional zurücksetzen (hilft für saubere Zyklen)
        sem_lock(semid);
        shm->resp_ready = 0;
        sem_unlock(semid);
    }

    if (shmdt(shm) == -1) perror("shmdt");
    return 0;
}
