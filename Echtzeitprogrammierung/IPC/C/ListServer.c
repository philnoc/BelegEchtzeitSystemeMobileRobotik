#include "listappl.h"

// Globale Handles System-V-Objekte
static int shmid = -1, semid = -1;              // IDs für SHM und Semaphore-Set                     
static ShmBlock *shm = NULL;                    // Pointer auf SHM Objekt

// Datei für ftok (eindeutiger Schlüssel für IPC)
static void ensure_ftok_file(void){
    FILE *f = fopen(FTOK_PATH, "a");
    if (f) fclose(f);
}

// Sauberes Aufräumen nach Ctrl+C 
static void cleanup(int sig){
    (void)sig;
    if (shm && shm != (void*)-1) shmdt(shm);                            // lösen von Shared Memory
    if (shmid != -1) shmctl(shmid, IPC_RMID, NULL);                     // SHM-Objekt löschen
    if (semid != -1) semctl(semid, 0, IPC_RMID, (union semun){0});      // Semaphor-Set löschen
    fprintf(stderr, "\n[server] cleaned up.\n");
    _exit(0);
}

int main(void){
    signal(SIGINT,  cleanup);
    signal(SIGTERM, cleanup);

    // Schlüssel erzeugen
    ensure_ftok_file();
    key_t key = ftok(FTOK_PATH, FTOK_PROJ); // Key aus Datei & Projekt ID
    if (key == -1) die("ftok");

    // SHM anlegen oder öffnen
    int created = 0;
    shmid = shmget(key, sizeof(ShmBlock), IPC_CREAT | IPC_EXCL | 0666); // SHM ID durch Erzeugung oder Öffnen von SHM Segment durch Schlüssel 
    if (shmid == -1) {
        if (errno != EEXIST) die("shmget(create)");
        shmid = shmget(key, sizeof(ShmBlock), 0666);
        if (shmid == -1) die("shmget(open)");
    } else {
        created = 1;
    }

    // SHM an freier Stelle im Prozessraum einhängen (Read and Write Access)
    shm = (ShmBlock*) shmat(shmid, NULL, 0); 
    if (shm == (void*)-1) die("shmat");
    if (created) {
        memset((void*)shm, 0, sizeof(*shm)); // Nullen des SHM
    }

    // Semaphore anlegen
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
        union semun u; u.val = 1;
        if (semctl(semid, 0, SETVAL, u) == -1) die("semctl(SETVAL)");
    }

    // Nutzerhinweise (Konsolenausgabe)
    printf("[server] ready\n  shmid=%d  semid=%d\n", shmid, semid);
    printf("  ipcs -m -s\n");
    printf("  ipcrm -m %d ; ipcrm -s %d\n", shmid, semid);
    printf("  Warte auf Anfragen: Client setzt req_ready=1 und füllt op/request.\n");

    struct timespec ts = {0, 50 * 1000 * 1000}; // 50 ms

    // Mainloop: auf Anfragen im SHM reagieren
    for (;;) {
        if (!shm->req_ready) { nanosleep(&ts, NULL); continue; } // Zyklisches Polling Neuer Request bereit/ anliegend, sonst 50 ms Pause und Sprung zu Beginn Forschleife

        sem_lock(semid);                                                                            // Zugriffssperre für SHM --> Exklusiver Zugriff nur dieses Prozesses

        if (shm->req_ready) {                                                                       // Überprüfung für Sicherheitstest bzgl Race Condition
            int  op = shm->op;                                                                      // Auslesen Operationscode
            char in[REQ_SIZE]; strncpy(in, shm->request, REQ_SIZE-1); in[REQ_SIZE-1] = '\0';        // Anfrage wird kopiert in lokalen Puffer "in"

            shm->req_ready = 0;                                                                     // Anfrage verbrauchen (als bearbeitet markiert)

            
            if (op == OP_APPEND) { // Eintrag zur Liste hinzufügen
                if (shm->list.count < MAX_MSG) {                                                    // Überprüfung Anzahl Listenelemente
                    int idx = shm->list.count++;                                                    // Bestimmung Index für neuen Eintrag
                    snprintf(shm->list.entries[idx].text, MSG_LEN, "%s", in);                       // Text aus Infrage in Liste schreiben
                    shm->list.entries[idx].ts = time(NULL);                                         // setzen Zeitstempel
                    snprintf(shm->response, RESP_SIZE, "OK: Eintrag (%d) hinzugefügt", idx);    // Antwortnachricht für Client
                } else {
                    snprintf(shm->response, RESP_SIZE, "Fehler: Liste voll (%d)", MAX_MSG);         // Fehlermeldung für Client
                }

            } else if (op == OP_READ) { // Liste formatiert ausgeben
                char buf[RESP_SIZE]; buf[0] = '\0';                                                            // lokaler Puffer, beginnt mit Zeilenumbruch für Darstellung
                for (int i = 0; i < shm->list.count; i++) {                                                    // Schleife über alle Listeneinträge
                    char tbuf[32] = {0};                                                                       // Puffer für Zeitstempel
                    struct tm tm; localtime_r(&shm->list.entries[i].ts, &tm);                                  // Umwandlung Zeitstempel in lesbares Format
                    strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm);

                    char line[MSG_LEN  + 64];
                    snprintf(line, sizeof(line), "(%d) [%s] %s\n", i, tbuf, shm->list.entries[i].text);    // Zeile Text für Listeneintrag für AUsgabe

                    if (strlen(buf) + strlen(line) + 1 < sizeof(buf)) {                                        // Zeile wird an den Sammelpuffer angehangen
                        strcat(buf, line);
                    } else {
                        break;
                    }
                }
                snprintf(shm->response, RESP_SIZE, "%s", buf[0] ? buf : "(leer)\n");                            //Sammelpuffer wird nach Response kopiert --> Client

            } else if (op == OP_CLEAR) { // Liste löschen
                shm->list.count = 0; // logisch leeren (Zurücksetzen Zählvariable)
                memset(shm->list.entries, 0, sizeof(shm->list.entries)); // Speicherinhalt nullen
                snprintf(shm->response, RESP_SIZE, "Liste wurde gelöscht");

            } else if (op == OP_DELETE) { // Eintrag aus Liste anhand von ID entfernen
                char *endp = NULL;                                                                    
                long id = strtol(in, &endp, 10);                                                        // Umwandlung Nutzereingabe in Zahl
                if (endp == in || id < 0 || id >= shm->list.count) {                                     // Überprüfung ob gültige Eingabe
                    snprintf(shm->response, RESP_SIZE, "Fehler: ungültige ID '%s'", in);                // Fehlermeldung
                } else {
 
                    for (int i = id; i < shm->list.count - 1; i++) {                                    // Verschiebung der nachvollgenden Einträge um position nach oben
                        shm->list.entries[i] = shm->list.entries[i+1];
                    }
                    shm->list.count--;                                                                  // Reduziere Anzahl Listeneinträge um 1
                    snprintf(shm->response, RESP_SIZE, "OK: Eintrag (%ld) gelöscht", id);               // Antwort an den Client 
                }

            } else { // unbekannte Operation
                snprintf(shm->response, RESP_SIZE, "error: unknown op=%d", op);
            }

            shm->resp_ready = 1;  // Antwort liegt an
        }

        sem_unlock(semid); // Kritischen Bereich verlassen --> für andere Prozesse nun zulässig
    }

    // (nie erreicht)
    return 0;
}