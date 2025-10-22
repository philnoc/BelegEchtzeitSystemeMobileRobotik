#include "listappl.h"

int main(void){
    // Key wie beim Server gleiche ProjektID und Dateipfad
    key_t key = ftok(FTOK_PATH, FTOK_PROJ);
    if (key == -1) die("ftok");                                 // Errorhandling

    // SHM & Semaphor öffnen
    int shmid = shmget(key, sizeof(ShmBlock), 0666);            // Existierendes SHM Segment öffnen mit Read Write Access
    if (shmid == -1) die("shmget(open)");                       // Errorhandling
    ShmBlock *shm = (ShmBlock*) shmat(shmid, NULL, 0);          // SHM Segment in Adressraum einhängen
    if (shm == (void*)-1) die("shmat");                         // Errorhandling

    int semid = semget(key, 1, 0666);                           // Vorhandenes Semaphoren Set öffnen
    if (semid == -1) die("semget(open)");                       // Errorhandling

    // Konsoelnausgabe mit Nutzungshinweisen
    printf("Befehle: /read | /clear | /delete N | /quit\n");
    printf("Textzeile ohne Slash = append\n");

char line[REQ_SIZE * 2];                                        // Eingabepuffer für Konsoleneingabe 
struct timespec ts = {0, 20 * 1000 * 1000};                     // Wartezeit für Warten auf Antwort = 20 ms

for (;;) {
    printf("> ");
    fflush(stdout);

    if (!fgets(line, sizeof(line), stdin)) break; // EOF/Ctrl+D

    size_t len = strlen(line);
    if (len && line[len-1] == '\n') line[len-1] = '\0';

    if (strcmp(line, "/quit") == 0) break; // Falls quitbefehl dann Schleife verlassen

    int op = 0;                                   // Variable für Befehl    
    char payload[REQ_SIZE]; payload[0] = '\0';    // Variable für Nutzlast -> Text oder ID

    // Befehle intepretieren aus Nutzereingabe und entsprechenden OP Mode zuweisen
    if (strcmp(line, "/read") == 0) {
        op = OP_READ;
    } else if (strcmp(line, "/clear") == 0) {
        op = OP_CLEAR;
    } else if (strncmp(line, "/delete ", 8) == 0) {
        op = OP_DELETE;
        // alles hinter "/delete " als ID-String
        snprintf(payload, REQ_SIZE, "%.*s", REQ_SIZE - 1, line + 8);
    } else { // Falls kein /command dann als Append interpretieren und Text anhängen zur Liste
        op = OP_APPEND; // normale Zeile -> anhängen
        snprintf(payload, REQ_SIZE, "%.*s", REQ_SIZE - 1, line + 8);
    }

    // Anfrage in SHM schreiben
    // Beginn kritischer Bereich, exklusiver Zutritt durch Semaphore
    sem_lock(semid);
    shm->op = op;                                              // gewünschte Operation in SHM schreiben
    shm->sender = getpid();                                    // PID schreiben
    if (op == OP_APPEND || op == OP_DELETE) {                  // Bei Delete oder Append Befehl wird (Request-) Payload gesetzt 
        snprintf(shm->request, REQ_SIZE, "%.*s", REQ_SIZE - 1, payload);
    } else {
        shm->request[0] = '\0';                                // sonst kein Payload Notwendig
    }
    shm->resp_ready = 0;                                       // Antwort-Flag zurückstellen (alte Antwort als gelesen markieren)
    shm->req_ready  = 1;                                       // Anfrage-Flag setzen und somit als bereit markieren --> Signal für Server
    sem_unlock(semid);                                         // kritischen Bereich verlassen

    // Auf Antwort von Server warten, einfaches warten mit Nanosleep
    while (!shm->resp_ready) nanosleep(&ts, NULL);

    // Formatierungsergänzung, bei Ausgabe der Liste sollen alle Elemente auf gleiche Einrückung befinden, deshalb Zeilenumbruch nach Server:
    if (op == OP_READ) {
        printf("SERVER:\n\n%s", shm->response);
    } else {
        printf("SERVER: %s\n", shm->response);
    }

    // zurücksetzen des resp-ready Flag, --> Serverantwort als gelesen markiert
    sem_lock(semid);            // Zurtritt durch Semaphore
    shm->resp_ready = 0;        // Setzen der Flag
    sem_unlock(semid);          // Lösen der Semaphore
}


    if (shmdt(shm) == -1) perror("shmdt");
    return 0;
}
