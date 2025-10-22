#define _XOPEN_SOURCE 700
#include "simpleHeader.h"
#include <time.h>
#include <stdio.h>
#include <string.h>

static void ts_now(char out[32]){
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(out, 32, "%Y-%m-%d %H:%M:%S", &tm);
}

int main(void){
    // Key & SHM öffnen (ohne Semaphor-Nutzung)
    ensure_ftok_file();
    key_t key = ftok(FTOK_PATH, FTOK_PROJ);
    if (key == -1) die("ftok");

    int shmid = shmget(key, sizeof(ShmBlock), 0666);
    if (shmid == -1) die("shmget(open)");

    ShmBlock *shm = (ShmBlock*) shmat(shmid, NULL, 0);
    if (shm == (void*)-1) die("shmat");

    printf("[listener] verbunden. Beenden mit Ctrl+C.\n");

    // Vorherige Flagwerte (für Kanten-Erkennung)
    int prev_req  = shm->req_ready;   // <- wichtig: initialen Zustand mitnehmen
    int prev_resp = shm->resp_ready;

    // „aktuell beobachtete“ Anfrage (für Korrelation)
    int64_t cur_value = 0;
    pid_t   cur_sender= -1;
    int     have_pending_request = 0; // haben wir eine Anfrage geloggt, auf deren Antwort wir warten?

    // Falls wir mitten in einer bereits signalisierten Anfrage starten (req_ready==1),
    // logge sie sofort (damit die „erste“ nicht verloren geht).
    if (prev_req == 1) {
        char t[32]; ts_now(t);
        cur_value  = shm->value;
        cur_sender = shm->sender;
        have_pending_request = 1;

        printf("[listener][%s] Anfrage erkannt (beim Start bereits aktiv):\n", t);
        printf("  sender : %d\n", (int)cur_sender);
        printf("  value  : %lld\n\n", (long long)cur_value);
    }

    // Hauptloop: Kanten erkennen, keine verschachtelten Warte-Schleifen.
    struct timespec tick = {0, 1 * 1000 * 1000}; // 1 ms, aggressiv, damit wir resp_ready erwischen

    for (;;) {
        // --- aktuellen Zustand lesen (ohne Lock, bewusst) ---
        int req  = shm->req_ready;
        int resp = shm->resp_ready;

        // Kante: neue Anfrage (0 -> 1)
        if (prev_req == 0 && req == 1) {
            char t[32]; ts_now(t);
            cur_value  = shm->value;
            cur_sender = shm->sender;
            have_pending_request = 1;

            printf("[listener][%s] Anfrage erkannt:\n", t);
            printf("  Sender : %d\n", (int)cur_sender);
            printf("  Wert  : %lld\n\n", (long long)cur_value);
        }

        // Kante: neue Antwort (0 -> 1)
        // Nur ausgeben, wenn wir vorher eine Anfrage geloggt haben
        // (so korrelieren wir das grob, ohne Lock/seq).
        if (have_pending_request && prev_resp == 0 && resp == 1) {
            char t[32]; ts_now(t);
            int  result = shm->result;
            char info[128]; info[0] = '\0';
            snprintf(info, sizeof(info), "%s", shm->info);

            printf("[listener][%s] Antwort erkannt:\n", t);
            printf("  (zu) Sender : %d\n", (int)cur_sender);
            printf("  (zu) Eingabewert  : %lld\n", (long long)cur_value);
            printf("  Ergebnis      : %d\n", result);
            printf("\n");

            // Wir bleiben in „bereit für nächste Anfrage“.
            // Falls resp sofort wieder 0 wird, sehen wir das in der nächsten Iteration.
            have_pending_request = 0;  // Zyklus abgeschlossen
        }

        // Für nächste Runde merken
        prev_req  = req;
        prev_resp = resp;

        // sanft schlafen, um CPU zu schonen, aber Antwortfenster nicht zu verpassen
        nanosleep(&tick, NULL);
    }

    // (nie erreicht)
    if (shmdt(shm) == -1) perror("shmdt");
    return 0;
}
