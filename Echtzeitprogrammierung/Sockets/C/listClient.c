// list_client_repl.c
// Interaktiver Client: bleibt verbunden, sendet Befehle, liest Antworten bis --END--.
// /quit beendet den Client (und schickt /quit an den Server, der dann die Verbindung schließt).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PORT 5000
#define BUFSZ 1024
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

int main(void) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return EXIT_FAILURE; }

    struct sockaddr_in srv; memset(&srv, 0, sizeof(srv));
    srv.sin_family = AF_INET; srv.sin_port = htons(PORT);
    if (inet_pton(AF_INET, "127.0.0.1", &srv.sin_addr) != 1) { perror("inet_pton"); close(fd); return EXIT_FAILURE; }

    if (connect(fd, (struct sockaddr*)&srv, sizeof(srv)) < 0) { perror("connect"); close(fd); return EXIT_FAILURE; }

    puts("Verbunden. Befehle: /read | /clear | /delete N | <Text> | /quit");

    char line[BUFSZ];
    for (;;) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break; // EOF

        size_t len = strnlen(line, sizeof(line));
        if (len == 0) continue;

        if (send_all(fd, line, len) < 0) { perror("send"); break; }

        // Antworten bis Delimiter --END-- sammeln
        char acc[8192]; acc[0] = '\0';
        for (;;) {
            char buf[1024 + 1];
            ssize_t n = recv(fd, buf, 1024, 0);
            if (n <= 0) { // Server hat geschlossen
                if (n < 0) perror("recv");
                goto out;
            }
            buf[n] = '\0';
            if (strlen(acc) + (size_t)n + 1 >= sizeof(acc)) {
                fwrite(acc, 1, strlen(acc), stdout);
                fwrite(buf, 1, (size_t)n, stdout);
                acc[0] = '\0';
            } else {
                strcat(acc, buf);
            }
            char *mark = strstr(acc, ENDMARK);
            if (mark) {
                *mark = '\0'; // Marker entfernen
                printf("%s", acc);
                // wenn der Server wegen /quit schließt, folgt gleich n<=0 im nächsten recv
                break;
            }
        }

        // lokale /quit-Logik: Beende den Client, wenn der Nutzer /quit eingegeben hat
        if (strncmp(line, "/quit", 5) == 0) break;
    }

out:
    close(fd);
    return EXIT_SUCCESS;
}
