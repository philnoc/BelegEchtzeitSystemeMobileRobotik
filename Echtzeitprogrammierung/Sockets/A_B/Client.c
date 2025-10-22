#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT 5000
#define BUFSZ 127

static int send_all(int fd, const char *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, buf + sent, len - sent, 0);
        if (n < 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

int main(void) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return EXIT_FAILURE; }

    struct sockaddr_in srv;
    memset(&srv, 0, sizeof(srv));
    srv.sin_family = AF_INET;
    srv.sin_port   = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &srv.sin_addr);

    if (connect(fd, (struct sockaddr*)&srv, sizeof(srv)) < 0) {
        perror("connect"); close(fd); return EXIT_FAILURE;
    }
    puts("Verbunden.");

    char sendbuf[BUFSZ + 1];
    printf("Nachricht eingeben: ");
    if (!fgets(sendbuf, sizeof(sendbuf), stdin)) { fprintf(stderr, "Eingabe fehlgeschlagen\n"); close(fd); return EXIT_FAILURE; }

    size_t len = strnlen(sendbuf, sizeof(sendbuf));
    if (len && send_all(fd, sendbuf, len) < 0) { perror("send"); close(fd); return EXIT_FAILURE; }

    char recvbuf[BUFSZ + 1];
    ssize_t n = recv(fd, recvbuf, BUFSZ, 0);
    if (n < 0) { perror("recv"); close(fd); return EXIT_FAILURE; }
    recvbuf[(n < BUFSZ) ? n : BUFSZ] = '\0';
    printf("Antwort vom Server: %s", recvbuf);

    close(fd);
    return EXIT_SUCCESS;
}
