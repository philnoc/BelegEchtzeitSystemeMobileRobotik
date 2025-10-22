#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
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

// vermeidet Zombie-Prozesse
static void reap(int sig) {
    (void)sig;
    while (waitpid(-1, NULL, WNOHANG) > 0) {}
}

int main(void) {
    signal(SIGCHLD, reap);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { perror("socket"); return EXIT_FAILURE; }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt"); close(server_fd); return EXIT_FAILURE;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(PORT);
    if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
        perror("inet_pton"); close(server_fd); return EXIT_FAILURE;
    }

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(server_fd); return EXIT_FAILURE;
    }
    if (listen(server_fd, 16) < 0) {
        perror("listen"); close(server_fd); return EXIT_FAILURE;
    }

    puts("Server hört auf 127.0.0.1:5000 ...");

    for (;;) {
        struct sockaddr_in peer; socklen_t peerlen = sizeof(peer);
        int cfd = accept(server_fd, (struct sockaddr*)&peer, &peerlen);
        if (cfd < 0) { perror("accept"); continue; }

        pid_t pid = fork();
        if (pid < 0) {
            perror("fork"); close(cfd); continue;
        }
        if (pid == 0) {
            // Kind-Prozess: bearbeitet genau EINEN Client
            close(server_fd); // Kind braucht den Listening-Socket nicht

            char remote[64];
            inet_ntop(AF_INET, &peer.sin_addr, remote, sizeof(remote));
            printf("[child %d] Verbunden mit %s:%u (fd=%d)\n",
                   getpid(), remote, ntohs(peer.sin_port), cfd);

            char buf[BUFSZ + 1];
            ssize_t n = recv(cfd, buf, BUFSZ, 0);
            if (n <= 0) {
                if (n < 0) perror("recv");
                else printf("[child %d] Client hat geschlossen.\n", getpid());
                close(cfd);
                _exit(0);
            }
            buf[(n < BUFSZ) ? n : BUFSZ] = '\0';
            printf("[child %d] Nachricht: %s", getpid(), buf);

            const char reply[] = "Hello from Server\n";
            if (send_all(cfd, reply, sizeof(reply) - 1) < 0) perror("send");

            close(cfd);
            _exit(0);
        } else {
            // Parent: macht sofort weiter und akzeptiert nächste Clients
            close(cfd); // Parent braucht den Client-FD nicht
        }
    }

    close(server_fd);
    return EXIT_SUCCESS;
}
