#ifndef LISTAPPL_H
#define LISTAPPL_H

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

#define MSG_LEN    256    // pro Listeneintrag
#define REQ_SIZE   256    // max. Client-Eingabetext
#define RESP_SIZE  4096   // Antwortpuffer für /read
#define MAX_MSG    256    // Anzahl Einträge

#define FTOK_PATH "/tmp/ipc_shm_demo_key"
#define FTOK_PROJ 'S'

enum OpCode { OP_NONE = 0, OP_APPEND=1, OP_READ=2, OP_CLEAR=3, OP_DELETE=4};

typedef struct {
    char   text[MSG_LEN];
    time_t ts;
} ListEntry;

typedef struct {
    int       count;
    ListEntry entries[MAX_MSG];
} SharedList;

typedef struct {
    volatile int req_ready;
    volatile int resp_ready;
    int   op;
    pid_t sender;
    char  request[REQ_SIZE];
    char  response[RESP_SIZE];
    SharedList list;
} ShmBlock;

union semun { int val; struct semid_ds *buf; unsigned short *array; };

static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

static void sem_lock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = -1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(-1)");
}
static void sem_unlock(int semid){
    struct sembuf op = { .sem_num = 0, .sem_op = +1, .sem_flg = SEM_UNDO };
    if (semop(semid, &op, 1) == -1) die("semop(+1)");
}
#endif
