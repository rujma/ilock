#include <QCoreApplication>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include "RFID.h"
#include "Lock.h"
#include "Query.h"

#define PSHARED 0

using namespace std;

/* Synchronization */
// Semaphores
sem_t sem_rfid;
sem_t sem_validate;
sem_t sem_manage;
// Mutexes
pthread_mutex_t mutex_rfid = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_admin = PTHREAD_MUTEX_INITIALIZER;

/* Object pointers */
CRFID rfid;
CQuery query;

/* Timers */
timer_t timer_rfid;

/* Global variables */
bool admin = false;

void* tAcquireRFID(void* arg)
{
    while(1)
    {
        // Wait for semaphore post
        sem_wait(&sem_rfid);
        // Check RFID presence
        if(!rfid.checkRFIDPresence())
            continue;
        // Take RFID mutex to write to new RFID value
        pthread_mutex_lock(&mutex_rfid);
        if(rfid.readRFIDCard())
        {
            pthread_mutex_lock(&mutex_admin);
            // Check admin status from validation task (admin card was detected on mentioned task)
            if(!admin)
                sem_post(&sem_validate);   // triggers task to add new user
            else
                sem_post(&sem_manage);     // triggers validation task
            pthread_mutex_unlock(&mutex_admin);
        }
        // Give RFID mutex back
        pthread_mutex_unlock(&mutex_rfid);
    }
}

void* tValidation(void* arg)
{
    char id_rfid[RFID_BYTE_SIZE];

    while(1)
    {
        // Wait for new RFID value
        sem_wait(&sem_validate);
        // Lock mutex to read new RFID
        pthread_mutex_lock(&mutex_rfid);
        // Read latest RFID
        rfid.getLastRFID(id_rfid);
        // Unlock RFID mutex
        pthread_mutex_unlock(&mutex_rfid);
        // Send query - UNSAFE USAGE!
        string query_result(query.sendQueryGetResponse("admin", "rfid", "idRFID", id_rfid));
        cout << "Query result: " << query_result << endl;
        // Check if query is empty (if it is empty, means that the RFID doesn't exist in db)
        if(query_result.empty())
        {
            cout << "RFID doesn't exist in db" << endl;
            continue;
        }

        else
        {
            // Get admin status from result
            if(query_result.find("admin = 1") != string::npos)
            {
                cout << "ADMIN"<<endl;
                continue;
            }
        }
    }
}

static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    if (sig == SIGRTMIN)
    {
        cout << "TIMER TRIGGER" << endl;
        sem_post(&sem_rfid);
    }
}
static int makeTimer( timer_t *timerID, int expireS, int intervalS )
{
    struct sigevent te;
    struct itimerspec its;
    struct sigaction sa;
    int sigNo = SIGRTMIN;

    /* Set up signal handler */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timerHandler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(sigNo, &sa, NULL) == -1) {
        perror("sigaction");
    }

    /* Set and enable alarm */
    te.sigev_notify = SIGEV_SIGNAL;
    te.sigev_signo = sigNo;
    te.sigev_value.sival_ptr = timerID;
    timer_create(CLOCK_REALTIME, &te, timerID);

    its.it_interval.tv_sec = expireS;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = intervalS;
    its.it_value.tv_nsec = 0;
    timer_settime(*timerID, 0, &its, NULL);

    return 1;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    // Initialize timer
    makeTimer(&timer_rfid, 2, 2);
    // Initialize semaphores
    sem_init(&sem_rfid, PSHARED, 0);
    sem_init(&sem_validate, PSHARED, 0);
    sem_init(&sem_manage, PSHARED, 0);
    // Initialize threads
    pthread_t tRfidID;
    pthread_create(&tRfidID, NULL, tAcquireRFID, NULL);
    pthread_t tValidationID;
    pthread_create(&tValidationID, NULL, tValidation, NULL);
    // Join threads
    pthread_join(tRfidID, NULL);
    pthread_join(tValidationID, NULL);
    // Wait for threads to finish
    pthread_exit(NULL);
    return a.exec();
}
