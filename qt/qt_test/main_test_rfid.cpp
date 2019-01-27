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

#define UNLOCK 0
#define LOCK 1

using namespace std;


/* Object pointers */
CRFID rfid;

/* Timers */
timer_t sTimeRFID;

/* Global variables */
bool admin = false;



void* tAcquireRFID(void* arg)
{
    char id[4];
    // Check RFID presence
    if(!rfid.checkRFIDPresence())
    {
        cout << "NO PRESENCE" << endl;
        return NULL;
    }
    if(rfid.readRFIDCard())
    {

        rfid.getLastRFID(id);
        for(byte i = 0; i < RFID_BYTE_SIZE; ++i)
        {
            cout << hex << (int)id[i] << " ";
        }
        cout << endl;
    }
    else
        cout << "CANT READ" << endl;
}



static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    if (sig == SIGRTMIN)
    {
        cout << "TIMER TRIGGER" << endl;
        tAcquireRFID(NULL);
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
    makeTimer(&sTimeRFID, 1, 1);
    while(1);
    return a.exec();
}
