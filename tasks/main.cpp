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
sem_t semRFID;
pthread_mutex_t mutexRFID = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condNewRFID = PTHREAD_COND_INITIALIZER;
pthread_cond_t condRFID = PTHREAD_COND_INITIALIZER;

/* OS Timers */
timer_t sTimeRFID;

/* Global variables */
bool admin = false; 

void* tActuateLock(void* arg)
{
	CLock *p_Lock = (CLock*)arg;
	while(1)
	{
		// Wait for lock actuation semaphore after sucessfull validation
		sem_wait(&semLock);
		// Unlock
		if(!p_Lock->actuateLock(UNLOCK))
			continue;
		// Wait for door to open
		while(p_Lock->checkReedSwitch());
		// Wait for door to close
		while(!p_Lock->checkReedSwitch());
		// Lock when door is closed
		if(!p_Lock->actuateLock(LOCK))
			continue;
	}
}

void* tAcquireRFID(void* arg)
{
	while(1)
	{
		// Wait for semaphore post
		sem_wait(&semRFID);
		// Check RFID presence
		printf("Near an RFID card\n");
		if(!p_RFID->checkRFIDPresence())
		{
			// If it's not present, go back to while(1)
			printf("RFID not present!\n");
			continue;
		}
		// Take RFID mutex to write to new RFID value
		pthread_mutex_lock(&mutexRFID);
		if(p_RFID->readRFIDCard())
		{
			// Check admin status from validation task (admin card was detected on mentioned task)
			if(admin) 
				pthread_cond_signal(&condNewRFID, &mutexRFID);  // triggers task to add new user
			else 
				pthread_cond_signal(&condRFID, &mutexRFID);     // triggers validation task
		}
		else
		{
			// Error while reading card
			printf("Card ccouldn't be read!\n");
		}
		// Give RFID mutex back
		pthread_mutex_unlock(&mutexRFID);	
	}
}

void* tValidation(void* arg)
{
	char rfid[RFID_BYTE_SIZE];

	while(1)
	{
		// Lock mutex to read new RFID
		pthread_mutex_lock(&mutexRFID);
		// Wait for new RFID value
		pthread_cond_wait(&condRFID, &mutexRFID);
		// Read latest RFID
		p_RFID->getLastRFID(rfid);
		// Send query
		if(!p_Query.selectQuery("admin", "rfid", "idRFID", rfid))
			continue;
		// Get query result
		if(!p_Query.receiveQuery())
			continue;
		string query_result(p_Query.getLastQueryResult());
		// Get admin status from result	
		if(query_result.empty())
			continue;
		else
		{
			if(query_result.find("admin = 1") != string::npos)
			{
				admin =  true;
				sem_post(&semManage);
				continue;
			}
			else admin = false;		
		}
		// Unlock RFID mutex
		pthread_mutex_unlock(&mutexRFID);

		// Start face recognition
		/* START CAMERA SAMPLING */
		pthread_mutex_lock(&mutexUpdate);
		/* Update GUI */
		pthread_mutex_unlock(&mutexUpdate);
		while(n_predictions < 10)
		{
			// Wait predictions
			pthread_mutex_lock(&mutexPrediction);
			pthread_cond_wait(&condPrediction, &mutexPrediction);
			/* Store face ID and predictions */
			pthread_mutex_unlock(&mutexPrediction);
		}
		/* CHECK ALL PREDICTIONS FOR LOWER THAN VALUE AND SAME ID */
		if( /* check is good */)
		{
			// Send query
			if(!p_Query.selectQuery("idRFID", "face", "faceID", id);
				continue;
			// Get query result
			if(!p_Query.receiveQuery())
				continue;
			string query_result(p_Query.getLastQueryResult());
			if(query_result.find(rfid) != string::npos)
				sem_post(&semLock);
		}
		else
		{
			pthread_mutex_lock(&mutexUpdate);
			/* Error message */
			pthread_cond_wait(&mutexUpdate);
		}
	}
}

static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    if (sig == SIGRTMIN)
        sem_post(&semRFID);
}

static int makeTimer( timer_t *timerID, int expireMS, int intervalMS )
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

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = intervalMS * 1000000;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = expireMS * 1000000;
    timer_settime(*timerID, 0, &its, NULL);

    return 1;
}

int main(int argc, char *argv[])
{

	makeTimer(&sTimeRFID, 2000, 2000); //2000ms
	return 0;
}