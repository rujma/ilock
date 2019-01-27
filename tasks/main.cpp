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
sem_t semValidateRFID;
sem_t semManageRFID;
/*
	If instead of a semaphore, a contition variable is used:
	struct binary_semaphore
	{
		pthread_mutex_t mutex;
		pthread_cond_t cvar;
		int v;
	} 

	post()
	{
		pthread_mutex_lock(sem.mutex);
		v+=1;
		pthread_cond_signal(sem.cvar);
		pthread_mutex_unlock(sem.mutex);
	}

	wait()
	{
		pthread_mutex_lock(sem.mutex);
		while(sem.v == 0)
			pthread_cond_wait(sem.cvar, sem.mutex);
		sem.v-=1;
		pthread_mutex_unlock(sem.mutex);
	}
*/
pthread_mutex_t mutexRFID = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexAdmin = PTHREAD_MUTEX_INITIALIZER;

/* Timers */
timer_t sTimeRFID;

/* Global variables */
bool admin = false; 


static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    if (sig == SIGRTMIN)
    	// Each two seconds
        sem_post(&semRFID);
}

void* tActuateLock(void* arg)
{
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
		if(!p_RFID->checkRFIDPresence())
			// If it's not present, go back to while(1)
			continue;
		// Take RFID mutex to write to new RFID value
		pthread_mutex_lock(&mutexRFID);
		if(p_RFID->readRFIDCard())
		{
			pthread_mutex_lock(&mutexAdmin);
			// Check admin status from validation task (admin card was detected on mentioned task)
			if(admin) 
				sem_post(&semValidateRFID);   // triggers task to add new user
			else 
				sem_post(&semManageRFID);     // triggers validation task
			pthread_mutex_unlock(&mutexAdmin);
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
		// Wait for new RFID value
		sem_wait(&semValidateRFID);
		// Lock mutex to read new RFID
		pthread_mutex_lock(&mutexRFID);
		// Read latest RFID
		p_RFID->getLastRFID(rfid);
		// Unlock RFID mutex
		pthread_mutex_unlock(&mutexRFID);
		// Send query
		if(!p_Query.selectQuery("admin", "rfid", "idRFID", rfid)) 
			continue;
		// Get query result
		if(!p_Query.receiveQuery()) 
			continue;
		// Save query result
		string query_result(p_Query.getLastQueryResult());
		// Check if query is empty (if it is empty, means that the RFID doesn't exist in db)
		if(query_result.empty()) 
			continue;
		else
		{
			// Get admin status from result	
			if(query_result.find("admin = 1") != string::npos)
			{
				pthread_mutex_lock(&mutexAdmin);
				admin =  true;
				pthread_mutex_unlock(&mutexAdmin);
				sem_post(&semManage);
				continue;
			}
		}
		// Start face recognition
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

void* tManagement(void* arg)
{
	while(1)
	{
		sem_wait(&semManage);
		pthread_mutex_lock(&mutexUpdate);
		/* UPDATE GUI */
		pthread_mutex_unlock(&mutexUpdate);
		// Events
		pthread_mutex_lock(&mutexEvent);
		while(/* NO EVENT */)
		{
			pthread_cond_wait(&condEvent, &mutexEvent);
			/* STORE DATA ON OBJECT */
		}
		pthread_mutex_unlock(&mutexEvent);

		

	}
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

    its.it_interval.tv_sec = 2;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = 2;
    its.it_value.tv_nsec = 0;
    timer_settime(*timerID, 0, &its, NULL);

    return 1;
}

int main(int argc, char *argv[])
{

	makeTimer(&sTimeRFID, 2000, 2000); //2000ms
	return 0;
}