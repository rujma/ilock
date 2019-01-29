#ifndef _QUERY_H_
#define _QUERY_H_

/* This is the daemon database communication driver */

#include <pthread.h>
#include <mqueue.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <sstream>
#include <iostream>

#define MAX_MSG_LEN 10000

using namespace std;


class CQuery{
public:
	CQuery();
	~CQuery();
	bool openQueryQueue();
	bool openCallbackQueue();
	bool closeQueryQueue();
	bool closeCallbackQueue();
	bool selectQuery(const char*, const char*, const char*, const char*);
	bool selectQuery(const char*, const char*, const char*, int cond);
	bool selectQuery(const char*, const char*);
	bool sendQuery(string);
	bool receiveQuery();
	string getLastQueryResult();

private:
	char query[MAX_MSG_LEN];
	char result[MAX_MSG_LEN];
	const char * msgq_query;
	const char * msgq_callback;
	mqd_t msgq_id_query;
	mqd_t msgq_id_callback;
};

#endif
