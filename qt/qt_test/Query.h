#ifndef _QUERY_H_
#define _QUERY_H_

/* This is the daemon database communication driver */

#include <pthread.h>
#include <mqueue.h>
#include <string>
#include <string.h>

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

    bool insertRFIDQuery(const char*, const char *, bool);    // RFID entity
    bool insertFaceQuery(const char*);                   // Face entity
    bool insertImageQuery(int, const char*);             // Image entity

    bool deleteByNameQuery(const char*);                      // Delete RFID by name (on delete cascade)

	bool selectQuery(const char*, const char*, const char*, const char*);
	bool selectQuery(const char*, const char*, const char*, int cond);
	bool selectQuery(const char*, const char*);

    string selectQueryGetResponse(const char*, const char*, const char*, const char*);    // Unsafe version of selectQuery + receiveQuery
    string selectQueryGetResponse(const char*, const char*, const char*, int);            // Unsafe version of selectQuery + receiveQuery
    string selectQueryGetResponse(const char*, const char*);                              // Unsafe version of selectQuery + receiveQuery

    bool sendQuery(string);
	bool receiveQuery();
	string getLastQueryResult();
    bool lastInsertValue(int*);
    bool getMaxID(int*, const char*, const char*);

private:
	char query[MAX_MSG_LEN];
	char result[MAX_MSG_LEN];

	const char * msgq_query;
	const char * msgq_callback;

	mqd_t msgq_id_query;
	mqd_t msgq_id_callback;
};

#endif
