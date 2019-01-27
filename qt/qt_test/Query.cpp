#include "Query.h"

/* This is the daemon database communication driver */

using namespace std;

CQuery::CQuery()
{
	msgq_query = "/database_query";
	msgq_callback = "/database_callback";
}

CQuery::~CQuery()
{
	closeQueryQueue();
	closeCallbackQueue();
}

bool CQuery::openQueryQueue()
{
    msgq_id_query = mq_open(msgq_query, O_RDWR | O_EXCL, S_IRWXU | S_IRWXG, NULL);
    if (msgq_id_query == (mqd_t) - 1) 
    {
        perror("In query queue mq_open()");
     	return false;   
    }
    return true;
}

bool CQuery::openCallbackQueue()
{
	msgq_id_callback = mq_open(msgq_callback, O_RDONLY);
	 if (msgq_id_callback == (mqd_t) - 1) 
    {
        perror("In callback queue mq_open()");
     	return false;   
    }
    return true;
}

bool CQuery::closeQueryQueue()
{
	if(mq_close(msgq_id_query) == 0)
		return true;
	return false;
}

bool CQuery::closeCallbackQueue()
{
	if(mq_close(msgq_id_callback) == 0)
		return true;
	return false;
}

bool CQuery::sendQuery(string custom_query) // change from const char *
{
	int msgsz;
	if(openQueryQueue())
	{
		memset(query, 0, sizeof(query));
		sprintf(query, custom_query.c_str());
		msgsz = mq_send(msgq_id_query, query, strlen(query)+1, 1);
		if (msgsz == -1) 
    	{
        	perror("In query queue mq_send()");
        	return false;
    	}    	
    	return closeQueryQueue();
	}
	return false;
}

bool CQuery::selectQuery(const char* column, const char* table, const char* cond_column, int cond)
{
	stringstream ss;
	ss << "select " << column << " from " << table << " where " << cond_column << "=" << cond;
	return sendQuery(ss.str());
}


bool CQuery::selectQuery(const char* column, const char* table, const char* cond_column, const char* cond)
{
	stringstream ss;
	ss << "select " << column << " from " << table << " where " << cond_column << "='" << cond << "'";
	return sendQuery(ss.str());
}

bool CQuery::selectQuery(const char* column, const char* table)
{
	stringstream ss;
	ss << "select " << column << " from " << table;
	return sendQuery(ss.str());
}

bool CQuery::receiveQuery()
{
	int msgsz;
	unsigned int sender;
	if(openCallbackQueue())
	{
		memset(result, 0, sizeof(result));
		msgsz = mq_receive(msgq_id_callback, result, MAX_MSG_LEN, &sender);
    	if (msgsz == -1) 
    	{
        	perror("In callback queue mq_receive()");
        	return false;
    	}
    	return closeCallbackQueue();
    }
    return false;
}

string CQuery::getLastQueryResult()
{
	string str(result);
	return str;
}

string CQuery::sendQueryGetResponse(const char* column, const char* table, const char* cond_column, const char* cond)
{
    selectQuery(column, table, cond_column, cond);
    receiveQuery();
    return getLastQueryResult();
}
