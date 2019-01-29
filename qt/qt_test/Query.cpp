#include <stdio.h>
#include <errno.h>
#include <string>
#include <sstream>
#include <iostream>
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

/* Insert functions */
bool CQuery::insertRFIDQuery(const char* id_rfid, const char * name_rfid, bool admin)
{
    stringstream ss;
    ss << "insert into rfid values ('" << id_rfid << "','" << name_rfid << "'," << (int)admin << ")";
    return sendQuery(ss.str());
}

bool CQuery::insertFaceQuery(const char* id_rfid)
{
    stringstream ss;
    ss << "insert into face(idRFID) values ('" << id_rfid << "')";
    return sendQuery(ss.str());
}

bool CQuery::insertImageQuery(int id_face, const char* face_path)
{
    stringstream ss;
    ss << "insert into image(idFace,facePATH) values (" << id_face << ",'" << face_path << "')";
    return sendQuery(ss.str());
}

/* Delete functions */
bool CQuery::deleteByNameQuery(const char* name_rfid)
{
    stringstream ss;
    ss << "delete from rfid where nameRFID='" << name_rfid << "'";
    return sendQuery(ss.str());
}

/* Select functions */
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

/* Unsafe select functions */
string CQuery::selectQueryGetResponse(const char* column, const char* table, const char* cond_column, const char* cond)
{
    selectQuery(column, table, cond_column, cond);
    receiveQuery();
    return getLastQueryResult();
}

/* Worker functions */
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

bool CQuery::lastInsertValue(int* value)
{
    int temp;
    if(sendQuery("select last_insert_rowid()"))
    {
        receiveQuery();
        string str(getLastQueryResult());
        istringstream iss(str);

        iss >> temp;
        *value = temp;
        return true;
    }
    else
        return false;
}

bool CQuery::getMaxID(int* value, const char* column, const char* table)
{
    int temp;
    stringstream ss;
    ss << "select max(" << column << ") from " << table;
    if(sendQuery(ss.str()))
    {
        receiveQuery();
        string str(getLastQueryResult());
        istringstream iss(str);

        iss >> temp;
        *value = temp;
        return true;
    }
    else
        return false;
}
