#ifndef _DATABASE_H_
#define _DATABASE_H_

#include <sqlite3.h>
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>


#define TRUE 1
#define FALSE 0

using namespace std;

class CDatabase{
public:
	CDatabase();
	~CDatabase();
	bool openDB();
	bool closeDB();
	bool executeQuery(char *query);
	static int callback(void* arg, int argc, char** argv, char** azColName);
	void getResult(char*);


private:
	sqlite3* database;
	string result;
	const char * databaseName;
};

#endif