#include "Database.h"

using namespace std;


CDatabase::CDatabase()
{
	databaseName = "ilock.db";
	openDB();
}

CDatabase::~CDatabase()
{
	closeDB();
}

bool CDatabase::openDB()
{
	// sqlite3_open(database name, database object)
	sqlite3_open(databaseName, &database);
}

bool CDatabase::closeDB()
{
	sqlite3_close(database);
}

bool CDatabase::executeQuery(char* query)
{
	char * errormsg;
	int rc = sqlite3_exec(database, query, callback, (void*)this, &errormsg);
	if( rc != SQLITE_OK){
		return FALSE;
	}
}

int CDatabase::callback(void* result_ptr, int argc, char** argv, char** azColName)
{
	CDatabase* obj = (CDatabase*) result_ptr;

	stringstream ss;
	for(int i = 0; i < argc; i++)
	{
		ss << azColName[i] << " = " << (argv[i] ? argv[i] : "NULL") << '\n';
	}

	obj->result = obj->result + ss.str();
	return 0;
}


void CDatabase::getResult(char* memory)
{
	sprintf(memory, "%s", result.c_str());	
	result.clear();
}
