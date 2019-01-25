#include "Query.h"
#include "RFID.h"
#include <cstdio>
#include <iostream>
#include <string.h>
#include <string>
#include <sstream>

using namespace std;


int main(int argc, char *argv[])
{
	CQuery query;
	CRFID rfid;
	char id[4];

	while(1)
	{
		if(!rfid.checkRFIDPresence()) 
			continue;

		if(!rfid.readRFIDCard())
			continue;
		else
		{
			// Read RFID value
			rfid.getLastRFID(id);
			// Show the RFID value
		//	for(byte i = 0; i < RFID_BYTE_SIZE; ++i)
		//	{
		//		cout << hex << (int)id[i] << " ";
		//	}
		//	cout << endl;
			// Prepare query to insert
			//stringstream ss;
			//ss << "select * from rfid where idRFID='" << id << "'";
			//query.sendQuery(ss.str());
			query.selectQuery("idRFID", "rfid", "idRFID", id);

			if(!query.receiveQuery())
			cout << "Receive query error" << endl;
			else
			{ 
				string buf(query.getLastQueryResult());
				cout << buf;
				cout << "RFID Read first: " << id << endl;

				if(buf.find(id) != string::npos)
					cout << "RFID is the same!" << endl;

				if(buf.find("admin = 1") != string::npos)
					cout << "Admin" << endl;
			}

			// Break loop
			break;			
		}
	}
	return 0;
}