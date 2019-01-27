#ifndef _RFID_H_
#define _RFID_H_

#include "MFRC522.h"

using namespace std;

#define RFID_BYTE_SIZE 4

// RFID wrapper class
class CRFID{
public:
	CRFID();
	~CRFID();
	bool checkRFIDPresence(void);
	bool readRFIDCard(void);
	void saveRFID(void);
	void getLastRFID(char*);
private:
	char idRFID[RFID_BYTE_SIZE];
	MFRC522 mfrc;
	
};

#endif