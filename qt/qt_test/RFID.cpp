#include "RFID.h"

#include <cstdio>
#include <iostream>
#include <string.h>
#include <string>
#include <sstream>

using namespace std;

CRFID::CRFID() : mfrc()
{
	mfrc.PCD_Init();
}

CRFID::~CRFID()
{

}

bool CRFID::checkRFIDPresence()
{
	byte bufferATQA[2];
  	byte bufferSize = sizeof(bufferATQA);
  	byte result = mfrc.PICC_RequestA(bufferATQA, &bufferSize);
  	return (result == mfrc.STATUS_OK || result == mfrc.STATUS_COLLISION);
}

bool CRFID::readRFIDCard()
{
  	byte result = mfrc.PICC_Select(&mfrc.uid);
  	if(result == mfrc.STATUS_OK)
  	{
        memcpy(idRFID, mfrc.uid.uidByte, RFID_BYTE_SIZE);
  		return 1;
  	}
  	else
  		return 0;
}

void CRFID::getLastRFID(char* buf)
{
    sprintf(buf, idRFID);
}
