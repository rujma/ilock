#include <stdio.h>
#include <sstream>
#include <iostream>
#include "Manage.h"

using namespace std;

CManage::CManage()
{

}

CManage::~CManage()
{

}

bool CManage::addUser(user_data_t user)
{
    int value;
    // Insert the RFID
    if(!insertRFIDQuery(user.user_id_rfid, user.user_name, false))
        return false;
    // Insert face entry
    if(!insertFaceQuery(user.user_id_rfid))
    {
        // If it fails, the RFID entry must be removed
        deleteByNameQuery(user.user_name);
        return false;
    }
    // Get the last inserted row id from face entity
    if(!lastInsertValue(&value))
    {
        // If it fails, the RFID entry must be removed
        deleteByNameQuery(user.user_name);
        return false;
    }
    // Add the images to the face
    for(int i = 0; i < user.n_pics; i++)
    {

    }

}

bool CManage::removeUser(user_data_t user)
{
    return deleteByNameQuery(user.user_name);
}
