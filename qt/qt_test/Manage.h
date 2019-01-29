#ifndef MANAGE_H
#define MANAGE_H

#include "Query.h"
#include "RFID.h"

typedef struct
{
    char user_id_rfid[RFID_BYTE_SIZE];
    const char * user_name;
    const char * face_path;
    int n_pics;
}user_data_t;


class CManage: public CQuery
{
public:
    CManage();
    ~CManage();
    bool addUser(user_data_t);
    bool removeUser(user_data_t);
};


#endif // MANAGE_H
