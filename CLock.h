#ifndef _CLock_H_
#define _CLock_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

using namespace std;

class CLock{
public:
	CLock();
	~CLock();
	int checkReedSwitch();
	bool actuateLock(char value); 
private:
	const char * reedDD;
	const char * lockDD;
};


#endif