#ifndef _CLock_H_
#define _CLock_H_

using namespace std;

class CLock{
public:
	CLock();
	~CLock();

	int checkReedSwitch();
	bool actuateLock(char value); 

    enum state {LOCKED = 1, UNLOCKED = 0};
    enum actuate {LOCK = 0, UNLOCK = 1};
private:
	const char * reedDD;
	const char * lockDD;
};


#endif
