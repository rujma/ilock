#include "Lock.h"

using namespace std;

CLock::CLock()
{
	reedDD = "/dev/ReedDevice";
	lockDD = "/dev/LockDevice";
}

CLock::~CLock()
{
	
}


int CLock::checkReedSwitch()
{
	int ret, receive;
	int fd = open(reedDD, O_RDONLY);
	if(fd < 0){
		perror("Cant open reed switch device driver\n");
		return -1;
	}

	ret = read(fd, &receive, 1);
	if(ret < 0){
		perror("Failed to read message from reed swtich device driver\n");
		return -1;
	}

	return receive;
}

bool CLock::actuateLock(char value)
{
	int ret;
	int fd = open(lockDD, O_WRONLY);
	if(fd < 0){
		perror("Cant open lock switch device driver\n");
		return false;
	}

	ret = write(fd, &value, 1);
	if(ret < 0){
		perror("Cant write to lock device driver\n");
		return false;
	}
	else return true;
}
