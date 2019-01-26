#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>


int main()
{
	
	int ret, fd;
	char receive = 0;
   	fd = open("/dev/ReedDevice", O_RDONLY);
   	if(fd < 0)
   	{            
   	   perror("Failed to open the device...");
   	   return errno;
   	}
   	ret = read(fd, &receive, 1);        
   	if (ret < 0){
   	   perror("Failed to read the message from the device.");
   	   return errno;
   	}
   	printf("Device driver: %c\n", receive);

   	return 0;
}