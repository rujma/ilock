CC = arm-buildroot-linux-uclibcgnueabihf-g++
RM = /bin/rm -f
all: daemon

daemon:
	$(CC) -o daemon daemon.cpp Database.cpp -lpthread -lsqlite3
clean:
	${RM} daemon
