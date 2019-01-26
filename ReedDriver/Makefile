ifneq ($(KERNELRELEASE),)
# Called by kbuild
obj-m += ReedDeviceModule.o

else # Called from command line
KDIR := /home/rui/buildroot-2018.08/output/build/linux-custom
PWD := $(shell pwd)
CROSS=/home/rui/buildroot-2018.08/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabihf-
default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS) modules
endif # End kbuild check

clean:
	rm -f -r *.o *.ko .*cmd .tmp* core *.i
