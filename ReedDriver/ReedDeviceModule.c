#include <linux/module.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>	/* for put_user */
#include <linux/uaccess.h>

#ifdef CONFIG_ARCH_MULTI_V7
#define BCM2708_PERI_BASE 0x3F000000
#else
#define BCM2708_PERI_BASE 0x20000000 
#endif

#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#define DEVICE_NAME "ReedDevice"
#define CLASS_NAME  "ReedDeviceClass"

MODULE_LICENSE("GPL");

/* Device variables */
static struct class* ReedDevice_class = NULL;
static dev_t ReedDevice_majorminor;
static struct cdev c_dev;  // Character device structure
void __iomem *base;       // Pointer to GPIO memory

// Define register adresses (datasheet page 90)
#define GPIOFSEL(x) (0x00+(x)*4)  
#define GPIOCLR(x)  (0x28+(x)*4)   
#define GPIOLEV(x)  (0x34+(x)*4)

static const int ReedPin = 22;

static void setGPIOFunction(int GPIO, int functionCode)
{
	int registerIndex = GPIO / 10;
	int bit = (GPIO % 10) * 3;
	unsigned long gpiodir;

	gpiodir = ioread32(base + GPIOFSEL(registerIndex));
	gpiodir &= ~(7 << bit);
	gpiodir |= functionCode << bit;
	iowrite32(gpiodir, base + GPIOFSEL(registerIndex));
	gpiodir = ioread32(base + GPIOFSEL(registerIndex));
}

static unsigned int readGPIOLevel(int GPIO) 
{
	unsigned gpio_pin = GPIO / 32;
	unsigned int pinValue;

	// Get 32 bit register of LEV0
	pinValue = ioread32(base + GPIOLEV(gpio_pin));
	
	return pinValue;
}

static ssize_t ReedDeviceRead(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	int result;
	char level;

	result = readGPIOLevel(ReedPin);
	// Shift the 32 bit register in search for the ReedPin bit
	level = (char)(result >> ReedPin) & 0b01;
	// Send either 0 or 1 character to user space
	put_user((char)(level+48), buf);
  	return len;
}

static struct file_operations ReedDevice_fops = {
	.owner = THIS_MODULE,
	.read = ReedDeviceRead
};


static int __init ReedDeviceModule_init(void)
{
	int ret;
	struct device *dev_ret;

	if((ret = alloc_chrdev_region(&ReedDevice_majorminor, 0, 1, DEVICE_NAME)) < 0)
	{
		return ret;
	}

	if(IS_ERR(ReedDevice_class = class_create(THIS_MODULE, CLASS_NAME)))
	{
		unregister_chrdev_region(ReedDevice_majorminor, 1);
		return PTR_ERR(ReedDevice_class);
	}

	if(IS_ERR(dev_ret = device_create(ReedDevice_class, NULL, ReedDevice_majorminor, NULL, DEVICE_NAME)))
	{
		class_destroy(ReedDevice_class);
		unregister_chrdev_region(ReedDevice_majorminor, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&c_dev, &ReedDevice_fops);
	c_dev.owner = THIS_MODULE;
	if((ret = cdev_add(&c_dev, ReedDevice_majorminor, 1)) < 0)
	{
		printk(KERN_NOTICE "Error %d adding device", ret);
		device_destroy(ReedDevice_class, ReedDevice_majorminor);
		class_destroy(ReedDevice_class);
		unregister_chrdev_region(ReedDevice_majorminor, 1);
		return ret;
	}
	
	base = ioremap(GPIO_BASE, 0xB0);
	setGPIOFunction(ReedPin, 0);

	return 0;
}


static void __exit ReedDeviceModule_exit(void)
{
	cdev_del(&c_dev);
	device_destroy(ReedDevice_class, ReedDevice_majorminor);
	class_destroy(ReedDevice_class);
	unregister_chrdev_region(ReedDevice_majorminor, 1);
}

module_init(ReedDeviceModule_init);
module_exit(ReedDeviceModule_exit);


