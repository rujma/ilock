#include <linux/module.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

#ifdef CONFIG_ARCH_MULTI_V7
#define BCM2708_PERI_BASE 0x3F000000
#else
#define BCM2708_PERI_BASE 0x20000000 
#endif

#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define DEVICE_NAME "LockDevice"
#define CLASS_NAME  "LockDeviceClass"

MODULE_LICENSE("GPL");

/* Device variables */
static struct class* LockDevice_class = NULL;
static dev_t LockDevice_majorminor;
static struct cdev c_dev;  // Character device structure
void __iomem *base;       // Pointer to GPIO memory

// Define register adresses (datasheet page 90)
#define GPIOFSEL(x) (0x00+(x)*4)  
#define GPIOSET(x)  (0x1c+(x)*4)  
#define GPIOCLR(x)  (0x28+(x)*4)   

static const int LockPin = 27;


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

static void setGPIOOutputValue(int GPIO, bool outputValue)
{
	unsigned gpio_pin = GPIO / 32;
	unsigned gpio_field_offset = (GPIO - 32 * gpio_pin);

	if(outputValue)
	{
		iowrite32(1 << gpio_field_offset, base + GPIOSET(gpio_pin));
	}
	else
	{
		iowrite32(1 << gpio_field_offset, base + GPIOCLR(gpio_pin));
	}
}

static ssize_t LockDeviceWrite(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
	if(buf[0] == '0')
		setGPIOOutputValue(LockPin, 0);
	else
		setGPIOOutputValue(LockPin, 1);
	return len;
}

static struct file_operations LockDevice_fops = {
	.owner = THIS_MODULE,
	.write = LockDeviceWrite
};

static int __init LockDeviceModule_init(void)
{
	int ret;
	struct device *dev_ret;

	if((ret = alloc_chrdev_region(&LockDevice_majorminor, 0, 1, DEVICE_NAME)) < 0)
	{
		return ret;
	}

	if(IS_ERR(LockDevice_class = class_create(THIS_MODULE, CLASS_NAME)))
	{
		unregister_chrdev_region(LockDevice_majorminor, 1);
		return PTR_ERR(LockDevice_class);
	}

	if(IS_ERR(dev_ret = device_create(LockDevice_class, NULL, LockDevice_majorminor, NULL, DEVICE_NAME)))
	{
		class_destroy(LockDevice_class);
		unregister_chrdev_region(LockDevice_majorminor, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&c_dev, &LockDevice_fops);
	c_dev.owner = THIS_MODULE;
	if((ret = cdev_add(&c_dev, LockDevice_majorminor, 1)) < 0)
	{
		printk(KERN_NOTICE "Error %d adding device", ret);
		device_destroy(LockDevice_class, LockDevice_majorminor);
		class_destroy(LockDevice_class);
		unregister_chrdev_region(LockDevice_majorminor, 1);
		return ret;
	}

	base = ioremap(GPIO_BASE, 0xB0);
	setGPIOFunction(LockPin, 0b001);
	
	return 0;
}


static void __exit LockDeviceModule_exit(void)
{
	setGPIOFunction(LockPin, 0);
	cdev_del(&c_dev);
	device_destroy(LockDevice_class, LockDevice_majorminor);
	class_destroy(LockDevice_class);
	unregister_chrdev_region(LockDevice_majorminor, 1);
}

module_init(LockDeviceModule_init);
module_exit(LockDeviceModule_exit);



