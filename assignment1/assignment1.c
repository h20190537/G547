#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/random.h>
#include <linux/uaccess.h>
#include "ioctl1.h" //this is an user defined header file   

static int channel; //stores selected the adc channel number
static unsigned short adc_reading;//stores the adc reading
static int byte_setting;//indicates the bits in which data is present '0'->lower bits, '1'->upper bits

static struct cdev c_dev;
static dev_t first;
static struct class *cls;
static int device_open = 0;

//step4: driver callback functions
static int my_open(struct inode *i, struct file *f)
{	
	if(device_open) //in order to ensure that only process is able to access it at a time.
		return -EBUSY; 
	else
	{	
		device_open++;
		printk(KERN_INFO "Starting ADC\n");
		return 0;
	}
}

static int my_close(struct inode *i, struct file *f)
{	
	device_open--;
	printk(KERN_INFO "Releasing the device\n");
	return 0;
}

static ssize_t my_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	printk(KERN_INFO "Reading ADC\n");
	get_random_bytes(&adc_reading,sizeof(adc_reading));
	adc_reading = adc_reading%1023;
	if(byte_setting)
	adc_reading = (adc_reading*64); //shifting data into the upper 10 bits
	if(copy_to_user(buf,&adc_reading,sizeof(adc_reading))) //copying the random generated data into the user space buffer
	printk(KERN_ALERT "DATA missing\n");
	return len;
}


static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	switch(cmd){
		case SET_CHANNEL:
		channel = arg;		
		printk(KERN_INFO "Selected ADC channel is:%d\n",channel);
		break;
		case SET_ALIGNMENT:
		byte_setting = arg;
		if(byte_setting)		
		printk(KERN_INFO "Set to upper Bits\n");
		else
		printk(KERN_INFO "Set to lower Bits\n");		
		break;
		default:
		return -ENOTTY;
}
return 0;
}

//###########################################################################################


static struct file_operations fops =
{
  .owner 	  = THIS_MODULE,
  .open 	  = my_open,
  .release 	  = my_close,
  .read 	  = my_read,
  .unlocked_ioctl = my_ioctl
};

static int __init adc8_init(void)
{
	printk(KERN_INFO "ADC8 Driver registering\n");

	//step1:reserve major and minor number
	if(alloc_chrdev_region(&first,0,1,"ADC")<0)
	{
		return -1;
	}
	
	//step2:creation of the device file.
	if((cls=class_create(THIS_MODULE,"adc_dev"))==NULL)
	{
		unregister_chrdev_region(first,1);
		return -1;	
	}
	if(device_create(cls,NULL,first,NULL, "adc8")==NULL)//this file will appear in dev directory
	{
		class_destroy(cls);
		unregister_chrdev_region(first,1);
		return -1;		
	}
	
	//step3:creation of the cdev structure and link fops
	cdev_init(&c_dev,&fops);
	if(cdev_add(&c_dev,first,1)==-1)
	{
		device_destroy(cls,first);
		class_destroy(cls);
		unregister_chrdev_region(first,1);
		return -1;
	}
	return 0;
}

static void __exit adc8_exit(void)
	{	
		cdev_del(&c_dev);		
		device_destroy(cls,first);
		class_destroy(cls);
		unregister_chrdev_region(first,1);
		printk(KERN_INFO "Removing adc8 driver\n\n");
	}

module_init(adc8_init); //defining the entry point
module_exit(adc8_exit); //defining the exit point

MODULE_DESCRIPTION("ADC8 Driver");
MODULE_AUTHOR("Sudhanshu Surana <sudhanshusurana3@gmail.com>");
MODULE_LICENSE("GPL"); //GNU public license
