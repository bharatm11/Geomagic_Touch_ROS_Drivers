#include <linux/module.h>
#include <linux/kernel.h>

int init_module(void)
{
	printk(KERN_INFO "Loaded dummy raw1394 module\n");
	return 0;
}
