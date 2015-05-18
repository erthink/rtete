#if !defined (__GNUC__) || (__GNUC__ < 2)
#       error The GNU C-compiler version 2 or higer is required.
#endif

/* Common includes */
#include <linux/version.h>
#if LINUX_VERSION_CODE < 0x020613
#	include <linux/config.h>
#endif
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/netdevice.h>
#include <linux/in.h>
#include <linux/skbuff.h>
#include <linux/if.h>
#include <linux/inetdevice.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mempool.h>

#include <linux/init.h>

static int __init proba_init(void)
{
	printk ("proba-init/loaded.\n");
	return 0;
}

static void __exit proba_exit(void)
{
	printk ("proba-exit/unloaded.\n");
	return;
}

module_init(proba_init);
module_exit(proba_exit);

MODULE_LICENSE("GPL");
