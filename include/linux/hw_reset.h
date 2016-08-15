#ifndef _LINUX_HARDWARE_RESET_H
#define _LINUX_HARDWARE_RESET_H

#include <linux/notifier.h>

#define HW_RESET_ENABLE		0x0000 /* enable force hardware reset */
#define HW_RESET_DISABLE	0x0001 /* disable force hardware reset */

extern int register_hw_reset_notifier(struct notifier_block *nb);
extern int unregister_hw_reset_notifier(struct notifier_block *nb);

#endif /* _LINUX_HARDWARE_RESET_H */
