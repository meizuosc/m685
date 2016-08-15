#ifndef __TFA_TAPDET_H__
#define __TFA_TAPDET_H__

#include "config.h"

int tfa98xx_tapdet_enable(struct tfa98xx *tfa98xx, bool enable);

// register a misc device to sensor hal
int tfa98xx_tapdet_register_device(struct tfa98xx *tfa98xx);

int tfa98xx_tapdet_deregister_device(struct tfa98xx *tfa98xx);

int tfa98xx_tapdet_report_check(struct tfa98xx *tfa98xx);

#endif //__TFA_TAPDET_H__
