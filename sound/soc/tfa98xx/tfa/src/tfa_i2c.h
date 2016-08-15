#ifndef __TFA_I2C_H__
#define __TFA_I2C_H__

#include "config.h"

#ifndef I2C_SLAVE
#define I2C_SLAVE			0x0703
#define I2C_SLAVE_FORCE		0x0706
#endif

#define RW_BUFFER_LENGTH (256)

int tfa98xx_i2c_register_device(struct tfa98xx *tfa98xx);
int tfa98xx_i2c_deregister_device(struct tfa98xx *tfa98xx);

#endif