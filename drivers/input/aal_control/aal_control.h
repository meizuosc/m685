#ifndef __AAL_CONTROL_H__
#define __AAL_CONTROL_H__

#define AAL_TAG                  "[ALS/AAL]"
#define AAL_LOG(fmt, args...)	 pr_debug(AAL_TAG fmt, ##args)
#define AAL_ERR(fmt, args...)    pr_err(AAL_TAG fmt, ##args)

/*copy from sensor_io.h*/
#define ALSPS							0X84

#define AAL_SET_ALS_MODE			_IOW(ALSPS, 0x14, int)
#define AAL_GET_ALS_MODE			_IOR(ALSPS, 0x15, int)
#define AAL_GET_ALS_DATA			_IOR(ALSPS, 0x16, int)

#endif

