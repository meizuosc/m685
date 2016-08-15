/*
 * Driver for CAM_CAL
 *
 *
 */

#define CONFIG_MTK_I2C_EXTENSION
#include <linux/i2c.h>
#undef CONFIG_MTK_I2C_EXTENSION
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k4e6otp.h"
#include "meizu_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define PFX "SUB_CAMERA_OTP"

#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 3
#define SUB_DEVICE_ID 0x6C

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_SUB_DRV"
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, SUB_DEVICE_ID>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

extern s5k4e6_otp_struct *sunny_otp_data;
extern s5k4e6_otp_struct *qtech_otp_data;
extern ov5695_otp_struct *ov5695_sunny_otp;
extern ov5695_otp_struct *ov5695_qtech_otp;

static int ReadOtp(u32 address,unsigned char *iBuffer,unsigned int buffersize)
{
		
		int i;
		
		CAM_CALDB("[SUB_CAMERA_OTP]ENTER  address:0x%x buffersize:%d\n ", address, buffersize);
		
		/*
		 * otp data had been read out in sensor driver,
		 * so just copy it to user.
		 */
		
		/*OV5695 sunny module OTP buffer data*/
		if(ov5695_sunny_otp != NULL){
			if ( ov5695_sunny_otp->module_id == OV5695_SUNNY_MODULE) {
				/*copy struct data*/	
				*(iBuffer)   = (unsigned char)ov5695_sunny_otp->module_id;
				*(iBuffer+1) = (unsigned char)ov5695_sunny_otp->lens_id;
				*(iBuffer+2) = (unsigned char)ov5695_sunny_otp->awb_rg_msb;
				*(iBuffer+3) = (unsigned char)ov5695_sunny_otp->awb_bg_msb;
				*(iBuffer+4) = (unsigned char)ov5695_sunny_otp->awb_lsb;
			}
			else{
				CAM_CALDB("[SUB_CAMERA_OTP] Get otp data fail! ov5695 Sunny module id is wrong!\n");
				return -1;
			}
		}
		/*OV5695 Qtech module OTP buffer data*/
		else if(ov5695_qtech_otp != NULL){
			if ( ov5695_qtech_otp->module_id == OV5695_QTECH_MODULE) {
				/*copy struct data*/	
				*(iBuffer)   = (unsigned char)ov5695_qtech_otp->module_id;
				*(iBuffer+1) = (unsigned char)ov5695_qtech_otp->lens_id;
				*(iBuffer+2) = (unsigned char)ov5695_qtech_otp->awb_rg_msb;
				*(iBuffer+3) = (unsigned char)ov5695_qtech_otp->awb_bg_msb;
				*(iBuffer+4) = (unsigned char)ov5695_qtech_otp->awb_lsb;
			}
			else{
				CAM_CALDB("[SUB_CAMERA_OTP] Get otp data fail! ov5695 Qtech module id is wrong!\n");
				return -1;
			}
		}
		/*S5K4E6 Sunny module OTP buffer data*/
		else if(sunny_otp_data != NULL){
			if ( sunny_otp_data->module_id == SUNNY_MODULE) {
				/*copy module id and awb data*/	
				*(iBuffer)   = (unsigned char)sunny_otp_data->module_id;
				*(iBuffer+1) = (unsigned char)sunny_otp_data->AWB_R;
				*(iBuffer+2) = (unsigned char)(sunny_otp_data->AWB_R>>8);
				*(iBuffer+3) = (unsigned char)sunny_otp_data->AWB_B;
				*(iBuffer+4) = (unsigned char)(sunny_otp_data->AWB_B>>8);
				*(iBuffer+5) = (unsigned char)sunny_otp_data->AWB_Gr_Gb;
				*(iBuffer+6) = (unsigned char)(sunny_otp_data->AWB_Gr_Gb>>8);
				/*copy lsc data*/
				memcpy(iBuffer+7, (sunny_otp_data->lsc_data), S5K4E6_LSC_SIZE);
			}
			else{
				CAM_CALDB("[SUB_CAMERA_OTP] Get otp data fail! s5k4e6 sunny module id is wrong!\n");
				return -1;
			}
		}
		/*S5K4E6 Qtech module OTP buffer data*/
		else if(qtech_otp_data != NULL){
			if ( qtech_otp_data->module_id == QTECH_MODULE) {
				/*copy struct data*/	
				*(iBuffer)   = (unsigned char)qtech_otp_data->module_id;
				*(iBuffer+1) = (unsigned char)qtech_otp_data->AWB_R;
				*(iBuffer+2) = (unsigned char)(qtech_otp_data->AWB_R>>8);
				*(iBuffer+3) = (unsigned char)qtech_otp_data->AWB_B;
				*(iBuffer+4) = (unsigned char)(qtech_otp_data->AWB_B>>8);
				*(iBuffer+5) = (unsigned char)qtech_otp_data->AWB_Gr_Gb;
				*(iBuffer+6) = (unsigned char)(qtech_otp_data->AWB_Gr_Gb>>8);
				/*copy lsc data*/
				memcpy(iBuffer+7, (qtech_otp_data->lsc_data), S5K4E6_LSC_SIZE);
			}
			else{
				CAM_CALDB("[SUB_CAMERA_OTP] Get otp data fail! qtech module id is wrong!\n");
				return -1;
			}
		}
		else{
		
			CAM_CALDB("[SUB_CAMEAR OTP] Get otp data fail! Otp data is NULL!\n");
			return -1;
		}

		for(i=0;i<buffersize;i++)
			CAM_CALDB("otp data addr=%d,data=0x%x\n",i,*(iBuffer+i));

		return 0;
}

/* Burst Read Data */
static int iReadData(u32 ui4_offset, unsigned int  ui4_length, u8 *pinputdata)
{
   int  i4RetValue = 0;
	
    /* read otp */
    ReadOtp(ui4_offset,pinputdata, ui4_length);
    for(i4RetValue = 0;i4RetValue<ui4_length;i4RetValue++){
    CAM_CALDB( "[SUB_CAMERA_OTP]pinputdata[%d]=0x%x\n", i4RetValue,*(pinputdata+i4RetValue));}
    CAM_CALDB(" [SUB_CAMERA_OTP]ui4_length = %d,ui4_offset =0x%x\n ",ui4_length,ui4_offset);
    CAM_CALDB("[SUB_CAMERA_OTP] iReadData done\n" );
   return 0;
}



//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	CAM_CALDB("[SUB_CAMERA_OTP]not implemented!");
	return 0;
}




#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k4e6otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    CAM_CALDB("[SUB_CAMERA_OTP] COMPAT_CAM_CALIOC_G_READ\n");
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	  CAM_CALDB("[SUB_CAMERA_OTP] s5k4e6otp_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[SUB_CAMERA_OTP] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	CAM_CALDB("[SUB_CAMERA_OTP] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[SUB_CAMERA_OTP] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALDB("ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALDB("[SUB_CAMERA_OTP] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("[SUB_CAMERA_OTP] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[SUB_CAMERA_OTP] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            CAM_CALDB("[SUB_CAMERA_OTP] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[SUB_CAMERA_OTP] length %d \n", ptempbuf->u4Length);
            //CAM_CALDB("[SUB_CAMERA_OTP] Before read Working buffer address 0x%p \n", pu1Params);

            i4RetValue = iReadData((u16)(ptempbuf->u4Offset), ptempbuf->u4Length, pu1Params);
            CAM_CALDB("[SUB_CAMERA_OTP] After read Working buffer data  0x%x \n", *pu1Params);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALDB("[SUB_CAMERA_OTP] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[SUB_CAMERA_OTP] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[SUB_CAMERA_OTP] to user  Working buffer address 0x%p \n", pu1Params);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALDB("[SUB_CAMERA_OTP] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[SUB_CAMERA_OTP] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[SUB_CAMERA_OTP] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    mdelay(2);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = s5k4e6otp_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[SUB_CAMERA_OTP] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[SUB_CAMERA_OTP] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[SUB_CAMERA_OTP] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[SUB_CAMERA_OTP] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_sub");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};



static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,
    .remove = CAM_CAL_i2c_remove,
//   .detect = CAM_CAL_i2c_detect,
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
int i4RetValue = 0;
    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = SUB_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP

    CAM_CALDB("[SUB_CAMERA_OTP] g_pstI2Cclient->addr = 0x%x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[SUB_CAMERA_OTP] register char device failed!\n");
        return i4RetValue;
    }

    CAM_CALDB("[SUB_CAMERA_OTP] Attached!! \n");
    return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

//MODULE_DESCRIPTION("CAM_CAL driver");
//MODULE_AUTHOR("lcz<lcz@meizu.com>");
//MODULE_LICENSE("GPL");


