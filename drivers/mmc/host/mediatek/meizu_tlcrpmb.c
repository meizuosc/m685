#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/random.h>
#include <linux/memory.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <crypto/hash.h>

#include <linux/scatterlist.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include "drivers/mmc/card/queue.h"

#include "meizu_tlcrpmb.h"
#include "mt_sd.h"

/* TEE usage */
#include "mobicore_driver_api.h"
//#include "drrpmb_Api.h"
#include "tlrpmb_Api.h"

static struct mc_uuid_t tlcrpmb_uuid = MEIZU_TCIRPMB_AP_UUID;
static struct mc_session_handle *rpmb_session = NULL;
static u32 rpmb_devid = MC_DEVICE_ID_DEFAULT;
static tciMessage_t *rpmb_tci;

#define RPMB_NAME "meizu_tlcrpmb"

/* Debug message event */
#define DBG_EVT_NONE (0) /* No event */
#define DBG_EVT_CMD  (1 << 0)/* SEC CMD related event */
#define DBG_EVT_FUNC (1 << 1)/* SEC function event */
#define DBG_EVT_INFO (1 << 2)/* SEC information event */
#define DBG_EVT_WRN  (1 << 30) /* Warning event */
#define DBG_EVT_ERR  (1 << 31) /* Error event */
#define DBG_EVT_ALL  (0xffffffff)

#define DBG_EVT_MASK (DBG_EVT_ALL)

#define MSG(evt, fmt, args...) \
do {\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		pr_err("[%s] "fmt, RPMB_NAME, ##args); \
	} \
} while (0)

static struct task_struct *loader_th;
static DEFINE_MUTEX(meizu_tlcrpmb_mutex);

#define MEIZU_LOCK_BLOCK_ADDR 0
#define MEIZU_LOCK_DEVICE_TAG "lock"

/*
 * End of above.
 *
 **********************************************************************************/


static int __meizu_tlcrpmb_open_session(void)
{
	int cnt = 0;
	enum mc_result mc_ret = MC_DRV_ERR_UNKNOWN;

	MSG(INFO, "%s start\n", __func__);

	do {
		msleep(2000);

		rpmb_session = (void *)kmalloc(sizeof(*rpmb_session), GFP_KERNEL);
		if(!rpmb_session) {
			rpmb_session = NULL;
			cnt++;
		}

		/* open device */
		mc_ret = mc_open_device(rpmb_devid);
		if (mc_ret != MC_DRV_OK) {
			MSG(ERR, "%s, mc_open_device failed: %d\n", __func__, mc_ret);
			kfree(rpmb_session);
			rpmb_session = NULL;
			cnt++;
			continue;
		}

		MSG(INFO, "%s, mc_open_device success.\n", __func__);


		/* allocating WSM for TCI */
		mc_ret = mc_malloc_wsm(rpmb_devid, 0, sizeof(tciMessage_t), (uint8_t **)&rpmb_tci, 0);
		if (mc_ret != MC_DRV_OK) {
			mc_close_device(rpmb_devid);
			MSG(ERR, "%s, mc_malloc_wsm failed: %d\n", __func__, mc_ret);
			kfree(rpmb_session);
			rpmb_session = NULL;
			cnt++;
			continue;
		}

		MSG(INFO, "%s, mc_malloc_wsm success.\n", __func__);
		MSG(INFO, "uuid[0]=%d, uuid[1]=%d, uuid[2]=%d, uuid[3]=%d\n",
			tlcrpmb_uuid.value[0], tlcrpmb_uuid.value[1], tlcrpmb_uuid.value[2], tlcrpmb_uuid.value[3]);

		rpmb_session->device_id = rpmb_devid;

		/* open session */
		mc_ret = mc_open_session(rpmb_session,
								 &tlcrpmb_uuid,
								 (uint8_t *) rpmb_tci,
								 sizeof(tciMessage_t));

		if (mc_ret != MC_DRV_OK) {
			MSG(ERR, "%s, mc_open_session failed.(%d)\n", __func__, cnt);

			mc_ret = mc_free_wsm(rpmb_devid, (uint8_t *)rpmb_tci);
			MSG(ERR, "%s, free wsm result (%d)\n", __func__, mc_ret);

			mc_ret = mc_close_device(rpmb_devid);
			MSG(ERR, "%s, try free wsm and close device\n", __func__);

			kfree(rpmb_session);
			rpmb_session = NULL;
			cnt++;
			continue;
		} else {
			break;
		}

	} while (cnt < 30);

	if (cnt >= 30)
		MSG(ERR, "%s, open session failed!!!\n", __func__);


	MSG(INFO, "%s end, mc_ret = %x\n", __func__, mc_ret);

	return mc_ret;
}

static int meizu_tlcrpmb_read_block(unsigned char *buf, size_t buf_len, int blk)
{
	int ret = 0;
	unsigned char nonce[RPMB_SZ_NONCE] = {0};

	struct mmc_card *card = mtk_msdc_host[0]->mmc->card;
	struct emmc_rpmb_req rpmb_req;
	struct s_rpmb *rpmb_frame;

	MSG(INFO, "%s: RPMB_CMD_READ_DATA. addr = %d\n", __func__, blk);

	memset(buf, 0, buf_len);
	get_random_bytes(nonce, RPMB_SZ_NONCE);

	rpmb_req.type = RPMB_READ_DATA;
	rpmb_req.blk_cnt = 1;
	rpmb_req.addr = blk;
	rpmb_req.data_frame = buf;

	rpmb_frame = (void *)rpmb_req.data_frame;
	rpmb_frame->request = cpu_to_be16p(&rpmb_req.type);
	rpmb_frame->address = cpu_to_be16p(&rpmb_req.addr); /* BUG */
	memcpy(rpmb_frame->nonce, nonce, RPMB_SZ_NONCE);

	ret = emmc_rpmb_req_handle(card, &rpmb_req);
	if (ret)
		MSG(ERR, "%s, emmc_rpmb_req_read_data failed!!(%x)\n", __func__, ret);

	return ret;
}

static int meizu_tlcrpmb_execute_cmd(tciMessage_t *tci)
{
	int ret = 0;

	tci = tci;

	/* STEP 1: open session*/
	if(!rpmb_session) {
		ret = __meizu_tlcrpmb_open_session();
		if(ret || !rpmb_session) {
			goto exit;
		}
	}

	/* STEP 2: notify */
	ret = mc_notify(rpmb_session);
	if (MC_DRV_OK != ret) {
		MSG(ERR, "Notify failed: %d\n", ret);
		goto exit;
	}

	/* STEP 3: wait for response */
	ret = mc_wait_notification(rpmb_session, MC_INFINITE_TIMEOUT);
	if (MC_DRV_OK != ret) {
		MSG(ERR, "Wait for response notification failed: 0x%x\n", ret);
		goto exit;
	}

exit:
	return ret;
}


///////////////////////////////////////////////export symbol//////////////////////////////////////

int meizu_tlcrpmb_get_rsa_seed(unsigned char *buf, size_t buf_len)
{
	tciMessage_t *tci = NULL;
	int ret = -1;

	mutex_lock(&meizu_tlcrpmb_mutex);

	if(buf_len != MEIZU_RSA_SEED_LEN) {
		MSG(ERR, "ras seed buffer length error\n");
		goto exit;
	}

	/*　STEP 1: Execute cmd*/
	tci = rpmb_tci;
	tci->cmdrpmb.header.commandId = CMD_MZSS_GET_RSA_SEED;
	ret = meizu_tlcrpmb_execute_cmd(tci);
	if(ret) {
		MSG(ERR, "get rsa seed failed,\n");
		goto exit;
	}

	/* STEP 2: copy to buffer*/
	memcpy(buf, tci->Buf, buf_len);

exit:
	mutex_unlock(&meizu_tlcrpmb_mutex);
	return ret;
}

int meizu_tlcrpmb_lock_device(void)
{
	tciMessage_t *tci = NULL;
	int ret = -1;

	mutex_lock(&meizu_tlcrpmb_mutex);

	/*　STEP 1: Execute cmd*/
	tci = rpmb_tci;
	tci->cmdrpmb.header.commandId = CMD_MZSS_LOCK_DEVICE;
	ret = meizu_tlcrpmb_execute_cmd(tci);
	if(ret) {
		MSG(ERR, "get rsa seed failed,\n");
		goto exit;
	}

exit:
	mutex_unlock(&meizu_tlcrpmb_mutex);
	return ret;
}

int meizu_tlcrpmb_unlock_device(unsigned char *buf, size_t buf_len)
{
	tciMessage_t *tci = NULL;
	int ret = -1;

	mutex_lock(&meizu_tlcrpmb_mutex);

	tci = rpmb_tci;

	/* STEP 1: copy to buffer*/
	if(buf_len > sizeof(tci->Buf)) {
		goto exit;
	}
	memcpy(tci->Buf, buf, buf_len);

	/*　STEP 2: Execute cmd*/
	tci->cmdrpmb.header.commandId = CMD_MZSS_UNLOCK_DEVICE;
	ret = meizu_tlcrpmb_execute_cmd(tci);
	if(ret) {
		MSG(ERR, "get rsa seed failed,\n");
		goto exit;
	}

	/* STEP 3: check */

	
exit:
	mutex_unlock(&meizu_tlcrpmb_mutex);
	return ret;
}

int meizu_tlcrpmb_is_dev_locked(void)
{
	unsigned char buf[512];
	int ret = 0, campare;
	struct s_rpmb *rpmb_frame;

	ret = meizu_tlcrpmb_read_block(buf, sizeof(buf), MEIZU_LOCK_BLOCK_ADDR);

	rpmb_frame = (void *)buf;
	if(rpmb_frame->result) {
		printk("[rpmb]ERROR: read rpmb error, errno = %d.\n", rpmb_frame->result);
		ret = -1;
		goto exit;
	}

	campare = strncmp(rpmb_frame->data, MEIZU_LOCK_DEVICE_TAG, sizeof(MEIZU_LOCK_DEVICE_TAG));

#if 0
	printk("block msg:\n");
	for(int i=0; i<sizeof(buf); i++) {
		if(i % 10 == 0) {
			printk("\n");
		}
		printk("0x%x, ", buf[i]);
	}
	printk("\n");
#endif

	/* the same */
	if(!campare) {
		ret = 1;
	}

exit:
	return ret;
}
//EXPORT_SYMBOL(meizu_tlcrpmb_is_dev_locked);

int meizu_tlcrpmb_root_device(void)
{
	//TODO: send request to rpmb TA
	return 0;
}

int meizu_tlcrpmb_unroot_device(void)
{
	//TODO:send request to rpmb TA
	return 0;
}

int meizu_tlcrpmb_is_dev_rooted(void)
{
	//TODO:read from rpmb
	return 0;
}
//EXPORT_SYMBOL(meizu_tlcrpmb_is_dev_rooted);

///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG
static int meizu_tlcrpmb_test(void)
{
	unsigned char buf[20];
	int ret = 0, i;

	/* test lock or not*/
	printk("[rpmb] devlock = %d\n", meizu_tlcrpmb_is_dev_locked());

	/* test lock */
	printk("[rpmb] test lock device----------------------------------\n");
	ret = meizu_tlcrpmb_lock_device();
	if(ret) {
		MSG(ERR, "meizu_tlcrpmb_lock_device test failed\n");
	}

	/* test lock or not*/
	printk("[rpmb] devlock = %d\n", meizu_tlcrpmb_is_dev_locked());

	/* test seed */
	printk("[rpmb] test rsa seed  ----------------------------------\n");
	ret = meizu_tlcrpmb_get_rsa_seed(buf, sizeof(buf));
	if(ret) {
		MSG(ERR, "meizu_tlcrpmb_get_rsa_seed test failed\n");
	} else {
		MSG(INFO, "RSA SEED:");
		for(i=0; i<sizeof(buf); i++) {
			printk("0x%x, ", buf[i]);
		}
		printk("\n");
	}

	/* test lock or not*/
	printk("[rpmb] devlock = %d\n", meizu_tlcrpmb_is_dev_locked());

	/* test unlock */
	printk("[rpmb] test unlock device----------------------------------\n");
	ret = meizu_tlcrpmb_unlock_device(buf, sizeof(buf));
	if(ret) {
		MSG(ERR, "meizu_tlcrpmb_unlock_device test failed\n");
	}

	printk("[rpmb] devlock = %d\n", meizu_tlcrpmb_is_dev_locked());

	return 0;
}
#endif /*DEBUG*/

static int meizu_tlcrpmb_loader_thread(void *context)
{
	int ret = 0;

	MSG(INFO, "%s start\n", __func__);

	mutex_lock(&meizu_tlcrpmb_mutex);
	if(!rpmb_session) {
		ret = __meizu_tlcrpmb_open_session();
	}
	mutex_unlock(&meizu_tlcrpmb_mutex);

	MSG(INFO, "%s end, ret = %x\n", __func__, ret);

#ifdef DEBUG
	meizu_tlcrpmb_test();
#endif /*DEBUG*/

	return 0;
}

static int __init meizu_tlcrpmb_init(void)
{
	MSG(INFO, "%s start\n", __func__);

	loader_th = kthread_run(meizu_tlcrpmb_loader_thread, NULL, "meizu_tlcrpmb_loader");
	if (IS_ERR(loader_th))
		MSG(ERR, "%s, init kthread_run failed!\n", __func__);

	MSG(INFO, "emmc_rpmb_init end!!!!\n");
	return 0;
}

late_initcall(meizu_tlcrpmb_init);

