#ifndef _MEIZU_TLCRPMB_H
#define _MEIZU_TLCRPMB_H

#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include "emmc_rpmb.h"

extern struct msdc_host *mtk_msdc_host[];
extern int emmc_rpmb_req_handle(struct mmc_card *card, struct emmc_rpmb_req *rpmb_req);

#define MEIZU_RSA_SEED_LEN 20
#define MEIZU_TCIRPMB_AP_UUID {{6, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}

#endif
