#ifndef _PMIC_REGULATOR_H_
#define _PMIC_REGULATOR_H_

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#endif
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

//#include <mach/pmic.h>
//#include <mach/pmic_irq.h>

#define GETSIZE(array) (sizeof(array)/sizeof(array[0]))


#define BOOSTTAG                "[BOOST] "
#ifdef BOOST_DEBUG
#define BOOSTDEB(fmt, arg...) pr_debug(BOOSTTAG "cpuid=%d, " fmt, raw_smp_processor_id(), ##arg)
#define BOOSTFUC(fmt, arg...) pr_debug(BOOSTTAG "cpuid=%d, %s\n", raw_smp_processor_id(), __func__)
#endif  /*-- defined PMIC_DEBUG --*/
#if defined BOOST_DEBUG_PR_DBG
#define BOOSTLOG(fmt, arg...)   pr_err(BOOSTTAG fmt, ##arg)
#else
#define BOOSTLOG(fmt, arg...)
#endif  /*-- defined PMIC_DEBUG_PR_DBG --*/
#define BOOSTERR(fmt, arg...)   pr_err(BOOSTTAG "ERROR,line=%d " fmt, __LINE__, ##arg)
#define BOOSTREG(fmt, arg...)   pr_debug(BOOSTTAG fmt, ##arg)


#define BOOST_EN REGULATOR_CHANGE_STATUS
#define BOOST_VOL REGULATOR_CHANGE_VOLTAGE
#define BOOST_EN_VOL (BOOST_EN | BOOST_VOL)



//extern struct mtk_regulator mtk_ldos[];
extern struct of_regulator_match pmic_regulator_matches[];



struct mtk_boost_regulator {
	struct regulator_desc desc;
	struct regulator_init_data init_data;
	struct regulator_config config;
	struct device_attribute en_att;
	struct device_attribute voltage_att;
	struct regulator_dev *rdev;
	//const void *pvoltages;
	//bool isUsedable;
	struct regulator *reg;
	//int vsleep_en_saved;
	/*--- Add to record selector ---*/
	//struct mtk_regulator_vosel vosel;
	/*--- BUCK/LDO ---*/
	const char *type;
};

extern unsigned int fan49101_read_interface(unsigned char RegNum, unsigned char *val,
					    unsigned char MASK, unsigned char SHIFT);
extern unsigned int fan49101_config_interface(unsigned char RegNum, unsigned char val,
					      unsigned char MASK, unsigned char SHIFT);



#endif				/* _PMIC_REGULATOR_H_ */
