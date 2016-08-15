/*****************************************************************************
 *
 * Filename:
 * ---------
 *    pmic_regulator.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines PMIC functions
 *
 * Author:
 * -------
 * Argus Lin
 *
 ****************************************************************************/
#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/syscalls.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <asm/uaccess.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/of_regulator.h>


#define BOOST_DEBUG
#define BOOST_DEBUG_PR_DBG



#include <mt-plat/mt_boost_regulator.h>


/*****************************************************************************
 * Global variable
 ******************************************************************************/


static int en = 0;
static int min_vol = 0;
static int max_vol = 0;
static struct mtk_boost_regulator mtk_boost_regulator_cb;
static int boost_voltaget_mapping[3][2]; //this size should not a harcode fix value, but should from OF
static int boost_voltaget_select;
static u32 boost_regulator_mask = 0; //debug purpose, only be set in attr store function

#ifdef BOOST_REGULATOR_LOG_HISTORY
int history[][];

#endif


#ifdef CONFIG_OF
struct platform_device mt_boost_regulator_device = {
	.name = "boost_regulator",
	.id = -1,
};

static const struct platform_device_id boost_regulator_id[] = {
	{"boost_regulator", 0},
	{},
};


static const struct of_device_id boost_regulator_of_ids[] = {
	{.compatible = "mediatek,buck_boost",},
	{},
};

static struct of_regulator_match boost_regulator_matches[] = {
	{.name = "buck_battery_boost", .driver_data = &mtk_boost_regulator_cb,},
	{},
};



MODULE_DEVICE_TABLE(of, boost_regulator_of_ids);
#endif








extern unsigned int fan49101_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT);

int mtk_boost_regulator_ops_enable(struct regulator_dev *rdev)
{
	en = 1;
	BOOSTLOG("mtk_regulator_enable\n");
	return en;
}

int mtk_boost_regulator_ops_disable(struct regulator_dev *rdev)
{
	en = 0;
	BOOSTLOG("mtk_regulator_disable\n");
	return en;
}


int mtk_boost_regulator_ops_is_enabled(struct regulator_dev *rdev)
{

	BOOSTLOG("mtk_boost_regulator_ops_is_enabled\n");
	return en;
}


int mtk_boost_regulator_ops_get_voltage(struct regulator_dev *rdev)
{
	BOOSTLOG("mtk_boost_regulator_ops_get_voltage\n");
	return boost_voltaget_mapping[boost_voltaget_select][0];
	//return min_vol;	
}


int mtk_boost_regulator_ops_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV, unsigned *selector)
{
	int i;
	int ret=0;

	if(boost_regulator_mask & 0x1){
		BOOSTLOG("mtk_boost_regulator_ops_set_voltage: disabled from layer to regulator : 0x%x\n", boost_regulator_mask);
		return ret;
	}
	
	if(max_uV != rdev->constraints->max_uV){
		BOOSTLOG("mtk_boost_regulator_ops_set_voltage: high risky!! max_vol = %d != constriant %d\n", max_uV, rdev->constraints->max_uV);
	}
		
	BOOSTLOG("mtk_boost_regulator_ops_set_voltage: %d, %d\n", min_uV, max_uV);
	min_vol = min_uV;
	max_vol = max_uV;

	//use the min_vol

	//find out the minimum register setting for this voltage
	for(i=0; i<(sizeof(boost_voltaget_mapping)/sizeof(boost_voltaget_mapping[0])); i++)
	{
		BOOSTLOG("mtk_boost_regulator_ops_set_voltage: lookup table : %d, 0x%x\n", boost_voltaget_mapping[i][0], boost_voltaget_mapping[i][1]);	
		if(min_uV <= boost_voltaget_mapping[i][0])
			break;
	}
	if((3 <= i) || (3400000 < boost_voltaget_mapping[i][0])){
		BOOSTLOG("mtk_boost_regulator_ops_set_voltage: index overflow : %d\n", i);	
		return -ESPIPE;
	}

	boost_voltaget_select = i;
	BOOSTLOG("mtk_boost_regulator_ops_set_voltage: use setting : %d, 0x%x\n", boost_voltaget_mapping[boost_voltaget_select][0], boost_voltaget_mapping[boost_voltaget_select][1]);
	if(0 == (boost_regulator_mask & 0x2)){
		ret = fan49101_config_interface(0x1, boost_voltaget_mapping[i][1], 0xff, 0x0);
		BOOSTLOG("mtk_boost_regulator_ops_set_voltage: set fan49101, result : %d\n", ret);	
	}
	return ret;
}




static struct regulator_ops mtk_boost_regulator_ops = {
	.enable = mtk_boost_regulator_ops_enable,
	.disable = mtk_boost_regulator_ops_disable,
	.is_enabled = mtk_boost_regulator_ops_is_enabled,
	.get_voltage = mtk_boost_regulator_ops_get_voltage,
	.set_voltage = mtk_boost_regulator_ops_set_voltage,
};




static int proc_utilization_show(struct seq_file *m, void *v)
{
	pr_crit("[boost]boost_regulator_init log test\n");
	BOOSTLOG("boost 2");
	seq_puts(m, "********** boost regulator debug shit**********\n");


	return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations boost_regulator_debug_proc_fops = {
	.open = proc_utilization_open,
	.read = seq_read,
};


void boost_regulator_debug_init(struct platform_device *dev)
{
	struct dentry *boost_regulator_dir;

	boost_regulator_dir = debugfs_create_dir("boost_regulator", NULL);
	if (!boost_regulator_dir) {
		BOOSTERR("fail to mkdir /sys/kernel/debug/boost_regulator\n");
		return;
	}

	debugfs_create_file("dump_status", S_IRUGO | S_IWUSR, boost_regulator_dir, NULL, &boost_regulator_debug_proc_fops);

	BOOSTLOG("proc_create boost_regulator_debug_proc_fops done\n");
	dev_crit(&dev->dev, "boost_regulator_debug_init\n");

}


struct regulator *testReg[2];
static int  userSetting[2][2];




void boost_regulator_single_test(struct device *dev, int userNum, int min_vol, int max_vol)
{
	int ret1, ret2;
	struct regulator *reg;
	int rvoltage;


	BOOSTLOG("boost_regulator_single_test : user %d, %d, %d\n", userNum, min_vol, max_vol);
	if(1<userNum)
		return;
	if(!testReg[userNum]){
		BOOSTLOG("boost_regulator_single_test : reg null, get one\n");
		testReg[userNum] = regulator_get(dev, "vBuck_boost");
		if (!testReg[userNum])
			return;
	}
	
		
	reg = testReg[userNum];
	/*on/off status*/
	ret1 = regulator_is_enabled(reg);
	if(0 < ret1){
		BOOSTLOG("boost_regulator_single_test : regulator enabled\n");
	}
	else if(0 == ret1){
		ret2 = regulator_enable(reg);
		BOOSTLOG("boost_regulator_single_test : turn on boost_regulator %d\n", ret2);
	}
	else{
		BOOSTLOG("boost_regulator_single_test : regulator invalid %d\n", ret1);
		return;
	}

	ret1 = regulator_set_voltage(reg, min_vol, max_vol);
	rvoltage = regulator_get_voltage(reg);
	BOOSTLOG("boost_regulator_single_test : result- %d, %d\n", ret1, rvoltage);
}




/*
	boost_regulator consumer can refer to following code
*/
int boost_regulator_user_get_setting(char *userName, int *setting)
{
	struct device_node *node_1, *node_2, *node_3;
	int ret1 = 0;
	
	node_1 = of_find_compatible_node(NULL, NULL, "mediatek,mt_boost");
	if(!node_1){
		BOOSTLOG("boost_regulator_user_test : find node_1 failed\n");				
		return -ENODEV;
	}
	node_2 = of_get_child_by_name(node_1, "buck_regulators");
	if(!node_2){
		BOOSTLOG("boost_regulator_user_test : find node_2 failed\n");				
		return -ENODEV;
	}
	node_3 = of_get_child_by_name(node_2, "buck_battery_boost");
	if(!node_3){
		BOOSTLOG("boost_regulator_user_test : find node_3 failed\n");				
		return -ENODEV;
	}
	ret1 = of_property_read_u32_array(node_3, userName, setting, 2);
	return ret1;
}



void boost_regulator_user_test(struct device *dev, int userNum, int scenario)
{
	int ret1, ret2;
	struct regulator *reg;
	int rvoltage;


	BOOSTLOG("boost_regulator_user_test : user %d, %d\n", userNum, scenario);
	if(1<userNum)
		return;
	if(!testReg[userNum]){
		BOOSTLOG("boost_regulator_user_test : reg null, get one\n");
		testReg[userNum] = regulator_get(dev, "vBuck_boost");
		if (!testReg[userNum])
			return;
	}


	
		
	reg = testReg[userNum];
	/*on/off status*/
	ret1 = regulator_is_enabled(reg);
	if(0 < ret1){
		BOOSTLOG("boost_regulator_user_test : regulator enabled\n");
	}
	else if(0 == ret1){
		ret2 = regulator_enable(reg);
		BOOSTLOG("boost_regulator_user_test : turn on boost_regulator %d\n", ret2);
	}
	else{
		BOOSTLOG("boost_regulator_user_test : regulator invalid %d\n", ret1);
		return;
	}

	if(0 == userSetting[userNum][scenario]){
		
		char *desp;

		BOOSTLOG("boost_regulator_user_test : setting empty, get it\n");
		if(userNum)
			desp = "nfc";
		else
			desp = "vcn33";

		BOOSTLOG("boost_regulator_user_test : find %s\n", desp);

		ret1 = boost_regulator_user_get_setting(desp, userSetting[userNum]);
		if(ret1){
			BOOSTLOG("boost_regulator_user_test : get setting failed : %d\n", ret1);				
			return;
		}
		BOOSTLOG("boost_regulator_user_test : get setting : %d, %d\n", userSetting[userNum][0],userSetting[userNum][1]);
	}

	BOOSTLOG("boost_regulator_user_test : will set %d\n", userSetting[userNum][scenario]);
	ret1 = regulator_set_voltage(reg, userSetting[userNum][scenario], 3400000);
	rvoltage = regulator_get_voltage(reg);
	BOOSTLOG("boost_regulator_user_test : result- %d, %d\n", ret1, rvoltage);
}


static ssize_t boost_regulator_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;
	int bufLen = PAGE_SIZE;
	
	BOOSTLOG("boost_regulator_status_show\n");
	//reg = mtk_boost_regulator_cb.reg;
	len += snprintf(buf + len, bufLen - len, "boost_regulator_status_show\n");
	//len += snprintf(buf + len, bufLen - len, "regulator status : always on : %d\n", value);
	len += snprintf(buf + len, bufLen - len, "regulator status : min_uV : %d\n", min_vol);
	len += snprintf(buf + len, bufLen - len, "regulator status : max_uV : %d\n", max_vol);
	len += snprintf(buf + len, bufLen - len, "regulator status : mask : 0x%x\n", boost_regulator_mask);
	
	return len;
}

static ssize_t boost_regulator_constraint_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;
	int bufLen = PAGE_SIZE;
	struct regulator_dev *rdev = mtk_boost_regulator_cb.rdev;
	
	BOOSTLOG("boost_regulator_constraint_show\n");
	if(!rdev){
		len += snprintf(buf + len, bufLen - len, "regulator constraints : regulator not registerred !\n");
		return len;
	}
	
	len += snprintf(buf + len, bufLen - len, "regulator constraints : constraints : %p\n", rdev->constraints);
	if(rdev->constraints){
		len += snprintf(buf + len, bufLen - len, "regulator constraints : constraints namne : %s\n", rdev->constraints->name);
		len += snprintf(buf + len, bufLen - len, "regulator constraints : constraints valid_ops : %d\n", rdev->constraints->valid_ops_mask);		
		len += snprintf(buf + len, bufLen - len, "regulator constraints : constraints vol : %d, %d\n", rdev->constraints->min_uV, rdev->constraints->max_uV);		
	}
	len += snprintf(buf + len, bufLen - len, "regulator matches : constraints : %p\n", boost_regulator_matches[0].init_data);
	if(boost_regulator_matches[0].init_data){
		len += snprintf(buf + len, bufLen - len, "regulator matches : constraints namne : %s\n", boost_regulator_matches[0].init_data->constraints.name);
		len += snprintf(buf + len, bufLen - len, "regulator matches : constraints valid_ops : %d\n", boost_regulator_matches[0].init_data->constraints.valid_ops_mask);		
		len += snprintf(buf + len, bufLen - len, "regulator matches : constraints vol : %d, %d\n", boost_regulator_matches[0].init_data->constraints.min_uV, boost_regulator_matches[0].init_data->constraints.max_uV);		
	}

	len += snprintf(buf + len, bufLen - len, "regulator init_data : constraints : %p\n", mtk_boost_regulator_cb.config.init_data);
	if(mtk_boost_regulator_cb.config.init_data){
		len += snprintf(buf + len, bufLen - len, "regulator init_data : constraints namne : %s\n", mtk_boost_regulator_cb.config.init_data->constraints.name);
		len += snprintf(buf + len, bufLen - len, "regulator init_data : constraints valid_ops : %d\n", mtk_boost_regulator_cb.config.init_data->constraints.valid_ops_mask);		
		len += snprintf(buf + len, bufLen - len, "regulator init_data : constraints vol : %d, %d\n", mtk_boost_regulator_cb.config.init_data->constraints.min_uV, mtk_boost_regulator_cb.config.init_data->constraints.max_uV);		
	}
	
	return len;
}



static ssize_t boost_regulator_batch_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	BOOSTLOG("boost_regulator_boost_batch_test_show\n");
	return sprintf(buf, "boost_regulator_boost_batch_test_show\n");
}

static ssize_t boost_regulator_batch_test_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	BOOSTLOG("boost_regulator_boost_batch_test_store\n");
	
	boost_regulator_single_test(dev, 1, 3100000, 3400000);
	boost_regulator_single_test(dev, 2, 3200000, 3300000);
	boost_regulator_single_test(dev, 1, 3340000, 3400000);
	return size;
}


static ssize_t boost_regulator_single_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	BOOSTLOG("boost_regulator_boost_single_test_show\n");
	return sprintf(buf, "format :   echo user(0/1) min_uVol max_uVol > single_test\n");
}

static ssize_t boost_regulator_single_test_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int user, min_uVol,max_uVol;

	/*format :   echo user(1/2) min_uVol max_uVol > single_test*/
	BOOSTLOG("boost_regulator_boost_single_test_store:%s\n", buf);
	//BOOSTLOG("%s\n", buf);
	sscanf(buf, "%d %d %d", &user, &min_uVol, &max_uVol);	
	if(2 <= user)
		return size;
	boost_regulator_single_test(dev, user, min_uVol, max_uVol);
	
	return size;
}


static ssize_t boost_regulator_user_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	BOOSTLOG("boost_regulator_boost_single_test_show\n");
	return sprintf(buf, "format :   echo user(0:VCN33/1:NFC) state(0:low_power/1:normal) > user_test\n");
}


static ssize_t boost_regulator_user_test_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int user, scenario;

	/*format :   echo user(0:VCN33/1:NFC) state(0:low_power/1:normal) > user_test*/
	BOOSTLOG("boost_regulator_boost_user_test_store:%s\n", buf);
	//BOOSTLOG("%s\n", buf);
	sscanf(buf, "%d %d", &user, &scenario);
	if((2 <= user) || (2 <= scenario))
		return size;
	boost_regulator_user_test(dev, user, scenario);
	
	return size;
}

static ssize_t boost_regulator_disable_dv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len=0;
	int bufLen = PAGE_SIZE;

	BOOSTLOG("boost_regulator_disable_dv_show\n");
	len += snprintf(buf + len, bufLen - len, "disable_dv is to disable voltage change from different layers : 0x%x\n", boost_regulator_mask);
	len += snprintf(buf + len, bufLen - len, "format :   echo switch(0:clear/1:set) bit_mask(hex, 0x1:layer to regulator/0x2:to buck driver) > disable_dv\n");

	return len;
}


static ssize_t boost_regulator_disable_dv_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int set;
	u32 mask;

	/*format :   echo user(0:VCN33/1:NFC) state(0:low_power/1:normal) > user_test*/
	BOOSTLOG("boost_regulator_disable_dv_store:%s\n", buf);
	//BOOSTLOG("%s\n", buf);
	sscanf(buf, "%d %x", &set, &mask);
	if((2 <= set) || (0x4 <= mask))
		return size;
	if(set){
		boost_regulator_mask |= mask;
	}
	else{
		boost_regulator_mask &= ~mask;
	}
	return size;
}




static DEVICE_ATTR(status, 0664, boost_regulator_status_show, NULL);	/* 664 */
static DEVICE_ATTR(constraint, 0664, boost_regulator_constraint_show, NULL);	/* 664 */
static DEVICE_ATTR(batch_test, 0664, boost_regulator_batch_test_show, boost_regulator_batch_test_store);	/* 664 */
static DEVICE_ATTR(single_test, 0664, boost_regulator_single_test_show, boost_regulator_single_test_store);	/* 664 */
static DEVICE_ATTR(user_test, 0664, boost_regulator_user_test_show, boost_regulator_user_test_store);	/* 664 */
static DEVICE_ATTR(disable_dv, 0664, boost_regulator_disable_dv_show, boost_regulator_disable_dv_store);	/* 664 */
//static DEVICE_ATTR(fix_vol, 0664, boost_regulator_fix_vol_show, boost_regulator_fix_vol_store);	/* 664 */


static struct device_attribute *boost_regulator_attr_list[] = {
	&dev_attr_status,     
	&dev_attr_constraint,	
	&dev_attr_batch_test,   
	&dev_attr_single_test,	 
	&dev_attr_user_test,	 
	&dev_attr_disable_dv,	 
};



#ifdef CONFIG_OF

static int boost_regulator_init(struct platform_device *pdev)
{

	struct device_node *np, *regulators, *buck_boost;
	int matched, ret;
	const __be32 *table=0;
	u32 len;
	//u32 num_1,num_2,num_3,num_4;


	pdev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,mt_boost");
	np = of_node_get(pdev->dev.of_node);
	if (!np){
		BOOSTERR("[boost]find of failed\n");
		return -EINVAL;
	}

	regulators = of_get_child_by_name(np, "buck_regulators");
	if (!regulators) {
		BOOSTERR("[boost]regulators node not found\n");
		ret = -EINVAL;
	}
	matched = of_regulator_match(&pdev->dev, regulators, boost_regulator_matches, 1);
	if (matched < 0) {
		BOOSTERR("[boost]Error parsing regulator init data: %d \n", matched );
		return matched;
	}


	


	mtk_boost_regulator_cb.config.dev = &(pdev->dev);
	mtk_boost_regulator_cb.config.init_data = boost_regulator_matches[0].init_data;
	mtk_boost_regulator_cb.config.of_node = boost_regulator_matches[0].of_node;
	mtk_boost_regulator_cb.config.driver_data = boost_regulator_matches[0].driver_data;

	mtk_boost_regulator_cb.desc.name = "boost_regulator";
	mtk_boost_regulator_cb.desc.type = REGULATOR_VOLTAGE;
	mtk_boost_regulator_cb.desc.n_voltages = 3; //currently we only has 3.1, 3.2, 3.4, but we cannot hardcoding here
	mtk_boost_regulator_cb.desc.ops = &mtk_boost_regulator_ops;
	mtk_boost_regulator_cb.desc.owner = THIS_MODULE;
	mtk_boost_regulator_cb.rdev = regulator_register(&mtk_boost_regulator_cb.desc, &mtk_boost_regulator_cb.config);
	//mtk_boost_regulator_cb.reg = regulator_get(&(pdev->dev), "vBuck_boost");
	BOOSTLOG("[boost]boost_regulator_init done:%p, %p\n", mtk_boost_regulator_cb.rdev, mtk_boost_regulator_cb.reg);

	buck_boost = of_get_child_by_name(regulators, "buck_battery_boost");
	if(!buck_boost){
		BOOSTERR("[boost]buck_boost node not found\n");
		return 0;
	}
	table = of_get_property(buck_boost, "buck_setting_array", &len);
	if(table){
		int i;
		
		BOOSTLOG("[boost]boost_regulator_init:table:%d,%d,0x%x,%d,0x%x\n", len,be32_to_cpu(*(table + 0)), be32_to_cpu(*(table + 1)), be32_to_cpu(*(table + 2)), be32_to_cpu(*(table + 3)));
		for(i=0; i<(sizeof(boost_voltaget_mapping)/sizeof(boost_voltaget_mapping[0])) ;i++){
			boost_voltaget_mapping[i][0] = be32_to_cpu(*(table +(2*i)+ 0));
			boost_voltaget_mapping[i][1] = be32_to_cpu(*(table +(2*i)+ 1));
			BOOSTLOG("boost_regulator_init: fill table : %d, 0x%x\n", boost_voltaget_mapping[i][0], boost_voltaget_mapping[i][1]);			
		}
	}


	return 0;
}

static int mt_boost_regulator_driver_probe(struct platform_device *pdev)
{
	int ret;
	int num, idx, err;

	ret = boost_regulator_init(pdev); //pdev is boost_regulator itself, pass this to boost regulator init, fill necessary infomration from DT to boost device
	if(ret){
		return ret;
	}
	boost_regulator_debug_init(pdev);
    num = (int)(sizeof(boost_regulator_attr_list)/sizeof(boost_regulator_attr_list[0]));
    for (idx = 0; idx < num; idx++) {
        if ((err = device_create_file(&pdev->dev, boost_regulator_attr_list[idx])))
            break;
    }



	BOOSTLOG("[boost]boost regulator probe done\n");
	return 0;


}

static int mt_boost_regulator_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mt_boost_regulator_driver = {
	.driver = {
		   .name = "boost_regulator",
		   .owner = THIS_MODULE,
		   .of_match_table = boost_regulator_of_ids,
		   },
	.probe = mt_boost_regulator_driver_probe,
	.remove = mt_boost_regulator_driver_remove,
};
#endif				/* End of #ifdef CONFIG_OF */







static int __init mtk_boost_regulator_module_init(void)
{

	int ret = 0;


#ifdef CONFIG_OF


	ret = platform_device_register(&mt_boost_regulator_device);
	if (ret) {
		BOOSTERR("****[mtk_boost_regulator_init] Unable to device register(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&mt_boost_regulator_driver);
	if (ret) {
		BOOSTERR("****[mtk_boost_regulator_init] Unable to register driver by DT(%d)\n", ret);
		return ret;
	}
#endif				/* End of #ifdef CONFIG_OF */
	

	return ret;
}

static void __exit mtk_boost_regulator_module_exit(void)
{
#ifdef CONFIG_OF
	platform_driver_unregister(&mt_boost_regulator_driver);
#endif
}


module_init(mtk_boost_regulator_module_init);
module_exit(mtk_boost_regulator_module_exit);



MODULE_AUTHOR("JY Lan");
MODULE_DESCRIPTION("Boost Regulator driver");
MODULE_LICENSE("GPL");
