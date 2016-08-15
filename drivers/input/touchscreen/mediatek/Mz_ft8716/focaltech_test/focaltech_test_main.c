/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: focaltech_test_main.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test entry for all IC
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>

#include "focaltech_test_main.h"
#include "focaltech_test_global.h"
#include "focaltech_test_config_ft5x46.h"
#include "focaltech_test_ft5x46.h"
#include "focaltech_test_config_ft8716.h"
#include "focaltech_test_ft8716.h"
#include "focaltech_test_ini.h"
#include "focaltech_comm.h"
#include "focaltech_core.h"

#define FTS_DRIVER_LIB_INFO  "Test_Lib_Version   V1.6.0 2016-03-01"

#define FTS_TEST_STORE_DATA_SIZE		80*1024

FTS_I2C_READ_FUNCTION fts_i2c_read_test;
FTS_I2C_WRITE_FUNCTION fts_i2c_write_test;

char *g_testparamstring = NULL;

/////////////////////IIC communication
int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
	unsigned char value = 0;
	unsigned char recode = 0;

	FTS_TEST_DBG("");
	
	fts_i2c_read_test = fpI2C_Read;
	if(NULL == fts_i2c_read_test)
	{
		FTS_TEST_DBG("[focal] %s fts_i2c_read_test == NULL ",  __func__);
	}
	
	//debug start
	recode = ReadReg(0xa6, &value);
	if(recode != ERROR_CODE_OK)
	{
		FTS_TEST_DBG("[focal] ReadReg Error, code: %d ",  recode);	
	}
	else
	{
		FTS_TEST_DBG("[focal] ReadReg successed, Addr: 0xa6, value: 0x%02x ",  value);	
	}		
	//debug end
	
	return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
	FTS_TEST_DBG("");
	
	fts_i2c_write_test = fpI2C_Write;
	if(NULL == fts_i2c_write_test)
	{
		FTS_TEST_DBG("[focal] fts_i2c_read_test == NULL ");
	}	
	return 0;
}

/************************************************************************
* Name: set_param_data
* Brief:  load Config. Set IC series, init test items, init basic threshold, int detailThreshold, and set order of test items
* Input: TestParamData, from ini file.
* Output: none
* Return: 0. No sense, just according to the old format.
***********************************************************************/
int set_param_data(char * TestParamData)
{
	int ret = 0;
	
	FTS_TEST_DBG("Enter  set_param_data.");
	g_testparamstring = TestParamData;//get param of ini file
	ret = ini_get_key_data(g_testparamstring);//get param to struct
	if(ret < 0)
	{
		FTS_TEST_DBG("ini_get_key_data error.");
		return ret;
	}
	
	//从配置读取所选芯片类?
	//Set g_ScreenSetParam.iSelectedIC
	OnInit_InterfaceCfg(g_testparamstring);

	/*Get IC Name*/
	fts_ic_table_get_ic_name_from_ic_code(g_ScreenSetParam.iSelectedIC, g_strIcName);

	if(IC_FT8716>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT8716_TestItem(g_testparamstring);
		OnInit_FT8716_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);//测试项详细配置 
		SetTestItem_FT8716();
	}
	FTS_TEST_DBG("end of set_param_data.");
	return 0;
}

/************************************************************************
* Name: start_test_tp
* Brief:  Test entry. Select test items based on IC series
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/

boolean start_test_tp(void) 
{
	boolean bTestResult = false;
	
	FTS_TEST_DBG("[focal] %s start ",  __func__);
	FTS_TEST_DBG("IC_%s Test",  g_strIcName);
	
	switch(g_ScreenSetParam.iSelectedIC>>4)
		{
		case IC_FT8716>>4:
			#if CTP_ESD_PROTECT
			fts_esd_protection_suspend();
			fts_esd_protection_notice();
			#endif
			
			bTestResult = FT8716_StartTest();

			#if CTP_ESD_PROTECT
			fts_esd_protection_resume();
			#endif

			break;			
		default:
			FTS_TEST_DBG("[focal]  Error IC, IC Name: %s, IC Code:  %d",  g_strIcName, g_ScreenSetParam.iSelectedIC);
			break;
		}

	EnterWork();

	return bTestResult;
}
/************************************************************************
* Name: get_test_data
* Brief:  Get test data based on IC series
* Input: none
* Output: pTestData, External application for memory, buff size >= 1024*8
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int get_test_data(char *pTestData)
{
	int iLen = 0;
	FTS_TEST_DBG("[focal] %s start ",  __func__);	
	switch(g_ScreenSetParam.iSelectedIC>>4)
		{
		case IC_FT8716>>4:
			iLen = FT8716_get_test_data(pTestData);
			break;		

		default:
			FTS_TEST_DBG("[focal]  Error IC, IC Name: %s, IC Code:  %d",  g_strIcName, g_ScreenSetParam.iSelectedIC);
			break;
		}


	return iLen;	
}

int focaltech_test_main_init(void)
{
	int ret = 0;
	
	FTS_TEST_DBG("[focal] %s ",  FTS_DRIVER_LIB_INFO);	//show lib version
	
	/*申请内存，存储测试结果*/
	g_pStoreAllData = NULL;	
	if(NULL == g_pStoreAllData)
		g_pStoreAllData = kmalloc(FTS_TEST_STORE_DATA_SIZE, GFP_ATOMIC);
	if(NULL == g_pStoreAllData)
		return -1;
	
	/*申请内存，分配给详细阈值的结构体*/
	ret = malloc_struct_DetailThreshold();
	if(ret < 0)
		return ret;

	return 0;
}
/************************************************************************
* Name: free_test_param_data
* Brief:  release printer memory
* Input: none
* Output: none
* Return: none. 
***********************************************************************/
int focaltech_test_main_exit(void)
{

	FTS_TEST_DBG("[focal] release memory -start.");
	
	/*释放配置参数文件内容*/
	if(g_testparamstring)
		kfree(g_testparamstring);
	g_testparamstring = NULL;

	/*释放存储测试结果的内存*/
	if(g_pStoreAllData)
		kfree(g_pStoreAllData);
	
	/*释放详细阈值结构体的内存*/
	free_struct_DetailThreshold();	

	/*释放详细阈值结构体的内存*/
	release_key_data();//release memory of key data for ini file
		FTS_TEST_DBG("[focal] release memory -end.");
	return 0;
}

