/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Global.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: global function for test
*
************************************************************************/
#ifndef _FOCALTECH_GLOBAL_H
#define _FOCALTECH_GLOBAL_H

#include "focaltech_ic_table.h"

extern int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
extern int fts_test_i2c_write(unsigned char *writebuf, int writelen);
extern int fts_test_get_ini_size(char *config_name);
extern int fts_test_read_ini_data(char *config_name, char *config_buf);
extern int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
extern int fts_test_get_testparam_from_ini(char *config_name);
extern int fts_test_entry(char *ini_file_name);
#endif
