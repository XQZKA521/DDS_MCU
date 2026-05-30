/**
 * @file         key.h
 * @details      矩阵键盘驱动头文件，声明 key.c 中开放的函数
 * @author       zjs
 */
#ifndef _KEY_H_
#define _KEY_H_
/*
===========================
头文件包含
===========================
*/
#include "ti_msp_dl_config.h"
/*
===========================
函数声明
===========================
*/

/* 获取矩阵键盘键值 */
int getKeyValue(void);

#endif