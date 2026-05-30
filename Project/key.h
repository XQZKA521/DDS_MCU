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

/* 矩阵键盘统一键值定义 */
#define KEY_DIGIT_0   0
#define KEY_DIGIT_1   1
#define KEY_DIGIT_2   2
#define KEY_DIGIT_3   3
#define KEY_DIGIT_4   4
#define KEY_DIGIT_5   5
#define KEY_DIGIT_6   6
#define KEY_DIGIT_7   7
#define KEY_DIGIT_8   8
#define KEY_DIGIT_9   9
#define KEY_PLUS      11
#define KEY_MINUS     12
#define KEY_CHENG     13
#define KEY_CHU       14
#define KEY_NONE      20
#define KEY_EQUAL     30
#define KEY_NEGATE    40

/* 获取矩阵键盘键值 */
int getKeyValue(void);

#endif
