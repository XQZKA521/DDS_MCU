#include "key.h"

/* 特殊键值定义 */
#define KEY_PLUS   11   /* 加号键 */
#define KEY_MINUS  12   /* 减号键 */
#define KEY_cheng  13   /* 乘号键 */
#define KEY_chu    14   /* 除号键 */
#define KEY_equal  30   /* 等号键 */
#define KEY_negate 40   /* 取反键 */
#define KEY_point       /* 小数点键 */

/* 行引脚和列引脚数组 */
int h_arr[4] = {KEY_H1_PIN, KEY_H2_PIN, KEY_H3_PIN, KEY_H4_PIN};
int v_arr[4] = {KEY_V1_PIN, KEY_V2_PIN, KEY_V3_PIN, KEY_V4_PIN};

/* 获取矩阵键盘键值
 * 返回值: 0~9 = 数字键, 11~16 = 功能键, 20 = 无按键按下
 */
int getKeyValue(void)
{
    int h_arr[4] = {KEY_H1_PIN, KEY_H2_PIN, KEY_H3_PIN, KEY_H4_PIN};
    int v_arr[4] = {KEY_V1_PIN, KEY_V2_PIN, KEY_V3_PIN, KEY_V4_PIN};
    int i, j = 0;
    int key_value = 20;  /* 默认值 20 表示无按键按下 */

    /* 逐行扫描 */
    for (i = 0; i < 4; i++)
    {
        delay_cycles(1000);

        /* 设置当前行为高电平，其余行为低电平 */
        DL_GPIO_setPins(KEY_PORT, h_arr[i]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 1) % 4]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 2) % 4]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 3) % 4]);

        delay_cycles(100000);

        /* 读取列状态 */
        for (j = 0; j < 4; j++)
        {
            if (DL_GPIO_readPins(KEY_PORT, v_arr[j]) != 0)
            {
                key_value = j * 4 + i + 1;
            }
            delay_cycles(1000);
        }

        delay_cycles(1000);
    }

    /* 键值映射：将矩阵位置转换为实际键值 */
    if(key_value == 1)     {key_value = 1;}       /* 数字 1 */
    else if(key_value == 5)  {key_value = 2;}     /* 数字 2 */
    else if(key_value == 9)  {key_value = 3;}     /* 数字 3 */
    else if(key_value == 2)  {key_value = 4;}     /* 数字 4 */
    else if(key_value == 6)  {key_value = 5;}     /* 数字 5 */
    else if(key_value == 10) {key_value = 6;}     /* 数字 6 */
    else if(key_value == 3)  {key_value = 7;}     /* 数字 7 */
    else if(key_value == 7)  {key_value = 8;}     /* 数字 8 */
    else if(key_value == 11) {key_value = 9;}     /* 数字 9 */
    else if(key_value == 8)  {key_value = 0;}     /* 数字 0 */

    else if(key_value == 13) {key_value = KEY_PLUS;}   /* 加号 */
    else if(key_value == 14) {key_value = KEY_MINUS;}  /* 减号 */
    else if(key_value == 15) {key_value = KEY_cheng;}  /* 乘号 */
    else if(key_value == 16) {key_value = KEY_chu;}    /* 除号 */
    else if(key_value == 12) {key_value = KEY_equal;}  /* 等号 */
    else if(key_value == 4)  {key_value = KEY_negate;} /* 取反 */


    return key_value; // 无按键按下返回 20
}
