#include "key.h"

#define KEY_PLUS  11
#define KEY_MINUS 12
#define KEY_cheng  13
#define KEY_chu 14
#define KEY_equal 30
#define KEY_negate 40
#define KEY_point 

int h_arr[4] = {KEY_H1_PIN, KEY_H2_PIN, KEY_H3_PIN, KEY_H4_PIN};
int v_arr[4] = {KEY_V1_PIN, KEY_V2_PIN, KEY_V3_PIN, KEY_V4_PIN};

int getKeyValue(void)
{
    int h_arr[4] = {KEY_H1_PIN, KEY_H2_PIN, KEY_H3_PIN, KEY_H4_PIN};
    int v_arr[4] = {KEY_V1_PIN, KEY_V2_PIN, KEY_V3_PIN, KEY_V4_PIN};
    int i, j = 0;
    int key_value = 20;
    for (i = 0; i < 4; i++)
    {
        delay_cycles(1000);

        DL_GPIO_setPins(KEY_PORT, h_arr[i]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 1) % 4]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 2) % 4]);
        DL_GPIO_clearPins(KEY_PORT, h_arr[(i + 3) % 4]);

        delay_cycles(100000);

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
						
		
if(key_value == 1){key_value = 1;}
else if(key_value == 5){key_value = 2;}
else if(key_value == 9){key_value = 3;}		
else if(key_value == 2){key_value = 4;}
else if(key_value == 6){key_value = 5;}
else if(key_value == 10){key_value = 6;}
else if(key_value == 3){key_value = 7;}
else if(key_value == 7){key_value = 8;}
else if(key_value == 11){key_value = 9;}		
else if(key_value == 8){key_value = 0;}

else if(key_value == 13){key_value = KEY_PLUS;}
else if(key_value == 14){key_value = KEY_MINUS;}
else if(key_value == 15){key_value = KEY_cheng;}
else if(key_value == 16){key_value = KEY_chu;}
else if(key_value == 12){key_value = KEY_equal;}		
else if(key_value == 4){key_value = KEY_negate;}


    return key_value; // 没有按下，返回0
}