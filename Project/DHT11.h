#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>

// DHT11 读取结果
typedef struct {
    uint8_t humidity;      // 湿度整数部分
    uint8_t temperature;   // 温度整数部分
    uint8_t checksum_ok;   // 1=校验成功
    uint8_t timeout;       // 1=通信超时
		uint8_t step;  
} DHT11_Data_t;

// 主读取函数
DHT11_Data_t DHT11_Read(void);

// 可选：打包返回版本
uint16_t DHT11_ReadPacked(void);

#endif