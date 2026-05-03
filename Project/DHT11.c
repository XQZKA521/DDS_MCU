#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "DHT11.h"

//该函数为驱动温湿度传感器DHT11的驱动

/*
typedef struct {
    uint8_t humidity;      // 湿度整数部分
    uint8_t temperature;   // 温度整数部分
    uint8_t checksum_ok;   // 1=校验成功, 0=失败
    uint8_t timeout;       // 1=通信超时, 0=正常
} DHT11_Data_t;
*/
// ===== 基础延时 =====
// 你当前工程主频接近 80MHz，这里按 80MHz 粗略延时
#define DHT11_START_LOW_MS          20U
#define DHT11_RELEASE_US            40U
#define DHT11_MODE_SETTLE_US        8U
#define DHT11_RESPONSE_TIMEOUT_US   1000U
#define DHT11_BIT_LOW_TIMEOUT_US    100U
#define DHT11_BIT_HIGH_TIMEOUT_US   100U
#define DHT11_BIT_END_TIMEOUT_US    120U
#define DHT11_BIT_SAMPLE_US         40U
#define DHT11_RETRY_COUNT           3U
#define DHT11_RETRY_GAP_MS          2U

static void DHT11_DelayUs(uint32_t us)
{
    delay_cycles(us * 80);
}

static void DHT11_DelayMs(uint32_t ms)
{
    while (ms--) {
        delay_cycles(80000);
    }
}

// ===== 动态切换 DHT11 引脚为输出/输入 =====
static void DHT11_PinMode_Output(void)
{
    DL_GPIO_initDigitalOutput(DHT11_DHT1_IOMUX);
    DL_GPIO_clearPins(DHT11_PORT, DHT11_DHT1_PIN);
    DL_GPIO_enableOutput(DHT11_PORT, DHT11_DHT1_PIN);
    DHT11_DelayUs(DHT11_MODE_SETTLE_US);
}

static void DHT11_PinMode_Input(void)
{
    DL_GPIO_disableOutput(DHT11_PORT, DHT11_DHT1_PIN);
    DL_GPIO_initDigitalInputFeatures(
        DHT11_DHT1_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE
    );
    DHT11_DelayUs(DHT11_MODE_SETTLE_US);
}

static void DHT11_WriteHigh(void)
{
    DL_GPIO_setPins(DHT11_PORT, DHT11_DHT1_PIN);
}

static void DHT11_WriteLow(void)
{
    DL_GPIO_clearPins(DHT11_PORT, DHT11_DHT1_PIN);
}

static uint8_t DHT11_ReadPin(void)
{
    return (DL_GPIO_readPins(DHT11_PORT, DHT11_DHT1_PIN) ? 1 : 0);
}

// 等待引脚变成指定电平，带超时，返回 1 成功，0 超时
static uint8_t DHT11_WaitLevel(uint8_t level, uint32_t timeout_us)
{
    while (timeout_us--) {
        if (DHT11_ReadPin() == level) {
            return 1;
        }
        DHT11_DelayUs(1);
    }
    return 0;
}

// 读取 1bit
static uint8_t DHT11_ReadBit(uint8_t *bit)
{
    // 每一位开始：先有约 50us 低电平
    if (!DHT11_WaitLevel(0, DHT11_BIT_LOW_TIMEOUT_US)) return 0;
    if (!DHT11_WaitLevel(1, DHT11_BIT_HIGH_TIMEOUT_US)) return 0;

    // 拉高后等待一段时间采样：
    // 高电平持续约 26~28us -> 0
    // 高电平持续约 70us    -> 1
    DHT11_DelayUs(DHT11_BIT_SAMPLE_US);
    *bit = DHT11_ReadPin();

    // 等待这位结束，回到低电平
    if (!DHT11_WaitLevel(0, DHT11_BIT_END_TIMEOUT_US)) return 0;

    return 1;
}

// 读取 1byte
static uint8_t DHT11_ReadByte(uint8_t *data)
{
    uint8_t i, bitVal;
    uint8_t value = 0;

    for (i = 0; i < 8; i++) {
        value <<= 1;
        if (!DHT11_ReadBit(&bitVal)) {
            return 0;
        }
        value |= bitVal;
    }

    *data = value;
    return 1;
}

static DHT11_Data_t DHT11_ReadOnce(void)
{
    DHT11_Data_t result = {0, 0, 0, 0, 0};
    uint8_t hum_int, hum_dec, temp_int, temp_dec, checksum;

    __disable_irq();

    // 1. 起始信号：主机拉低 >=18ms
    DHT11_PinMode_Output();
    DHT11_WriteLow();
    DHT11_DelayMs(DHT11_START_LOW_MS);

    // 2. 主机拉高 20~40us 后切输入
    DHT11_WriteHigh();
    DHT11_DelayUs(DHT11_RELEASE_US);
    DHT11_PinMode_Input();

    // 3. 等待 DHT11 响应
    result.step = 1;
    if (!DHT11_WaitLevel(0, DHT11_RESPONSE_TIMEOUT_US)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 2;
    if (!DHT11_WaitLevel(1, DHT11_RESPONSE_TIMEOUT_US)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 3;
    if (!DHT11_WaitLevel(0, DHT11_RESPONSE_TIMEOUT_US)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    // 4. 读 5 字节
    result.step = 4;
    if (!DHT11_ReadByte(&hum_int)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 5;
    if (!DHT11_ReadByte(&hum_dec)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 6;
    if (!DHT11_ReadByte(&temp_int)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 7;
    if (!DHT11_ReadByte(&temp_dec)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    result.step = 8;
    if (!DHT11_ReadByte(&checksum)) {
        result.timeout = 1;
        __enable_irq();
        return result;
    }

    __enable_irq();

    // 5. 校验
    result.step = 9;
    if (((uint8_t)(hum_int + hum_dec + temp_int + temp_dec)) == checksum) {
        result.humidity = hum_int;
        result.temperature = temp_int;
        result.checksum_ok = 1;
    } else {
        result.checksum_ok = 0;
    }

    return result;
}

DHT11_Data_t DHT11_Read(void)
{
    DHT11_Data_t result = {0, 0, 0, 0, 0};

    for (uint8_t attempt = 0; attempt < DHT11_RETRY_COUNT; attempt++) {
        result = DHT11_ReadOnce();
        if (!result.timeout) {
            return result;
        }
        DHT11_DelayMs(DHT11_RETRY_GAP_MS);
    }

    return result;
}
