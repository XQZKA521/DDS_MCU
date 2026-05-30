#include "ti_msp_dl_config.h"
#include "oled.h"
#include "key.h"
#include "delay.h"

#include <stdint.h>
#include <string.h>

/* ----- DDS 寄存器地址定义 ----- */
#define DDS_ADDR_FREQ_L        0x01
#define DDS_ADDR_FREQ_H        0x02
#define DDS_ADDR_MOD_TYPE      0x03
#define DDS_ADDR_AM_DEPTH      0x04
#define DDS_ADDR_MOD_FREQ_L    0x05
#define DDS_ADDR_MOD_FREQ_H    0x06

/* ----- DDS 调制类型 ----- */
#define DDS_MOD_CW             0x00
#define DDS_MOD_AM             0x01
#define DDS_MOD_FM             0x02

/* ----- AM 调制度寄存器换算 ----- */
#define DDS_AM_DEPTH_FULL_SCALE 32768U

/* ----- 频率范围约束 ----- */
#define DDS_CARRIER_FREQ_MIN_HZ 1UL
#define DDS_CARRIER_FREQ_MAX_HZ 10000000UL
#define DDS_MOD_FREQ_MIN_HZ    1UL
#define DDS_MOD_FREQ_MAX_HZ    1000000UL

/* ----- 串口波特率（默认 921600，匹配 FPGA） ----- */
#define UART_DEFAULT_BAUD      921600UL

/* ----- 串口接收缓冲区（预留，暂未使用） ----- */
#define UART_RX_BUF_SIZE       64U

/* ----- 输入模式 ----- */
#define MODE_IDLE              0U
#define MODE_FREQ              1U
#define MODE_MOD_FREQ          2U

static volatile uint8_t  gUartRxBuffer[UART_RX_BUF_SIZE];
static volatile uint16_t gUartRxIndex = 0U;

static uint32_t gCarrierFreqHz    = 2000UL;
static uint32_t gModFreqHz        = 1000UL;
static uint8_t  gModType          = DDS_MOD_CW;
static uint8_t  gAmDepthPercent   = 50U;
static uint8_t  gInputMode        = MODE_IDLE;
static uint32_t gInputValue       = 0UL;

static void UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, data);
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

/* 帧格式:
 * [数据高8位][数据低8位][控制地址][0xFF][0xFF][0xFF]
 */
static void DDS_SendFrame(uint16_t data, uint8_t addr)
{
    UART_SendByte((uint8_t)(data >> 8));
    UART_SendByte((uint8_t)(data & 0xFFU));
    UART_SendByte(addr);
    UART_SendByte(0xFFU);
    UART_SendByte(0xFFU);
    UART_SendByte(0xFFU);
}

static uint16_t DDS_AmDepthPercentToReg(uint8_t depth_percent)
{
    return (uint16_t)((((uint32_t) depth_percent) * DDS_AM_DEPTH_FULL_SCALE + 50U) / 100U);
}

static void DDS_SetCarrierFrequency(uint32_t freq_hz)
{
    DDS_SendFrame((uint16_t)(freq_hz & 0xFFFFU), DDS_ADDR_FREQ_L);
    DDS_SendFrame((uint16_t)((freq_hz >> 16) & 0xFFFFU), DDS_ADDR_FREQ_H);
    gCarrierFreqHz = freq_hz;
}

static void DDS_SetModFrequency(uint32_t freq_hz)
{
    DDS_SendFrame((uint16_t)(freq_hz & 0xFFFFU), DDS_ADDR_MOD_FREQ_L);
    DDS_SendFrame((uint16_t)((freq_hz >> 16) & 0xFFFFU), DDS_ADDR_MOD_FREQ_H);
    gModFreqHz = freq_hz;
}

static void DDS_SetModType(uint8_t mod_type)
{
    DDS_SendFrame((uint16_t) mod_type, DDS_ADDR_MOD_TYPE);
    gModType = mod_type;
}

static void DDS_SetAmDepthPercent(uint8_t depth_percent)
{
    DDS_SendFrame(DDS_AmDepthPercentToReg(depth_percent), DDS_ADDR_AM_DEPTH);
    gAmDepthPercent = depth_percent;
}

static void DDS_ApplyStartupConfig(void)
{
    DDS_SetCarrierFrequency(gCarrierFreqHz);
    DDS_SetModFrequency(gModFreqHz);
    DDS_SetAmDepthPercent(gAmDepthPercent);
    DDS_SetModType(gModType);
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint8_t ch = (uint8_t) DL_UART_Main_receiveData(UART_0_INST);

            if (gUartRxIndex < (UART_RX_BUF_SIZE - 1U)) {
                gUartRxBuffer[gUartRxIndex++] = ch;
                gUartRxBuffer[gUartRxIndex] = '\0';
            } else {
                gUartRxIndex = 0U;
                memset((void *) gUartRxBuffer, 0, UART_RX_BUF_SIZE);
            }
            break;
        }
        default:
            break;
    }
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint8_t ch = (uint8_t) DL_UART_Main_receiveData(UART_1_INST);

            if (gUartRxIndex < (UART_RX_BUF_SIZE - 1U)) {
                gUartRxBuffer[gUartRxIndex++] = ch;
                gUartRxBuffer[gUartRxIndex] = '\0';
            } else {
                gUartRxIndex = 0U;
                memset((void *) gUartRxBuffer, 0, UART_RX_BUF_SIZE);
            }
            break;
        }
        default:
            break;
    }
}

/* 公式: BaudRate = CLK / (16 * (IBRD + FBRD/64)) */
static void UART_SetBaudRate(uint32_t baud)
{
    uint32_t clk0 = UART_0_INST_FREQUENCY;
    uint32_t clk1 = UART_1_INST_FREQUENCY;
    uint32_t ibrd;
    uint32_t fbrd;

    ibrd = clk0 / (16U * baud);
    fbrd = (((clk0 % (16U * baud)) * 64U) + (8U * baud)) / (16U * baud);
    DL_UART_Main_disable(UART_0_INST);
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, ibrd, fbrd);
    DL_UART_Main_enable(UART_0_INST);

    ibrd = clk1 / (16U * baud);
    fbrd = (((clk1 % (16U * baud)) * 64U) + (8U * baud)) / (16U * baud);
    DL_UART_Main_disable(UART_1_INST);
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, ibrd, fbrd);
    DL_UART_Main_enable(UART_1_INST);

}

static void UART_Init(void)
{
    UART_SetBaudRate(UART_DEFAULT_BAUD);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

static uint8_t IsCarrierFrequencyValid(uint32_t freq_hz)
{
    return (freq_hz >= DDS_CARRIER_FREQ_MIN_HZ) && (freq_hz <= DDS_CARRIER_FREQ_MAX_HZ);
}

static uint8_t IsModFrequencyValid(uint32_t freq_hz)
{
    return (freq_hz >= DDS_MOD_FREQ_MIN_HZ) && (freq_hz <= DDS_MOD_FREQ_MAX_HZ);
}

static void EnterInputMode(uint8_t mode)
{
    gInputMode = mode;
    gInputValue = 0UL;
}

static void StepAmDepth(int8_t step)
{
    int16_t next_depth = (int16_t) gAmDepthPercent + step;

    if (next_depth < 10) {
        next_depth = 10;
    } else if (next_depth > 100) {
        next_depth = 100;
    }

    DDS_SetAmDepthPercent((uint8_t) next_depth);
}

static void ToggleAmMode(void)
{
    if (gModType == DDS_MOD_AM) {
        DDS_SetModType(DDS_MOD_CW);
    } else {
        DDS_SetModType(DDS_MOD_AM);
    }
}

static void ConfirmInputValue(void)
{
    if (gInputMode == MODE_FREQ) {
        if (IsCarrierFrequencyValid(gInputValue)) {
            DDS_SetCarrierFrequency(gInputValue);
            gInputMode = MODE_IDLE;
            gInputValue = 0UL;
        }
    } else if (gInputMode == MODE_MOD_FREQ) {
        if (IsModFrequencyValid(gInputValue)) {
            DDS_SetModFrequency(gInputValue);
            gInputMode = MODE_IDLE;
            gInputValue = 0UL;
        }
    }
}

static void Display_IdleScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1:Fc 2:Fm 3:AM");

    OLED_ShowString(0, 2, (u8 *) "Fc:");
    OLED_ShowNum(24, 2, (u32) gCarrierFreqHz, 8, 16);
    OLED_ShowString(96, 2, (u8 *) "Hz");

    OLED_ShowString(0, 4, (u8 *) "Fm:");
    OLED_ShowNum(24, 4, (u32) gModFreqHz, 7, 16);
    OLED_ShowString(88, 4, (u8 *) "Hz");

    if (gModType == DDS_MOD_AM) {
        OLED_ShowString(0, 6, (u8 *) "AM:ON ");
    } else {
        OLED_ShowString(0, 6, (u8 *) "AM:CW ");
    }
    OLED_ShowNum(48, 6, (u32) gAmDepthPercent, 3, 16);
    OLED_ShowString(72, 6, (u8 *) "% +/-");
}

static void Display_InputScreen(void)
{
    if (gInputMode == MODE_FREQ) {
        OLED_ShowString(0, 0, (u8 *) "Input Fc (Hz)  ");
        OLED_ShowString(0, 4, (u8 *) "1Hz-10MHz      ");
    } else {
        OLED_ShowString(0, 0, (u8 *) "Input Fm (Hz)  ");
        OLED_ShowString(0, 4, (u8 *) "Range:1-1MHz   ");
    }

    OLED_ShowString(0, 2, (u8 *) ">");
    OLED_ShowNum(8, 2, (u32) gInputValue, 10, 16);
    OLED_ShowString(0, 6, (u8 *) "=OK /CLR *ESC  ");
}

static void Display_Status(void)
{
    OLED_Clear();

    if (gInputMode == MODE_IDLE) {
        Display_IdleScreen();
    } else {
        Display_InputScreen();
    }
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    UART_Init();

    OLED_ShowString(20, 2, (u8 *) "DDS Starting...");
    delay_ms(1000);

    DDS_ApplyStartupConfig();
    Display_Status();

    while (1) {
        int key = getKeyValue();

        if (key == KEY_NONE) {
            continue;
        }

        delay_ms(10);
        if (getKeyValue() != key) {
            continue;
        }

        if ((key >= KEY_DIGIT_0) && (key <= KEY_DIGIT_9)) {
            if (gInputMode == MODE_IDLE) {
                if (key == KEY_DIGIT_1) {
                    EnterInputMode(MODE_FREQ);
                } else if (key == KEY_DIGIT_2) {
                    EnterInputMode(MODE_MOD_FREQ);
                } else if (key == KEY_DIGIT_3) {
                    ToggleAmMode();
                }
            } else if (gInputValue <= 429496728UL) {
                gInputValue = gInputValue * 10UL + (uint32_t) key;
            }
        } else if (key == KEY_EQUAL) {
            ConfirmInputValue();
        } else if ((key == KEY_CHU) && (gInputMode != MODE_IDLE)) {
            gInputValue = 0UL;
        } else if ((key == KEY_CHENG) && (gInputMode != MODE_IDLE)) {
            gInputMode = MODE_IDLE;
            gInputValue = 0UL;
        } else if ((key == KEY_PLUS) && (gInputMode == MODE_IDLE)) {
            StepAmDepth(10);
        } else if ((key == KEY_MINUS) && (gInputMode == MODE_IDLE)) {
            StepAmDepth(-10);
        }

        Display_Status();

        while (getKeyValue() != KEY_NONE) {
        }
        delay_ms(10);
    }
}
