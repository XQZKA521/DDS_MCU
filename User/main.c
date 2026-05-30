
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "key.h"
#include "delay.h"

#include <stdint.h>
#include <string.h>

/* ----- DDS 寄存器地址定义 ----- */
#define DDS_ADDR_FREQ_L     0x01    /* 频率低16位寄存器地址 */
#define DDS_ADDR_FREQ_H     0x02    /* 频率高16位寄存器地址 */
#define DDS_ADDR_MOD_TYPE   0x03    /* 调制类型寄存器地址 */
#define DDS_ADDR_AMP        0x04    /* 幅度控制寄存器地址 */

/* ----- DDS 调制类型 ----- */
#define DDS_MOD_CW          0x00    /* 等幅波（纯正弦） */
#define DDS_MOD_AM          0x01    /* 调幅 */
#define DDS_MOD_FM          0x02    /* 调频 */

/* ----- DDS 幅度默认值（0x8000 = 32768 = 满幅度 1.0 倍） ----- */
#define DDS_AMP_DEFAULT     0x8000

/* ----- 串口波特率（默认 921600，匹配 FPGA） ----- */
#define UART_DEFAULT_BAUD   921600
static volatile uint32_t gUartBaudRate = UART_DEFAULT_BAUD;

/* ----- 串口接收缓冲区（预留，暂未使用） ----- */
#define UART_RX_BUF_SIZE    64
static volatile uint8_t  gUartRxBuffer[UART_RX_BUF_SIZE];
static volatile uint16_t gUartRxIndex = 0;

/* ----- 当前频率（每次确认后更新） ----- */
static volatile uint32_t gCurrentFreqHz = 2000;

/* ----- 输入状态 -----
 *  MODE_IDLE    = 待机，等待按 1 或 2 进入输入模式
 *  MODE_FREQ    = 正在输入频率
 *  MODE_BAUD    = 正在输入波特率
 */
#define MODE_IDLE   0
#define MODE_FREQ   1
#define MODE_BAUD   2
static volatile uint8_t  gInputMode   = MODE_IDLE;
static volatile uint32_t gInputValue  = 0;       /* 当前正在输入的数字 */

/* ========================================================================
 *  底层串口发送函数
 * ======================================================================== */

/* 底层串口发送（两路同步发送） */
static void UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, data);
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

/* ========================================================================
 *  DDS 帧发送函数
 *
 *  帧格式（6 字节）：
 *    [数据高8位][数据低8位][控制地址][0xFF][0xFF][0xFF]
 * ======================================================================== */

void DDS_SendFrame(uint16_t data, uint8_t addr)
{
    UART_SendByte((uint8_t)(data >> 8));    /* 字节1：数据高8位 */
    UART_SendByte((uint8_t)(data & 0xFF));  /* 字节2：数据低8位 */
    UART_SendByte(addr);                     /* 字节3：控制地址   */
    UART_SendByte(0xFF);                     /* 字节4：帧尾       */
    UART_SendByte(0xFF);                     /* 字节5：帧尾       */
    UART_SendByte(0xFF);                     /* 字节6：帧尾       */
}

void DDS_SetFrequency(uint32_t freq_hz)
{
    uint16_t freq_low  = (uint16_t)(freq_hz & 0xFFFF);
    uint16_t freq_high = (uint16_t)((freq_hz >> 16) & 0xFFFF);

    DDS_SendFrame(freq_low,       DDS_ADDR_FREQ_L);   /* 帧1：频率低16位 */
    DDS_SendFrame(freq_high,      DDS_ADDR_FREQ_H);   /* 帧2：频率高16位 */
    DDS_SendFrame(DDS_AMP_DEFAULT, DDS_ADDR_AMP);      /* 帧3：满幅度      */

    gCurrentFreqHz = freq_hz;
}

void DDS_SetModType(uint8_t mod_type)
{
    DDS_SendFrame((uint16_t)mod_type, DDS_ADDR_MOD_TYPE);
}

/* ========================================================================
 *  串口接收中断服务函数（预留功能，仅读取数据以清除中断标志）
 * ======================================================================== */

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint8_t ch = (uint8_t)DL_UART_Main_receiveData(UART_0_INST);
            if (gUartRxIndex < (UART_RX_BUF_SIZE - 1)) {
                gUartRxBuffer[gUartRxIndex++] = ch;
                gUartRxBuffer[gUartRxIndex] = '\0';
            } else {
                gUartRxIndex = 0;
                memset((void *)gUartRxBuffer, 0, UART_RX_BUF_SIZE);
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
            uint8_t ch = (uint8_t)DL_UART_Main_receiveData(UART_1_INST);
            if (gUartRxIndex < (UART_RX_BUF_SIZE - 1)) {
                gUartRxBuffer[gUartRxIndex++] = ch;
                gUartRxBuffer[gUartRxIndex] = '\0';
            } else {
                gUartRxIndex = 0;
                memset((void *)gUartRxBuffer, 0, UART_RX_BUF_SIZE);
            }
            break;
        }
        default:
            break;
    }
}

/* 设置两路串口统一波特率（运行时可调）
 * 公式：BaudRate = CLK / (16 * (IBRD + FBRD/64))
 */
static void UART_SetBaudRate(uint32_t baud)
{
    uint32_t ibrd, fbrd;

    /* UART_0（接电脑） */
    uint32_t clk0 = UART_0_INST_FREQUENCY;
    ibrd = clk0 / (16 * baud);
    fbrd = (((clk0 % (16 * baud)) * 64) + (8 * baud)) / (16 * baud);
    DL_UART_Main_disable(UART_0_INST);
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, ibrd, fbrd);
    DL_UART_Main_enable(UART_0_INST);

    /* UART_1（接 FPGA） */
    uint32_t clk1 = UART_1_INST_FREQUENCY;
    ibrd = clk1 / (16 * baud);
    fbrd = (((clk1 % (16 * baud)) * 64) + (8 * baud)) / (16 * baud);
    DL_UART_Main_disable(UART_1_INST);
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, ibrd, fbrd);
    DL_UART_Main_enable(UART_1_INST);

    gUartBaudRate = baud;
}

static void UART_Init(void)
{
    UART_SetBaudRate(UART_DEFAULT_BAUD);

    /* UART_0 中断配置（接电脑，可收数据） */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    /* UART_1 中断配置（接 FPGA） */
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/* ========================================================================
 *  OLED 显示函数
 * ======================================================================== */

static void Display_Status(void)
{
    /* 第0行：当前模式提示 */
    switch (gInputMode) {
        case MODE_IDLE:
            OLED_ShowString(0, 0, (u8 *)"1:Freq 2:Baud  ");
            break;
        case MODE_FREQ:
            OLED_ShowString(0, 0, (u8 *)"Input Freq:    ");
            break;
        case MODE_BAUD:
            OLED_ShowString(0, 0, (u8 *)"Input Baud:    ");
            break;
    }

    /* 第2行：输入中的数字 或 当前频率 */
    if (gInputMode != MODE_IDLE) {
        /* 输入模式：显示正在输入的数字 */
        OLED_ShowString(0, 2, (u8 *)">");
        OLED_ShowNum(8, 2, (u32)gInputValue, 10, 16);
    } else {
        /* 待机模式：显示当前生效的频率 */
        OLED_ShowString(0, 2, (u8 *)"Freq:");
        OLED_ShowNum(40, 2, (u32)gCurrentFreqHz, 10, 16);
    }

    /* 第4行：Hz（待机时显示） */
    if (gInputMode == MODE_IDLE) {
        OLED_ShowString(0, 4, (u8 *)"Hz             ");
    } else {
        OLED_ShowString(0, 4, (u8 *)"=OK  -=CLR     ");
    }

    /* 第6行：当前波特率 */
    OLED_ShowString(0, 6, (u8 *)"Baud:");
    OLED_ShowNum(40, 6, (u32)gUartBaudRate, 7, 16);
}

/* ========================================================================
 *  主函数
 * ======================================================================== */

int main(void)
{
    /* --- 硬件初始化 --- */
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    UART_Init();

    /* --- 开机画面 --- */
    OLED_ShowString(20, 2, (u8 *)"DDS Starting...");
    delay_ms(1000);
    OLED_Clear();

    /* --- 发送初始频率并显示 --- */
    DDS_SetFrequency(gCurrentFreqHz);
    Display_Status();

    /* --- 主循环 --- */
    while (1) {
        int key = getKeyValue();

        if (key != 20) {                        /* 20 = 无按键按下 */
            delay_ms(10);
            if (getKeyValue() == key) {         /* 消抖确认 */

                /* ---------- 数字键 0~9：拼接输入 ---------- */
                if (key >= 0 && key <= 9) {
                    /* 按键 1 在待机时进入频率模式，按键 2 进入波特率模式 */
                    if (gInputMode == MODE_IDLE) {
                        if (key == 1) {
                            gInputMode  = MODE_FREQ;
                            gInputValue = 0;
                        } else if (key == 2) {
                            gInputMode  = MODE_BAUD;
                            gInputValue = 0;
                        }
                        /* 其他数字键在待机模式下无效 */
                    } else {
                        /* 输入模式下，追加数字（防溢出：最大 10 位） */
                        if (gInputValue <= 999999999) {
                            gInputValue = gInputValue * 10 + (uint32_t)key;
                        }
                    }
                    OLED_Clear();
                }
                /* ---------- = 键（key12）：确认输入 ---------- */
                else if (key == 12) {
                    if (gInputMode == MODE_FREQ) {
                        DDS_SetFrequency(gInputValue);
                    } else if (gInputMode == MODE_BAUD) {
                        if (gInputValue >= 300 && gInputValue <= 921600) {
                            UART_SetBaudRate(gInputValue);
                        }
                    }
                    gInputMode  = MODE_IDLE;
                    gInputValue = 0;
                    OLED_Clear();
                }
                /* ---------- - 键（key14）：清除输入 ---------- */
                else if (key == 14) {
                    gInputValue = 0;
                    OLED_Clear();
                }

                Display_Status();

                /* 等待按键松开 */
                while (getKeyValue() != 20)
                    ;
                delay_ms(10);
            }
        }

        Display_Status();
    }
}
