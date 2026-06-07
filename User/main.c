/*
1 进正弦波模式，2 进 AM 模式，3 进 FM 模式，7 进 ASK 模式，8 进 PSK 模式

SIN 模式：4 设置载波频率
AM 模式：4 设置载波，5 设置调制频率，+/- 调 AM 深度
FM 模式：4 设置载波，5 设置调制频率，6 设置频偏，+ 选 10k 上限，- 选 5k 上限
ASK/PSK 模式：固定 100kHz 载波，10kbps PRBS 基带
取反键：全局输出幅度 10%-100%，每次增加 5%，超过上限回到下限

输入时：= 确认，/ 清零，* 退出输入

*/
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "key.h"
#include "delay.h"

#include <stdint.h>
#include <string.h>

/* ----- DDS register map ----- */
#define DDS_ADDR_FREQ_L         0x01
#define DDS_ADDR_FREQ_H         0x02
#define DDS_ADDR_MOD_TYPE       0x03
#define DDS_ADDR_AM_DEPTH       0x04
#define DDS_ADDR_MOD_FREQ_L     0x05
#define DDS_ADDR_MOD_FREQ_H     0x06
#define DDS_ADDR_FM_DEV         0x07
#define DDS_ADDR_OUTPUT_AMP     0x08

/* ----- DDS modulation types ----- */
#define DDS_MOD_CW              0x00
#define DDS_MOD_AM              0x01
#define DDS_MOD_FM              0x02
#define DDS_MOD_ASK             0x03
#define DDS_MOD_PSK             0x04

/* ----- AM depth register scaling ----- */
#define DDS_AM_DEPTH_FULL_SCALE 32768U

/* ----- Output amplitude scaling ----- */
#define DDS_OUTPUT_AMP_FULL_SCALE 32768U
#define DDS_OUTPUT_AMP_MIN_PERCENT 10U
#define DDS_OUTPUT_AMP_MAX_PERCENT 100U
#define DDS_OUTPUT_AMP_STEP_PERCENT 5U

/* ----- Frequency limits ----- */
#define DDS_CARRIER_FREQ_MIN_HZ    1UL
#define DDS_CARRIER_FREQ_MAX_HZ    10000000UL
#define DDS_FM_CARRIER_FREQ_MIN_HZ 100000UL
#define DDS_MOD_FREQ_MIN_HZ        1UL
#define DDS_MOD_FREQ_MAX_HZ        1000000UL

/* ----- FM deviation limits ----- */
#define DDS_FM_DEV_LIMIT_5K_HZ  5000U
#define DDS_FM_DEV_LIMIT_10K_HZ 10000U

/* ----- ASK/PSK fixed parameters ----- */
#define DDS_KEYING_CARRIER_HZ   100000UL
#define DDS_KEYING_BITRATE_BPS  10000UL

/* ----- UART baud rate (default 921600, matched with FPGA) ----- */
#define UART_DEFAULT_BAUD       921600UL

/* ----- UART RX buffer (reserved) ----- */
#define UART_RX_BUF_SIZE        64U

/* ----- UI mode ----- */
#define UI_MODE_SINE            0U
#define UI_MODE_AM              1U
#define UI_MODE_FM              2U
#define UI_MODE_ASK             3U
#define UI_MODE_PSK             4U

/* ----- Numeric input target ----- */
#define INPUT_NONE              0U
#define INPUT_SINE_FREQ         1U
#define INPUT_AM_CARRIER        2U
#define INPUT_AM_MOD_FREQ       3U
#define INPUT_FM_CARRIER        4U
#define INPUT_FM_MOD_FREQ       5U
#define INPUT_FM_DEV            6U

static volatile uint8_t  gUartRxBuffer[UART_RX_BUF_SIZE];
static volatile uint16_t gUartRxIndex = 0U;

static uint8_t  gUiMode          = UI_MODE_SINE;
static uint8_t  gInputMode       = INPUT_NONE;
static uint32_t gInputValue      = 0UL;

static uint32_t gSineCarrierHz   = 2000UL;
static uint32_t gAmCarrierHz     = 2000UL;
static uint32_t gAmModFreqHz     = 1000UL;
static uint8_t  gAmDepthPercent  = 50U;
static uint32_t gFmCarrierHz     = 100000UL;
static uint32_t gFmModFreqHz     = 1000UL;
static uint16_t gFmDevHz         = DDS_FM_DEV_LIMIT_5K_HZ;
static uint16_t gFmDevLimitHz    = DDS_FM_DEV_LIMIT_5K_HZ;
static uint8_t  gOutputAmpPercent = DDS_OUTPUT_AMP_MAX_PERCENT;

static void UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, data);
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

/* Frame format:
 * [data_hi][data_lo][addr][0xFF][0xFF][0xFF]
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

static uint16_t DDS_OutputAmpPercentToReg(uint8_t amp_percent)
{
    return (uint16_t)((((uint32_t) amp_percent) * DDS_OUTPUT_AMP_FULL_SCALE + 50U) / 100U);
}

static void DDS_SetCarrierFrequency(uint32_t freq_hz)
{
    DDS_SendFrame((uint16_t)(freq_hz & 0xFFFFU), DDS_ADDR_FREQ_L);
    DDS_SendFrame((uint16_t)((freq_hz >> 16) & 0xFFFFU), DDS_ADDR_FREQ_H);
}

static void DDS_SetModFrequency(uint32_t freq_hz)
{
    DDS_SendFrame((uint16_t)(freq_hz & 0xFFFFU), DDS_ADDR_MOD_FREQ_L);
    DDS_SendFrame((uint16_t)((freq_hz >> 16) & 0xFFFFU), DDS_ADDR_MOD_FREQ_H);
}

static void DDS_SetModType(uint8_t mod_type)
{
    DDS_SendFrame((uint16_t) mod_type, DDS_ADDR_MOD_TYPE);
}

static void DDS_SetAmDepthPercent(uint8_t depth_percent)
{
    DDS_SendFrame(DDS_AmDepthPercentToReg(depth_percent), DDS_ADDR_AM_DEPTH);
}

static void DDS_SetFmDeviation(uint16_t deviation_hz)
{
    DDS_SendFrame(deviation_hz, DDS_ADDR_FM_DEV);
}

static void DDS_SetOutputAmplitudePercent(uint8_t amp_percent)
{
    DDS_SendFrame(DDS_OutputAmpPercentToReg(amp_percent), DDS_ADDR_OUTPUT_AMP);
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

/* Formula: BaudRate = CLK / (16 * (IBRD + FBRD / 64)) */
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

static uint8_t IsValueInRange(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    return (value >= min_value) && (value <= max_value);
}

static void DDS_ApplyCurrentMode(void)
{
    if (gUiMode == UI_MODE_SINE) {
        DDS_SetCarrierFrequency(gSineCarrierHz);
        DDS_SetModType(DDS_MOD_CW);
    } else if (gUiMode == UI_MODE_AM) {
        DDS_SetCarrierFrequency(gAmCarrierHz);
        DDS_SetModFrequency(gAmModFreqHz);
        DDS_SetAmDepthPercent(gAmDepthPercent);
        DDS_SetModType(DDS_MOD_AM);
    } else if (gUiMode == UI_MODE_FM) {
        DDS_SetCarrierFrequency(gFmCarrierHz);
        DDS_SetModFrequency(gFmModFreqHz);
        DDS_SetFmDeviation(gFmDevHz);
        DDS_SetModType(DDS_MOD_FM);
    } else if (gUiMode == UI_MODE_ASK) {
        DDS_SetModType(DDS_MOD_ASK);
    } else {
        DDS_SetModType(DDS_MOD_PSK);
    }
}

static void DDS_ApplyStartupConfig(void)
{
    DDS_SetAmDepthPercent(gAmDepthPercent);
    DDS_SetFmDeviation(gFmDevHz);
    DDS_SetOutputAmplitudePercent(gOutputAmpPercent);
    DDS_ApplyCurrentMode();
}

static void EnterInputMode(uint8_t mode)
{
    gInputMode = mode;
    gInputValue = 0UL;
}

static void ExitInputMode(void)
{
    gInputMode = INPUT_NONE;
    gInputValue = 0UL;
}

static void SelectUiMode(uint8_t next_mode)
{
    gUiMode = next_mode;
    ExitInputMode();
    DDS_ApplyCurrentMode();
}

static void StepAmDepth(int8_t step)
{
    int16_t next_depth = (int16_t) gAmDepthPercent + step;

    if (next_depth < 10) {
        next_depth = 10;
    } else if (next_depth > 100) {
        next_depth = 100;
    }

    gAmDepthPercent = (uint8_t) next_depth;
    if (gUiMode == UI_MODE_AM) {
        DDS_SetAmDepthPercent(gAmDepthPercent);
    }
}

static void SetFmDeviationLimit(uint16_t limit_hz)
{
    gFmDevLimitHz = limit_hz;
    if (gFmDevHz > gFmDevLimitHz) {
        gFmDevHz = gFmDevLimitHz;
    }

    if (gUiMode == UI_MODE_FM) {
        DDS_SetFmDeviation(gFmDevHz);
    }
}

static void StepOutputAmplitude(void)
{
    if (gOutputAmpPercent >= DDS_OUTPUT_AMP_MAX_PERCENT) {
        gOutputAmpPercent = DDS_OUTPUT_AMP_MIN_PERCENT;
    } else {
        gOutputAmpPercent += DDS_OUTPUT_AMP_STEP_PERCENT;
        if (gOutputAmpPercent > DDS_OUTPUT_AMP_MAX_PERCENT) {
            gOutputAmpPercent = DDS_OUTPUT_AMP_MIN_PERCENT;
        }
    }

    DDS_SetOutputAmplitudePercent(gOutputAmpPercent);
}

static void ConfirmInputValue(void)
{
    switch (gInputMode) {
        case INPUT_SINE_FREQ:
            if (IsValueInRange(gInputValue, DDS_CARRIER_FREQ_MIN_HZ, DDS_CARRIER_FREQ_MAX_HZ)) {
                gSineCarrierHz = gInputValue;
                DDS_SetCarrierFrequency(gSineCarrierHz);
                DDS_SetModType(DDS_MOD_CW);
                ExitInputMode();
            }
            break;

        case INPUT_AM_CARRIER:
            if (IsValueInRange(gInputValue, DDS_CARRIER_FREQ_MIN_HZ, DDS_CARRIER_FREQ_MAX_HZ)) {
                gAmCarrierHz = gInputValue;
                DDS_SetCarrierFrequency(gAmCarrierHz);
                ExitInputMode();
            }
            break;

        case INPUT_AM_MOD_FREQ:
            if (IsValueInRange(gInputValue, DDS_MOD_FREQ_MIN_HZ, DDS_MOD_FREQ_MAX_HZ)) {
                gAmModFreqHz = gInputValue;
                DDS_SetModFrequency(gAmModFreqHz);
                ExitInputMode();
            }
            break;

        case INPUT_FM_CARRIER:
            if (IsValueInRange(gInputValue, DDS_FM_CARRIER_FREQ_MIN_HZ, DDS_CARRIER_FREQ_MAX_HZ)) {
                gFmCarrierHz = gInputValue;
                DDS_SetCarrierFrequency(gFmCarrierHz);
                ExitInputMode();
            }
            break;

        case INPUT_FM_MOD_FREQ:
            if (IsValueInRange(gInputValue, DDS_MOD_FREQ_MIN_HZ, DDS_MOD_FREQ_MAX_HZ)) {
                gFmModFreqHz = gInputValue;
                DDS_SetModFrequency(gFmModFreqHz);
                ExitInputMode();
            }
            break;

        case INPUT_FM_DEV:
            if (gInputValue <= (uint32_t) gFmDevLimitHz) {
                gFmDevHz = (uint16_t) gInputValue;
                DDS_SetFmDeviation(gFmDevHz);
                ExitInputMode();
            }
            break;

        default:
            break;
    }
}

static void DisplaySineScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1S 2A 3F 4Fc ");
    OLED_ShowString(0, 2, (u8 *) "Mode:SIN");
    OLED_ShowString(0, 4, (u8 *) "Fc:");
    OLED_ShowNum(24, 4, (u32) gSineCarrierHz, 8, 16);
    OLED_ShowString(96, 4, (u8 *) "Hz");
    OLED_ShowString(0, 6, (u8 *) "A:");
    OLED_ShowNum(16, 6, (u32) gOutputAmpPercent, 3, 16);
    OLED_ShowString(40, 6, (u8 *) "% NEG");
}

static void DisplayAmScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1S 2A 3F 4/5  ");
    OLED_ShowString(0, 2, (u8 *) "Fc:");
    OLED_ShowNum(24, 2, (u32) gAmCarrierHz, 8, 16);
    OLED_ShowString(96, 2, (u8 *) "Hz");

    OLED_ShowString(0, 4, (u8 *) "Fm:");
    OLED_ShowNum(24, 4, (u32) gAmModFreqHz, 7, 16);
    OLED_ShowString(88, 4, (u8 *) "Hz");

    OLED_ShowString(0, 6, (u8 *) "D:");
    OLED_ShowNum(24, 6, (u32) gAmDepthPercent, 3, 16);
    OLED_ShowString(48, 6, (u8 *) "% A:");
    OLED_ShowNum(80, 6, (u32) gOutputAmpPercent, 3, 16);
    OLED_ShowString(104, 6, (u8 *) "%");
}

static void DisplayFmScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1S 2A 3F 4/5/6");
    OLED_ShowString(0, 2, (u8 *) "Fc:");
    OLED_ShowNum(24, 2, (u32) gFmCarrierHz, 8, 16);
    OLED_ShowString(96, 2, (u8 *) "Hz");

    OLED_ShowString(0, 4, (u8 *) "Fm:");
    OLED_ShowNum(24, 4, (u32) gFmModFreqHz, 7, 16);
    OLED_ShowString(88, 4, (u8 *) "Hz");

    OLED_ShowString(0, 6, (u8 *) "D:");
    OLED_ShowNum(24, 6, (u32) gFmDevHz, 5, 16);
    OLED_ShowString(64, 6, (u8 *) " A:");
    OLED_ShowNum(88, 6, (u32) gOutputAmpPercent, 3, 16);
    OLED_ShowString(112, 6, (u8 *) "%");
}

static void DisplayAskScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1S 2A 3F 7A 8P");
    OLED_ShowString(0, 2, (u8 *) "Mode:ASK PRBS  ");
    OLED_ShowString(0, 4, (u8 *) "Fc:");
    OLED_ShowNum(24, 4, (u32) (DDS_KEYING_CARRIER_HZ / 1000UL), 3, 16);
    OLED_ShowString(48, 4, (u8 *) "k A:");
    OLED_ShowNum(80, 4, (u32) gOutputAmpPercent, 3, 16);
    OLED_ShowString(104, 4, (u8 *) "%");
    OLED_ShowString(0, 6, (u8 *) "Rb:");
    OLED_ShowNum(24, 6, (u32) (DDS_KEYING_BITRATE_BPS / 1000UL), 2, 16);
    OLED_ShowString(40, 6, (u8 *) "kbps PRBS");
}

static void DisplayPskScreen(void)
{
    OLED_ShowString(0, 0, (u8 *) "1S 2A 3F 7A 8P");
    OLED_ShowString(0, 2, (u8 *) "Mode:PSK PRBS  ");
    OLED_ShowString(0, 4, (u8 *) "Fc:");
    OLED_ShowNum(24, 4, (u32) (DDS_KEYING_CARRIER_HZ / 1000UL), 3, 16);
    OLED_ShowString(48, 4, (u8 *) "k A:");
    OLED_ShowNum(80, 4, (u32) gOutputAmpPercent, 3, 16);
    OLED_ShowString(104, 4, (u8 *) "%");
    OLED_ShowString(0, 6, (u8 *) "Rb:");
    OLED_ShowNum(24, 6, (u32) (DDS_KEYING_BITRATE_BPS / 1000UL), 2, 16);
    OLED_ShowString(40, 6, (u8 *) "kbps PRBS");
}

static void DisplayInputScreen(void)
{
    switch (gInputMode) {
        case INPUT_SINE_FREQ:
            OLED_ShowString(0, 0, (u8 *) "Set SIN Fc     ");
            OLED_ShowString(0, 4, (u8 *) "1Hz-10MHz      ");
            break;

        case INPUT_AM_CARRIER:
            OLED_ShowString(0, 0, (u8 *) "Set AM Fc      ");
            OLED_ShowString(0, 4, (u8 *) "1Hz-10MHz      ");
            break;

        case INPUT_AM_MOD_FREQ:
            OLED_ShowString(0, 0, (u8 *) "Set AM Fm      ");
            OLED_ShowString(0, 4, (u8 *) "1Hz-1MHz       ");
            break;

        case INPUT_FM_CARRIER:
            OLED_ShowString(0, 0, (u8 *) "Set FM Fc      ");
            OLED_ShowString(0, 4, (u8 *) "100k-10MHz     ");
            break;

        case INPUT_FM_MOD_FREQ:
            OLED_ShowString(0, 0, (u8 *) "Set FM Fm      ");
            OLED_ShowString(0, 4, (u8 *) "1Hz-1MHz       ");
            break;

        case INPUT_FM_DEV:
            OLED_ShowString(0, 0, (u8 *) "Set FM Dev     ");
            OLED_ShowString(0, 4, (u8 *) "0-");
            OLED_ShowNum(16, 4, (u32) gFmDevLimitHz, 5, 16);
            OLED_ShowString(56, 4, (u8 *) "Hz");
            break;

        default:
            break;
    }

    OLED_ShowString(0, 2, (u8 *) ">");
    OLED_ShowNum(8, 2, (u32) gInputValue, 10, 16);
    OLED_ShowString(0, 6, (u8 *) "=OK /CLR *ESC  ");
}

static void DisplayStatus(void)
{
    OLED_Clear();

    if (gInputMode != INPUT_NONE) {
        DisplayInputScreen();
    } else if (gUiMode == UI_MODE_SINE) {
        DisplaySineScreen();
    } else if (gUiMode == UI_MODE_AM) {
        DisplayAmScreen();
    } else if (gUiMode == UI_MODE_FM) {
        DisplayFmScreen();
    } else if (gUiMode == UI_MODE_ASK) {
        DisplayAskScreen();
    } else {
        DisplayPskScreen();
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
    DisplayStatus();

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
            if (gInputMode == INPUT_NONE) {
                if (key == KEY_DIGIT_1) {
                    SelectUiMode(UI_MODE_SINE);
                } else if (key == KEY_DIGIT_2) {
                    SelectUiMode(UI_MODE_AM);
                } else if (key == KEY_DIGIT_3) {
                    SelectUiMode(UI_MODE_FM);
                } else if (key == KEY_DIGIT_7) {
                    SelectUiMode(UI_MODE_ASK);
                } else if (key == KEY_DIGIT_8) {
                    SelectUiMode(UI_MODE_PSK);
                } else if ((key == KEY_DIGIT_4) && (gUiMode == UI_MODE_SINE)) {
                    EnterInputMode(INPUT_SINE_FREQ);
                } else if ((key == KEY_DIGIT_4) && (gUiMode == UI_MODE_AM)) {
                    EnterInputMode(INPUT_AM_CARRIER);
                } else if ((key == KEY_DIGIT_5) && (gUiMode == UI_MODE_AM)) {
                    EnterInputMode(INPUT_AM_MOD_FREQ);
                } else if ((key == KEY_DIGIT_4) && (gUiMode == UI_MODE_FM)) {
                    EnterInputMode(INPUT_FM_CARRIER);
                } else if ((key == KEY_DIGIT_5) && (gUiMode == UI_MODE_FM)) {
                    EnterInputMode(INPUT_FM_MOD_FREQ);
                } else if ((key == KEY_DIGIT_6) && (gUiMode == UI_MODE_FM)) {
                    EnterInputMode(INPUT_FM_DEV);
                }
            } else if (gInputValue <= 429496728UL) {
                gInputValue = gInputValue * 10UL + (uint32_t) key;
            }
        } else if (key == KEY_EQUAL) {
            ConfirmInputValue();
        } else if ((key == KEY_CHU) && (gInputMode != INPUT_NONE)) {
            gInputValue = 0UL;
        } else if ((key == KEY_CHENG) && (gInputMode != INPUT_NONE)) {
            ExitInputMode();
        } else if ((key == KEY_PLUS) && (gInputMode == INPUT_NONE) && (gUiMode == UI_MODE_AM)) {
            StepAmDepth(10);
        } else if ((key == KEY_MINUS) && (gInputMode == INPUT_NONE) && (gUiMode == UI_MODE_AM)) {
            StepAmDepth(-10);
        } else if ((key == KEY_PLUS) && (gInputMode == INPUT_NONE) && (gUiMode == UI_MODE_FM)) {
            SetFmDeviationLimit(DDS_FM_DEV_LIMIT_10K_HZ);
        } else if ((key == KEY_MINUS) && (gInputMode == INPUT_NONE) && (gUiMode == UI_MODE_FM)) {
            SetFmDeviationLimit(DDS_FM_DEV_LIMIT_5K_HZ);
        } else if ((key == KEY_NEGATE) && (gInputMode == INPUT_NONE)) {
            StepOutputAmplitude();
        }

        DisplayStatus();

        while (getKeyValue() != KEY_NONE) {
        }
        delay_ms(10);
    }
}
