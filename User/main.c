#include "ti_msp_dl_config.h"
#include "key.h"
#include "oled.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti/driverlib/dl_dac12.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DHT11.h"
extern uint8_t OLED_GRAM[128][8];

// ==========================================================
// 常量区
// ==========================================================
#define PI_VAL                  3.1415926f

// ---------------- 信号源参数 ----------------
#define DAC_TABLE_SIZE          256U
#define DAC_UPDATE_RATE_HZ      20000U      // DAC 固定更新率 20kHz

// ---------------- 示波器参数 ----------------
#define OSC_SAMPLE_COUNT        256U
#define OLED_WAVE_WIDTH         128U
#define OLED_WAVE_Y_MIN         16U
#define OLED_WAVE_Y_MAX         63U
#define OLED_WAVE_Y_CENTER      40U

// TIMER_2 的时钟来自 sysconfig：80MHz / (1 * (255+1)) = 312500 Hz
#define TIMER2_TICK_HZ          312500U

#define OSC_TIMEOUT_COUNT       1000000UL

#define ADC_FULL_SCALE          4095.0f
#define DAC_FULL_SCALE          4095.0f
#define DAC_MID_CODE            2048U

// ==========================================================
// 数据结构
// ==========================================================
typedef struct {
    int16_t year;
    int8_t  month;
    int8_t  day;
    int8_t  hour;
    int8_t  minute;
    int8_t  second;
} Calendar_t;

typedef enum {
    MODE_CLOCK = 0,
    MODE_OSC,
    MODE_GEN,
    MODE_SETTING
} Mode_e;

typedef enum {
    GEN_WAVE_SINE = 0,
    GEN_WAVE_SQUARE,
    GEN_WAVE_TRIANGLE,
    GEN_WAVE_DC,
    GEN_WAVE_CUSTOM
} GenWave_e;

typedef enum {
    GEN_EDIT_WAVE = 0,
    GEN_EDIT_AMP,
    GEN_EDIT_FREQ
} GenEditItem_e;

typedef enum {
    OSC_PAGE_WAVE = 0,
    OSC_PAGE_INFO
} OscPage_e;

typedef enum {
    OSC_EDIT_X = 0,
    OSC_EDIT_Y
} OscEditItem_e;

// ==========================================================
// 全局变量
// ==========================================================
volatile Calendar_t gTime = {2024, 1, 23, 16, 50, 0};
volatile int gMilliSeconds = 0;
volatile Mode_e gCurrentMode = MODE_CLOCK;

// ---------------- 信号源 ----------------
volatile GenWave_e gGenWaveType = GEN_WAVE_SINE;
volatile GenEditItem_e gGenEditItem = GEN_EDIT_WAVE;

volatile int gGenAmp = 50;                 // 0~100 (%)
volatile float gGenFreq = 100.0f;          // Hz

volatile uint16_t dac_wave_table[DAC_TABLE_SIZE];
volatile uint32_t gDdsPhaseAcc = 0;
volatile uint32_t gDdsPhaseStep = 0;
volatile uint8_t  gCustomWaveReady = 0;
volatile uint8_t  gCustomWaitingCmd = 0;
volatile uint8_t  gUartCmdReady = 0;
volatile uint16_t gUartRxIndex = 0;
char gUartRxBuffer[96];
char gCustomDisplayBuffer[32];
float gCustomRawData[DAC_TABLE_SIZE];

volatile uint16_t count_DHT11 = 0;


// ---------------- 示波器 ----------------
static const uint32_t gOscSampleRateTable[] = {2000, 5000, 10000, 20000, 31250};
#define OSC_RATE_TABLE_SIZE (sizeof(gOscSampleRateTable)/sizeof(gOscSampleRateTable[0]))

volatile uint8_t  gOscRateIndex = 2;       // 默认 10kHz
volatile uint32_t gOscSampleRate = 10000;  // Hz
volatile float    gYScale = 1.0f;          // 0.5 ~ 5.0

volatile uint16_t gOscRawBuffer[OSC_SAMPLE_COUNT];
volatile uint16_t gOscCaptureIndex = 0;
volatile uint8_t  gOscCapturing = 0;

volatile float gSignalAvgVoltage = 0.0f;
volatile float gSignalVpp = 0.0f;
volatile float gSignalFreq = 0.0f;

volatile OscPage_e gOscPage = OSC_PAGE_WAVE;
volatile OscEditItem_e gOscEditItem = OSC_EDIT_X;
volatile uint16_t gOscCenterCode = 2048;
volatile uint16_t gOscAdcMin = 0;
volatile uint16_t gOscAdcMax = 0;
volatile uint16_t gOscAdcAvgCode = 2048;

//PWM波
volatile uint8_t gPWMOneShot = 0;

// ==========================================================
// 函数声明
// ==========================================================
void Delay_Safe(int ms);

void Update_Calendar(void);

void Init_Generator(void);
void Update_Wave_Table(void);
void Update_DDS_PhaseStep(void);
void Clear_Uart_CommandBuffer(void);
void Parse_Custom_Function(char *str);
void Process_Vofa_Command(void);

void Osc_SetSampleRate(uint32_t sampleRateHz);
void Osc_CaptureFrame(void);
void Osc_ProcessFrame(void);
float Osc_MeasureFrequency(const uint16_t *buf, uint16_t len, uint32_t sampleRateHz);
uint8_t ADC_To_OLED_Y(uint16_t adc);
void Osc_AutoAdjust(void);
uint8_t Osc_FindNearestRateIndex(uint32_t targetRate);

void Display_Clock(void);
void Display_Oscilloscope(void);
void Display_Oscilloscope_WavePage(void);
void Display_Oscilloscope_InfoPage(void);
void Display_Generator(void);
void Display_Setting(void);

void Int2Str(u8 *p, int num);
void Int3Str(u8 *p, int num);
void Int4Str(u8 *p, int num);
void UIntToStr(char *str, uint32_t num);
void OLED_ShowFloat(u8 x, u8 y, float num, u8 decimal_digits);


void IntToString(char *str, int number);
void UART_SendString(char *str);
void Send_Wave_To_VOFA(void);

void Test_DHT11(void);

void PWM_0_OutputOnePeriod(void);

float my_sin(float x);
float my_cos(float x);
float my_tan(float x);
float my_sa(float x);
float my_abs(float x);
float my_sqrt(float x);
float my_exp(float x);
float my_pow(float base, float p);

float my_sin(float x);
float my_cos(float x);
float my_tan(float x);
float my_sa(float x);
float my_abs(float x);
float my_sqrt(float x);
float my_exp(float x);
float my_pow(float base, float p);
// ==========================================================
// 基础函数
// ==========================================================
void Delay_Safe(int ms)
{
    for (int i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

void Int2Str(u8 *p, int num)
{
    p[0] = (num / 10) + '0';
    p[1] = (num % 10) + '0';
}

void Int3Str(u8 *p, int num)
{
    p[0] = (num / 100) + '0';
    p[1] = ((num % 100) / 10) + '0';
    p[2] = (num % 10) + '0';
}

void Int4Str(u8 *p, int num)
{
    p[0] = (num / 1000) + '0';
    p[1] = ((num % 1000) / 100) + '0';
    p[2] = ((num % 100) / 10) + '0';
    p[3] = (num % 10) + '0';
}

void UIntToStr(char *str, uint32_t num)
{
    char temp[16];
    int i = 0, j = 0;

    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    while (num > 0) {
        temp[i++] = (char)('0' + (num % 10));
        num /= 10;
    }

    while (i > 0) {
        str[j++] = temp[--i];
    }
    str[j] = '\0';
}

void OLED_ShowFloat(u8 x, u8 y, float num, u8 decimal_digits)
{
    unsigned char i;
    unsigned int integer_part;
    float fraction_part;

    if (num < 0) {
        OLED_ShowChar(x, y, '-');
        x += 8;
        num = -num;
    }

    integer_part = (unsigned int)num;
    fraction_part = num - integer_part;

    unsigned int temp_int = integer_part;
    unsigned char int_str[10];
    unsigned char int_len = 0;

    if (temp_int == 0) {
        int_str[0] = 0;
        int_len = 1;
    } else {
        while (temp_int > 0) {
            int_str[int_len] = temp_int % 10;
            temp_int /= 10;
            int_len++;
        }
    }

    for (i = 0; i < int_len; i++) {
        OLED_ShowChar(x, y, int_str[int_len - 1 - i] + '0');
        x += 8;
    }

    OLED_ShowChar(x, y, '.');
    x += 8;

    for (i = 0; i < decimal_digits; i++) {
        fraction_part *= 10.0f;
        unsigned int digit = (unsigned int)fraction_part;
        OLED_ShowChar(x, y, digit + '0');
        fraction_part -= digit;
        x += 8;
    }
}

// ==========================================================
// 时间
// ==========================================================
void Update_Calendar(void)
{
    gTime.second++;
    if (gTime.second >= 60) {
        gTime.second = 0;
        gTime.minute++;

        if (gTime.minute >= 60) {
            gTime.minute = 0;
            gTime.hour++;

            if (gTime.hour >= 24) {
                gTime.hour = 0;
                gTime.day++;

                int daysInMonth = 31;
                if (gTime.month == 4 || gTime.month == 6 || gTime.month == 9 || gTime.month == 11) {
                    daysInMonth = 30;
                } else if (gTime.month == 2) {
                    if ((gTime.year % 4 == 0 && gTime.year % 100 != 0) || (gTime.year % 400 == 0)) {
                        daysInMonth = 29;
                    } else {
                        daysInMonth = 28;
                    }
                }

                if (gTime.day > daysInMonth) {
                    gTime.day = 1;
                    gTime.month++;
                    if (gTime.month > 12) {
                        gTime.month = 1;
                        gTime.year++;
                    }
                }
            }
        }
    }
}

// ==========================================================
// 信号源：DDS
// ==========================================================
void Init_Generator(void)
{
    Update_Wave_Table();
    Update_DDS_PhaseStep();
}

void Update_DDS_PhaseStep(void)
{
    double step = ((double)gGenFreq * 4294967296.0) / (double)DAC_UPDATE_RATE_HZ;
    if (step < 1.0) step = 1.0;
    if (step > 4294967295.0) step = 4294967295.0;
    gDdsPhaseStep = (uint32_t)step;
}

void Clear_Uart_CommandBuffer(void)
{
    memset(gUartRxBuffer, 0, sizeof(gUartRxBuffer));
    gUartRxIndex = 0;
}

float my_sin(float x)
{
    while (x > PI_VAL) x -= 2.0f * PI_VAL;
    while (x < -PI_VAL) x += 2.0f * PI_VAL;
    return sinf(x);
}

float my_cos(float x)
{
    return cosf(x);
}

float my_tan(float x)
{
    float c = my_cos(x);
    if (fabsf(c) < 0.001f) return (my_sin(x) >= 0.0f) ? 1000.0f : -1000.0f;
    return my_sin(x) / c;
}

float my_sa(float x)
{
    if (fabsf(x) < 0.0001f) return 1.0f;
    return my_sin(x) / x;
}

float my_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

float my_sqrt(float x)
{
    return (x <= 0.0f) ? 0.0f : sqrtf(x);
}

float my_exp(float x)
{
    return expf(x);
}

float my_pow(float base, float p)
{
    if (base < 0.0f) {
        int pInt = (int)p;
        if ((float)pInt == p) {
            float result = 1.0f;
            int count = (pInt < 0) ? -pInt : pInt;
            for (int i = 0; i < count; i++) result *= base;
            return (pInt < 0) ? (1.0f / result) : result;
        }
        return 0.0f;
    }
    return powf(base, p);
}

static int is_match(const char *str, const char *target, int len)
{
    for (int i = 0; i < len; i++) {
        if (str[i] != target[i]) return 0;
    }
    return 1;
}

static const char *gExprPtr;
static float gCurrentExprX;
static float Parse_Expr(void);

static float Parse_Factor(void)
{
    float v = 0.0f;
    int sign = 1;

    if (*gExprPtr == '-') {
        sign = -1;
        gExprPtr++;
    } else if (*gExprPtr == '+') {
        gExprPtr++;
    }

    if (is_match(gExprPtr, "sin", 3)) {
        gExprPtr += 3;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_sin(v);
    }
    if (is_match(gExprPtr, "cos", 3)) {
        gExprPtr += 3;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_cos(v);
    }
    if (is_match(gExprPtr, "tan", 3)) {
        gExprPtr += 3;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_tan(v);
    }
    if (is_match(gExprPtr, "sa", 2) || is_match(gExprPtr, "Sa", 2)) {
        gExprPtr += 2;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_sa(v);
    }
    if (is_match(gExprPtr, "abs", 3)) {
        gExprPtr += 3;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_abs(v);
    }
    if (is_match(gExprPtr, "sqrt", 4)) {
        gExprPtr += 4;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_sqrt(v);
    }
    if (is_match(gExprPtr, "exp", 3)) {
        gExprPtr += 3;
        if (*gExprPtr == '(') gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * my_exp(v);
    }

    if (*gExprPtr == 'x' || *gExprPtr == 't') {
        gExprPtr++;
        return sign * gCurrentExprX;
    }

    if (*gExprPtr == '(') {
        gExprPtr++;
        v = Parse_Expr();
        if (*gExprPtr == ')') gExprPtr++;
        return sign * v;
    }

    while (*gExprPtr >= '0' && *gExprPtr <= '9') {
        v = v * 10.0f + (*gExprPtr - '0');
        gExprPtr++;
    }
    if (*gExprPtr == '.') {
        float frac = 0.1f;
        gExprPtr++;
        while (*gExprPtr >= '0' && *gExprPtr <= '9') {
            v += (*gExprPtr - '0') * frac;
            frac *= 0.1f;
            gExprPtr++;
        }
    }

    return sign * v;
}

static float Parse_Power(void)
{
    float v = Parse_Factor();
    while (*gExprPtr == '^') {
        gExprPtr++;
        v = my_pow(v, Parse_Factor());
    }
    return v;
}

static float Parse_Term(void)
{
    float v = Parse_Power();
    while (*gExprPtr == '*' || *gExprPtr == '/') {
        char op = *gExprPtr++;
        float rhs = Parse_Power();
        if (op == '*') v *= rhs;
        else if (fabsf(rhs) > 0.000001f) v /= rhs;
    }
    return v;
}

static float Parse_Expr(void)
{
    float v = Parse_Term();
    while (*gExprPtr == '+' || *gExprPtr == '-') {
        char op = *gExprPtr++;
        float rhs = Parse_Term();
        if (op == '+') v += rhs;
        else v -= rhs;
    }
    return v;
}

void Parse_Custom_Function(char *str)
{
    char compact[96];
    uint16_t j = 0;

    for (uint16_t i = 0; str[i] != '\0' && j < (sizeof(compact) - 1); i++) {
        if (str[i] != ' ' && str[i] != '\r' && str[i] != '\n' && str[i] != '\t') {
            compact[j++] = str[i];
        }
    }
    compact[j] = '\0';

    for (uint16_t i = 0; i < DAC_TABLE_SIZE; i++) {
        gCurrentExprX = (2.0f * PI_VAL * (float)i) / (float)DAC_TABLE_SIZE;
        gExprPtr = compact;
        gCustomRawData[i] = Parse_Expr();
    }
}

void Process_Vofa_Command(void)
{
    if (!gUartCmdReady) return;

    __disable_irq();
    gUartCmdReady = 0;
    __enable_irq();

    Parse_Custom_Function(gUartRxBuffer);
    gGenWaveType = GEN_WAVE_CUSTOM;
    gCurrentMode = MODE_GEN;
    gGenEditItem = GEN_EDIT_WAVE;
    gCustomWaveReady = 1;
    gCustomWaitingCmd = 0;
    strncpy(gCustomDisplayBuffer, gUartRxBuffer, sizeof(gCustomDisplayBuffer) - 1);
    gCustomDisplayBuffer[sizeof(gCustomDisplayBuffer) - 1] = '\0';
    Update_Wave_Table();
    Clear_Uart_CommandBuffer();
    OLED_Clear();
}

void Update_Wave_Table(void)
{
    uint32_t i;
    uint16_t code;
    uint16_t swing = (uint16_t)((2047.0f * gGenAmp) / 100.0f);

    if (gGenWaveType == GEN_WAVE_CUSTOM && gCustomWaveReady) {
        float minVal = gCustomRawData[0];
        float maxVal = gCustomRawData[0];
        for (i = 1; i < DAC_TABLE_SIZE; i++) {
            if (gCustomRawData[i] < minVal) minVal = gCustomRawData[i];
            if (gCustomRawData[i] > maxVal) maxVal = gCustomRawData[i];
        }

        {
            float range = maxVal - minVal;
            if (fabsf(range) < 0.000001f) range = 1.0f;

            for (i = 0; i < DAC_TABLE_SIZE; i++) {
                float norm = ((gCustomRawData[i] - minVal) / range) * 2.0f - 1.0f;
                float val = (float)DAC_MID_CODE + ((float)swing * norm);
                if (val < 0.0f) val = 0.0f;
                if (val > DAC_FULL_SCALE) val = DAC_FULL_SCALE;
                dac_wave_table[i] = (uint16_t)val;
            }
        }
        return;
    }

    for (i = 0; i < DAC_TABLE_SIZE; i++) {
        float phase = (2.0f * PI_VAL * (float)i) / (float)DAC_TABLE_SIZE;
        float val = 0.0f;

        switch (gGenWaveType) {
            case GEN_WAVE_SINE:
                val = (float)DAC_MID_CODE + (float)swing * sinf(phase);
                break;

            case GEN_WAVE_SQUARE:
                val = (i < (DAC_TABLE_SIZE / 2)) ? ((float)DAC_MID_CODE + swing)
                                                 : ((float)DAC_MID_CODE - swing);
                break;

            case GEN_WAVE_TRIANGLE: {
                uint32_t q1 = DAC_TABLE_SIZE / 4;
                uint32_t q3 = (DAC_TABLE_SIZE * 3) / 4;
                uint32_t half = DAC_TABLE_SIZE / 2;

                if (i < q1) {
                    val = (float)DAC_MID_CODE + ((float)swing / q1) * i;
                } else if (i < q3) {
                    val = ((float)DAC_MID_CODE + swing) - ((float)swing * 2.0f / half) * (i - q1);
                } else {
                    val = ((float)DAC_MID_CODE - swing) + ((float)swing / q1) * (i - q3);
                }
                break;
            }

            case GEN_WAVE_DC:
                val = (DAC_FULL_SCALE * gGenAmp) / 100.0f;
                break;

            default:
                val = DAC_MID_CODE;
                break;
        }

        if (val < 0.0f) val = 0.0f;
        if (val > DAC_FULL_SCALE) val = DAC_FULL_SCALE;

        code = (uint16_t)val;
        dac_wave_table[i] = code;
    }
}

// ==========================================================
// 示波器：采样率设置、采样、处理
// ==========================================================
void Osc_SetSampleRate(uint32_t sampleRateHz)
{
    uint32_t loadValue;

    if (sampleRateHz < 1000U) sampleRateHz = 1000U;
    if (sampleRateHz > TIMER2_TICK_HZ / 2U) sampleRateHz = TIMER2_TICK_HZ / 2U;

    gOscSampleRate = sampleRateHz;

    loadValue = (TIMER2_TICK_HZ / sampleRateHz);
    if (loadValue > 0) loadValue -= 1U;
    if (loadValue < 1U) loadValue = 1U;

    DL_TimerA_stopCounter(TIMER_2_INST);
    DL_TimerA_setLoadValue(TIMER_2_INST, loadValue);
    DL_TimerA_setTimerCount(TIMER_2_INST, loadValue);
    DL_TimerA_clearInterruptStatus(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
}

uint8_t ADC_To_OLED_Y(uint16_t adc)
{
    float centered = ((float)((int32_t)adc - (int32_t)gOscCenterCode) * gYScale) / 2048.0f;
    int y = (int)(OLED_WAVE_Y_CENTER - centered * 24.0f);

    if (y < OLED_WAVE_Y_MIN) y = OLED_WAVE_Y_MIN;
    if (y > OLED_WAVE_Y_MAX) y = OLED_WAVE_Y_MAX;
    return (uint8_t)y;
}

void Osc_CaptureFrame(void)
{
    uint32_t wait_cnt = 0;

    gOscCaptureIndex = 0;
    gOscCapturing = 1;

    DL_TimerA_stopCounter(TIMER_2_INST);
    DL_TimerA_clearInterruptStatus(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_setTimerCount(TIMER_2_INST, DL_TimerA_getLoadValue(TIMER_2_INST));

    DL_ADC12_clearInterruptStatus(ADC12_0_INST, 0xFFFFFFFF);
    DL_ADC12_startConversion(ADC12_0_INST);

    DL_TimerA_startCounter(TIMER_2_INST);

    while (gOscCaptureIndex < OSC_SAMPLE_COUNT) {
        wait_cnt++;
        if (wait_cnt > OSC_TIMEOUT_COUNT) {
            break;
        }
    }

    DL_TimerA_stopCounter(TIMER_2_INST);
    gOscCapturing = 0;
}

float Osc_MeasureFrequency(const uint16_t *buf, uint16_t len, uint32_t sampleRateHz)
{
    uint32_t i;
    uint32_t sum = 0;
    uint16_t mid;
    int firstCross = -1;
    int secondCross = -1;

    if (len < 8) return 0.0f;

    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    mid = (uint16_t)(sum / len);

    for (i = 1; i < len; i++) {
        if ((buf[i - 1] < mid) && (buf[i] >= mid)) {
            if (firstCross < 0) {
                firstCross = (int)i;
            } else {
                secondCross = (int)i;
                break;
            }
        }
    }

    if (firstCross >= 0 && secondCross > firstCross) {
        uint32_t periodSamples = (uint32_t)(secondCross - firstCross);
        if (periodSamples > 0) {
            return ((float)sampleRateHz) / (float)periodSamples;
        }
    }

    return 0.0f;
}

void Osc_ProcessFrame(void)
{
    uint32_t i, x;
    uint16_t adcMin = 4095;
    uint16_t adcMax = 0;
    uint32_t adcSum = 0;

    for (i = 0; i < OSC_SAMPLE_COUNT; i++) {
        uint16_t v = gOscRawBuffer[i];
        if (v < adcMin) adcMin = v;
        if (v > adcMax) adcMax = v;
        adcSum += v;
    }

    gOscAdcMin = adcMin;
    gOscAdcMax = adcMax;
    gOscAdcAvgCode = (uint16_t)(adcSum / OSC_SAMPLE_COUNT);

    gSignalAvgVoltage = 3.3f * ((float)adcSum / (float)OSC_SAMPLE_COUNT) / ADC_FULL_SCALE;
    gSignalVpp = 3.3f * ((float)(adcMax - adcMin)) / ADC_FULL_SCALE;
    gSignalFreq = Osc_MeasureFrequency(gOscRawBuffer, OSC_SAMPLE_COUNT, gOscSampleRate);

    for (uint8_t page = 2; page < 8; page++) {
        for (x = 0; x < 128; x++) {
            OLED_GRAM[x][page] = 0x00;
        }
    }

    for (x = 0; x < OLED_WAVE_WIDTH; x++) {
        uint16_t idx0 = x * 2;
        uint16_t idx1 = idx0 + 1;
        uint16_t localMin, localMax;
        uint8_t y1, y2;

        if (idx1 >= OSC_SAMPLE_COUNT) idx1 = OSC_SAMPLE_COUNT - 1;

        localMin = gOscRawBuffer[idx0];
        localMax = gOscRawBuffer[idx0];

        if (gOscRawBuffer[idx1] < localMin) localMin = gOscRawBuffer[idx1];
        if (gOscRawBuffer[idx1] > localMax) localMax = gOscRawBuffer[idx1];

        y1 = ADC_To_OLED_Y(localMax);
        y2 = ADC_To_OLED_Y(localMin);

        if (y1 > y2) {
            uint8_t t = y1;
            y1 = y2;
            y2 = t;
        }

        for (uint8_t y = y1; y <= y2; y++) {
            OLED_DrawPoint((u8)x, y, 1);
        }
    }
}

uint8_t Osc_FindNearestRateIndex(uint32_t targetRate)
{
    uint8_t bestIndex = 0;
    uint32_t bestDiff = 0xFFFFFFFF;

    for (uint8_t i = 0; i < OSC_RATE_TABLE_SIZE; i++) {
        uint32_t rate = gOscSampleRateTable[i];
        uint32_t diff = (rate > targetRate) ? (rate - targetRate) : (targetRate - rate);

        if (diff < bestDiff) {
            bestDiff = diff;
            bestIndex = i;
        }
    }
    return bestIndex;
}

void Osc_AutoAdjust(void)
{
    gOscCenterCode = gOscAdcAvgCode;

    {
        uint16_t devHigh = (gOscAdcMax > gOscAdcAvgCode) ? (gOscAdcMax - gOscAdcAvgCode) : 0;
        uint16_t devLow  = (gOscAdcAvgCode > gOscAdcMin) ? (gOscAdcAvgCode - gOscAdcMin) : 0;
        uint16_t peakDev = (devHigh > devLow) ? devHigh : devLow;

        if (peakDev < 8) peakDev = 8;

        float targetPixels = 22.0f;
        float newScale = (targetPixels * 2048.0f) / (24.0f * (float)peakDev);

        if (newScale < 0.5f) newScale = 0.5f;
        if (newScale > 5.0f) newScale = 5.0f;

        gYScale = newScale;
    }

    if (gSignalFreq > 1.0f) {
        float targetCycles = 1.5f;
        uint32_t targetRate = (uint32_t)((gSignalFreq * OSC_SAMPLE_COUNT) / targetCycles);

        uint8_t idx = Osc_FindNearestRateIndex(targetRate);
        gOscRateIndex = idx;
        Osc_SetSampleRate(gOscSampleRateTable[gOscRateIndex]);
    }
}

// ==========================================================
// 显示
// ==========================================================
void Display_Clock(void)
{
    u8 buff[16];

    Int4Str(&buff[0], gTime.year);
    buff[4] = '-';
    Int2Str(&buff[5], gTime.month);
    buff[7] = '-';
    Int2Str(&buff[8], gTime.day);
    buff[10] = '\0';
    OLED_ShowString(0, 0, buff);

    Int2Str(&buff[0], gTime.hour);
    buff[2] = ':';
    Int2Str(&buff[3], gTime.minute);
    buff[5] = ':';
    Int2Str(&buff[6], gTime.second);
    buff[8] = '\0';
    OLED_ShowString(16, 3, buff);
	
		if(count_DHT11 >3000)
		{
			OLED_ShowString(0, 5, (u8 *)"                ");
			Test_DHT11();
			
			
		//	if(!gPWMOneShot)		PWM_0_OutputOnePeriod();  //控制pwm波输出几个周期停止后自动启动
					
			
			count_DHT11 = 0;
		}
	
}

void Display_Oscilloscope_WavePage(void)
{
    char buf[12];
    uint32_t xUsPerPoint = 1000000UL / gOscSampleRate;

 //   OLED_ShowString(0, 0, (u8 *)"                ");
 //   OLED_ShowString(0, 2, (u8 *)"                ");

    OLED_ShowString(0, 0, (u8 *)"X:");
    UIntToStr(buf, xUsPerPoint);
    OLED_ShowString(16, 0, (u8 *)buf);
    OLED_ShowString(48, 0, (u8 *)"us");

    OLED_ShowString(72, 0, (u8 *)"Y:");
    OLED_ShowFloat(88, 0, gYScale, 1);

    //OLED_ShowString(100, 0, (u8 *)"M:");
    if (gOscEditItem == OSC_EDIT_X) OLED_ShowString(120, 0, (u8 *)"X");
    else                            OLED_ShowString(120, 0, (u8 *)"Y");

    for (uint8_t page = 2; page < 8; page++) {
        OLED_WR_Byte(0xb0 + page, 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (uint8_t x = 0; x < 128; x++) {
            OLED_WR_Byte(OLED_GRAM[x][page], 1);
        }
    }
}

void Display_Oscilloscope_InfoPage(void)
{
    char buf[12];

    for (uint8_t page = 2; page < 8; page++) {
        for (uint8_t x = 0; x < 128; x++) {
            OLED_GRAM[x][page] = 0x00;
        }
    }


    OLED_ShowString(0, 0, (u8 *)"A:");
    OLED_ShowFloat(16, 0, gSignalAvgVoltage, 2);
    OLED_ShowString(64, 0, (u8 *)"P:");
    OLED_ShowFloat(80, 0, gSignalVpp, 2);

    OLED_ShowString(0, 2, (u8 *)"F:");
    UIntToStr(buf, (uint32_t)(gSignalFreq + 0.5f));
    OLED_ShowString(16, 2, (u8 *)buf);
		delay_cycles(8000000);
		OLED_ShowString(16, 2, (u8 *)"      ");

    OLED_ShowString(56, 2, (u8 *)"Hz");

    OLED_ShowString(0, 4, (u8 *)"S:");
    UIntToStr(buf, gOscSampleRate);
    OLED_ShowString(16, 4, (u8 *)buf);

/*
    for (uint8_t page = 2; page < 8; page++) {
        OLED_WR_Byte(0xb0 + page, 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (uint8_t x = 0; x < 128; x++) {
            OLED_WR_Byte(OLED_GRAM[x][page], 1);
        }
    }
*/
}

void Display_Oscilloscope(void)
{
    Osc_CaptureFrame();
    Osc_ProcessFrame();

	    // 采样完成后，把波形发给 VOFA
    Send_Wave_To_VOFA();
	
    if (gOscPage == OSC_PAGE_WAVE) {
        Display_Oscilloscope_WavePage();
    } else {
        Display_Oscilloscope_InfoPage();
    }
}

void Display_Generator(void)
{
    char freqStr[12];

    OLED_ShowString(0, 0, (u8 *)"-- SIGNAL GEN --");

    OLED_ShowString(3, 2, (u8 *)"W:");
    if (gGenWaveType == GEN_WAVE_SINE)          OLED_ShowString(16, 2, (u8 *)"Sine    ");
    else if (gGenWaveType == GEN_WAVE_SQUARE)   OLED_ShowString(16, 2, (u8 *)"Square  ");
    else if (gGenWaveType == GEN_WAVE_TRIANGLE) OLED_ShowString(16, 2, (u8 *)"Triangle");
    else if (gGenWaveType == GEN_WAVE_DC)       OLED_ShowString(16, 2, (u8 *)"DC      ");
    else if (gGenWaveType == GEN_WAVE_CUSTOM)   OLED_ShowString(16, 2, (u8 *)"Custom  ");

    OLED_ShowString(0, 4, (u8 *)"A:");
    OLED_ShowNum(16, 4, (u32)gGenAmp, 3, 16);
    OLED_ShowString(40, 4, (u8 *)"%");

    OLED_ShowString(56, 4, (u8 *)"F:");
    UIntToStr(freqStr, (uint32_t)(gGenFreq + 0.5f));
	  OLED_ShowNum(72, 4, (u32)gGenFreq, 3, 16);
  //  OLED_ShowString(72, 4, (u8 *)freqStr);
    OLED_ShowString(112, 4, (u8 *)"H");

    OLED_ShowString(0, 6, (u8 *)"                ");
    if (gGenWaveType == GEN_WAVE_CUSTOM) {
        if (gCustomWaitingCmd) {
            OLED_ShowString(0, 6, (u8 *)"WAIT FUNC");
        } else {
            OLED_ShowString(0, 6, (u8 *)gCustomDisplayBuffer);
        }
    } else {
        OLED_ShowString(85, 2, (u8 *)"now:");
        if (gGenEditItem == GEN_EDIT_WAVE)      OLED_ShowString(120, 2, (u8 *)"W");
        else if (gGenEditItem == GEN_EDIT_AMP)  OLED_ShowString(120, 2, (u8 *)"A");
        else if (gGenEditItem == GEN_EDIT_FREQ) OLED_ShowString(120, 2, (u8 *)"F");
    }
}

void Display_Setting(void)
{
    OLED_ShowString(0, 0, (u8 *)"-- SETTINGS --");
    OLED_ShowString(3, 2, (u8 *)"Generator and");
    OLED_ShowString(0, 4, (u8 *)"Scope decoupled");
    OLED_ShowString(0, 6, (u8 *)"K4 Back Clock");
}

// ==========================================================
// 主函数
// ==========================================================
int main(void)
{
    SYSCFG_DL_init();

    DL_DAC12_setAmplifier(DAC0, DL_DAC12_AMP_ON);
    DL_DAC12_enableOutputPin(DAC0);
    DL_DAC12_enable(DAC0);

    OLED_Init();
    OLED_Clear();
    memset(gCustomDisplayBuffer, 0, sizeof(gCustomDisplayBuffer));

		delay_cycles(80000000 * 2);
	
    Init_Generator();
    Osc_SetSampleRate(gOscSampleRateTable[gOscRateIndex]);

    DL_DAC12_output12(DAC0, dac_wave_table[0]);

    DL_ADC12_enableConversions(ADC12_0_INST);
    DL_ADC12_startConversion(ADC12_0_INST);

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_startCounter(TIMER_0_INST);

    NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
    DL_TimerA_clearInterruptStatus(TIMER_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(TIMER_1_INST);

    NVIC_ClearPendingIRQ(TIMER_2_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_2_INST_INT_IRQN);
    DL_TimerA_clearInterruptStatus(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);

    NVIC_ClearPendingIRQ(UART0_INT_IRQn);
    DL_UART_Main_enableInterrupt(UART0, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART0_INT_IRQn);

    NVIC_ClearPendingIRQ(UART1_INT_IRQn);
    DL_UART_Main_enableInterrupt(UART1, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART1_INT_IRQn);

    OLED_ShowString(20, 2, (u8 *)"System Ready");
    delay_cycles(32000000);
    OLED_Clear();

		NVIC_ClearPendingIRQ(PWM_0_INST_INT_IRQN);
		NVIC_EnableIRQ(PWM_0_INST_INT_IRQN);

		DL_TimerG_clearInterruptStatus(PWM_0_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
		DL_TimerG_enableInterrupt(PWM_0_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);

    while (1) {
        Process_Vofa_Command();
        int key_val = getKeyValue();

        if (gCurrentMode == MODE_GEN &&
            gGenWaveType == GEN_WAVE_CUSTOM &&
            gCustomWaitingCmd &&
            key_val != 4) {
            key_val = 0;
        }

        if (key_val != 0 && key_val != 20) {
            Delay_Safe(10);
            if (getKeyValue() == key_val) {

                if (gCurrentMode == MODE_OSC) {
                    if (key_val == 1) {
                        gCurrentMode = MODE_GEN;
                        OLED_Clear();
                    }
                    else if (key_val == 4) {
                        gCurrentMode = MODE_CLOCK;
                        OLED_Clear();
                    }
                    else if (key_val == 2) {
                        if (gOscEditItem == OSC_EDIT_X) gOscEditItem = OSC_EDIT_Y;
                        else                            gOscEditItem = OSC_EDIT_X;
                    }
                    else if (key_val == 3) {
                        if (gOscPage == OSC_PAGE_WAVE) gOscPage = OSC_PAGE_INFO;
                        else                           gOscPage = OSC_PAGE_WAVE;
                        OLED_Clear();
                    }
                    else if (key_val == 5) {
                        if (gOscEditItem == OSC_EDIT_X) {
                            if (gOscRateIndex < (OSC_RATE_TABLE_SIZE - 1)) {
                                gOscRateIndex++;
                                Osc_SetSampleRate(gOscSampleRateTable[gOscRateIndex]);
                            }
                        } else {
                            gYScale += 0.5f;
                            if (gYScale > 5.0f) gYScale = 5.0f;
                        }
                    }
                    else if (key_val == 6) {
                        if (gOscEditItem == OSC_EDIT_X) {
                            if (gOscRateIndex > 0) {
                                gOscRateIndex--;
                                Osc_SetSampleRate(gOscSampleRateTable[gOscRateIndex]);
                            }
                        } else {
                            gYScale -= 0.5f;
                            if (gYScale < 0.5f) gYScale = 0.5f;
                        }
                    }
                    else if (key_val == 7) {
                        Osc_AutoAdjust();
                    }
                }
                else {
                    if (gCurrentMode == MODE_GEN &&
                        gGenWaveType == GEN_WAVE_CUSTOM &&
                        gCustomWaitingCmd &&
                        key_val == 4) {
                        gCustomWaitingCmd = 0;
                        strcpy(gCustomDisplayBuffer, "UNCLOCKED");
                        OLED_Clear();
                    }
                    else
                    {
                    if (key_val == 1) {
                        OLED_Clear();
                        if (gCurrentMode == MODE_CLOCK)      gCurrentMode = MODE_OSC;
                        else if (gCurrentMode == MODE_GEN)   gCurrentMode = MODE_SETTING;
                        else if (gCurrentMode == MODE_SETTING) gCurrentMode = MODE_CLOCK;
                        else                                 gCurrentMode = MODE_CLOCK;
                    }
                    else if (key_val == 4) {
                        gCurrentMode = MODE_CLOCK;
                        OLED_Clear();
                    }
                    }

                    if (gCurrentMode == MODE_GEN) {
                        if (key_val == 2) {
                            if (gGenEditItem == GEN_EDIT_WAVE) gGenEditItem = GEN_EDIT_FREQ;
                            else gGenEditItem = (GenEditItem_e)(gGenEditItem - 1);
                        }
                        else if (key_val == 3) {
                            if (gGenEditItem == GEN_EDIT_FREQ) gGenEditItem = GEN_EDIT_WAVE;
                            else gGenEditItem = (GenEditItem_e)(gGenEditItem + 1);
                        }
                        else if (key_val == 5) {
                            if (gGenEditItem == GEN_EDIT_WAVE) {
                                gGenWaveType++;
                                if (gGenWaveType > GEN_WAVE_CUSTOM) gGenWaveType = GEN_WAVE_SINE;
                                if (gGenWaveType == GEN_WAVE_CUSTOM) {
                                    gCustomWaitingCmd = 1;
                                    gCustomWaveReady = 0;
                                    strcpy(gCustomDisplayBuffer, "WAIT FUNC");
                                    Clear_Uart_CommandBuffer();
                                }
                                Update_Wave_Table();
                            } else if (gGenEditItem == GEN_EDIT_AMP) {
                                gGenAmp += 5;
                                if (gGenAmp > 100) gGenAmp = 100;
                                Update_Wave_Table();
                            } else if (gGenEditItem == GEN_EDIT_FREQ) {
                                if (gGenFreq < 100.0f)       gGenFreq += 10.0f;
                                else if (gGenFreq < 1000.0f) gGenFreq += 50.0f;
                                else                         gGenFreq += 100.0f;
                                if (gGenFreq > 5000.0f) gGenFreq = 5000.0f;
                                Update_DDS_PhaseStep();
                            }
                        }
                        else if (key_val == 6) {
                            if (gGenEditItem == GEN_EDIT_WAVE) {
                                if (gGenWaveType == GEN_WAVE_SINE) gGenWaveType = GEN_WAVE_CUSTOM;
                                else gGenWaveType--;
                                if (gGenWaveType == GEN_WAVE_CUSTOM) {
                                    gCustomWaitingCmd = 1;
                                    gCustomWaveReady = 0;
                                    strcpy(gCustomDisplayBuffer, "WAIT FUNC");
                                    Clear_Uart_CommandBuffer();
                                }
                                Update_Wave_Table();
                            } else if (gGenEditItem == GEN_EDIT_AMP) {
                                gGenAmp -= 5;
                                if (gGenAmp < 0) gGenAmp = 0;
                                Update_Wave_Table();
                            } else if (gGenEditItem == GEN_EDIT_FREQ) {
                                if (gGenFreq <= 100.0f)       gGenFreq -= 10.0f;
                                else if (gGenFreq <= 1000.0f) gGenFreq -= 50.0f;
                                else                          gGenFreq -= 100.0f;
                                if (gGenFreq < 1.0f) gGenFreq = 1.0f;
                                Update_DDS_PhaseStep();
                            }
                        }
                    }
										
										if (gCurrentMode == MODE_CLOCK){
										
										if(key_val == 2)
										{
											PWM_0_OutputOnePeriod();//启动舵机
										}
										
										
										}
                }
            }

            while (getKeyValue() != 20) {
                switch (gCurrentMode) {
                    case MODE_CLOCK:   Display_Clock();        break;
                    case MODE_OSC:     Display_Oscilloscope(); break;
                    case MODE_GEN:     Display_Generator();    break;
                    case MODE_SETTING: Display_Setting();      break;
                    default: break;
                }
            }

            Delay_Safe(10);
        }

        switch (gCurrentMode) {
            case MODE_CLOCK:   Display_Clock();        break;
            case MODE_OSC:     Display_Oscilloscope(); break;
            case MODE_GEN:     Display_Generator();    break;
            case MODE_SETTING: Display_Setting();      break;
            default: break;
        }
    }
}

// ==========================================================
// 中断
// ==========================================================
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            gMilliSeconds++;
				
						count_DHT11++;
				
            if (gMilliSeconds >= 1000) {
                gMilliSeconds = 0;
                Update_Calendar();
            }
            break;
        default:
            break;
    }
}

void TIMER_1_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_1_INST)) {
        case DL_TIMER_IIDX_ZERO:
        {
            uint16_t outCode;

            if (gGenWaveType == GEN_WAVE_DC) {
                outCode = (uint16_t)((DAC_FULL_SCALE * gGenAmp) / 100.0f);
            } else {
                gDdsPhaseAcc += gDdsPhaseStep;
                {
                    uint8_t idx = (uint8_t)(gDdsPhaseAcc >> 24);
                    outCode = dac_wave_table[idx];
                }
            }

            DL_DAC12_output12(DAC0, outCode);
            break;
        }
        default:
            break;
    }
}

void TIMER_2_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_2_INST)) {
        case DL_TIMER_IIDX_ZERO:
            if (gOscCapturing && gOscCaptureIndex < OSC_SAMPLE_COUNT) {
                uint16_t adc_val = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
                gOscRawBuffer[gOscCaptureIndex] = adc_val;
                gOscCaptureIndex++;

                if (gOscCaptureIndex < OSC_SAMPLE_COUNT) {
                    DL_ADC12_startConversion(ADC12_0_INST);
                }
            }
            break;

        default:
            break;
    }
}

void UART0_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART0)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            char ch = (char)DL_UART_Main_receiveData(UART0);

            if (gUartRxIndex < (sizeof(gUartRxBuffer) - 1)) {
                gUartRxBuffer[gUartRxIndex++] = ch;
                gUartRxBuffer[gUartRxIndex] = '\0';

                if (gUartRxIndex >= 3 &&
                    gUartRxBuffer[gUartRxIndex - 3] == 'e' &&
                    gUartRxBuffer[gUartRxIndex - 2] == 'n' &&
                    gUartRxBuffer[gUartRxIndex - 1] == 'd') {
                    gUartRxBuffer[gUartRxIndex - 3] = '\0';
                    gUartCmdReady = 1;
                }
            } else {
                Clear_Uart_CommandBuffer();
            }
            break;
        }
        default:
            break;
    }
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART1)) {
        case DL_UART_MAIN_IIDX_RX:
            (void)DL_UART_Main_receiveData(UART1);
            break;
        default:
            break;
    }
}

void IntToString(char *str, int number)
{
    char temp[16];
    int i = 0, j = 0;

    if (number == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    if (number < 0) {
        str[j++] = '-';
        number = -number;
    }

    while (number > 0) {
        temp[i++] = (number % 10) + '0';
        number /= 10;
    }

    while (i > 0) {
        str[j++] = temp[--i];
    }

    str[j] = '\0';
}

void UART_SendString(char *str)
{
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART0, *str);
		//	  DL_UART_Main_transmitDataBlocking(UART1, *str);
        str++;
    }
}

void Send_Wave_To_VOFA(void)
{
    char print_buffer[16];

    // 这里发 128 点，减轻阻塞发送压力
    // 256 点也能发，但界面会更卡
    for (uint16_t i = 0; i < OSC_SAMPLE_COUNT; i++) {
        uint16_t sample = gOscRawBuffer[i];   

        IntToString(print_buffer, sample);
        UART_SendString(print_buffer);

        // VOFA 常见兼容格式：每个点一行，结尾 \r\n
        DL_UART_Main_transmitDataBlocking(UART0, '\r');
       DL_UART_Main_transmitDataBlocking(UART0, '\n');
		//	  DL_UART_Main_transmitDataBlocking(UART1, '\r');
     //   DL_UART_Main_transmitDataBlocking(UART1, '\n');
			delay_cycles(80);
    }
}

void Test_DHT11(void)
{
    DHT11_Data_t dht;
    u8 buf[4];

    dht = DHT11_Read();

    if (dht.timeout) {
        OLED_ShowString(0, 5, (u8 *)"Timeout");
        Int2Str(buf, dht.step);
        buf[2] = '\0';
        OLED_ShowString(72, 5, (u8 *)"S:");
        OLED_ShowString(88, 5, buf);
    }
    else if (!dht.checksum_ok) {
        OLED_ShowString(0, 5, (u8 *)"CheckErr");
        OLED_ShowString(72, 5, (u8 *)"S:9");
    }
    else {
        OLED_ShowString(0, 5, (u8 *)"Temp:");
        Int2Str(buf, dht.temperature);
        buf[2] = '\0';
        OLED_ShowString(48, 5, buf);

        OLED_ShowString(68, 5, (u8 *)"Humi:");
        Int2Str(buf, dht.humidity);
        buf[2] = '\0';
        OLED_ShowString(110, 5, buf);
    }
}

void PWM_0_OutputOnePeriod(void)
{
    gPWMOneShot = 1;

    DL_TimerG_stopCounter(PWM_0_INST);
    DL_TimerG_setTimerCount(PWM_0_INST, 0);
    DL_TimerG_clearInterruptStatus(PWM_0_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
    DL_TimerG_startCounter(PWM_0_INST);
}


void PWM_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PWM_0_INST)) {
        case DL_TIMER_IIDX_LOAD:
            gPWMOneShot++;
            if (gPWMOneShot > 37) {
                DL_TimerG_stopCounter(PWM_0_INST);
                gPWMOneShot = 0;
            }
            break;

        default:
            break;
    }
}

