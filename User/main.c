#include "ti_msp_dl_config.h"
#include "oled.h"

// 50Hz交流电，6400Hz采样频率，一个周期正好切成 128 份
#define SAMPLE_POINTS 128
// 128个电压点 + 128个电流点
#define BUFFER_SIZE (SAMPLE_POINTS * 2)
// 50Hz 每周期计算一次，25 次约等于 0.5 秒刷新一次 OLED
#define OLED_REFRESH_DIVIDER 25
#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_CODE 4095.0f

// 交错存放ADC结果，偶数存电压(0,2,4)，奇数存电流(1,3,5)
// 加 volatile 是警告编译器：别瞎优化，这个数据在后台会被DMA悄悄改掉！
volatile uint16_t adc_buffer[BUFFER_SIZE]; 


volatile bool g_dataReady = false;
static uint8_t g_oledRefreshCount = 0;


#include <math.h>


float g_V_rms_real = 0.0f;     // 真实的电压有效值 (V)
float g_I_rms_real = 0.0f;     // 真实的电流有效值 (A)
float g_Active_Power = 0.0f;   // 真实的有功功率 (W)
float g_Power_Factor = 0.0f;   // 功率因数
float g_V_rms_display = 0.0f;
float g_I_rms_display = 0.0f;
float g_Active_Power_display = 0.0f;
float g_Power_Factor_display = 0.0f;
static float g_vRmsSum = 0.0f;
static float g_iRmsSum = 0.0f;
static float g_activePowerSum = 0.0f;
static float g_powerFactorSum = 0.0f;

// 标定系数 (Calibration Factors)
// 这两个值需要你接上真实的万用表/功率计后，慢慢调整修改
// 直连 ADC 测试时，电压数字量 1.0 对应 ADC_REF_VOLTAGE / ADC_MAX_CODE。
#define VOLT_CAL_FACTOR (ADC_REF_VOLTAGE / ADC_MAX_CODE)
#define CURR_CAL_FACTOR 0.002f 

static uint32_t Float_Scale_Pow10(uint8_t frac_len)
{
    uint32_t scale = 1;

    while (frac_len--) {
        scale *= 10;
    }

    return scale;
}

static uint8_t OLED_ShowText(uint8_t x, uint8_t y, const char *text)
{
    OLED_ShowString(x, y, (u8 *)text);

    while (*text != '\0') {
        x += 8;
        text++;
    }

    return x;
}

static void OLED_ShowFraction(uint8_t x, uint8_t y, uint32_t fraction, uint8_t frac_len)
{
    uint32_t divisor = Float_Scale_Pow10(frac_len) / 10;

    while (frac_len--) {
        OLED_ShowChar(x, y, (fraction / divisor) % 10 + '0');
        x += 8;
        divisor /= 10;
    }
}

static void OLED_ShowInteger(uint8_t x, uint8_t y, uint32_t value, uint8_t len)
{
    uint32_t max_value = Float_Scale_Pow10(len) - 1;

    if (value > max_value) {
        while (len--) {
            OLED_ShowChar(x, y, '*');
            x += 8;
        }
    } else {
        OLED_ShowNum(x, y, value, len, 16);
    }
}

static void OLED_ShowFixed(uint8_t x, uint8_t y, const char *label, float value,
    uint8_t int_len, uint8_t frac_len, const char *unit)
{
    uint32_t scale = Float_Scale_Pow10(frac_len);
    uint32_t scaled_value;
    uint32_t integer_part;
    uint32_t fraction_part;

    OLED_ShowString(0, y, (u8 *)"                ");
    x = OLED_ShowText(x, y, label);

    if (value < 0.0f) {
        OLED_ShowChar(x, y, '-');
        x += 8;
        value = -value;
    }

    scaled_value = (uint32_t)(value * (float)scale + 0.5f);
    integer_part = scaled_value / scale;
    fraction_part = scaled_value % scale;

    OLED_ShowInteger(x, y, integer_part, int_len);
    x += int_len * 8;
    OLED_ShowChar(x, y, '.');
    x += 8;
    OLED_ShowFraction(x, y, fraction_part, frac_len);
    x += frac_len * 8;
    OLED_ShowText(x, y, unit);
}

static void OLED_ShowPowerAndRMS(void)
{
    OLED_ShowFixed(0, 0, "U=", g_V_rms_display, 4, 3, "V");
    OLED_ShowFixed(0, 2, "I=", g_I_rms_display, 1, 3, "A");
    OLED_ShowFixed(0, 4, "P=", g_Active_Power_display, 4, 2, "W");
    OLED_ShowFixed(0, 6, "PF=", g_Power_Factor_display, 1, 3, "");
}

static void DMA_Reload_ADC_Buffer(void)
{
    DL_DMA_disableChannel(DMA, DMA_UI_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_setSrcAddr(DMA, DMA_UI_CHAN_ID,
        (uint32_t) DL_ADC12_getFIFOAddress(ADC12_UI_INST));
    DL_DMA_setDestAddr(DMA, DMA_UI_CHAN_ID, (uint32_t) &adc_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_UI_CHAN_ID, BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_UI_CHAN_ID);
}


//计算有效值与功率

void Calculate_Power_And_RMS(void) {
    long long v_sum = 0, i_sum = 0;
    
    
    // 第一步：计算直流偏置 (求一个周期内的平均值)
   
    for (int i = 0; i < BUFFER_SIZE; i += 2) {
        v_sum += adc_buffer[i];       // 累加所有电压点
        i_sum += adc_buffer[i + 1];   // 累加所有电流点
    }
    float v_dc_offset = (float)v_sum / SAMPLE_POINTS;
    float i_dc_offset = (float)i_sum / SAMPLE_POINTS;
    
   
    // 第二步：去直流偏置，并累加平方和与乘积
    
    float v_sq_sum = 0.0f; // 电压平方和
    float i_sq_sum = 0.0f; // 电流平方和
    float p_sum = 0.0f;    // 瞬时功率乘积累加和
    
    for (int i = 0; i < BUFFER_SIZE; i += 2) {
        // 减去偏置，还原真实交流波形 (此时波形有正有负)
        float v_ac = (float)adc_buffer[i] - v_dc_offset;
        float i_ac = (float)adc_buffer[i + 1] - i_dc_offset;
        
        v_sq_sum += (v_ac * v_ac);
        i_sq_sum += (i_ac * i_ac);
        p_sum += (v_ac * i_ac);
    }
    
   
    // 第三步：计算纯数字量下的 RMS 和 有功功率
   
    // RMS = sqrt(平方和 / 采样点数)
    float v_rms_adc = sqrt(v_sq_sum / SAMPLE_POINTS);
    float i_rms_adc = sqrt(i_sq_sum / SAMPLE_POINTS);
    float p_adc = p_sum / SAMPLE_POINTS;
    
    
    // 第四步：乘以标定系数，还原为真实世界的物理量 //
    g_V_rms_real = v_rms_adc * VOLT_CAL_FACTOR;
    g_I_rms_real = i_rms_adc * CURR_CAL_FACTOR;
    g_Active_Power = p_adc * VOLT_CAL_FACTOR * CURR_CAL_FACTOR;
    
    // 视在功率 S = U_rms * I_rms
    float apparent_power = g_V_rms_real * g_I_rms_real;
    
    // 功率因数 PF = 有功功率 / 视在功率
    if (apparent_power > 0.001f) {
        g_Power_Factor = g_Active_Power / apparent_power;
    } else {
        g_Power_Factor = 0.0f; // 防死机：避免除以 0
    }
}

void DMA_IRQHandler(void) {
    // 查一下到底是谁触发的中断
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH0: 
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
            g_dataReady = true;  
            break;
        default:
            break;
    }
}

int main(void) {
    // 把 SysConfig 里画好的外设全部初始化一遍
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ShowString(0, 0, (u8 *)"Power Meter");
    OLED_ShowString(0, 2, (u8 *)"Waiting ADC...");
    
    // 在内核中断控制器里，允许 DMA 触发中断
    NVIC_EnableIRQ(DMA_INT_IRQn);
    
    // ADC 序列的第二个结果进 FIFO 后，触发 DMA 一次搬走 2 个采样值。
    DL_ADC12_disableConversions(ADC12_UI_INST);
    DL_ADC12_enableDMATrigger(ADC12_UI_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);
    DL_ADC12_clearDMATriggerStatus(ADC12_UI_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC12_UI_INST);
    DMA_Reload_ADC_Buffer();
    
    // ADC 启动，但先不急着采，挂起等待定时器的硬件触发信号
    DL_ADC12_startConversion(ADC12_UI_INST);
    
    // 定时器开始以 6400Hz 的频率狂跳，整个流水线正式转起来了！
    DL_TimerA_startCounter(TIMER_TIMG_INST);
    
    while (1) {
        
        if (g_dataReady) {
            g_dataReady = false;
            
            
            Calculate_Power_And_RMS();

            g_vRmsSum += g_V_rms_real;
            g_iRmsSum += g_I_rms_real;
            g_activePowerSum += g_Active_Power;
            g_powerFactorSum += g_Power_Factor;
            g_oledRefreshCount++;
            if (g_oledRefreshCount >= OLED_REFRESH_DIVIDER) {
                g_V_rms_display = g_vRmsSum / OLED_REFRESH_DIVIDER;
                g_I_rms_display = g_iRmsSum / OLED_REFRESH_DIVIDER;
                g_Active_Power_display = g_activePowerSum / OLED_REFRESH_DIVIDER;
                g_Power_Factor_display = g_powerFactorSum / OLED_REFRESH_DIVIDER;

                g_oledRefreshCount = 0;
                g_vRmsSum = 0.0f;
                g_iRmsSum = 0.0f;
                g_activePowerSum = 0.0f;
                g_powerFactorSum = 0.0f;
                OLED_ShowPowerAndRMS();
            }
            
            
            //指针已经跑到数组外面去了，得把它拽回数组开头 [0]
            DMA_Reload_ADC_Buffer();
        }
        
        
        __WFI(); 
    }
}
