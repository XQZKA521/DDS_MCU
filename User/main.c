#include "ti_msp_dl_config.h"
#include "oled.h"

#define ADC_SAMPLE_RATE_HZ 6400.0f
// 1024 个电压采样点，采样窗口约 160ms，可覆盖 10Hz 以上信号的完整周期
#define SAMPLE_POINTS 1024
// 电压、电流交错存放，各 SAMPLE_POINTS 个点
#define BUFFER_SIZE (SAMPLE_POINTS * 2)
// 每个 DMA 缓冲区约 160ms，3 次约等于 0.5 秒刷新一次 OLED
#define OLED_REFRESH_DIVIDER 3
#define MIN_SIGNAL_VPP_CODE 20
#define MIN_EDGE_HYS_CODE 5
#define VOFA_SEND_POINTS 128
#define VOFA_SAMPLE_STRIDE (SAMPLE_POINTS / VOFA_SEND_POINTS)
#define ADC_REF_VOLTAGE 3.3f
#define ADC_FULL_SCALE_COUNTS 4096.0f
#define ADC_CHANNEL_COUNT 2
#define ADC_VOLTAGE_CH 0
#define ADC_CURRENT_CH 1

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
float g_Signal_Frequency = 0.0f;
float g_V_rms_display = 0.0f;
float g_I_rms_display = 0.0f;
float g_Active_Power_display = 0.0f;
float g_Power_Factor_display = 0.0f;
static float g_vRmsSum = 0.0f;
static float g_iRmsSum = 0.0f;
static float g_activePowerSum = 0.0f;
static float g_powerFactorSum = 0.0f;

// 标定系数 (Calibration Factors)
// VOLT_FRONTEND_GAIN 是待测端到 ADC 引脚的反向比例：
// 直连 ADC 时为 1；若前端把 10V 缩放到 ADC 上 1V，则应填 10。
#define VOLT_FRONTEND_GAIN 0.7215386f
#define VOLT_CAL_FACTOR ((ADC_REF_VOLTAGE / ADC_FULL_SCALE_COUNTS) * VOLT_FRONTEND_GAIN)
// PA26 采到的是采样电阻/电流检测前端上的电压，电流按 I = U / R 换算。
// 这里的 0.402832f 等效于原来的 0.002A/code；实际使用时请改成真实采样电阻值。
#define CURRENT_SENSE_RESISTANCE_OHM 0.402832f
#define CURRENT_FRONTEND_GAIN 1.0f
#define ADC_CODE_TO_PIN_VOLT ((ADC_REF_VOLTAGE / ADC_FULL_SCALE_COUNTS))
#define CURR_CAL_FACTOR ((ADC_CODE_TO_PIN_VOLT * CURRENT_FRONTEND_GAIN) / CURRENT_SENSE_RESISTANCE_OHM)

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
    DL_DMA_setTransferMode(DMA, DMA_UI_CHAN_ID, DL_DMA_SINGLE_TRANSFER_MODE);
    DL_DMA_setSrcAddr(DMA, DMA_UI_CHAN_ID,
        (uint32_t) DL_ADC12_getFIFOAddress(ADC12_UI_INST));
    DL_DMA_setDestAddr(DMA, DMA_UI_CHAN_ID, (uint32_t) &adc_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_UI_CHAN_ID, BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_UI_CHAN_ID);
}

static uint16_t ADC_GetSampleCode(uint16_t sample_index, uint8_t channel)
{
    return adc_buffer[sample_index * ADC_CHANNEL_COUNT + channel];
}

static float ADC_CodeDeltaToVoltage(float code_delta)
{
    return code_delta * VOLT_CAL_FACTOR;
}

static float ADC_CodeDeltaToCurrent(float code_delta)
{
    return code_delta * CURR_CAL_FACTOR;
}

static void ADC_Stop_Sampling(void)
{
    DL_TimerA_stopCounter(TIMER_TIMG_INST);
    DL_ADC12_stopConversion(ADC12_UI_INST);
    DL_ADC12_disableConversions(ADC12_UI_INST);
}

static void ADC_Start_Sampling(void)
{
    DL_ADC12_disableFIFO(ADC12_UI_INST);
    DL_ADC12_enableFIFO(ADC12_UI_INST);
    DL_ADC12_clearDMATriggerStatus(ADC12_UI_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);
    DMA_Reload_ADC_Buffer();
    DL_ADC12_enableDMA(ADC12_UI_INST);
    DL_ADC12_enableConversions(ADC12_UI_INST);
    DL_TimerA_setTimerCount(TIMER_TIMG_INST, 0);
    DL_ADC12_startConversion(ADC12_UI_INST);
    DL_TimerA_startCounter(TIMER_TIMG_INST);
}

static void UART0_SendChar(char ch)
{
    while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
    }

    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
}

static void UART0_SendString(const char *str)
{
    while (*str != '\0') {
        UART0_SendChar(*str++);
    }
}

static void UART0_SendUint(uint32_t value)
{
    char buf[10];
    uint8_t len = 0;

    if (value == 0) {
        UART0_SendChar('0');
        return;
    }

    while ((value > 0) && (len < sizeof(buf))) {
        buf[len++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (len > 0) {
        UART0_SendChar(buf[--len]);
    }
}

static void UART0_SendInt(int32_t value)
{
    if (value < 0) {
        UART0_SendChar('-');
        value = -value;
    }

    UART0_SendUint((uint32_t)value);
}

static void UART0_SendFloat(float value, uint8_t frac_len)
{
    uint32_t scale = Float_Scale_Pow10(frac_len);
    uint32_t divisor;
    int32_t integer_part;
    uint32_t fraction_part;
    uint32_t scaled_fraction;

    if (value < 0.0f) {
        UART0_SendChar('-');
        value = -value;
    }

    integer_part = (int32_t)value;
    scaled_fraction = (uint32_t)((value - (float)integer_part) * (float)scale + 0.5f);

    if (scaled_fraction >= scale) {
        integer_part++;
        scaled_fraction -= scale;
    }

    fraction_part = scaled_fraction;
    UART0_SendInt(integer_part);

    if (frac_len > 0) {
        UART0_SendChar('.');
        divisor = scale / 10;

        while (frac_len--) {
            UART0_SendChar((char)('0' + ((fraction_part / divisor) % 10)));
            divisor /= 10;
        }
    }
}

static void VOFA_SendWaveform(void)
{
    float v_dc_offset = 0.0f;
    float i_dc_offset = 0.0f;

    for (uint16_t n = 0; n < SAMPLE_POINTS; n++) {
        v_dc_offset += ADC_GetSampleCode(n, ADC_VOLTAGE_CH);
        i_dc_offset += ADC_GetSampleCode(n, ADC_CURRENT_CH);
    }

    v_dc_offset /= SAMPLE_POINTS;
    i_dc_offset /= SAMPLE_POINTS;

    for (uint16_t n = 0; n < SAMPLE_POINTS; n += VOFA_SAMPLE_STRIDE) {
        float v_wave = ADC_CodeDeltaToVoltage(
            (float)ADC_GetSampleCode(n, ADC_VOLTAGE_CH) - v_dc_offset);
        float i_wave = ADC_CodeDeltaToCurrent(
            (float)ADC_GetSampleCode(n, ADC_CURRENT_CH) - i_dc_offset);

        UART0_SendFloat(v_wave, 4);
        UART0_SendChar(',');
        UART0_SendFloat(i_wave, 4);
        UART0_SendString("\r\n");
    }
}

static bool Find_ADC_Cycle_Window(uint8_t channel, uint16_t *start_sample,
    uint16_t *end_sample, float *dc_offset, uint16_t *vpp_code, float *frequency)
{
    long long sum = 0;
    uint16_t code_max = 0;
    uint16_t code_min = 4095;
    uint16_t dc_mid;
    uint16_t hys;
    uint16_t edges[32];
    uint16_t edge_count = 0;
    bool high_state = false;

    for (uint16_t n = 0; n < SAMPLE_POINTS; n++) {
        uint16_t code = ADC_GetSampleCode(n, channel);

        sum += code;
        if (code > code_max) {
            code_max = code;
        }
        if (code < code_min) {
            code_min = code;
        }
    }

    dc_mid = (uint16_t)(sum / SAMPLE_POINTS);
    *vpp_code = code_max - code_min;
    *dc_offset = (float)dc_mid;

    if (*vpp_code < MIN_SIGNAL_VPP_CODE) {
        *start_sample = 0;
        *end_sample = SAMPLE_POINTS;
        *frequency = 0.0f;
        return false;
    }

    hys = (*vpp_code / 10 < MIN_EDGE_HYS_CODE) ? MIN_EDGE_HYS_CODE : *vpp_code / 10;

    // 使用三点平均 + 滞回阈值检测上升沿，降低噪声导致的重复触发。
    for (uint16_t n = 1; n < SAMPLE_POINTS - 1; n++) {
        uint16_t smooth_code = (ADC_GetSampleCode(n - 1, channel) +
                                ADC_GetSampleCode(n, channel) +
                                ADC_GetSampleCode(n + 1, channel)) / 3;

        if (!high_state) {
            if (smooth_code > (uint16_t)(dc_mid + hys)) {
                if (edge_count < (sizeof(edges) / sizeof(edges[0]))) {
                    edges[edge_count++] = n;
                }
                high_state = true;
            }
        } else {
            if (smooth_code < (uint16_t)(dc_mid - hys)) {
                high_state = false;
            }
        }
    }

    if (edge_count < 2) {
        *start_sample = 0;
        *end_sample = SAMPLE_POINTS;
        *frequency = 0.0f;
        return false;
    }

    *start_sample = edges[0];
    *end_sample = edges[edge_count - 1];
    *frequency = ADC_SAMPLE_RATE_HZ * (float)(edge_count - 1) /
        (float)(*end_sample - *start_sample);
    return true;
}

static float ADC_CalculateWindowDC(uint8_t channel, uint16_t start_sample,
    uint16_t end_sample)
{
    float dc_offset = 0.0f;
    uint16_t points = end_sample - start_sample;

    if (points == 0) {
        return 0.0f;
    }

    for (uint16_t n = start_sample; n < end_sample; n++) {
        dc_offset += ADC_GetSampleCode(n, channel);
    }

    return dc_offset / points;
}

static float ADC_CalculateRmsCode(uint8_t channel, uint16_t start_sample,
    uint16_t end_sample, float dc_offset)
{
    float square_sum = 0.0f;
    uint16_t points = end_sample - start_sample;

    if (points == 0) {
        return 0.0f;
    }

    for (uint16_t n = start_sample; n < end_sample; n++) {
        float ac_code = (float)ADC_GetSampleCode(n, channel) - dc_offset;

        square_sum += ac_code * ac_code;
    }

    return sqrt(square_sum / points);
}


//计算有效值与功率

void Calculate_Power_And_RMS(void) {
    static float filt_freq = 0.0f;
    static float filt_v_rms = 0.0f;
    static float filt_i_rms = 0.0f;
    static float filt_power = 0.0f;
    static float filt_pf = 0.0f;
    uint16_t v_start_sample;
    uint16_t v_end_sample;
    uint16_t i_start_sample;
    uint16_t i_end_sample;
    uint16_t power_points;
    uint16_t v_vpp_code;
    uint16_t i_vpp_code;
    float voltage_frequency;
    float current_frequency;
    float v_dc_offset;
    float i_dc_offset;
    float power_v_dc_offset;
    float power_i_dc_offset;
    float p_sum = 0.0f;    // 瞬时功率乘积累加和
    float v_rms_adc;
    float i_rms_adc;
    float p_adc;
    float raw_v_rms;
    float raw_i_rms;
    float raw_power;
    float raw_pf;
    float apparent_power;
    float alpha;

    // 电压通道使用 PA27/ADC CH0，电流通道使用 PA26/ADC CH1。
    // 两个通道各自找周期窗口和直流偏置，避免把电压采样窗口套到电流 RMS 上。
    Find_ADC_Cycle_Window(ADC_VOLTAGE_CH, &v_start_sample, &v_end_sample,
        &v_dc_offset, &v_vpp_code, &voltage_frequency);
    Find_ADC_Cycle_Window(ADC_CURRENT_CH, &i_start_sample, &i_end_sample,
        &i_dc_offset, &i_vpp_code, &current_frequency);
    g_Signal_Frequency = voltage_frequency;

    if ((v_end_sample <= v_start_sample) || (i_end_sample <= i_start_sample)) {
        return;
    }

    v_dc_offset = ADC_CalculateWindowDC(ADC_VOLTAGE_CH, v_start_sample, v_end_sample);
    i_dc_offset = ADC_CalculateWindowDC(ADC_CURRENT_CH, i_start_sample, i_end_sample);
    v_rms_adc = ADC_CalculateRmsCode(ADC_VOLTAGE_CH, v_start_sample, v_end_sample,
        v_dc_offset);
    i_rms_adc = ADC_CalculateRmsCode(ADC_CURRENT_CH, i_start_sample, i_end_sample,
        i_dc_offset);

    power_points = v_end_sample - v_start_sample;
    power_v_dc_offset = ADC_CalculateWindowDC(ADC_VOLTAGE_CH, v_start_sample,
        v_end_sample);
    power_i_dc_offset = ADC_CalculateWindowDC(ADC_CURRENT_CH, v_start_sample,
        v_end_sample);

    for (uint16_t n = v_start_sample; n < v_end_sample; n++) {
        // 减去偏置，还原真实交流波形 (此时波形有正有负)
        float v_ac = (float)ADC_GetSampleCode(n, ADC_VOLTAGE_CH) - power_v_dc_offset;
        float i_ac = (float)ADC_GetSampleCode(n, ADC_CURRENT_CH) - power_i_dc_offset;
        
        p_sum += (v_ac * i_ac);
    }

    p_adc = p_sum / power_points;
    
    
    // 第四步：乘以标定系数，还原为真实世界的物理量 //
    raw_v_rms = v_rms_adc * VOLT_CAL_FACTOR;
    raw_i_rms = i_rms_adc * CURR_CAL_FACTOR;
    raw_power = p_adc * VOLT_CAL_FACTOR * CURR_CAL_FACTOR;
    
    // 视在功率 S = U_rms * I_rms
    apparent_power = raw_v_rms * raw_i_rms;
    
    // 功率因数 PF = 有功功率 / 视在功率
    if (apparent_power > 0.001f) {
        raw_pf = raw_power / apparent_power;
    } else {
        raw_pf = 0.0f; // 防死机：避免除以 0
    }

    // 借用参考项目的显示稳定策略：变化大时立即更新，稳定时一阶低通滤波。
    if ((filt_freq < 1.0f) ||
        (fabs(g_Signal_Frequency - filt_freq) > filt_freq * 0.10f) ||
        (fabs(raw_v_rms - filt_v_rms) > filt_v_rms * 0.10f)) {
        alpha = 1.0f;
    } else {
        alpha = 0.20f;
    }

    filt_freq = alpha * g_Signal_Frequency + (1.0f - alpha) * filt_freq;
    filt_v_rms = alpha * raw_v_rms + (1.0f - alpha) * filt_v_rms;
    filt_i_rms = alpha * raw_i_rms + (1.0f - alpha) * filt_i_rms;
    filt_power = alpha * raw_power + (1.0f - alpha) * filt_power;
    filt_pf = alpha * raw_pf + (1.0f - alpha) * filt_pf;

    g_Signal_Frequency = filt_freq;
    g_V_rms_real = filt_v_rms;
    g_I_rms_real = filt_i_rms;
    g_Active_Power = filt_power;
    g_Power_Factor = filt_pf;
}

void DMA_IRQHandler(void) {
    // 查一下到底是谁触发的中断
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH0: 
            ADC_Stop_Sampling();
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
    
    // ADC 序列的第二个结果进 FIFO 后，触发 DMA 搬走一组电压/电流采样值。
    DL_ADC12_disableConversions(ADC12_UI_INST);
    DL_ADC12_enableDMATrigger(ADC12_UI_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);
    ADC_Start_Sampling();
    
    while (1) {
        if (g_dataReady) {
            g_dataReady = false;

            Calculate_Power_And_RMS();
            
            VOFA_SendWaveform();

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
            
            ADC_Start_Sampling();
        }
        __WFI();
    }
}
