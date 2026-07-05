#include "ti_msp_dl_config.h"
#include "oled.h"

#define ADC_SAMPLE_RATE_HZ 6400.0f
// 1024 个电压采样点，采样窗口约 160ms，可覆盖 10Hz 以上信号的完整周期
#define SAMPLE_POINTS 1024
// 电压、电流交错存放，各 SAMPLE_POINTS 个点
#define BUFFER_SIZE (SAMPLE_POINTS * 2)
#define MIN_SIGNAL_VPP_CODE 20
#define MIN_EDGE_HYS_CODE 5
#define VOFA_SEND_POINTS 128
#define VOFA_SAMPLE_STRIDE (SAMPLE_POINTS / VOFA_SEND_POINTS)
#define ADC_REF_VOLTAGE 3.3f
#define ADC_FULL_SCALE_COUNTS 4096.0f
#define ADC_CHANNEL_COUNT 2
#define ADC_VOLTAGE_CH 0
#define ADC_CURRENT_CH 1
#define ADC_READY_UI 0x01u
#define ADC_READY_I 0x02u
#define ADC_READY_BOTH (ADC_READY_UI | ADC_READY_I)
#define CURRENT_SENSE_RESISTANCE_OHM 400.0f

#if (DMA_UI_CHAN_ID == 0)
#define DMA_UI_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL0
#define DMA_UI_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH0
#elif (DMA_UI_CHAN_ID == 1)
#define DMA_UI_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL1
#define DMA_UI_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH1
#elif (DMA_UI_CHAN_ID == 2)
#define DMA_UI_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL2
#define DMA_UI_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH2
#else
#error "Unsupported DMA_UI_CHAN_ID"
#endif

#define DMA_I_CHAN_ID DMA_CH1_CHAN_ID
#if (DMA_I_CHAN_ID == 0)
#define DMA_I_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL0
#define DMA_I_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH0
#elif (DMA_I_CHAN_ID == 1)
#define DMA_I_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL1
#define DMA_I_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH1
#elif (DMA_I_CHAN_ID == 2)
#define DMA_I_INTERRUPT_MASK DL_DMA_INTERRUPT_CHANNEL2
#define DMA_I_INTERRUPT_IIDX DL_DMA_EVENT_IIDX_DMACH2
#else
#error "Unsupported DMA_I_CHAN_ID"
#endif

// adc_buffer 由 ADC0 DMA 写入，adc_i_dma_buffer 由 ADC1 DMA 写入
// ADC12_I 现在和 ADC12_UI 一样是双通道序列采样，缓冲区大小相同
volatile uint16_t adc_buffer[BUFFER_SIZE];
volatile uint16_t adc_i_dma_buffer[BUFFER_SIZE];

volatile bool g_dataReady = false;
static volatile uint8_t g_adcReadyMask = 0;
static volatile uint8_t g_adcValidMask = 0;

typedef enum {
    ADC_ACTIVE_UI = 0,
    ADC_ACTIVE_I,
} ADC_Active;

static volatile ADC_Active g_activeAdc = ADC_ACTIVE_UI;


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

// 标定系数 (Calibration Factors)
// VOLT_FRONTEND_GAIN 是待测端到 ADC 引脚的反向比例：
// 直连 ADC 时为 1；若前端把 10V 缩放到 ADC 上 1V，则应填 10。
#define VOLT_FRONTEND_GAIN 0.7215386f
#define VOLT_CAL_FACTOR ((ADC_REF_VOLTAGE / ADC_FULL_SCALE_COUNTS) * VOLT_FRONTEND_GAIN)
// DEBUG: 暂不除以采样电阻，两个 ADC 使用完全相同的电压标定，先确认 ADC12_I 能正常采样
#define CURR_CAL_FACTOR VOLT_CAL_FACTOR

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

static void OLED_ShowWaiting(uint8_t y, const char *label)
{
    OLED_ShowString(0, y, (u8 *)"                ");
    OLED_ShowText(0, y, label);
    OLED_ShowText((uint8_t)(2 * 8), y, "Waiting");
}

static void OLED_ShowPowerAndRMSByMask(uint8_t ready_mask)
{
    if (ready_mask & ADC_READY_UI) {
        OLED_ShowFixed(0, 0, "U=", g_V_rms_display, 4, 3, "V");
    } else {
        OLED_ShowWaiting(0, "U=");
    }

    if (ready_mask & ADC_READY_I) {
        OLED_ShowFixed(0, 2, "I=", g_I_rms_display, 1, 6, "A");
    } else {
        OLED_ShowWaiting(2, "I=");
    }

    if ((ready_mask & ADC_READY_BOTH) == ADC_READY_BOTH) {
        OLED_ShowFixed(0, 4, "P=", g_Active_Power_display, 4, 2, "W");
        OLED_ShowFixed(0, 6, "PF=", g_Power_Factor_display, 1, 3, "");
    } else {
        OLED_ShowWaiting(4, "P=");
        OLED_ShowWaiting(6, "PF=");
    }
}

static void DMA_Reload_ADC_Buffer(void)
{
    DL_DMA_disableChannel(DMA, DMA_UI_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DMA_UI_INTERRUPT_MASK);
    DL_DMA_setTransferMode(DMA, DMA_UI_CHAN_ID, DL_DMA_SINGLE_TRANSFER_MODE);
    DL_DMA_setSrcAddr(DMA, DMA_UI_CHAN_ID,
        (uint32_t) DL_ADC12_getFIFOAddress(ADC12_UI_INST));
    DL_DMA_setDestAddr(DMA, DMA_UI_CHAN_ID, (uint32_t) &adc_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_UI_CHAN_ID, BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_UI_CHAN_ID);
}

// 完全照搬 DMA_Reload_ADC_Buffer 的逻辑
static void DMA_Reload_ADC_I_Buffer(void)
{
    DL_DMA_disableChannel(DMA, DMA_I_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DMA_I_INTERRUPT_MASK);
    DL_DMA_setTransferMode(DMA, DMA_I_CHAN_ID, DL_DMA_SINGLE_TRANSFER_MODE);
    DL_DMA_setSrcAddr(DMA, DMA_I_CHAN_ID,
        (uint32_t) DL_ADC12_getFIFOAddress(ADC12_I_INST));
    DL_DMA_setDestAddr(DMA, DMA_I_CHAN_ID, (uint32_t) &adc_i_dma_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_I_CHAN_ID, BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_I_CHAN_ID);
}

static uint16_t ADC_GetSampleCode(uint16_t sample_index, uint8_t channel)
{
    if (channel == ADC_CURRENT_CH) {
        // ADC12_I 双通道序列采样，取 MEM0 数据（和 ADC12_UI 的 CH0 读取方式一致）
        return adc_i_dma_buffer[sample_index * ADC_CHANNEL_COUNT + 0];
    }

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

static void ADC_Stop_UI_Sampling(void)
{
    DL_ADC12_stopConversion(ADC12_UI_INST);
    DL_ADC12_disableDMA(ADC12_UI_INST);
    DL_ADC12_disableConversions(ADC12_UI_INST);
}

static void ADC_Stop_I_Sampling(void)
{
    DL_ADC12_stopConversion(ADC12_I_INST);
    DL_ADC12_disableDMA(ADC12_I_INST);
    DL_ADC12_disableConversions(ADC12_I_INST);
}

static void ADC_Start_Sampling(void)
{
    g_adcReadyMask = 0;
    DL_TimerA_stopCounter(TIMER_TIMG_INST);
    ADC_Stop_UI_Sampling();
    ADC_Stop_I_Sampling();

    DL_TimerA_setTimerCount(TIMER_TIMG_INST, 0);

    if (g_activeAdc == ADC_ACTIVE_UI) {
        DL_ADC12_disableFIFO(ADC12_UI_INST);
        DL_ADC12_enableFIFO(ADC12_UI_INST);
        DL_ADC12_clearDMATriggerStatus(ADC12_UI_INST,
            DL_ADC12_DMA_MEM1_RESULT_LOADED);
        DMA_Reload_ADC_Buffer();
        DL_ADC12_enableDMA(ADC12_UI_INST);
        DL_ADC12_enableConversions(ADC12_UI_INST);
        DL_ADC12_startConversion(ADC12_UI_INST);
    } else {
        DL_ADC12_disableFIFO(ADC12_I_INST);
        DL_ADC12_enableFIFO(ADC12_I_INST);
        DL_ADC12_clearDMATriggerStatus(ADC12_I_INST,
            DL_ADC12_DMA_MEM1_RESULT_LOADED);
        DMA_Reload_ADC_I_Buffer();
        DL_ADC12_enableDMA(ADC12_I_INST);
        DL_ADC12_enableConversions(ADC12_I_INST);
        DL_ADC12_startConversion(ADC12_I_INST);
    }

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
    for (uint16_t n = 0; n < SAMPLE_POINTS; n += VOFA_SAMPLE_STRIDE) {
        UART0_SendUint(ADC_GetSampleCode(n, ADC_VOLTAGE_CH));
        UART0_SendChar(',');
        UART0_SendUint(ADC_GetSampleCode(n, ADC_CURRENT_CH));
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

void Calculate_Power_And_RMS(uint8_t ready_mask) {
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
    float v_dc_offset = 0.0f;
    float i_dc_offset = 0.0f;
    float power_v_dc_offset;
    float power_i_dc_offset;
    float p_sum = 0.0f;    // 瞬时功率乘积累加和
    float v_rms_adc = 0.0f;
    float i_rms_adc = 0.0f;
    float p_adc;
    float raw_v_rms = filt_v_rms;
    float raw_i_rms = filt_i_rms;
    float raw_power;
    float raw_pf;
    float apparent_power;
    float alpha = 1.0f;
    bool voltage_ready = ((ready_mask & ADC_READY_UI) != 0);
    bool current_ready = ((ready_mask & ADC_READY_I) != 0);

    if (voltage_ready) {
        Find_ADC_Cycle_Window(ADC_VOLTAGE_CH, &v_start_sample, &v_end_sample,
            &v_dc_offset, &v_vpp_code, &voltage_frequency);
        if (v_end_sample <= v_start_sample) {
            voltage_ready = false;
        } else {
            v_dc_offset = ADC_CalculateWindowDC(ADC_VOLTAGE_CH, v_start_sample,
                v_end_sample);
            v_rms_adc = ADC_CalculateRmsCode(ADC_VOLTAGE_CH, v_start_sample,
                v_end_sample, v_dc_offset);
            raw_v_rms = v_rms_adc * VOLT_CAL_FACTOR;

            if ((filt_freq < 1.0f) ||
                (fabs(voltage_frequency - filt_freq) > filt_freq * 0.10f) ||
                (fabs(raw_v_rms - filt_v_rms) > filt_v_rms * 0.10f)) {
                alpha = 1.0f;
            } else {
                alpha = 0.20f;
            }

            filt_freq = alpha * voltage_frequency + (1.0f - alpha) * filt_freq;
            filt_v_rms = alpha * raw_v_rms + (1.0f - alpha) * filt_v_rms;
            g_Signal_Frequency = filt_freq;
            g_V_rms_real = filt_v_rms;
        }
    }

    if (current_ready) {
        Find_ADC_Cycle_Window(ADC_CURRENT_CH, &i_start_sample, &i_end_sample,
            &i_dc_offset, &i_vpp_code, &current_frequency);
        if (i_end_sample <= i_start_sample) {
            current_ready = false;
        } else {
            i_dc_offset = ADC_CalculateWindowDC(ADC_CURRENT_CH, i_start_sample,
                i_end_sample);
            i_rms_adc = ADC_CalculateRmsCode(ADC_CURRENT_CH, i_start_sample,
                i_end_sample, i_dc_offset);
            raw_i_rms = i_rms_adc * CURR_CAL_FACTOR;

            if ((filt_i_rms < 0.000001f) ||
                (fabs(raw_i_rms - filt_i_rms) > filt_i_rms * 0.10f)) {
                alpha = 1.0f;
            } else {
                alpha = 0.20f;
            }

            filt_i_rms = alpha * raw_i_rms + (1.0f - alpha) * filt_i_rms;
            g_I_rms_real = filt_i_rms;
        }
    }

    if (voltage_ready && current_ready) {
        power_points = v_end_sample - v_start_sample;
        power_v_dc_offset = ADC_CalculateWindowDC(ADC_VOLTAGE_CH, v_start_sample,
            v_end_sample);
        power_i_dc_offset = ADC_CalculateWindowDC(ADC_CURRENT_CH, v_start_sample,
            v_end_sample);

        for (uint16_t n = v_start_sample; n < v_end_sample; n++) {
            float v_ac = (float)ADC_GetSampleCode(n, ADC_VOLTAGE_CH) - power_v_dc_offset;
            float i_ac = (float)ADC_GetSampleCode(n, ADC_CURRENT_CH) - power_i_dc_offset;

            p_sum += (v_ac * i_ac);
        }

        p_adc = p_sum / power_points;
        raw_power = p_adc * VOLT_CAL_FACTOR * CURR_CAL_FACTOR;
        apparent_power = raw_v_rms * raw_i_rms;

        if (apparent_power > 0.001f) {
            raw_pf = raw_power / apparent_power;
        } else {
            raw_pf = 0.0f;
        }

        if ((filt_power < 0.001f) ||
            (fabs(raw_power - filt_power) > fabs(filt_power) * 0.10f)) {
            alpha = 1.0f;
        } else {
            alpha = 0.20f;
        }

        filt_power = alpha * raw_power + (1.0f - alpha) * filt_power;
        filt_pf = alpha * raw_pf + (1.0f - alpha) * filt_pf;
        g_Active_Power = filt_power;
        g_Power_Factor = filt_pf;
    }
}

void DMA_IRQHandler(void) {
    bool hasPendingInterrupt = true;
    uint8_t ready_mask = 0;

    while (hasPendingInterrupt) {
        switch (DL_DMA_getPendingInterrupt(DMA)) {
            case DMA_UI_INTERRUPT_IIDX:
                DL_DMA_clearInterruptStatus(DMA, DMA_UI_INTERRUPT_MASK);
                ready_mask |= ADC_READY_UI;
                break;
            case DMA_I_INTERRUPT_IIDX:
                DL_DMA_clearInterruptStatus(DMA, DMA_I_INTERRUPT_MASK);
                ready_mask |= ADC_READY_I;
                break;
            default:
                hasPendingInterrupt = false;
                break;
        }
    }

    if (ready_mask != 0) {
        if ((ready_mask & ADC_READY_UI) && (g_activeAdc == ADC_ACTIVE_UI)) {
            ADC_Stop_UI_Sampling();
            g_adcReadyMask = ADC_READY_UI;
            g_adcValidMask |= ADC_READY_UI;
            DL_TimerA_stopCounter(TIMER_TIMG_INST);
            g_dataReady = true;
        }
        if ((ready_mask & ADC_READY_I) && (g_activeAdc == ADC_ACTIVE_I)) {
            ADC_Stop_I_Sampling();
            g_adcReadyMask = ADC_READY_I;
            g_adcValidMask |= ADC_READY_I;
            DL_TimerA_stopCounter(TIMER_TIMG_INST);
            g_dataReady = true;
        }
    }
}

void ADC0_IRQHandler(void)
{
    // ADC0 结果由 DMA 从 FIFO 搬运，这里不做软件取样。
}

void ADC1_IRQHandler(void)
{
    // ADC1 结果由 DMA 从 FIFO 搬运，这里不做软件取样。
}

int main(void) {
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ShowString(0, 0, (u8 *)"Power Meter");
    OLED_ShowString(0, 2, (u8 *)"Waiting ADC...");

    // 使能 DMA 中断（ADC0 和 ADC1 的 DMA 共用 DMA_INT_IRQn）
    NVIC_EnableIRQ(DMA_INT_IRQn);

    // 两个 ADC 的 SysConfig 初始化已开启转换，此处先关闭
    // 等 DMA 配置就绪后，在 ADC_Start_Sampling() 中统一开启
    DL_ADC12_disableConversions(ADC12_UI_INST);
    DL_ADC12_disableConversions(ADC12_I_INST);

    // ADC12_UI: DMA 在 MEM1 结果就绪后触发（双通道序列的第二个转换完成）
    DL_ADC12_enableDMATrigger(ADC12_UI_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);
    // ADC12_I: 完全照搬 ADC12_UI，DMA 在 MEM1 结果就绪后触发
    DL_ADC12_enableDMATrigger(ADC12_I_INST, DL_ADC12_DMA_MEM1_RESULT_LOADED);

    ADC_Start_Sampling();

    while (1) {
        if (g_dataReady) {
            uint8_t ready_mask = g_adcReadyMask;
            uint8_t valid_mask = g_adcValidMask;
            ADC_Active completed_adc = g_activeAdc;

            g_dataReady = false;

            Calculate_Power_And_RMS(
                ((valid_mask & ADC_READY_BOTH) == ADC_READY_BOTH) ?
                    ADC_READY_BOTH : ready_mask);

            if (ready_mask & ADC_READY_UI) {
                g_V_rms_display = g_V_rms_real;
            }
            if (ready_mask & ADC_READY_I) {
                g_I_rms_display = g_I_rms_real / CURRENT_SENSE_RESISTANCE_OHM;
            }
            if ((valid_mask & ADC_READY_BOTH) == ADC_READY_BOTH) {
                g_Active_Power_display = g_Active_Power;
                g_Power_Factor_display = g_Power_Factor;
                VOFA_SendWaveform();
            }

            OLED_ShowPowerAndRMSByMask(valid_mask);
            g_activeAdc = (completed_adc == ADC_ACTIVE_UI) ?
                ADC_ACTIVE_I : ADC_ACTIVE_UI;
            ADC_Start_Sampling();
        }
        __WFI();
    }
}
