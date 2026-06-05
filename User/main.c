#include "bsp.h"
#include "delay.h"
#include "key.h"
#include "oled.h"

#define SERVO_MIN_ANGLE_DEG      0U
#define SERVO_MAX_ANGLE_DEG      90U
#define SERVO_DEFAULT_ANGLE_DEG  45U
/*
 * MG946R 官方页面给出了供电/扭矩等规格，但没有公开控制脉宽范围。
 * 这里先采用更保守的标准舵机测试范围 1000us~2000us，避免直接顶到机械极限。
 * 如果实测仍有偏差，可继续微调为 900/2100 或者缩小范围。
 */
#define SERVO_MIN_PULSE_US       1000U
#define SERVO_MAX_PULSE_US       2000U

#define STEP_MIN_DEG             1U
#define STEP_MAX_DEG             90U
#define STEP_DEFAULT_DEG         5U

#define INPUT_NONE               0xFFFFU

static uint16_t servo_angle = SERVO_DEFAULT_ANGLE_DEG;
static uint16_t servo_step = STEP_DEFAULT_DEG;
static uint16_t input_value = INPUT_NONE;

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint16_t servo_angle_to_pulse_us(uint16_t angle_deg)
{
    uint32_t span = (uint32_t) (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return (uint16_t) (SERVO_MIN_PULSE_US +
        ((uint32_t) angle_deg * span) / SERVO_MAX_ANGLE_DEG);
}

static void servo_apply_angle(uint16_t angle_deg)
{
    uint16_t pulse_us;

    servo_angle = clamp_u16(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    pulse_us = servo_angle_to_pulse_us(servo_angle);
    DL_TimerG_setCaptureCompareValue(
        SERVO_PWM_INST, pulse_us, GPIO_SERVO_PWM_C0_IDX);
}

static void servo_init(void)
{
    servo_apply_angle(SERVO_DEFAULT_ANGLE_DEG);
    DL_TimerG_startCounter(SERVO_PWM_INST);
}

static void input_append_digit(uint8_t digit)
{
    if (digit > 9U) {
        return;
    }

    if (input_value == INPUT_NONE) {
        input_value = digit;
        return;
    }

    input_value = (uint16_t) (input_value * 10U + digit);
    if (input_value > 999U) {
        input_value = 999U;
    }
}

static void input_clear(void)
{
    input_value = INPUT_NONE;
}

static uint16_t input_or_default(uint16_t default_value)
{
    if (input_value == INPUT_NONE) {
        return default_value;
    }
    return input_value;
}

static void oled_show_fixed3(uint8_t x, uint8_t y, uint16_t value)
{
    OLED_ShowChar(x, y, (value / 100U) % 10U + '0');
    OLED_ShowChar(x + 8U, y, (value / 10U) % 10U + '0');
    OLED_ShowChar(x + 16U, y, value % 10U + '0');
}

static void oled_show_input(uint8_t x, uint8_t y, uint16_t value)
{
    if (value == INPUT_NONE) {
        OLED_ShowString(x, y, (u8 *) "---");
        return;
    }
    oled_show_fixed3(x, y, value);
}

static void ui_refresh(void)
{
    uint16_t pulse_us = servo_angle_to_pulse_us(servo_angle);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *) "MG946R TEST");

    OLED_ShowString(0, 2, (u8 *) "A:");
    oled_show_fixed3(16, 2, servo_angle);
    OLED_ShowString(48, 2, (u8 *) "S:");
    oled_show_fixed3(64, 2, servo_step);

    OLED_ShowString(0, 4, (u8 *) "I:");
    oled_show_input(16, 4, input_value);
    OLED_ShowString(48, 4, (u8 *) "P:");
    OLED_ShowNum(64, 4, pulse_us, 4, 16);

    OLED_ShowString(0, 6, (u8 *) "+-MV *S /A");
}

static int keypad_get_event(void)
{
    int key_value = getKeyValue();

    if (key_value == KEY_NONE) {
        return KEY_NONE;
    }

    delay_ms(20);
    if (getKeyValue() != key_value) {
        return KEY_NONE;
    }

    while (getKeyValue() != KEY_NONE) {
        delay_ms(10);
    }

    return key_value;
}

static void handle_key_event(int key_value)
{
    switch (key_value) {
        case KEY_DIGIT_0:
        case KEY_DIGIT_1:
        case KEY_DIGIT_2:
        case KEY_DIGIT_3:
        case KEY_DIGIT_4:
        case KEY_DIGIT_5:
        case KEY_DIGIT_6:
        case KEY_DIGIT_7:
        case KEY_DIGIT_8:
        case KEY_DIGIT_9:
            input_append_digit((uint8_t) key_value);
            break;

        case KEY_PLUS:
            servo_apply_angle((uint16_t) (servo_angle + servo_step));
            break;

        case KEY_MINUS:
            if (servo_angle > servo_step) {
                servo_apply_angle((uint16_t) (servo_angle - servo_step));
            } else {
                servo_apply_angle(SERVO_MIN_ANGLE_DEG);
            }
            break;

        case KEY_CHENG:
            servo_step = clamp_u16(input_or_default(servo_step), STEP_MIN_DEG, STEP_MAX_DEG);
            input_clear();
            break;

        case KEY_CHU:
            servo_apply_angle(clamp_u16(input_or_default(servo_angle),
                SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG));
            input_clear();
            break;

        case KEY_EQUAL:
            servo_apply_angle(SERVO_DEFAULT_ANGLE_DEG);
            input_clear();
            break;

        case KEY_NEGATE:
            if (input_value != INPUT_NONE) {
                input_clear();
            } else {
                servo_apply_angle(SERVO_MIN_ANGLE_DEG);
            }
            break;

        default:
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    servo_init();
    ui_refresh();

    while (1) {
        int key_value = keypad_get_event();

        if (key_value != KEY_NONE) {
            handle_key_event(key_value);
            ui_refresh();
        }

        delay_ms(10);
    }
}
