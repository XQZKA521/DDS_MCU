#include "bsp.h"
#include "delay.h"
#include "key.h"
#include "oled.h"

#define SERVO_MIN_ANGLE_DEG      0U
#define SERVO_MAX_ANGLE_DEG      180U
#define SERVO_DEFAULT_ANGLE_DEG  90U
/*
 * MG946R 官方页面给出了供电/扭矩等规格，但没有公开控制脉宽范围。
 * 这里先采用更保守的标准舵机测试范围 1000us~2000us，避免直接顶到机械极限。
 * 如果实测仍有偏差，可继续微调为 900/2100 或者缩小范围。
 */
#define SERVO_MIN_PULSE_US       1000U
#define SERVO_MAX_PULSE_US       2000U

#define STEP_MIN_DEG             1U
#define STEP_MAX_DEG             180U
#define STEP_DEFAULT_DEG         5U

#define INPUT_NONE               0xFFFFU

#define RFID_FRAME_STX           0x02U
#define RFID_FRAME_ETX           0x03U
#define RFID_PAYLOAD_LEN         12U
#define RFID_ID_LEN              10U
#define RFID_FRAME_TIMEOUT_TICKS 20U
#define RFID_RX_BUFFER_SIZE      64U
#define RFID_RAW_LOG_SIZE        240U
#define RFID_RAW_BYTES_PER_PAGE  15U
#define RFID_RAW_BYTES_PER_LINE  5U
#define LED8_BLINK_TICKS         5U
#define AUTO_CARD_ID             "0692CDEE24"
#define AUTO_HOLD_MS             3000U
#define AUTO_REARM_TICKS         100U

typedef enum {
    APP_MODE_SERVO = 0,
    APP_MODE_RFID,
    APP_MODE_AUTO
} app_mode_t;

static uint16_t servo_angle = SERVO_DEFAULT_ANGLE_DEG;
static uint16_t servo_step = STEP_DEFAULT_DEG;
static uint16_t input_value = INPUT_NONE;
static app_mode_t app_mode = APP_MODE_SERVO;
static char rfid_card_id[RFID_ID_LEN + 1U] = "----------";
static uint8_t rfid_payload[RFID_PAYLOAD_LEN];
static uint8_t rfid_rx_index = 0U;
static uint8_t rfid_rx_active = 0U;
static uint8_t rfid_idle_ticks = 0U;
static volatile uint8_t rfid_rx_buffer[RFID_RX_BUFFER_SIZE];
static volatile uint8_t rfid_rx_head = 0U;
static volatile uint8_t rfid_rx_tail = 0U;
static volatile uint8_t rfid_rx_overflow = 0U;
static volatile uint8_t led8_blink_ticks = 0U;
static volatile uint8_t rfid_uart_rx_seen = 0U;
static volatile uint8_t rfid_raw_log[RFID_RAW_LOG_SIZE];
static volatile uint16_t rfid_raw_len = 0U;
static volatile uint8_t rfid_raw_overflow = 0U;
static volatile uint8_t rfid_raw_updated = 0U;
static uint8_t rfid_raw_view = 0U;
static uint8_t rfid_raw_page = 0U;
static uint8_t auto_target_handled = 0U;
static uint16_t auto_idle_ticks = 0U;

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

static void oled_show_fixed2(uint8_t x, uint8_t y, uint8_t value)
{
    OLED_ShowChar(x, y, (value / 10U) % 10U + '0');
    OLED_ShowChar(x + 8U, y, value % 10U + '0');
}

static void oled_show_input(uint8_t x, uint8_t y, uint16_t value)
{
    if (value == INPUT_NONE) {
        OLED_ShowString(x, y, (u8 *) "---");
        return;
    }
    oled_show_fixed3(x, y, value);
}

static void servo_ui_refresh(void)
{
    uint16_t pulse_us = servo_angle_to_pulse_us(servo_angle);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *) "MODE: SERVO");

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

static void rfid_ui_refresh(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *) "MODE: RFID");
    OLED_ShowString(0, 2, (u8 *) "WAIT ID CARD");
    OLED_ShowString(0, 4, (u8 *) "ID:");
    OLED_ShowString(24, 4, (u8 *) rfid_card_id);
    OLED_ShowString(0, 6, (u8 *) "RX:");
    OLED_ShowChar(24, 6, (rfid_uart_rx_seen != 0U) ? 'Y' : 'N');
    OLED_ShowString(48, 6, (u8 *) "NEG MODE");
}

static void auto_ui_refresh(const char *status)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *) "MODE: AUTO");
    OLED_ShowString(0, 2, (u8 *) "WAIT 0692CDEE24");
    OLED_ShowString(0, 4, (u8 *) "ID:");
    OLED_ShowString(24, 4, (u8 *) rfid_card_id);
    OLED_ShowString(0, 6, (u8 *) (status != 0 ? status : "READY"));
}

static char hex_nibble_to_char(uint8_t value)
{
    value &= 0x0FU;
    if (value < 10U) {
        return (char) ('0' + value);
    }

    return (char) ('A' + value - 10U);
}

static uint8_t rfid_raw_page_count(void)
{
    uint16_t len = rfid_raw_len;
    uint16_t pages;

    if (len == 0U) {
        return 1U;
    }

    pages = (uint16_t) ((len + RFID_RAW_BYTES_PER_PAGE - 1U) /
        RFID_RAW_BYTES_PER_PAGE);
    if (pages > 99U) {
        return 99U;
    }

    return (uint8_t) pages;
}

static uint8_t rfid_raw_log_get(uint16_t index, uint8_t *byte)
{
    if (index >= rfid_raw_len) {
        return 0U;
    }

    __disable_irq();
    if (index >= rfid_raw_len) {
        __enable_irq();
        return 0U;
    }

    *byte = rfid_raw_log[index];
    __enable_irq();
    return 1U;
}

static void rfid_raw_ui_refresh(void)
{
    uint8_t line;
    uint8_t col;
    uint8_t page_count = rfid_raw_page_count();
    uint16_t base_index;

    if (rfid_raw_page >= page_count) {
        rfid_raw_page = (uint8_t) (page_count - 1U);
    }

    base_index = (uint16_t) rfid_raw_page * RFID_RAW_BYTES_PER_PAGE;

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *) "RAW ");
    oled_show_fixed2(32, 0, (uint8_t) (rfid_raw_page + 1U));
    OLED_ShowChar(48, 0, '/');
    oled_show_fixed2(56, 0, page_count);
    if (rfid_raw_overflow != 0U) {
        OLED_ShowString(80, 0, (u8 *) "OVF");
    }

    for (line = 0U; line < 3U; line++) {
        for (col = 0U; col < RFID_RAW_BYTES_PER_LINE; col++) {
            uint8_t byte;
            uint16_t index = (uint16_t) (base_index +
                (uint16_t) line * RFID_RAW_BYTES_PER_LINE + col);
            uint8_t x = (uint8_t) (col * 24U);
            uint8_t y = (uint8_t) (line * 2U + 2U);

            if (rfid_raw_log_get(index, &byte) == 0U) {
                return;
            }

            OLED_ShowChar(x, y, hex_nibble_to_char((uint8_t) (byte >> 4U)));
            OLED_ShowChar((uint8_t) (x + 8U), y, hex_nibble_to_char(byte));
            OLED_ShowChar((uint8_t) (x + 16U), y, ' ');
        }
    }
}

static void ui_refresh(void)
{
    if (app_mode == APP_MODE_SERVO) {
        servo_ui_refresh();
    } else if (app_mode == APP_MODE_AUTO) {
        auto_ui_refresh(0);
    } else if (rfid_raw_view != 0U) {
        rfid_raw_ui_refresh();
    } else {
        rfid_ui_refresh();
    }
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

static uint8_t rfid_is_ascii_hex(uint8_t value)
{
    return (((value >= '0') && (value <= '9')) ||
            ((value >= 'A') && (value <= 'F')) ||
            ((value >= 'a') && (value <= 'f')));
}

static void rfid_reset_parser(void)
{
    rfid_rx_index = 0U;
    rfid_rx_active = 0U;
    rfid_idle_ticks = 0U;
}

static void rfid_clear_uart_rx(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
        (void) DL_UART_Main_receiveData(UART_1_INST);
    }
}

static void rfid_clear_buffer(void)
{
    __disable_irq();
    rfid_rx_head = 0U;
    rfid_rx_tail = 0U;
    rfid_rx_overflow = 0U;
    __enable_irq();
}

static void rfid_clear_raw_log(void)
{
    __disable_irq();
    rfid_raw_len = 0U;
    rfid_raw_overflow = 0U;
    rfid_raw_updated = 0U;
    __enable_irq();
    rfid_raw_page = 0U;
}

static void rfid_uart_note_activity(void)
{
    rfid_uart_rx_seen = 1U;
    led8_blink_ticks = LED8_BLINK_TICKS;
    DL_GPIO_setPins(LED_LED8_PORT, LED_LED8_PIN);
}

static void rfid_raw_log_push(uint8_t byte)
{
    if (rfid_raw_len >= RFID_RAW_LOG_SIZE) {
        rfid_raw_overflow = 1U;
        return;
    }

    rfid_raw_log[rfid_raw_len] = byte;
    rfid_raw_len++;
    rfid_raw_updated = 1U;
}

static void rfid_rx_buffer_push(uint8_t byte)
{
    uint8_t next_head = (uint8_t) ((rfid_rx_head + 1U) % RFID_RX_BUFFER_SIZE);

    if (next_head == rfid_rx_tail) {
        rfid_rx_overflow = 1U;
        return;
    }

    rfid_rx_buffer[rfid_rx_head] = byte;
    rfid_rx_head = next_head;
}

static uint8_t rfid_rx_buffer_pop(uint8_t *byte)
{
    if (rfid_rx_head == rfid_rx_tail) {
        return 0U;
    }

    __disable_irq();
    if (rfid_rx_head == rfid_rx_tail) {
        __enable_irq();
        return 0U;
    }

    *byte = rfid_rx_buffer[rfid_rx_tail];
    rfid_rx_tail = (uint8_t) ((rfid_rx_tail + 1U) % RFID_RX_BUFFER_SIZE);
    __enable_irq();

    return 1U;
}

static void led8_task(void)
{
    if (led8_blink_ticks == 0U) {
        return;
    }

    led8_blink_ticks--;
    if (led8_blink_ticks == 0U) {
        DL_GPIO_clearPins(LED_LED8_PORT, LED_LED8_PIN);
    }
}

static void rfid_uart_init(void)
{
    rfid_clear_uart_rx();
    rfid_clear_buffer();
    rfid_clear_raw_log();
    DL_GPIO_clearPins(LED_LED8_PORT, LED_LED8_PIN);

    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(UART_1_INST, 1U);
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE |
        DL_UART_MAIN_INTERRUPT_RXD_NEG_EDGE |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

static uint8_t rfid_process_byte(uint8_t byte)
{
    uint8_t i;

    if (byte == RFID_FRAME_STX) {
        rfid_rx_active = 1U;
        rfid_rx_index = 0U;
        rfid_idle_ticks = 0U;
        return 0U;
    }

    if (rfid_rx_active == 0U) {
        return 0U;
    }

    rfid_idle_ticks = 0U;

    if (byte == RFID_FRAME_ETX) {
        if (rfid_rx_index == RFID_PAYLOAD_LEN) {
            for (i = 0U; i < RFID_ID_LEN; i++) {
                if (rfid_is_ascii_hex(rfid_payload[i]) == 0U) {
                    rfid_reset_parser();
                    return 0U;
                }
                rfid_card_id[i] = (char) rfid_payload[i];
            }
            rfid_card_id[RFID_ID_LEN] = '\0';
            rfid_reset_parser();
            return 1U;
        }

        rfid_reset_parser();
        return 0U;
    }

    if (rfid_rx_index >= RFID_PAYLOAD_LEN) {
        rfid_reset_parser();
        return 0U;
    }

    rfid_payload[rfid_rx_index] = byte;
    rfid_rx_index++;

    return 0U;
}

static uint8_t rfid_poll(void)
{
    uint8_t byte;
    uint8_t card_updated = 0U;

    if (rfid_rx_overflow != 0U) {
        rfid_rx_overflow = 0U;
        rfid_reset_parser();
    }

    while (rfid_rx_buffer_pop(&byte) != 0U) {
        if (rfid_process_byte(byte) != 0U) {
            card_updated = 1U;
        }
    }

    if (rfid_rx_active != 0U) {
        rfid_idle_ticks++;
        if (rfid_idle_ticks > RFID_FRAME_TIMEOUT_TICKS) {
            rfid_reset_parser();
        }
    }

    return card_updated;
}

static uint8_t rfid_card_id_is_auto_target(void)
{
    uint8_t i;

    for (i = 0U; i < RFID_ID_LEN; i++) {
        if (rfid_card_id[i] != AUTO_CARD_ID[i]) {
            return 0U;
        }
    }

    return 1U;
}

static void auto_run_servo_action(void)
{
    auto_ui_refresh("MATCH");
    servo_apply_angle(SERVO_MIN_ANGLE_DEG);
    delay_ms(100);
    servo_apply_angle(SERVO_MAX_ANGLE_DEG);
    delay_ms(AUTO_HOLD_MS);
    servo_apply_angle(SERVO_MIN_ANGLE_DEG);
    rfid_reset_parser();
    rfid_clear_uart_rx();
    rfid_clear_buffer();
    auto_ui_refresh("DONE");
}

static void app_toggle_mode(void)
{
    if (app_mode == APP_MODE_SERVO) {
        app_mode = APP_MODE_RFID;
        rfid_raw_view = 0U;
        rfid_reset_parser();
        rfid_clear_uart_rx();
        rfid_clear_buffer();
        rfid_clear_raw_log();
    } else if (app_mode == APP_MODE_RFID) {
        app_mode = APP_MODE_AUTO;
        rfid_raw_view = 0U;
        auto_target_handled = 0U;
        auto_idle_ticks = 0U;
        rfid_reset_parser();
        rfid_clear_uart_rx();
        rfid_clear_buffer();
        rfid_clear_raw_log();
    } else {
        app_mode = APP_MODE_SERVO;
        rfid_raw_view = 0U;
        auto_target_handled = 0U;
        auto_idle_ticks = 0U;
    }
}

static void handle_rfid_key_event(int key_value)
{
    uint8_t page_count = rfid_raw_page_count();

    switch (key_value) {
        case KEY_CHU:
            rfid_raw_view = 1U;
            if (rfid_raw_page >= page_count) {
                rfid_raw_page = (uint8_t) (page_count - 1U);
            }
            break;

        case KEY_PLUS:
            if (rfid_raw_view != 0U) {
                if ((uint8_t) (rfid_raw_page + 1U) < page_count) {
                    rfid_raw_page++;
                }
            }
            break;

        case KEY_MINUS:
            if (rfid_raw_view != 0U) {
                if (rfid_raw_page > 0U) {
                    rfid_raw_page--;
                }
            }
            break;

        default:
            break;
    }
}

static void handle_key_event(int key_value)
{
    if (key_value == KEY_NEGATE) {
        app_toggle_mode();
        return;
    }

    if (app_mode == APP_MODE_RFID) {
        handle_rfid_key_event(key_value);
        return;
    }

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

        default:
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    rfid_uart_init();
    OLED_Init();
    servo_init();
    ui_refresh();

    while (1) {
        int key_value = keypad_get_event();

        if (key_value != KEY_NONE) {
            handle_key_event(key_value);
            ui_refresh();
        }

        if (app_mode == APP_MODE_RFID) {
            if (rfid_poll() != 0U) {
                ui_refresh();
            } else if ((rfid_raw_view != 0U) && (rfid_raw_updated != 0U)) {
                rfid_raw_updated = 0U;
                rfid_raw_ui_refresh();
            }
        } else if (app_mode == APP_MODE_AUTO) {
            if (rfid_poll() != 0U) {
                auto_idle_ticks = 0U;
                if (rfid_card_id_is_auto_target() != 0U) {
                    if (auto_target_handled == 0U) {
                        auto_target_handled = 1U;
                        auto_run_servo_action();
                    }
                } else {
                    auto_target_handled = 0U;
                    auto_ui_refresh("NO MATCH");
                }
            } else if (auto_target_handled != 0U) {
                if (auto_idle_ticks < AUTO_REARM_TICKS) {
                    auto_idle_ticks++;
                } else {
                    auto_target_handled = 0U;
                    auto_idle_ticks = 0U;
                }
            }
        }

        led8_task();
        delay_ms(10);
    }
}

void UART4_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            rfid_uart_note_activity();
            while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
                uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);

                rfid_raw_log_push(byte);
                rfid_rx_buffer_push(byte);
            }
            break;

        case DL_UART_MAIN_IIDX_RXD_POS_EDGE:
        case DL_UART_MAIN_IIDX_RXD_NEG_EDGE:
            rfid_uart_note_activity();
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            rfid_uart_note_activity();
            while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
                (void) DL_UART_Main_receiveData(UART_1_INST);
            }
            rfid_rx_overflow = 1U;
            break;

        default:
            break;
    }
}
