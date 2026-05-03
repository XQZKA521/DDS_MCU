#include "ti_msp_dl_config.h"
#include "key.h"
#include "oled.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti/driverlib/dl_dac12.h"

#include <stdint.h>
#include <stdbool.h>


extern uint8_t OLED_GRAM[128][8];

/* TIMA0 的 SysConfig 周期是 0.125ms，所以采样率是 8000Hz。 */
#define SAMPLE_RATE_HZ                  (8000U)
#define SAMPLE_BUFFER_COUNT             (2U)
#define SAMPLE_BUFFER_SIZE              (8000U)

/*
 * 这里把采样数据放在 512KB 主 Flash 的高 256KB 区域，按连续数据追加。
 * 如果工程以后代码超过 256KB，需要在 scatter/linker 里把这段区域保留出来。
 */
#define FLASH_STORAGE_START_ADDRESS     (0x00040000U)
#define FLASH_STORAGE_BYTES             (0x00040000U)
#define FLASH_STORAGE_END_ADDRESS       (FLASH_STORAGE_START_ADDRESS + FLASH_STORAGE_BYTES)

/*
 * Flash 开头只放一个总头部，后面全部是连续的 16 位 ADC 数据。
 * word0 = magic，word1 = 采样率，word2 = 总采样点数，word3 = 完成标志。
 * word2/word3 在停止采样并保存完最后一个 buffer 后才写入。
 */
#define FLASH_STORAGE_MAGIC             (0x53414D50U)
#define FLASH_STORAGE_DONE_MAGIC        (0x444F4E45U)
#define FLASH_STORAGE_HEADER_WORDS      (4U)
#define FLASH_STORAGE_HEADER_BYTES      (FLASH_STORAGE_HEADER_WORDS * sizeof(uint32_t))
#define FLASH_DATA_START_ADDRESS        (FLASH_STORAGE_START_ADDRESS + FLASH_STORAGE_HEADER_BYTES)

/* 每个 32 位 word 打包两个 16 位 ADC 采样值。 */
#define FLASH_SAMPLE_WORDS              ((SAMPLE_BUFFER_SIZE + 1U) / 2U)
#define FLASH_IMAGE_MAX_WORDS           (FLASH_SAMPLE_WORDS + 1U)

typedef enum {
    OLED_STATE_READY = 0,
    OLED_STATE_ERASING,
    OLED_STATE_SAMPLING,
    OLED_STATE_SAVING,
    OLED_STATE_SAVED,
    OLED_STATE_OVERFLOW,
    OLED_STATE_FLASH_ERROR, 
    OLED_STATE_PLAYING,     // 新增：播放中
    OLED_STATE_PLAY_DONE,   // 新增：播放完成
    OLED_STATE_NO_DATA      // 新增：Flash中没有有效数据
} OLED_State;

typedef enum {
    BUFFER_STATE_EMPTY = 0,
    BUFFER_STATE_FILLING,
    BUFFER_STATE_READY,
    BUFFER_STATE_SAVING
} BufferState;

/* DAC 播放相关变量 */
static volatile bool g_isPlaying = false;
static volatile uint32_t g_playTotalCount = 0;
static volatile uint32_t g_playCurrentCount = 0;

/* 利用 uint16_t 指针直接访问 Flash，起始地址为 Header 之后的 0x00040010 */
static uint16_t *g_flashPlayPtr = (uint16_t *)FLASH_DATA_START_ADDRESS;

/* 这些变量会在主循环和定时器中断之间共享，所以用 volatile。 */
static volatile bool g_isSampling = false;
static volatile bool g_stopSaveRequest = false;
static volatile bool g_flashError = false;
static volatile bool g_overflowError = false;
static volatile uint32_t g_totalSampleCount = 0;

static volatile uint32_t g_activeBuffer = 0;
static volatile uint32_t g_activeIndex = 0;
static volatile BufferState g_bufferState[SAMPLE_BUFFER_COUNT];
static volatile uint32_t g_bufferSampleCount[SAMPLE_BUFFER_COUNT];
static volatile uint16_t g_sampleBuffer[SAMPLE_BUFFER_COUNT][SAMPLE_BUFFER_SIZE];

/* Flash 写入 API 按 32 位数据写入，先在 RAM 里打包好一个 buffer。 */
static uint32_t g_flashImage[FLASH_IMAGE_MAX_WORDS];
static uint32_t g_flashWriteAddress = FLASH_DATA_START_ADDRESS;
static uint32_t g_savedSampleCount = 0;

static void OLED_ShowStatus(OLED_State state, uint32_t sampleCount);
static bool Sampling_Start(void);
static void Sampling_Stop(bool requestSave);
static void Sampling_Abort(void);
static uint16_t ADC12_ReadOnce(void);
static bool Flash_EraseStorage(void);
static bool Flash_WriteStartHeader(void);
static bool Flash_FinalizeStorage(void);
static bool Flash_SaveBuffer(uint32_t bufferIndex);
static uint32_t Flash_BuildSampleImage(uint32_t bufferIndex);
static int32_t Buffer_TakeReadyForSave(void);
static void Buffer_ReleaseAfterSave(uint32_t bufferIndex);
static bool Buffers_HavePendingSave(void);

static bool Playback_Start(void);
static void Playback_Stop(void);

int main(void)
{
    bool key1PressedLast = false;
    bool key2PressedLast = false;
    OLED_State oledState = OLED_STATE_READY;
    uint32_t lastDisplayCount = 0xFFFFFFFFU;

    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_TimerA_stopCounter(TIMER_0_INST);

    OLED_ShowStatus(OLED_STATE_READY, 0);

    while (1) {
        int32_t bufferToSave = Buffer_TakeReadyForSave();

        /*
         * 优先保存已经满的 buffer，避免另一个 buffer 再满时没有空 buffer 可切。
         * Flash 写入时不整体关中断，TIMA0 中断仍然可以继续往另一个 buffer 采样。
         */
        if (bufferToSave >= 0) {
            oledState = OLED_STATE_SAVING;
            OLED_ShowStatus(oledState, g_savedSampleCount);

            if (Flash_SaveBuffer((uint32_t) bufferToSave)) {
                Buffer_ReleaseAfterSave((uint32_t) bufferToSave);
            } else {
                g_flashError = true;
                Sampling_Abort();
                Buffer_ReleaseAfterSave((uint32_t) bufferToSave);
                OLED_ShowStatus(OLED_STATE_FLASH_ERROR, g_savedSampleCount);
            }
        }

        if (g_overflowError) {
            OLED_ShowStatus(OLED_STATE_OVERFLOW, g_totalSampleCount);
        } else if (g_flashError) {
            OLED_ShowStatus(OLED_STATE_FLASH_ERROR, g_savedSampleCount);
        } else if (g_stopSaveRequest && !Buffers_HavePendingSave()) {
            if (Flash_FinalizeStorage()) {
                g_stopSaveRequest = false;
                oledState = OLED_STATE_SAVED;
                OLED_ShowStatus(oledState, g_savedSampleCount);
            } else {
                g_flashError = true;
                OLED_ShowStatus(OLED_STATE_FLASH_ERROR, g_savedSampleCount);
            }
        }

        {
            int keyValue = getKeyValue();
            bool key1Pressed = (keyValue == 1);
            bool key2Pressed = (keyValue == 2); // 新增按键 2

            /* 对按键 1 做上升沿检测：第一次开始，再按一次停止。 */
            if (key1Pressed && !key1PressedLast && !g_flashError) {
                if (g_isSampling) {
                    Sampling_Stop(true);
                    oledState = OLED_STATE_SAVING;
                    OLED_ShowStatus(oledState, g_totalSampleCount);
                } else if (!Buffers_HavePendingSave()) {
                    OLED_ShowStatus(OLED_STATE_ERASING, 0);
                    if (Sampling_Start()) {
                        oledState = OLED_STATE_SAMPLING;
                        lastDisplayCount = 0xFFFFFFFFU;
                        OLED_ShowStatus(oledState, 0);
                    } else {
                        g_flashError = true;
                        OLED_ShowStatus(OLED_STATE_FLASH_ERROR, 0);
                    }
                }
            }
            key1PressedLast = key1Pressed;

            /* 新增：按键 2 (DAC播放开始/停止) */
             if (key2Pressed && !key2PressedLast && !g_isSampling && !Buffers_HavePendingSave()) {
                 if (g_isPlaying) {
                Playback_Stop();
                oledState = OLED_STATE_READY;
                OLED_ShowStatus(oledState, g_playCurrentCount);
                 } 
                else {
                if (Playback_Start()) {
                    oledState = OLED_STATE_PLAYING;
                    OLED_ShowStatus(oledState, g_playTotalCount);
                    lastDisplayCount = 0xFFFFFFFFU;
                } else {
                    oledState = OLED_STATE_NO_DATA;
                    OLED_ShowStatus(oledState, 0);
                }
            }
            }
            key2PressedLast = key2Pressed;   
        }

        if (g_isSampling) {
            uint32_t currentCount = g_totalSampleCount;

            /* 采样中每秒刷新一次计数，减少 OLED 刷屏对主循环的影响。 */
            if ((currentCount != lastDisplayCount) &&
                ((currentCount % SAMPLE_RATE_HZ) == 0U)) {
                lastDisplayCount = currentCount;
                OLED_ShowStatus(OLED_STATE_SAMPLING, currentCount);
            }
        }
/* 播放进度 OLED 刷新逻辑 (每秒刷新一次，不阻塞主循环) */
        if (g_isPlaying) {
        uint32_t currentPlay = g_playCurrentCount;
        if ((currentPlay != lastDisplayCount) && ((currentPlay % SAMPLE_RATE_HZ) == 0U)) {
            lastDisplayCount = currentPlay;
            OLED_ShowStatus(OLED_STATE_PLAYING, currentPlay);
        }
         } else if (oledState == OLED_STATE_PLAYING) {
        /* 如果中断里自然结束了播放，更新 OLED 状态 */
        oledState = OLED_STATE_PLAY_DONE;
        OLED_ShowStatus(oledState, g_playTotalCount);
          }

        delay_cycles(20000);
    }
}

static void OLED_ShowStatus(OLED_State state, uint32_t sampleCount)
{
    OLED_Clear();

    switch (state) {
        case OLED_STATE_READY:
            OLED_ShowString(0, 0, (u8 *) "Ready");
            OLED_ShowString(0, 2, (u8 *) "Key1 Start");
            break;
        case OLED_STATE_ERASING:
            OLED_ShowString(0, 0, (u8 *) "Erase Flash");
            OLED_ShowString(0, 2, (u8 *) "Please Wait");
            break;
        case OLED_STATE_SAMPLING:
            OLED_ShowString(0, 0, (u8 *) "Sampling");
            OLED_ShowString(0, 2, (u8 *) "Key1 Stop");
            break;
        case OLED_STATE_SAVING:
            OLED_ShowString(0, 0, (u8 *) "Saving");
            break;
        case OLED_STATE_SAVED:
            OLED_ShowString(0, 0, (u8 *) "Saved");
            break;
        case OLED_STATE_OVERFLOW:
            OLED_ShowString(0, 0, (u8 *) "Buf Over");
            break;
        case OLED_STATE_FLASH_ERROR:
            OLED_ShowString(0, 0, (u8 *) "Flash Err");
            break;
        default:
            OLED_ShowString(0, 0, (u8 *) "Unknown");
            break;

        case OLED_STATE_PLAYING:
            OLED_ShowString(0, 0, (u8 *) "Playing DAC ");
            OLED_ShowString(0, 2, (u8 *) "Key2 Stop   ");
            break;
        case OLED_STATE_PLAY_DONE:
            OLED_ShowString(0, 0, (u8 *) "Play Done   ");
            OLED_ShowString(0, 2, (u8 *) "            ");
            break;
        case OLED_STATE_NO_DATA:
            OLED_ShowString(0, 0, (u8 *) "No ValidData");
            OLED_ShowString(0, 2, (u8 *) "            ");
            break;
    }

    OLED_ShowString(0, 4, (u8 *) "Cnt:");
    OLED_ShowNum(40, 4, sampleCount, 8, 16);
}

static bool Sampling_Start(void)
{
    uint32_t i;

    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    /* 开始新一轮采样前先擦除整段存储区，后续 ADC 数据就可以连续追加。 */
    if (!Flash_EraseStorage()) {
        return false;
    }
    if (!Flash_WriteStartHeader()) {
        return false;
    }

    __disable_irq();
    for (i = 0; i < SAMPLE_BUFFER_COUNT; i++) {
        g_bufferState[i] = BUFFER_STATE_EMPTY;
        g_bufferSampleCount[i] = 0;
    }

    g_activeBuffer = 0;
    g_activeIndex = 0;
    g_bufferState[0] = BUFFER_STATE_FILLING;

    g_totalSampleCount = 0;
    g_savedSampleCount = 0;
    g_flashWriteAddress = FLASH_DATA_START_ADDRESS;
    g_stopSaveRequest = false;
    g_flashError = false;
    g_overflowError = false;
    g_isSampling = true;

    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_TimerA_startCounter(TIMER_0_INST);
    __enable_irq();

    return true;
}

static void Sampling_Stop(bool requestSave)
{
    __disable_irq();
    g_isSampling = false;
    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    if (requestSave) {
        uint32_t buffer = g_activeBuffer;

        if ((g_bufferState[buffer] == BUFFER_STATE_FILLING) &&
            (g_activeIndex > 0U)) {
            g_bufferSampleCount[buffer] = g_activeIndex;
            g_bufferState[buffer] = BUFFER_STATE_READY;
            g_activeIndex = 0;
        }

        g_stopSaveRequest = true;
    }
    __enable_irq();
}

static void Sampling_Abort(void)
{
    __disable_irq();
    g_isSampling = false;
    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    __enable_irq();
}

static uint16_t ADC12_ReadOnce(void)
{
    uint32_t timeout = 1000U;

    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(ADC12_0_INST);

    while ((DL_ADC12_getRawInterruptStatus(ADC12_0_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U)) {
        timeout--;
    }

    DL_ADC12_stopConversion(ADC12_0_INST);

    return DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);
}

static bool Flash_EraseStorage(void)
{
    uint32_t address = FLASH_STORAGE_START_ADDRESS;

    while (address < FLASH_STORAGE_END_ADDRESS) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(
            FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);

        if (DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, address,
                DL_FLASHCTL_COMMAND_SIZE_SECTOR) !=
            DL_FLASHCTL_COMMAND_STATUS_PASSED) {
            return false;
        }

        address += DL_FLASHCTL_SECTOR_SIZE;
    }

    return true;
}

static bool Flash_WriteStartHeader(void)
{
    uint32_t header[2];
    DL_FLASHCTL_COMMAND_STATUS status;

    header[0] = FLASH_STORAGE_MAGIC;
    header[1] = SAMPLE_RATE_HZ;

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, FLASH_STORAGE_START_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);

    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, FLASH_STORAGE_START_ADDRESS, header, 2,
        DL_FLASHCTL_REGION_SELECT_MAIN);

    return (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
}

static bool Flash_FinalizeStorage(void)
{
    uint32_t finish[2];
    DL_FLASHCTL_COMMAND_STATUS status;

    finish[0] = g_savedSampleCount;
    finish[1] = FLASH_STORAGE_DONE_MAGIC;

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL,
        FLASH_STORAGE_START_ADDRESS + (2U * sizeof(uint32_t)),
        DL_FLASHCTL_REGION_SELECT_MAIN);

    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, FLASH_STORAGE_START_ADDRESS + (2U * sizeof(uint32_t)),
        finish, 2, DL_FLASHCTL_REGION_SELECT_MAIN);

    return (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
}

static bool Flash_SaveBuffer(uint32_t bufferIndex)
{
    uint32_t imageWords = Flash_BuildSampleImage(bufferIndex);
    uint32_t imageBytes = imageWords * sizeof(uint32_t);
    DL_FLASHCTL_COMMAND_STATUS status;

    if ((imageWords == 0U) ||
        ((g_flashWriteAddress + imageBytes) > FLASH_STORAGE_END_ADDRESS)) {
        return false;
    }

    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, g_flashWriteAddress, g_flashImage, imageWords,
        DL_FLASHCTL_REGION_SELECT_MAIN);

    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return false;
    }

    g_flashWriteAddress += imageBytes;
    g_savedSampleCount += g_bufferSampleCount[bufferIndex];

    return true;
}

/*
 * Flash 数据区格式：
 * 从 FLASH_DATA_START_ADDRESS 开始，所有 ADC 值按 16 位连续排列；
 * 这里打包成 32 位 word 只是为了配合 Flash 写入 API。
 */
static uint32_t Flash_BuildSampleImage(uint32_t bufferIndex)
{
    uint32_t i;
    uint32_t sampleCount = g_bufferSampleCount[bufferIndex];
    uint32_t sampleWords = (sampleCount + 1U) / 2U;
    uint32_t imageWords = sampleWords;

    if ((bufferIndex >= SAMPLE_BUFFER_COUNT) ||
        (sampleCount == 0U) ||
        (sampleCount > SAMPLE_BUFFER_SIZE) ||
        (imageWords > FLASH_IMAGE_MAX_WORDS)) {
        return 0;
    }

    for (i = 0; i < sampleWords; i++) {
        uint32_t index = i * 2U;
        uint32_t low = g_sampleBuffer[bufferIndex][index];
        uint32_t high = 0xFFFFU;

        if ((index + 1U) < sampleCount) {
            high = g_sampleBuffer[bufferIndex][index + 1U];
        }

        g_flashImage[i] = low | (high << 16);
    }

    /* 64-bit Flash 写入要求 32 位 word 数为偶数，不足时补 0xFFFFFFFF。 */
    if ((imageWords & 1U) != 0U) {
        g_flashImage[imageWords] = 0xFFFFFFFFU;
        imageWords++;
    }

    return imageWords;
}

static int32_t Buffer_TakeReadyForSave(void)
{
    uint32_t i;
    int32_t result = -1;

    __disable_irq();
    for (i = 0; i < SAMPLE_BUFFER_COUNT; i++) {
        if (g_bufferState[i] == BUFFER_STATE_READY) {
            g_bufferState[i] = BUFFER_STATE_SAVING;
            result = (int32_t) i;
            break;
        }
    }
    __enable_irq();

    return result;
}

static void Buffer_ReleaseAfterSave(uint32_t bufferIndex)
{
    if (bufferIndex >= SAMPLE_BUFFER_COUNT) {
        return;
    }

    __disable_irq();
    g_bufferSampleCount[bufferIndex] = 0;
    /*
     * 不整块 memset 数组，清空有效长度和状态即可。
     * 下一次填充会覆盖旧值，这样不会浪费正在采样时的主循环时间。
     */
    g_bufferState[bufferIndex] = BUFFER_STATE_EMPTY;
    __enable_irq();
}

static bool Buffers_HavePendingSave(void)
{
    bool pending = false;
    uint32_t i;

    __disable_irq();
    for (i = 0; i < SAMPLE_BUFFER_COUNT; i++) {
        if ((g_bufferState[i] == BUFFER_STATE_READY) ||
            (g_bufferState[i] == BUFFER_STATE_SAVING)) {
            pending = true;
            break;
        }
    }
    __enable_irq();

    return pending;
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            if (g_isSampling) {
                uint32_t buffer = g_activeBuffer;
                uint32_t index = g_activeIndex;

                if (index < SAMPLE_BUFFER_SIZE) {
                    g_sampleBuffer[buffer][index] = ADC12_ReadOnce();
                    index++;
                    g_activeIndex = index;
                    g_totalSampleCount++;
                }

                if (index >= SAMPLE_BUFFER_SIZE) {
                    uint32_t nextBuffer = buffer ^ 1U;

                    g_bufferSampleCount[buffer] = index;
                    g_bufferState[buffer] = BUFFER_STATE_READY;

                    if (g_bufferState[nextBuffer] == BUFFER_STATE_EMPTY) {
                        g_activeBuffer = nextBuffer;
                        g_activeIndex = 0;
                        g_bufferSampleCount[nextBuffer] = 0;
                        g_bufferState[nextBuffer] = BUFFER_STATE_FILLING;
                    } else {
                        /*
                         * 两个 buffer 都不可用，说明 Flash 保存速度追不上采样速度。
                         * 停止采样，避免覆盖还没保存的数据。
                         */
                        g_isSampling = false;
                        g_overflowError = true;
                        DL_TimerA_stopCounter(TIMER_0_INST);
                    }
                }
            }
            else if (g_isPlaying) {
                /* 播放逻辑：每次中断直接输出一个样本 */
                if (g_playCurrentCount < g_playTotalCount) {
                    /* 直接读取 16 位 Flash 数据，指针自增 */
                    uint16_t sample = *g_flashPlayPtr++;
                    
                    /* 屏蔽高 4 位，只保留 12 位有效数据，写入 DAC 寄存器 */
                    DL_DAC12_output12(DAC0, (sample & 0x0FFF));
                    
                    g_playCurrentCount++;
                }
                else {
                    /* 数据播放完毕，自动停止 */
                    g_isPlaying = false;
                    DL_TimerA_stopCounter(TIMER_0_INST);
                    DL_DAC12_disable(DAC0);
                }
            }
            break;
        default:
            break;
    }
}

static bool Playback_Start(void)
{
    /* 强转为 32 位指针读取 Flash 头部 (0x00040000) */
    uint32_t *header = (uint32_t *)FLASH_STORAGE_START_ADDRESS;
    
    /* 校验 Magic Word 和完成标志，确保有完整数据 */
    if (header[0] != FLASH_STORAGE_MAGIC || header[3] != FLASH_STORAGE_DONE_MAGIC) {
        return false; 
    }

    /* 提取参数：[1]为采样率(8000), [2]为总采样点数 */
    uint32_t savedSampleRate = header[1];
    g_playTotalCount = header[2];
    
    if (g_playTotalCount == 0 || savedSampleRate == 0) {
        return false;
    }

    /* 重置播放进度和数据指针 (指向 0x00040010) */
    g_playCurrentCount = 0;
    g_flashPlayPtr = (uint16_t *)FLASH_DATA_START_ADDRESS;

    /* 暂停定时器以安全切换状态 */
    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    /* 启用 DAC 外设 (SysConfig 默认宏通常为 DAC0) */
    DL_DAC12_enable(DAC0);

    g_isPlaying = true;
    
    /* 重启定时器，维持 8000Hz 中断率 */
    DL_TimerA_startCounter(TIMER_0_INST);

    return true;
}

static void Playback_Stop(void)
{
    g_isPlaying = false;
    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_DAC12_disable(DAC0);
}