/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG8
#define PWM_0_INST_IRQHandler                                   TIMG8_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_7
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM14)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM14_PF_TIMG8_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_0
#define GPIO_PWM_0_C1_IOMUX                                       (IOMUX_PINCM1)
#define GPIO_PWM_0_C1_IOMUX_FUNC                      IOMUX_PINCM1_PF_TIMG8_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_TIMG */
#define TIMER_TIMG_INST                                                  (TIMA0)
#define TIMER_TIMG_INST_IRQHandler                              TIMA0_IRQHandler
#define TIMER_TIMG_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define TIMER_TIMG_INST_LOAD_VALUE                                      (12499U)
#define TIMER_TIMG_INST_PUB_0_CH                                             (1)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART4
#define UART_1_INST_FREQUENCY                                           80000000
#define UART_1_INST_IRQHandler                                  UART4_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART4_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM28)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM27)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM28_PF_UART4_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM27_PF_UART4_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_80_MHZ_9600_BAUD                                       (520)
#define UART_1_FBRD_80_MHZ_9600_BAUD                                        (53)





/* Defines for ADC12_UI */
#define ADC12_UI_INST                                                       ADC0
#define ADC12_UI_INST_IRQHandler                                 ADC0_IRQHandler
#define ADC12_UI_INST_INT_IRQN                                   (ADC0_INT_IRQn)
#define ADC12_UI_ADCMEM_U                                     DL_ADC12_MEM_IDX_0
#define ADC12_UI_ADCMEM_U_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC12_UI_ADCMEM_1                                     DL_ADC12_MEM_IDX_1
#define ADC12_UI_ADCMEM_1_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC12_UI_INST_SUB_CH                                                 (1)
#define GPIO_ADC12_UI_C0_PORT                                              GPIOA
#define GPIO_ADC12_UI_C0_PIN                                      DL_GPIO_PIN_27
#define GPIO_ADC12_UI_IOMUX_C0                                   (IOMUX_PINCM60)
#define GPIO_ADC12_UI_IOMUX_C0_FUNC               (IOMUX_PINCM60_PF_UNCONNECTED)
#define GPIO_ADC12_UI_C1_PORT                                              GPIOA
#define GPIO_ADC12_UI_C1_PIN                                      DL_GPIO_PIN_26
#define GPIO_ADC12_UI_IOMUX_C1                                   (IOMUX_PINCM59)
#define GPIO_ADC12_UI_IOMUX_C1_FUNC               (IOMUX_PINCM59_PF_UNCONNECTED)



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (1)
#define DMA_CH0_TRIGGER_SEL_SW                               (DMA_SOFTWARE_TRIG)
/* Defines for DMA_UI */
#define DMA_UI_CHAN_ID                                                       (0)
#define ADC12_UI_INST_DMA_TRIGGER                     (DMA_ADC0_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for LED4: GPIOA.14 with pinCMx 36 on package pin 43 */
#define LED_LED4_PIN                                            (DL_GPIO_PIN_14)
#define LED_LED4_IOMUX                                           (IOMUX_PINCM36)
/* Port definition for Pin Group DHT11 */
#define DHT11_PORT                                                       (GPIOB)

/* Defines for DHT1: GPIOB.21 with pinCMx 49 on package pin 68 */
#define DHT11_DHT1_PIN                                          (DL_GPIO_PIN_21)
#define DHT11_DHT1_IOMUX                                         (IOMUX_PINCM49)
/* Defines for CS: GPIOC.9 with pinCMx 87 on package pin 66 */
#define GPIO_GRP_0_CS_PORT                                               (GPIOC)
#define GPIO_GRP_0_CS_PIN                                        (DL_GPIO_PIN_9)
#define GPIO_GRP_0_CS_IOMUX                                      (IOMUX_PINCM87)
/* Defines for DC: GPIOC.8 with pinCMx 86 on package pin 65 */
#define GPIO_GRP_0_DC_PORT                                               (GPIOC)
#define GPIO_GRP_0_DC_PIN                                        (DL_GPIO_PIN_8)
#define GPIO_GRP_0_DC_IOMUX                                      (IOMUX_PINCM86)
/* Defines for D0: GPIOB.3 with pinCMx 16 on package pin 19 */
#define GPIO_GRP_0_D0_PORT                                               (GPIOB)
#define GPIO_GRP_0_D0_PIN                                        (DL_GPIO_PIN_3)
#define GPIO_GRP_0_D0_IOMUX                                      (IOMUX_PINCM16)
/* Defines for D1: GPIOB.2 with pinCMx 15 on package pin 18 */
#define GPIO_GRP_0_D1_PORT                                               (GPIOB)
#define GPIO_GRP_0_D1_PIN                                        (DL_GPIO_PIN_2)
#define GPIO_GRP_0_D1_IOMUX                                      (IOMUX_PINCM15)
/* Defines for RES: GPIOB.23 with pinCMx 51 on package pin 70 */
#define GPIO_GRP_0_RES_PORT                                              (GPIOB)
#define GPIO_GRP_0_RES_PIN                                      (DL_GPIO_PIN_23)
#define GPIO_GRP_0_RES_IOMUX                                     (IOMUX_PINCM51)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for H1: GPIOB.6 with pinCMx 23 on package pin 30 */
#define KEY_H1_PIN                                               (DL_GPIO_PIN_6)
#define KEY_H1_IOMUX                                             (IOMUX_PINCM23)
/* Defines for H2: GPIOB.7 with pinCMx 24 on package pin 31 */
#define KEY_H2_PIN                                               (DL_GPIO_PIN_7)
#define KEY_H2_IOMUX                                             (IOMUX_PINCM24)
/* Defines for H3: GPIOB.8 with pinCMx 25 on package pin 32 */
#define KEY_H3_PIN                                               (DL_GPIO_PIN_8)
#define KEY_H3_IOMUX                                             (IOMUX_PINCM25)
/* Defines for H4: GPIOB.9 with pinCMx 26 on package pin 33 */
#define KEY_H4_PIN                                               (DL_GPIO_PIN_9)
#define KEY_H4_IOMUX                                             (IOMUX_PINCM26)
/* Defines for V1: GPIOB.20 with pinCMx 48 on package pin 67 */
#define KEY_V1_PIN                                              (DL_GPIO_PIN_20)
#define KEY_V1_IOMUX                                             (IOMUX_PINCM48)
/* Defines for V2: GPIOB.24 with pinCMx 52 on package pin 71 */
#define KEY_V2_PIN                                              (DL_GPIO_PIN_24)
#define KEY_V2_IOMUX                                             (IOMUX_PINCM52)
/* Defines for V3: GPIOB.25 with pinCMx 56 on package pin 75 */
#define KEY_V3_PIN                                              (DL_GPIO_PIN_25)
#define KEY_V3_IOMUX                                             (IOMUX_PINCM56)
/* Defines for V4: GPIOB.27 with pinCMx 58 on package pin 77 */
#define KEY_V4_PIN                                              (DL_GPIO_PIN_27)
#define KEY_V4_IOMUX                                             (IOMUX_PINCM58)


/* Defines for TRNG */
/*
 * The TRNG interrupt is part of INT_GROUP1. Refer to the TRM for more details
 * on interrupt grouping
 */
#define TRNG_INT_IRQN                                            (TRNG_INT_IRQn)
#define TRNG_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_TRNG)




/* Defines for DAC12 */
#define DAC12_IRQHandler                                         DAC0_IRQHandler
#define DAC12_INT_IRQN                                           (DAC0_INT_IRQn)
#define GPIO_DAC12_OUT_PORT                                                GPIOA
#define GPIO_DAC12_OUT_PIN                                        DL_GPIO_PIN_15
#define GPIO_DAC12_IOMUX_OUT                                     (IOMUX_PINCM37)
#define GPIO_DAC12_IOMUX_OUT_FUNC                   IOMUX_PINCM37_PF_UNCONNECTED


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_TIMG_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_ADC12_UI_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_TRNG_init(void);
void SYSCFG_DL_DAC12_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
