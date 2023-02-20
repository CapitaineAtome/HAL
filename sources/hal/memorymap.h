//
// Created by marmelade on 27/01/23.
//

#ifndef DRIVER_HAL_MEMORYMAP_H
#define DRIVER_HAL_MEMORYMAP_H

#ifndef FLASH_BASE
#define FLASH_BASE          (0x0800'0000U)
#endif // FLASH_BASE

#ifndef SRAM_BASE
#define SRAM_BASE           (0x2000'0000U)
#endif // SRAM_BASE

#ifndef PERIPH_BASE
#define PERIPH_BASE         (0x4000'0000U)
#endif // PERIPH_BASE

#ifndef SRAM_SIZE_MAX
#define SRAM_SIZE_MAX       (0x0000'2000U)
#endif // SRAM_SIZE_MAX


#define IOPORT_MEMORY       (0x5000'0000U)
#define PORTA_MEMORY        ((IOPORT_MEMORY) + (0x0000'0000U))
#define PORTB_MEMORY        ((IOPORT_MEMORY) + (0x0000'0400U))
#define PORTC_MEMORY        ((IOPORT_MEMORY) + (0x0000'0800U))
#define PORTD_MEMORY        ((IOPORT_MEMORY) + (0x0000'0C00U))
#define PORTE_MEMORY        ((IOPORT_MEMORY) + (0x0000'1000U))
#define PORTF_MEMORY        ((IOPORT_MEMORY) + (0x0000'1400U))

#define AHB_MEMORY          (0x4002'0000U)
#define DMA1_MEMORY         ((AHB_MEMORY) + (0x0000'0000U))
#define DMA2_MEMORY         ((AHB_MEMORY) + (0x0000'0400U))
#define DMAMUX_MEMORY       ((AHB_MEMORY) + (0x0000'0800U))
#define RCC_MEMORY          ((AHB_MEMORY) + (0x0000'1000U))
#define EXTI_MEMORY         ((AHB_MEMORY) + (0x0000'1800U))
#define FLASH_MEMORY        ((AHB_MEMORY) + (0x0000'2000U))
#define CRC_MEMORY          ((AHB_MEMORY) + (0x0000'3000U))
#define RNG_MEMORY          ((AHB_MEMORY) + (0x0000'5000U))
#define AES_MEMORY          ((AHB_MEMORY) + (0x0000'6000U))

#define APB_MEMORY          (0x4000'0000U)
#define TIM2_MEMORY         ((APB_MEMORY) + (0x0000'0000))
#define TIM3_MEMORY         ((APB_MEMORY) + (0x0000'0400))
#define TIM4_MEMORY         ((APB_MEMORY) + (0x0000'0800))
#define TIM6_MEMORY         ((APB_MEMORY) + (0x0000'1000))
#define TIM7_MEMORY         ((APB_MEMORY) + (0x0000'1400))
#define TIM14_MEMORY        ((APB_MEMORY) + (0x0000'2000))
#define RTC_MEMORY          ((APB_MEMORY) + (0x0000'2800))
#define WWDG_MEMORY         ((APB_MEMORY) + (0x0000'2C00))
#define IWDG_MEMORY         ((APB_MEMORY) + (0x0000'3000))
#define SPI2_I2S2_MEMORY    ((APB_MEMORY) + (0x0000'3800))
#define SPI3_MEMORY         ((APB_MEMORY) + (0x0000'3C00))
#define USART2_MEMORY       ((APB_MEMORY) + (0x0000'4400))
#define USART3_MEMORY       ((APB_MEMORY) + (0x0000'4800))
#define USART4_MEMORY       ((APB_MEMORY) + (0x0000'4C00))
#define USART5_MEMORY       ((APB_MEMORY) + (0x0000'5000))
#define I2C1_MEMORY         ((APB_MEMORY) + (0x0000'5400))
#define I2C2_MEMORY         ((APB_MEMORY) + (0x0000'5800))
#define USB_MEMORY          ((APB_MEMORY) + (0x0000'5C00))
#define FDCAN1_MEMORY       ((APB_MEMORY) + (0x0000'6400))
#define FDCAN2_MEMORY       ((APB_MEMORY) + (0x0000'6800))
#define CRS_MEMORY          ((APB_MEMORY) + (0x0000'6C00))
#define PWR_MEMORY          ((APB_MEMORY) + (0x0000'7000))
#define DAC_MEMORY          ((APB_MEMORY) + (0x0000'7400))
#define CEC_MEMORY          ((APB_MEMORY) + (0x0000'7800))
#define LPTIM1_MEMORY       ((APB_MEMORY) + (0x0000'7C00))
#define LPUART1_MEMORY      ((APB_MEMORY) + (0x0000'8000))
#define LPUART2_MEMORY      ((APB_MEMORY) + (0x0000'8400))
#define I2C3_MEMORY         ((APB_MEMORY) + (0x0000'8800))
#define LPTIM2_MEMORY       ((APB_MEMORY) + (0x0000'9400))
#define USBA_RAM_1_MEMORY   ((APB_MEMORY) + (0x0000'9800))
#define USB_RAM_2_MEMORY    ((APB_MEMORY) + (0x0000'9C00))
#define UCPD1_MEMORY        ((APB_MEMORY) + (0x0000'A000))
#define UCPD2_MEMORY        ((APB_MEMORY) + (0x0000'A400))
#define TAMP_MEMORY         ((APB_MEMORY) + (0x0000'B000))
#define FDCAN_MEMORY        ((APB_MEMORY) + (0x0000'B400))
#define SYSCFG_MEMORY       ((APB_MEMORY) + (0x0001'0000))
#define VREFBUF_MEMORY      ((APB_MEMORY) + (0x0001'0030))
#define SYSCFGITLINE_MEMORY ((APB_MEMORY) + (0x0001'0080))
#define COMP_MEMORY         ((APB_MEMORY) + (0x0001'0200))
#define ADC_MEMORY          ((APB_MEMORY) + (0x0001'2400))
#define TIM1_MEMORY         ((APB_MEMORY) + (0x0001'2C00))
#define SPI1_I2S1_MEMORY    ((APB_MEMORY) + (0x0001'3000))
#define USART1_MEMORY       ((APB_MEMORY) + (0x0001'3800))
#define USART6_MEMORY       ((APB_MEMORY) + (0x0001'3C00))
#define TIM15_MEMORY        ((APB_MEMORY) + (0x0001'4000))
#define TIM16_MEMORY        ((APB_MEMORY) + (0x0001'4400))
#define TIM17_MEMORY        ((APB_MEMORY) + (0x0001'4800))
#define DBG_MEMORY          ((APB_MEMORY) + (0x0001'5800))

#endif // DRIVER_HAL_MEMORYMAP_H
