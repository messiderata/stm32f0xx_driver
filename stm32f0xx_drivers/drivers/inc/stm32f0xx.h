/*
 * stm32f0xx.h
 *
 *  Created on: Jan 3, 2026
 *      Author: philip amista
 */

#ifndef INC_STM32F0XX_H_
#define INC_STM32F0XX_H_

#define FLASH_BASE_ADDR					0x08000000U	//BASE ADDRESS OF FLASH MAIN MEMORY
#define ROM_BASEADDR					0x1FFFEC00U //BASE ADDRESS OF SYSTEM MEMORY
#define SRAM 							0x20000000U //BASE ADDRESS OF SRAM

/*
 * AHBx and APB Bus Peripheral base addresses *
 */
#define PERIPH_BASE_ADDR				0x40000000U
#define APBPERIPH_BASEADDR 				PERIPH_BASE_ADDR
#define AHB1_BASE_ADDR					(PERIPH_BASE + 0x00020000U)
#define AHB2_BASE_ADDR					(PERIPH_BASE + 0x08000000U)

/*
 *
 * Base addressed of the Peripherals which are hanging on AHB1 bus
 */

#define	DMA_BASEADDR					(AHB1_BASE_ADDR + 0x0000)
#define DMA2_BASEADDR					(AHB1_BASE_ADDR + 0x0400)
#define RCC_BASEADDR					(AHB1_BASE_ADDR + 0x1000)
#define FLASH_INTERFACE_BASEADDR		(AHB1_BASE_ADDR + 0x2000)
#define CRC_BASEADDR					(AHB1_BASE_ADDR + 0x3000)
#define TRC_BASEADDR					(AHB1_BASE_ADDR + 0x4000)

/*
 * GPIOA to GPIOF base addresses
 * Base addressed of the Peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR 					(AHB2_BASE_ADDR + 0x0000)
#define GPIOB_BASEADDR 					(AHB2_BASE_ADDR + 0x0400)
#define GPIOC_BASEADDR 					(AHB2_BASE_ADDR + 0x0800)
#define GPIOD_BASEADDR 					(AHB2_BASE_ADDR + 0x0C00)
#define GPIOE_BASEADDR 					(AHB2_BASE_ADDR + 0x1000)
#define GPIOF_BASEADDR 					(AHB2_BASE_ADDR + 0x1400)

/*
 *
 * Base addressed of the Peripherals which are hanging on APB bus
 */
#define TIM2_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x0000)
#define TIM3_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x0400) 
#define TIM6_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x1000)
#define TIM7_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x1400)
#define TIM14_BASE_ADDR                  (APBPERIPH_BASEADDR + 0x2000)
#define RTC_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x2800)
#define WWDG_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x2C00)
#define IWDG_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x3000)
#define SPI2_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x3800)
#define USART2_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x4400)
#define USART3_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x4800)
#define USART4_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x4C00)
#define USART5_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x5000)
#define I2C1_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x5400)
#define I2C2_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x5800)
#define USB_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x5C00)
#define USB_CAN_SRAM_BASE_ADDR           (APBPERIPH_BASEADDR + 0x6000)
#define CAN_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x6400)
#define CRC_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x6C00)
#define PWR_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x7000)
#define DAC_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x7400)
#define CEC_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x7800)
#define SYSCFG_COMP_BASE_ADDR            (APBPERIPH_BASEADDR + 0x10000)
#define EXTI_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x10400)
#define USART6_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x11400)
#define USART7_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x11800)
#define USART8_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x11C00)
#define ADC_BASE_ADDR                    (APBPERIPH_BASEADDR + 0x12400)
#define TIM1_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x11C00)
#define SPI1_I2S1_BASE_ADDR              (APBPERIPH_BASEADDR + 0x12C00)
#define USART1_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x13000)
#define TIM15_BASE_ADDR                  (APBPERIPH_BASEADDR + 0x14000)
#define TIM16_BASE_ADDR                  (APBPERIPH_BASEADDR + 0x14400)
#define TIM16_BASE_ADDR                  (APBPERIPH_BASEADDR + 0x14400)
#define TIM17_BASE_ADDR                  (APBPERIPH_BASEADDR + 0x14800)
#define DBGMCU_BASE_ADDR                 (APBPERIPH_BASEADDR + 0x15800)
    
#endif /* INC_STM32F0XX_H_ */
