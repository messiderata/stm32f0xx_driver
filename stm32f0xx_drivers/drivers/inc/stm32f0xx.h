/*
 * stm32f0xx.h
 *
 *  Created on: Jan 3, 2026
 *      Author: philip amista
 */

#ifndef INC_STM32F0XX_H_
#define INC_STM32F0XX_H_

#include <stdint.h>

#define __vo volatile

#define FLASH_BASE_ADDR					0x08000000U	//BASE ADDRESS OF FLASH MAIN MEMORY
#define ROM_BASEADDR					0x1FFFEC00U //BASE ADDRESS OF SYSTEM MEMORY
#define SRAM 							0x20000000U //BASE ADDRESS OF SRAM

/*
 * AHBx and APB Bus Peripheral base addresses *
 */
#define PERIPH_BASE_ADDR				0x40000000U
#define APBPERIPH_BASEADDR 				PERIPH_BASE_ADDR
#define AHB1_BASE_ADDR					(PERIPH_BASE_ADDR + 0x00020000U)
#define AHB2_BASE_ADDR					(PERIPH_BASE_ADDR + 0x08000000U)
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
 * Base addresses of the Peripherals which are hanging on APB bus
 */
#define TIM2_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x0000)//Base address of Timer 2
#define TIM3_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x0400)//Base address of Timer 3
#define TIM6_BASE_ADDR                   (APBPERIPH_BASEADDR + 0x1000)//Base address of timer 6
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


/********************************Peripheral register definition structures**************************************************************** */
/*
    Note: Register of a peripheral are specific to MCU
    So, for different MCUs, the register structures for different peripherals may vary.
    Please refer the reference manual of the specific MCU you are using.
*/
typedef struct{
    __vo uint32_t MODER;                                        /*GPIO port mode register                               Address offset: 0x00        */
    __vo uint32_t OTYPER;                                       /*GPIO port output type register                        Address offset: 0x04        */
    __vo uint32_t OSPEEDR;                                      /*GPIO port output speed register                       Address offset: 0x08        */
    __vo uint32_t PUPDR;                                        /*GPIO port pull-up/pull-down register                  Address offset: 0x0C        */
    __vo uint32_t IDR;                                          /*GPIO port input data register                         Address offset: 0x10        */
    __vo uint32_t ODR;                                          /*GPIO port output data register                        Address offset: 0x14        */                                          
    __vo uint32_t BSRR;                                         /*GPIO port bit set/reset register                      Address offset: 0x18        */
    __vo uint32_t LCKR;                                         /*GPIO port configuration lock register                 Address offset: 0x1C        */       
    __vo uint32_t AFR[2];                                       /*GPIO alternate function low register   AF[0] = LOW, AF[1] = HIGH            Address offset: 0x20-0x24   */ 
    __vo uint32_t BRR;                                          /*GPIO port bit reset register                          Address offset: 0x28        */  
}GPIO_RegDef_t;


/*
    Note: Register of a peripheral are specific to MCU
    So, for different MCUs, the register structures for different peripherals may vary.
    Please refer the reference manual of the specific MCU you are using.
*/
typedef struct{

    __vo uint32_t  IMR;     /*Interrupt mask register                               Address offset: 0x00*/   
    __vo uint32_t  EMR;     /*Event mask register                                   Address offset: 0x04*/
    __vo uint32_t  RTSR;    /*Rising trigger selection register                     Address offset: 0x08*/
    __vo uint32_t  FTSR;    /*Falling trigger selection register                    Address offset: 0x0C*/
    __vo uint32_t  SWIER;   /*Software interrupt event register                     Address offset: 0x10*/
    __vo uint32_t  PR;      /*Pending register                                      Addres offset : 0x14*/
}EXTI_RegDef_t;



typedef struct 
{
    /* data */
    __vo uint32_t SYSCFG_CFGR1[2];
    __vo uint32_t SYSCFG_EXTICR[4];

}SYSCFG_RegDef_t;


/********************************Peripheral Clock register definition structures**************************************************************** */
/*
    Note: Register of a clock peripheral are specific to MCU
    So, for different MCUs, the register structures for different peripherals may vary.
    Please refer the reference manual of the specific MCU you are using.
*/

typedef struct{
    __vo uint32_t CR;            /* RCC clock control register,                      Address offset: 0x00 */
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t APB2RSTR;
    __vo uint32_t APB1RSTR;
    __vo uint32_t AHBENR;
    __vo uint32_t APB2ENR;
    __vo uint32_t APB1ENR;
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    __vo uint32_t AHBRSTR;
    __vo uint32_t CFGR2;
    __vo uint32_t CFGR3;
    __vo uint32_t CR2;
}RCC_RegDef_t;


/**
 * @brief 
 * Defining all GPIO peripheral definitions
 * Peripheral base addresses typecasted to GPIO_RegDef_t structure type pointers
 */
#define GPIOA                           ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t*)GPIOF_BASEADDR)


#define RCC                             ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                            ((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG                           ((SYSCFG_RegDef_t *))
/*
    Clock enable marcos for GPIOX peripherals
*/
#define GPIOA_PCLK_EN()           (RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()           (RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()           (RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()           (RCC->AHBENR |= (1<<20))
#define GPIOE_PCLK_EN()           (RCC->AHBENR |= (1<<21))
#define GPIOF_PCLK_EN()           (RCC->AHBENR |= (1<<22))

/*
    Clock disable marcos for GPIOX peripherals
*/
#define GPIOA_PCLK_DI()           (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()           (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()           (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()           (RCC->AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI()           (RCC->AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI()           (RCC->AHBENR &= ~(1<<22))

/*
    Clock reset marcos for GPIOX peripherals
*/
#define GPIOA_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<17)); (RCC->AHBRSTR  &= (1<<17));}while(0)
#define GPIOB_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<18)); (RCC->AHBRSTR  &= (1<<18));}while(0)
#define GPIOC_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<19)); (RCC->AHBRSTR  &= (1<<19));}while(0)
#define GPIOD_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<20)); (RCC->AHBRSTR  &= (1<<20));}while(0)
#define GPIOE_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<21)); (RCC->AHBRSTR  &= (1<<21));}while(0)
#define GPIOF_PCLK_RST()           do {(RCC->AHBRSTR  |= (1<<22)); (RCC->AHBRSTR  &= (1<<22));}while(0)

/*
    Clock enable marcos for I2C peripherals
*/
#define I2C1_EN()                   (RCC->APB1ENR |= (1<<21))
#define I2C2_EN()                   (RCC->APB1ENR |= (1<<22))

/*
    Clock enable marcos for SPI peripherals
*/
#define SPI1_EN()                   (RCC->APB2ENR |= (1<<12))
#define SPI2_EN()                   (RCC->APB1ENR |= (1<<14))


/*
    Clock enable marcos for USART peripherals
*/
#define USART1_EN()                   (RCC->APB2ENR |= (1<<14))
#define USART2_EN()                   (RCC->APB1ENR |= (1<<17))
#define USART3_EN()                   (RCC->APB1ENR |= (1<<18))
#define USART4_EN()                   (RCC->APB1ENR |= (1<<19))
#define USART5_EN()                   (RCC->APB1ENR |= (1<<20))
#define USART6_EN()                   (RCC->APB2ENR |= (1<<5))
#define USART7_EN()                   (RCC->APB2ENR |= (1<<6))
#define USART8_EN()                   (RCC->APB2ENR |= (1<<7))

/*
    Clock enable marcos for SYSCFGCOMPEN peripherals
*/  
#define SYSCFG_EN()                   (RCC->APB2ENR |= (1<<0))






/*
    Clock disable marcos for I2C peripherals
*/
#define I2C1_DI()                   (RCC->APB1ENR &= ~(0<<21))
#define I2C2_DI()                   (RCC->APB1ENR &= ~(0<<22))

/*
    Clock disable marcos for SPI peripherals
*/
#define SPI1_DI()                   (RCC->APB2ENR &= ~(0<<12))
#define SPI2_DI()                   (RCC->APB1ENR &= ~(0<<14))


/*
    Clock disable marcos for USART peripherals
*/
#define USART1_DI()                   (RCC->APB2ENR &= ~(0<<14))
#define USART2_DI()                   (RCC->APB1ENR &= ~(0<<17))
#define USART3_DI()                   (RCC->APB1ENR &= ~(0<<18))
#define USART4_DI()                   (RCC->APB1ENR &= ~(0<<19))
#define USART5_DI()                   (RCC->APB1ENR &= ~(0<<20))
#define USART6_DI()                   (RCC->APB2ENR &= ~(0<<5))
#define USART7_DI()                   (RCC->APB2ENR &= ~(0<<6))
#define USART8_DI()                   (RCC->APB2ENR &= ~(0<<7))

/*
disable*/  
#define SYSCFG_DI()                   (RCC->APB2ENR &= ~(1<<0))








#define ENABLE                                  1
#define DISABLE                                 0
#define SET                                     ENABLE
#define RESET                                   DISABLE   
#define GPIO_PIN_SET                            SET
#define GPIO_PIN_RESET                          RESET



#include "stm32f051xx_gpio.h"
    
    
#endif /* INC_STM32F0XX_H_ */
