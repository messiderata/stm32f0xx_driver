/*
 * stm32f051xx_gpio.h
 *
 *  Created on: Jan 4, 2026
 *      Author: philip
 */

#ifndef INC_STM32F051XX_GPIO_H_
#define INC_STM32F051XX_GPIO_H_

#include "stm32f0xx.h"


typedef struct{
    uint8_t GPIO_PinNumber;             /*POSSIBLE VALUES FROM @GPIO_PIN_MODES*/
    uint8_t GPIO_PinMode;               /*POSSIBLE VALUES FROM @GPIO_PIN_OUTPUT_TYPES*/
    uint8_t GPIO_PinSpeed;              /*POSSIBLE VALUES FROM @GPIO_OUTPUT_SPEED*/
    uint8_t GPIO_PinPuPdControl;        /*POSSIBLE VALUES FROM @GPIO_PULL_TYPES*/
    uint8_t GPIO_PinOPType;            
    uint8_t GPIO_PinAltFunMode;        
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct{
	// Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; /* This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig; /* This holds GPIO OIN CONFIGURATION SETTINGS*/
}GPIO_Handle_t;

/*
    @GPIO_PIN_NUMBERS
    GPIO pin numbers
*/
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
    @GPIO_PIN_MODES
    GPIO POSSIBLE OUTPUT MODES
*/
#define GPIO_MODE_IN        0   // INPUT MODE
#define GPIO_MODE_OUTPUT    1   // OUTPUT MODE
#define GPIO_MODE_AF        2   // ALTERNATE FUNCTION MODE
#define GPIO_MODE_ANALOG    3   // ANALOG MODE
#define GPIO_MODE_IT_FT     4   // FALLING EDGE MODE - INTERRUPT
#define GPIO_MODE_IT_RT     5   // RISING EDGE MODE -   INTERRUPT
#define GPIO_MODE_IT_RFT    6   // MODE IT RISING EDGE TRIGGER MODE - INTERRUPT


/*
    @GPIO_PIN_OUTPUT_TYPES
    GPIO POSSIBLE OUTPUT TYPES
*/
#define GPIO_OUTPUT_TYPE_PP 0   // PUSH PULL TYPE
#define GPIO_OUTPUT_TYPE_OP 1   //OPEN DRAIN TYPE



/*
    @GPIO_OUTPUT_SPEED
    GPIO POSSIBLE OUTPUT SPEED

*/
#define GPIO_LOW_SPEED          0   // LOW SPEED
#define GPIO_MEDIUM_SPEED       1   //MEDIUM SPEED
#define GPIO_HIGH_SPEED         2   //HIGH SPEED

/*  
    @GPIO_PULL_TYPES
    GPIO PIN PULL UP AND PULL DOWN CONFIGURATION MACROS
*/

#define GPIO_NO_PUPD            0 //: No pull-up, pull-down
#define GPIO_PIN_PU             1 //: Pull-up
#define GPIO_PIN_PD             2 //: Pull-down
#define GPIO_RESERVED           3 //: Reserved



/********************************************************************************************************************************************************************
*                                                       APIs supported by this driver
*                       Fore more information about the APIs check the function definition
********************************************************************************************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
    GPIO Initialization
*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
/*
    GPIO De-initialization
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
    Data read and write
*/
uint8_t GPIO_InputReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_InputReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,  uint8_t  value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx  ,uint8_t  value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx  ,uint8_t  PinNumber);

/*
    IRQ configuration and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F051XX_GPIO_H_ */
