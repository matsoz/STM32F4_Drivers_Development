/*
 * stm32f407xx.h
 *
 *  Created on: Dec 26, 2020
 *      Author: Matheus Sozza
 */

#ifndef INC_STM32F407XX_H_

#define INC_STM32F407XX_H_	1
#include <stdint.h>
#define __vo volatile

/**************** Peripheral base address definition ***************/

/* Memories Base Address */
#define FLASH_BASE_ADDRESS					0x08000000U
#define SRAM1_BASE_ADDRESS					0x20000000U
#define SRAM2_BASE_ADDRESS					SRAM1_BASE_ADDRESS + 0x1C00
#define ROM									0x1FFF0000U
#define SRAM 								SRAM_BASE_ADDRESS

/*  Main Buses address */
#define PERIPH_BASE 						0x400000000U
#define APB1_PERIPH_BASE_ADDRESS 			PERIPH_BASE
#define APB2_PERIPH_BASE_ADDRESS 			0x40010000U
#define AHB1_PERIPH_BASE_ADDRESS 			0x40020000U
#define AHB2_PERIPH_BASE_ADDRESS 			0x50000000U

/* AHB1 Peripherals */
#define GPIOA_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X0400U
#define GPIOB_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X0400U
#define GPIOC_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X0800U
#define GPIOD_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X0C00U
#define GPIOE_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X1000U

/* APB1 Peripherals */
#define I2C1_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X5400U
#define I2C2_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X5800U
#define I2C3_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X5C00U
#define UART8_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X7C00U
#define UART7_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X7800U
#define UART5_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X5000U
#define UART4_BASE_ADDRESS 					APB1_PERIPH_BASE_ADDRESS + 0X4C00U
#define USART3_BASE_ADDRESS					APB1_PERIPH_BASE_ADDRESS + 0X4800U
#define USART4_BASE_ADDRESS					APB1_PERIPH_BASE_ADDRESS + 0X4400U
#define SPI2_BASE_ADDRESS					APB1_PERIPH_BASE_ADDRESS + 0X3800U
#define SPI3_BASE_ADDRESS					APB1_PERIPH_BASE_ADDRESS + 0X3C00U

/* APB2 Peripherals */
#define EXTI_BASE_ADDRESS 					APB2_PERIPH_BASE_ADDRESS + 0X3C00U
#define SPI1_BASE_ADDRESS 					APB2_PERIPH_BASE_ADDRESS + 0X3000U
#define SYSCFG_BASE_ADDRESS 				APB2_PERIPH_BASE_ADDRESS + 0X3800U
#define USART1_BASE_ADDRESS 				APB2_PERIPH_BASE_ADDRESS + 0X1000U
#define USART6_BASE_ADDRESS 				APB2_PERIPH_BASE_ADDRESS + 0X1400U

/**************** Peripheral registers structure definition ***************/

/* GPIO Base Register Structure */
typedef struct //GPIO_RegDef_t register mapping definition
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRRL;
	__vo uint32_t BSRRH;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2]; //AFR[0] is the low register; AFR[1] is the high register
} GPIO_RegDef_t;

#endif /* INC_STM32F407XX_H_ */
