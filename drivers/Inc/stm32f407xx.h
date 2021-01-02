/*
 * stm32f407xx.h
 *
 *
 * General Microcontroller definitions header file
 *
 *
 *  Created on: Dec 26, 2020
 *      Author: Matheus Sozza
 */

#ifndef STM32F407XX_H_
#define	STM32F407XX_H_

	#include <stddef.h>
	#include <stdint.h>
	#define __vo volatile


 // **************************************** Processor specific details for STM32F4xx based MCU ***************************************

	// *** NVIC Registers
	#define NVIC_ISER0 								((__vo uint32_t*) 0xE000E100)
	#define NVIC_ISER1							 	((__vo uint32_t*) 0xE000E104)
	#define NVIC_ISER2							 	((__vo uint32_t*) 0xE000E108)
	#define NVIC_ISER3 								((__vo uint32_t*) 0xE000E10C)

	#define NVIC_ICER0 								((__vo uint32_t*) 0xE000E180)
	#define NVIC_ICER1 								((__vo uint32_t*) 0xE000E184)
	#define NVIC_ICER2							 	((__vo uint32_t*) 0xE000E188)
	#define NVIC_ICER3 								((__vo uint32_t*) 0xE000E18C)

	#define NVIC_PR_BASE_ADDRESS 					((__vo uint32_t*)0xE000E4000)

	#define	NVIC_NO_PR_BITS_IMPLEMENTED				4


 // **************************************** Define base address macros for STM32F4xx peripherals ***************************************
	/* RCC Clock Base Address */
	#define RCC_BASE_ADDRESS					0x40023800U

	/* Memories Base Address */
	#define FLASH_BASE_ADDRESS					0x08000000U
	#define SRAM1_BASE_ADDRESS					0x20000000U
	#define SRAM2_BASE_ADDRESS					(SRAM1_BASE_ADDRESS + 0x1C00)
	#define ROM									0x1FFF0000U
	#define SRAM 								SRAM_BASE_ADDRESS

	/*  Main Buses address */
	#define PERIPH_BASE 						0x40000000U
	#define APB1_PERIPH_BASE_ADDRESS 			PERIPH_BASE
	#define APB2_PERIPH_BASE_ADDRESS 			0x40010000U
	#define AHB1_PERIPH_BASE_ADDRESS 			0x40020000U
	#define AHB2_PERIPH_BASE_ADDRESS 			0x50000000U

	/* AHB1 Peripherals */
	#define GPIOA_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X0400U)
	#define GPIOB_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X0400U)
	#define GPIOC_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X0800U)
	#define GPIOD_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X0C00U)
	#define GPIOE_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X1000U)
	#define GPIOF_BASE_ADDRESS 					(AHB1_PERIPH_BASE_ADDRESS + 0X1400U)

	/* APB1 Peripherals */
	#define I2C1_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X5400U)
	#define I2C2_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X5800U)
	#define I2C3_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X5C00U)
	#define UART8_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X7C00U)
	#define UART7_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X7800U)
	#define UART5_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X5000U)
	#define UART4_BASE_ADDRESS 					(APB1_PERIPH_BASE_ADDRESS + 0X4C00U)
	#define USART3_BASE_ADDRESS					(APB1_PERIPH_BASE_ADDRESS + 0X4800U)
	#define USART2_BASE_ADDRESS					(APB1_PERIPH_BASE_ADDRESS + 0X4400U)
	#define SPI2_BASE_ADDRESS					(APB1_PERIPH_BASE_ADDRESS + 0X3800U)
	#define SPI3_BASE_ADDRESS					(APB1_PERIPH_BASE_ADDRESS + 0X3C00U)

	/* APB2 Peripherals */
	#define EXTI_BASE_ADDRESS 					(APB2_PERIPH_BASE_ADDRESS + 0X3C00U)
	#define SPI1_BASE_ADDRESS 					(APB2_PERIPH_BASE_ADDRESS + 0X3000U)
	#define SYSCFG_BASE_ADDRESS 				(APB2_PERIPH_BASE_ADDRESS + 0X3800U)
	#define USART1_BASE_ADDRESS 				(APB2_PERIPH_BASE_ADDRESS + 0X1000U)
	#define USART6_BASE_ADDRESS 				(APB2_PERIPH_BASE_ADDRESS + 0X1400U)

// **************************************** Define register mapping structures for STM32F4xx peripherals

	/*** GPIO Base Register Structure ***/
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
	}GPIO_RegDef_t;

	/*** RCC Base Register Structure ***/
	typedef struct //RCC_RegDef_t register mapping definition
	{
		#define RCC_RegDef_AHB1 0
		#define RCC_RegDef_AHB2 1
		#define RCC_RegDef_AHB3 2
		#define RCC_RegDef_APB1 4
		#define RCC_RegDef_APB2 5

		__vo uint32_t CR;
		__vo uint32_t PLLCFG;
		__vo uint32_t CFGR;
		__vo uint32_t CIR;
		__vo uint32_t RSTTR[8]; // 0-AHB1 / 1-AHB2 / 2-AHB3 / 4-APB1 / 5-APB2
		__vo uint32_t ENR[8]; // 0-AHB1 / 1-AHB2 / 2-AHB3 / 4-APB1 / 5-APB2
		__vo uint32_t LPENR[8]; // 0-AHB1 / 1-AHB2 / 2-AHB3 / 4-APB1 / 5-APB2
		__vo uint32_t BDCR;
		__vo uint32_t CSR;
		__vo uint32_t RESERVED0;
		__vo uint32_t PLLI2SCFGR;
		__vo uint32_t PLLSAICFGR;
		__vo uint32_t DCKCFGR;

	}RCC_RegDef_t;

	/*** EXTI Base Register Structure ***/
		typedef struct //EXTI_RegDef_t register mapping definition
		{
			__vo uint32_t IMR;
			__vo uint32_t EMR;
			__vo uint32_t RTSR;
			__vo uint32_t FTSR;
			__vo uint32_t SWIER;
			__vo uint32_t PR;
		}EXTI_RegDef_t;

	/*** SYSCFG Base Register Structure ***/
		typedef struct //SYSCFG_RegDef_t register mapping definition
		{
			__vo uint32_t MEMRMP;
			__vo uint32_t PMC;
			__vo uint32_t EXTICR[4];
			__vo uint32_t RESERVED1[2];
			__vo uint32_t CMPCR;
			__vo uint32_t RESERVED2[2];
			__vo uint32_t CFGR;
		}SYSCFG_RegDef_t;

	#define GPIOA	(GPIO_RegDef_t*)GPIOA_BASE_ADDRESS
	#define GPIOB	(GPIO_RegDef_t*)GPIOB_BASE_ADDRESS
	#define GPIOC	(GPIO_RegDef_t*)GPIOC_BASE_ADDRESS
	#define GPIOD	(GPIO_RegDef_t*)GPIOD_BASE_ADDRESS
	#define GPIOE	(GPIO_RegDef_t*)GPIOE_BASE_ADDRESS
	#define GPIOF	(GPIO_RegDef_t*)GPIOF_BASE_ADDRESS

	#define RCC		(RCC_RegDef_t*)RCC_BASE_ADDRESS

	#define EXTI	((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

	#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)

// **************************************** Register struct allocation for GPIO & RCC Device ****************************************

	#define pGPIOA_RegDef()			GPIO_RegDef_t *pGPIOA = GPIOA
	#define pGPIOB_RegDef()			GPIO_RegDef_t *pGPIOB = GPIOB
	#define pGPIOC_RegDef()			GPIO_RegDef_t *pGPIOC = GPIOC
	#define pGPIOD_RegDef()			GPIO_RegDef_t *pGPIOD = GPIOD
	#define pGPIOE_RegDef()			GPIO_RegDef_t *pGPIOE = GPIOE
	#define pGPIOF_RegDef()			GPIO_RegDef_t *pGPIOF = GPIOF

	#define pRCC_RegDef()			RCC_RegDef_t *pRCC = RCC

// **************************************** Define RCC clock enabling macros for STM32F4xx peripheral
	// GPIO Clock Enabling / Disabling macros
	#define GPIOA_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<0)
	#define GPIOB_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<1)
	#define GPIOC_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<2)
	#define GPIOD_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<3)
	#define GPIOE_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<4)
	#define GPIOF_PCLK_EN()		pRCC->ENR[RCC_RegDef_AHB1] |= (1<<5)

	#define GPIOA_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<0)
	#define GPIOB_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<1)
	#define GPIOC_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<2)
	#define GPIOD_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<3)
	#define GPIOE_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<4)
	#define GPIOF_PCLK_DI()		pRCC->ENR[RCC_RegDef_AHB1] &= ~(1<<5)

	#define GPIOA_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<0));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<0));} while(0)
	#define GPIOB_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<1));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<1));} while(0)
	#define GPIOC_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<2));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<2));} while(0)
	#define GPIOD_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<3));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<3));} while(0)
	#define GPIOE_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<4));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<5));} while(0)
	#define GPIOF_REG_RESET()	do{ (pRCC->RSTTR[RCC_RegDef_AHB1] |= (1<<4));	\
										(pRCC->RSTTR[RCC_RegDef_AHB1] &= ~(1<<5));} while(0)

	//SYSCFG Clock enabling / disabling macros

	#define SYSCFG_CLK_EN() 	pRCC->ENR[RCC_RegDef_APB2] |= (1<<14);
	#define SYSCFG_CLK_DI() 	pRCC->ENR[RCC_RegDef_APB2] &= ~(1<<14);

	// I2C Clock Enabling / Disabling macros
	#define I2C1_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<21)
	#define I2C2_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<22)
	#define I2C3_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<23)

	#define I2C1_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<21)
	#define I2C2_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<22)
	#define I2C3_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<23)

	// SPI Clock Enabling / Disabling macros
	#define SPI2_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<14)
	#define SPI3_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<15)

	#define SPI2_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<14)
	#define SPI3_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<15)

	// UART Clock Enabling / Disabling macros
	#define USART1_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB2] |= (1<<4)
	#define USART2_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<17)
	#define USART3_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<18)
	#define UART4_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<19)
	#define UART5_PCLK_EN() 	pRCC->ENR[RCC_RegDef_APB1] |= (1<<20)

	#define USART1_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB2] &= ~(1<<4)
	#define USART2_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<17)
	#define USART3_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<18)
	#define UART4_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<19)
	#define UART5_PCLK_DI() 	pRCC->ENR[RCC_RegDef_APB1] &= ~(1<<20)

// **************************************** Define generic application macros for STM32F4xx peripherals

	// IRQ Number macros
	#define IRQ_NO_EXTI0		6
	#define IRQ_NO_EXTI1		7
	#define IRQ_NO_EXTI2		8
	#define IRQ_NO_EXTI3		9
	#define IRQ_NO_EXTI4		10
	#define IRQ_NO_EXTI9_5		23
	#define IRQ_NO_EXTI15_10	40

	// Generic definitions
	#define ENABLE		1
	#define DISABLE		0
	#define SET			1
	#define RESET 		0

	// Generic Macros
	#define GPIO_BASE_ADDRESS_TO_CODE(x)	((x== GPIOA) ? 0 : \
											(x==GPIOB) ?  1 : \
											(x==GPIOC) ?  2 : \
											(x==GPIOD) ?  3 : \
											(x==GPIOE) ?  4 : \
											(x==GPIOF) ? 5 : 0)

// **************************************** Sub-libraries inclusion for STM32F4xx peripherals

	#include "stm32f407xx_gpio_driver.h"

#endif
