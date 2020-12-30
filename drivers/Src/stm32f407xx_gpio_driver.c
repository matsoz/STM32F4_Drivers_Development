/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 28, 2020
 *      Author: mathe
 */


#include "stm32f407xx_gpio_driver.h"


// ********** Register struct allocation for GPIO & RCC Device **********

GPIO_RegDef_t *pGPIOA = GPIOA;
GPIO_RegDef_t *pGPIOB = GPIOB;
GPIO_RegDef_t *pGPIOC = GPIOC;
GPIO_RegDef_t *pGPIOD = GPIOD;
GPIO_RegDef_t *pGPIOE = GPIOE;

RCC_RegDef_t *pRCC = RCC;

// ********** GPIO APIs function definition **********

// *** GPIO Initialization and Control ***
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) //Initialize GPIO Port and Pin
{
	uint8_t Temp = 0;

	//Configure MODE
	if(pGPIOHandle->GPIO_PinConfig.PinMode < 3) //Non interrupt mode
	{
		Temp = pGPIOHandle->GPIO_PinConfig.PinMode << (2* pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOBaseAddr->MODER &= ~(0x11 << 2* pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOBaseAddr->MODER |= Temp;
	}

	//Configure SPEED
	Temp = 0;
	Temp = pGPIOHandle->GPIO_PinConfig.PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR &= ~ (0x11 << 2* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR |= Temp;

	//Configure PUPD
	Temp = 0;
	Temp = pGPIOHandle->GPIO_PinConfig.PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->PUPDR &= ~(0x11 << 2* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->PUPDR |= Temp;

	//Configure POutput Type
	Temp = 0;
	Temp = pGPIOHandle->GPIO_PinConfig.PinOPType << (1* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OTYPER &= ~(0x1 << 1* pGPIOHandle->GPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OTYPER |= Temp;

	//Configure Pin Alt. Function
	if(pGPIOHandle->GPIO_PinConfig.PinAltFunMode == GPIO_MODE_AF)
	{
		if(pGPIOHandle->GPIO_PinConfig.PinNumber > 7) //High register
		{
			 Temp = pGPIOHandle->GPIO_PinConfig.PinAltFunMode<< (4*(pGPIOHandle->GPIO_PinConfig.PinNumber-8));
			 pGPIOHandle->pGPIOBaseAddr->AFR[1] &= ~(0x1111 << (4*(pGPIOHandle->GPIO_PinConfig.PinNumber-8)));
			 pGPIOHandle->pGPIOBaseAddr->AFR[1] |= Temp;
		}
		else //Low Register
		{
			Temp = pGPIOHandle->GPIO_PinConfig.PinAltFunMode<< (4*pGPIOHandle->GPIO_PinConfig.PinNumber);
			 pGPIOHandle->pGPIOBaseAddr->AFR[0] &= ~(0x1111 << (4*pGPIOHandle->GPIO_PinConfig.PinNumber));
			pGPIOHandle->pGPIOBaseAddr->AFR[0] |= Temp;
		}
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIO) //De-Initialize GPIO Port and Pin
{
	if(pGPIO == (GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)	GPIOA_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)	GPIOB_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)	GPIOC_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)	GPIOD_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)	GPIOE_REG_RESET();
}

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable) //Enable / Disable GPIO clock
{
	if(Enable == ENABLE)
	{
		if(pGPIOx == (GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)	GPIOA_PCLK_EN();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)	GPIOB_PCLK_EN();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)	GPIOC_PCLK_EN();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)	GPIOD_PCLK_EN();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)	GPIOE_PCLK_EN();
	}
	else
	{
		if(pGPIOx == (GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)	GPIOA_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)	GPIOB_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)	GPIOC_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)	GPIOD_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)	GPIOE_PCLK_DI();
	}
}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{

}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{

}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

// *** GPIO Interrupt Config and Handling ***
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
