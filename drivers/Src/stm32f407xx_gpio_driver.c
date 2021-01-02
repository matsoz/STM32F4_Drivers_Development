/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 28, 2020
 *      Author: mathe
 */


#include "stm32f407xx_gpio_driver.h"

// ********** GPIO APIs function definition **********

// *** GPIO Initialization and Control ***
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) //Initialize GPIO Port and Pin
{
	pRCC_RegDef();

	uint32_t Temp = 0;

	//Configure MODE
	if(pGPIOHandle->GPIO_PinConfig.PinMode < 3) //Non interrupt mode
	{
		Temp = pGPIOHandle->GPIO_PinConfig.PinMode << (2* pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOBaseAddr->MODER &= ~(0x11 << 2* pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOBaseAddr->MODER |= Temp;
	}
	else
	{
		//Configure interrupt trigger (Rising and/or falling edge)
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.PinNumber);
		}

		//Configure the GPIO Port selected in the EXTI line
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
		uint8_t portcode = GPIO_BASE_ADDRESS_TO_CODE(pGPIOHandle->pGPIOBaseAddr);

		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<temp2*4;

		//Enable EXTI interrupt delivery by IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.PinNumber;
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
	pRCC_RegDef();
	if(pGPIO == (GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)	GPIOA_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)	GPIOB_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)	GPIOC_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)	GPIOD_REG_RESET();
	if(pGPIO == (GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)	GPIOE_REG_RESET();
}

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable) //Enable / Disable GPIO clock
{
	pRCC_RegDef();
	if(Enable == ENABLE)
	{

		if(pGPIOx == (GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
		{
			pGPIOA_RegDef();
			GPIOA_PCLK_EN();
		}
		if(pGPIOx == (GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
		{
			pGPIOB_RegDef();
			GPIOB_PCLK_EN();
		}
		if(pGPIOx == (GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
		{
			pGPIOC_RegDef();
			GPIOC_PCLK_EN();
		}
		if(pGPIOx == (GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
		{
			pGPIOD_RegDef();
			GPIOD_PCLK_EN();
		}
		if(pGPIOx == (GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
		{
			pGPIOE_RegDef();
			GPIOE_PCLK_EN();
		}
		if(pGPIOx == (GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
		{
			pGPIOF_RegDef();
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == (GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)	GPIOA_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)	GPIOB_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)	GPIOC_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)	GPIOD_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)	GPIOE_PCLK_DI();
		if(pGPIOx == (GPIO_RegDef_t*) GPIOF_BASE_ADDRESS)	GPIOF_PCLK_DI();
	}
}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
		uint8_t Value;
		Value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
		return Value;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value = (uint16_t) (pGPIOx->IDR);
	return Value;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else if(Value == RESET)
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}

}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = (uint16_t) Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

// *** GPIO Interrupt Config and Handling ***
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable)
{
	//Set or Reset IRQ Number
	if(Enable == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
			}
		}
	else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}
		}

	//Set or Reset IRQ Priority

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NVIC_NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDRESS + iprx ) |=  ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear pending register for corresponding Pin Number
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber); //Clear
	}
}
