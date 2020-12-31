/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 28, 2020
 *      Author: mathe
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

	#include "stm32f407xx.h"


	// *************************** Config. structure for the GPIO Pin ******************************
	typedef struct
	{
		uint8_t PinNumber;
		uint8_t PinMode;
		uint8_t PinSpeed;
		uint8_t PinPuPdControl;
		uint8_t PinOPType;
		uint8_t PinAltFunMode;
	}GPIO_PinConfig_t;

	// ******************************** Handle structure for the GPIO Pin ******************************
	typedef struct
	{
		GPIO_RegDef_t *pGPIOBaseAddr; //Holds the base address of the GPIO port which the pin belongs
		GPIO_PinConfig_t GPIO_PinConfig;
	}GPIO_Handle_t;


	// ****************************** GPIO Initialization and Control prototypes ******************************
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle); //Initialize GPIO Port and Pin
	void GPIO_DeInit(GPIO_RegDef_t *pGPIO); //De-Initialize GPIO Port and Pin
	void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable); //Enable / Disable GPIO clock

	uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
	uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
	void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
	void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
	void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

	//GPIO Interrupt Config and Handling
	void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable);
	void GPIO_IRQHandling(uint8_t PinNumber);



	// ******************************** Specific definitions for GPIO Handle function ******************************

	//GPIO Pin Number
	#define GPIO_PINNUM_0				0
	#define GPIO_PINNUM_1				1
	#define GPIO_PINNUM_2				2
	#define GPIO_PINNUM_3				3
	#define GPIO_PINNUM_4				4
	#define GPIO_PINNUM_5				5
	#define GPIO_PINNUM_6				6
	#define GPIO_PINNUM_7				7
	#define GPIO_PINNUM_8				8
	#define GPIO_PINNUM_9				9
	#define GPIO_PINNUM_10				10
	#define GPIO_PINNUM_11				11
	#define GPIO_PINNUM_12				12
	#define GPIO_PINNUM_13				13
	#define GPIO_PINNUM_14				14
	#define GPIO_PINNUM_15				15

	// GPIO Modes
	#define GPIO_MODE_INPUT				0
	#define GPIO_MODE_OUTPUT			1
	#define GPIO_MODE_AF				2
	#define GPIO_MODE_ANALOG			3
	#define GPIO_MODE_IT_FT				4
	#define GPIO_MODE_IT_RT				5
	#define GPIO_MODE_IT_RFT			6

	//GPIO Output Types
	#define GPIO_OP_TYPE_PP				0
	#define GPIO_OP_TYPE_OD				1

	//GPIO Output Speeds
	#define GPIO_SPEED_LOW				0
	#define GPIO_SPEED_MED				1
	#define GPIO_SPEED_FST				2
	#define GPIO_SPEED_HI				3

	//GPIO Pin Pull Up / Down config
	#define GPIO_PUPD_NO_PUPD			0
	#define GPIO_PUPD_PU				1
	#define GPIO_PUPD_PD				2

#endif
