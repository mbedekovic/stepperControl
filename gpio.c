/* gpio.c */
#include <gpio.h>

void gpio_init(void)
{
		GPIO_InitTypeDef GPIOStruct;
		GPIO_StructInit(&GPIOStruct);
		//Turn on clock for peripheral 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
		//Pin configuration 
		GPIOStruct.GPIO_Pin		= GPIO_Pin_12;
		GPIOStruct.GPIO_Mode 	= GPIO_Mode_OUT;
		GPIOStruct.GPIO_OType = GPIO_OType_PP; 
		GPIOStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIOStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIOStruct);
	//Green LED
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
	//Blue LED
	GPIOStruct.GPIO_Pin    	= GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIOStruct);
	//Orange LED
	GPIOStruct.GPIO_Pin    	= GPIO_Pin_13;
	GPIO_Init(GPIOD,&GPIOStruct);
	
	RCC_AHB1PeriphClockCmd(MOTOR1_RCC_GPIOx, ENABLE);
		//Dir pin
		GPIOStruct.GPIO_Pin 	= MOTOR1_DIR;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		//Step pin
		GPIOStruct.GPIO_Pin 	= MOTOR1_STEP;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		//MS1
		GPIOStruct.GPIO_Pin 	= MOTOR1_MS1;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		//MS2
		GPIOStruct.GPIO_Pin 	= MOTOR1_MS2;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		//MS3
		GPIOStruct.GPIO_Pin 	= MOTOR1_MS3;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		
		//Blue Led
		GPIOStruct.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		
		//Reset dir pin
		GPIO_WriteBit(MOTOR1_GPIOx, MOTOR1_DIR, Bit_RESET);
		//Setting stepper driver in half step mode (400 pulses per revolution)
		GPIO_WriteBit(MOTOR1_GPIOx, MOTOR1_MS1, Bit_SET);
		GPIO_WriteBit(MOTOR1_GPIOx, MOTOR1_MS2, Bit_RESET);
		GPIO_WriteBit(MOTOR1_GPIOx, MOTOR1_MS3, Bit_RESET);
		
	//Motor 2 pin init 
	RCC_AHB1PeriphClockCmd(MOTOR2_RCC_GPIOx, ENABLE);
		//Dir pin motor2
		GPIOStruct.GPIO_Pin = MOTOR2_DIR;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		//Step pin motor2
		GPIOStruct.GPIO_Pin = MOTOR2_STEP;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		//MS1 motor2
		GPIOStruct.GPIO_Pin = MOTOR2_MS1;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		//MS2 motor2
		GPIOStruct.GPIO_Pin = MOTOR2_MS2;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		//MS3 motor2  
		GPIOStruct.GPIO_Pin = MOTOR2_MS3;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		
		//Reset dir pin
		GPIO_WriteBit(MOTOR2_GPIOx, MOTOR2_DIR, Bit_RESET);
		//Setting stepper driver in half step mode (400 pulses per revolution)
		GPIO_WriteBit(MOTOR2_GPIOx, MOTOR2_MS1, Bit_SET);
		GPIO_WriteBit(MOTOR2_GPIOx, MOTOR2_MS2, Bit_RESET);
		GPIO_WriteBit(MOTOR2_GPIOx, MOTOR2_MS3, Bit_RESET);
}
