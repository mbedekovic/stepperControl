/* controlAlgorithm.h */
#ifndef CONTROLALGORITHM_H
#define CONTROLALGORITHM_H

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <timer.h>
#include <usart.h>
#include <gpio.h>

//Typedefed message structs:
typedef struct
{
	int32_t delta;
	int16_t	 direction;
} ISR_message_t;


//Queue handles 

	//Motor control setpoint msg queues
	extern xQueueHandle xQueueMotorSetpoint[4];
	//Queue for sending delay between pulses to TIM4 ISR
	extern xQueueHandle xQueueMotorISR[4];
	//Queue for passing controller setup message from UART to motor controllers 
	extern xQueueHandle xQueueMotorSetup;
	
	
//Function declarations
	//Motor controller task function 
	void vTaskMotorController(void *pvParameters);
	//Absolute value function 
	int32_t absVal(int32_t value);
	//Function for determening a sign of integer number 
	int16_t sign(int32_t value);
	//Timer 4 interrupt service routine
	void TIM4_IRQHandler(void);
	
	//Crtitical section in reading pulse count
	int32_t getPulsCnt(uint32_t num);


#endif
