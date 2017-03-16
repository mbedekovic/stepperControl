/* usart.h */
#ifndef USART_H
#define USART_H

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"


#define BUFSIZE				20*20
#define BAUDRATE			115200
#define PACKSIZE			20					//size of data packet in bytes

void USART1_Init(void);						// USART1 init function
void USART1_SendChar(char c);			// blocking send character function
int USART1_Dequeue(char *c);			// pop character from receive FIFO


//Deffered interupt handling task for USART
void vDecodeMsgTask(void *pvParameters);
//For passing task handle to Rx ISR
extern xTaskHandle xDecodeMsgTaskHandle;

//Queue handles
	//Motor control setpoint msg queues
	extern xQueueHandle xQueueMotorSetpoint[4];
	//Queue for passing controller setup message from UART to motor controllers 
	extern xQueueHandle xQueueMotorSetup;

//sizeof(motor_control_t) = 20 -> 3 padding bytes added for word aligment of struct 
typedef struct
{
	int32_t moto1;
	int32_t moto2;
	int32_t moto3;
	int32_t moto4;
	uint8_t	 cmd;
} motor_control_t;

//Structure for motor controller setup message 
typedef struct
{
	uint32_t P;
	uint32_t wMax;
	uint32_t rLim;
	uint32_t dLim;
}	motor_setup_t;

struct __FILE
{
	int dummy;
};


#endif
