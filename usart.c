/* usart.c */

#include <usart.h>

//RX FIFO buffer
char RX_BUFFER[BUFSIZE];
int RX_BUFFER_HEAD, RX_BUFFER_TAIL;


//TX state flag
uint8_t TxReady;

//init USART1
void USART1_Init(void)
{
	GPIO_InitTypeDef 			GPIO_InitStruct;
	USART_InitTypeDef			USART_InitStruct;
	NVIC_InitTypeDef			NVIC_InitStructure;
	
	// enable peripheral clocks (note: different bus interfaces for each peripheral)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	// map port B pins for alternate function
	GPIO_InitStruct.GPIO_Pin 		= GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (Tx) and 7 (Rx) for USART
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;						 //alternate function of pins
	GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_50MHz; 			 // I/O pins speed (signal rise time)
	GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP; 					 // Push-pull output
	GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_UP; 					 // internal pullup resistor active
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//set alternate function to USART1 (from multiple possible alternate function choices)
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //pins will be automatically assigned
				// -> see datasheet
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	// configure USART1 peripheral with right typdef struct 
	USART_InitStruct.USART_BaudRate							= BAUDRATE;
	USART_InitStruct.USART_WordLength 					= USART_WordLength_8b;		// 8 data bits
	USART_InitStruct.USART_StopBits							= USART_StopBits_1;				// 1 stop bit
	USART_InitStruct.USART_Parity								= USART_Parity_No;				// no parity check
	USART_InitStruct.USART_HardwareFlowControl 	=	USART_HardwareFlowControl_None; // no HW flow ctl
	USART_InitStruct.USART_Mode									= USART_Mode_Tx | USART_Mode_Rx; // both Rx & Tx
	USART_Init(USART1, &USART_InitStruct);
	
	// set interrupt triggers for USART1 ISR but not enabeling interrupts jet
	USART_ITConfig(USART1, USART_IT_TXE, 	DISABLE);
	USART_ITConfig(USART1, USART_IT_TC, 	ENABLE); // enabled for registering completed event (TxReady)
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // for regisering new received char
	
	TxReady = 1;	// ready to send 
	RX_BUFFER_HEAD = 0;		RX_BUFFER_TAIL = 0;		// clear rx buffer
	
	// setting the NVIC to receive USART1 irqs
	NVIC_InitStructure.NVIC_IRQChannel										= USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0; //max priority 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	5;		//Priority must be logicali lower or equal to configMAX_SYSCALL_INTERRUPT_PRIORITY
																																//	when using FreeRTOS API functions in ISR
	NVIC_InitStructure.NVIC_IRQChannelSubPriority					= 0;	// max subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd									= ENABLE;	// enable USART1 interrupt IN NVIC
	NVIC_Init(&NVIC_InitStructure);	// write to NVIC register bank
	
	// enable irq generation in USART1
	USART_Cmd(USART1, ENABLE);
}


void USART1_IRQHandler(void)
{
	static int rx_char;
	static int rx_head;
	static uint16_t bytesRecieved = 0;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;			//For task notification mechanisam
	
	//Rx event
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			rx_char = USART_ReceiveData(USART1);
			// check for buffer overrun
			rx_head = RX_BUFFER_HEAD+1;
			if (rx_head == BUFSIZE) rx_head = 0;
			if (rx_head != RX_BUFFER_TAIL)
			{
				// adding new char willl not cause buffer overrun:
				RX_BUFFER[RX_BUFFER_HEAD]	=	rx_char;
				RX_BUFFER_HEAD = rx_head; // update head
				++bytesRecieved; 		//Incement the bytes counter
			}
			
			
			if (bytesRecieved == PACKSIZE)
			{
				bytesRecieved = 0;						//Reset the counter
				//after filling the buffer to message size,
				//notify the blocked task vDecodeMsgTask via it's handle to do it's thing
				vTaskNotifyGiveFromISR(xDecodeMsgTaskHandle, &xHigherPriorityTaskWoken);
			}
			//call context switch
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

	// Tx event
	if (USART_GetITStatus(USART1, USART_IT_TC) == SET)
	{
			USART_ClearITPendingBit(USART1, USART_IT_TC);
			TxReady = 1;
	}
}

void USART1_SendChar(char c)
{
	while(!TxReady);
	USART_SendData(USART1, c);
	TxReady = 0;
}

int USART1_Dequeue(char* c)
{
	int ret;
	
		ret = 0;
		*c	= 0;
		NVIC_DisableIRQ(USART1_IRQn);
		// start of critical section
		if (RX_BUFFER_HEAD != RX_BUFFER_TAIL)
		{
			*c	= RX_BUFFER[RX_BUFFER_TAIL];
			RX_BUFFER_TAIL++;
			if (RX_BUFFER_TAIL == BUFSIZE) RX_BUFFER_TAIL = 0;
			ret = 1;
		}
		// end of critical section
		NVIC_EnableIRQ(USART1_IRQn);
		return ret;
}



/**
	* @brief This function executes when there is one whole message frame in input 
	*				buffer. It decodes the received message and dispatches received info
	*				to other tasks. This task is unblocked by notfication from USART ISR handler 
	*/

void vDecodeMsgTask(void *pvParameters)
{
	union Data
		{
			motor_control_t broj;
			char str[20];
		} data;
		
	uint16_t i;
		
	while(1)
	{
		//If there is notification do some work  
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY  );			
		
		//Fill up data structure 
		for(i=0; i<20; i++)
		{
			USART1_Dequeue(&data.str[i]);
		}
		//Padding bytes set to zero 
		data.str[19] = 0;
		data.str[18] = 0;
		data.str[17] = 0;
		
		//From cmd data field decide where to send the data

		switch((char)data.broj.cmd)
		{
			case('C'):
			{
				//Send to motor setpoint queues
				int32_t ref = data.broj.moto1;
				//Send to appropriate setpoint queues. Queues hold only 1 element,
				// and xQueueOverwrite allways inserts the last received value to 
				// queue even if it's full (overwrite)
				xQueueOverwrite(xQueueMotorSetpoint[0],
												(void *)&ref);
												
				ref = data.broj.moto2;
				xQueueOverwrite(xQueueMotorSetpoint[1],
												(void *)&ref);
				
				ref = data.broj.moto3;
				xQueueOverwrite(xQueueMotorSetpoint[2],
												(void *)&ref);
				
				ref = data.broj.moto4;
				xQueueOverwrite(xQueueMotorSetpoint[3],
												(void *)&ref);
				
				//In final version send to all setpoint queues
				break;
										
			}
			case('S'):
			{
				// Setup msg for all motor controllers 
				
					//Form a message 
					motor_setup_t setupCmd;
						setupCmd.P 		= data.broj.moto1;
						setupCmd.wMax = data.broj.moto2;
						setupCmd.rLim = data.broj.moto3;
						setupCmd.dLim = data.broj.moto4;
					//Add to queue without blocking 
					xQueueOverwrite( xQueueMotorSetup[0],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[1],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[2],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[3],
														(void *)&setupCmd);
				break;
			}
			case('M'):
			{
				//Set global shared pule counter variables in critical section
				NVIC_DisableIRQ(TIM4_IRQn);
					pulsCnt1 = data.broj.moto1;
					pulsCnt2 = data.broj.moto2;
					pulsCnt3 = data.broj.moto3;
					pulsCnt4 = data.broj.moto4;
				NVIC_EnableIRQ(TIM4_IRQn);
			}
			default:
				break;
		}
				
	}
}
