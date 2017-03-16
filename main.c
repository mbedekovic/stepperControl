/* main.c */
#include <main.h>

/**
	* \author Bruno Maric
	*	\email <bruno.maric@fer.hr>
	* \author Gorna Popovic
	* \email <goran.popovic@fer.hr>
	* \author Matija Bedekovic 
	* \email <matija.bedekovic2@fer.hr>
	*/
uint64_t u64Ticks = 0; //Counts OS ticks (default = 1000 Hz)
uint64_t u64IdleTicks = 0; //Value of u64IdleTicksCnt is copied once per sec.
uint64_t u64IdleTicksCnt = 0; //Counts when the OS has no task to execute.
//This FreeRTOS callback funtion gets called once per tick (default 1000 Hz).
//---------------------------------------------------------------------------
void vApplicationTickHook(void)
{
	++u64Ticks;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ----------------------------------------------------------------------------
void vApplicationIdleHook( void )	
{
++u64IdleTicksCnt;
}
// A required FreeRTOS function.
// ----------------------------------------------------------------------------
void vApplicationMallocFailedHook( void )
{
configASSERT( 0 ); // Latch on any failure / error.
}

//Task handles
	xTaskHandle xDecodeMsgTaskHandle;	

//Queue Handles
 xQueueHandle xQueueMotorISR[4];
 xQueueHandle xQueueMotorSetpoint[4];
 xQueueHandle xQueueMotorSetup;

int main(void)
{
	
	
/*********************************************************************/	
	//Queue Create
	
	
	//Queue for setup task
	xQueueMotorSetup = xQueueCreate(3, sizeof(motor_setup_t));
	//Queues for setpoint tasks 
	xQueueMotorSetpoint[0] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[1] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[2] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[3] = xQueueCreate(1, sizeof(int32_t));
	//Queues for timer output compare ISR 
	xQueueMotorISR[0] = xQueueCreate(1, sizeof(ISR_message_t));
	xQueueMotorISR[1] = xQueueCreate(1, sizeof(ISR_message_t));
	xQueueMotorISR[2] = xQueueCreate(1, sizeof(ISR_message_t));
	xQueueMotorISR[3] = xQueueCreate(1, sizeof(ISR_message_t));
	
	//It is recomended to set all priority bits to preempt priority when using FreeRTOS 
	// on STM32 devices
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); 	
	USART1_Init();		//USART1 init function
	gpio_init();			//GPIO init function
	TimerInit();			//TIM4 init function 		
	
	
	
/*********************************************************************/
	//Task Create
		
	//Usart Task
	xTaskCreate(
		vDecodeMsgTask,
		(const char*)"MSG_DECODE_TASK",
		configMINIMAL_STACK_SIZE,
		NULL,
		2,		
		&xDecodeMsgTaskHandle); 
		

		
	//Controller tasks - for now only 2 tasks are created because we use only 2 motors 
		//									When we add remaining 2 motors new 2 task will be created 
		xTaskCreate(
			 vTaskMotorController,
			(const char*)"MOTOR1_TASK",
			configMINIMAL_STACK_SIZE,
			(void *)1,
			2,
			NULL); 
		
		xTaskCreate(
			 vTaskMotorController,
			(const char*)"MOTOR2_TASK",
			configMINIMAL_STACK_SIZE,
			(void *)2,
			2,
			NULL); 
			
		xTaskCreate(
			 vTaskMotorController,
			(const char*)"MOTOR3_TASK",
			configMINIMAL_STACK_SIZE,
			(void *)3,
			2,
			NULL); 
			
		xTaskCreate(
			 vTaskMotorController,
			(const char*)"MOTOR4_TASK",
			configMINIMAL_STACK_SIZE,
			(void *)4,
			2,
			NULL); 
			
	//run task scheduler
		vTaskStartScheduler();
	//While(1) for safety
			while(1)
			{
				//Should never get here 
			}
}

