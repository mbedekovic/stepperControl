/*timer.h*/
#ifndef TIMER_H
#define TIMER_H

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>

#define CONTROL_LOOP_FREQUENCY	(uint32_t)100
#define TIMER_FREQUENCY					(uint32_t)10
#define PERIOD									(uint32_t)64000
#define PRESCALER								(uint16_t)41


//Function declarations:
	
	//Timer init function 
	void TimerInit(void);
		
	//TIM4 Interrupt handler 
	void TIM4_IRQHandler(void);
	

	
#endif 
