/* gpio.h */
#ifndef GPIO_H
#define GPIO_H
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
// GPIO RCC Peripheral defines 
#define MOTOR1_RCC_GPIOx 				RCC_AHB1Periph_GPIOE
#define MOTOR2_RCC_GPIOx				RCC_AHB1Periph_GPIOA

// GPIO Peripheral defines
#define MOTOR1_GPIOx						GPIOE
#define MOTOR2_GPIOx						GPIOA

// Pin numbers for motors
	//Motor 1 
#define MOTOR1_DIR							GPIO_Pin_8
#define MOTOR1_STEP							GPIO_Pin_9
#define MOTOR1_MS1							GPIO_Pin_10
#define MOTOR1_MS2							GPIO_Pin_11
#define MOTOR1_MS3							GPIO_Pin_12

	//Motor 2
#define MOTOR2_DIR							GPIO_Pin_2
#define MOTOR2_STEP							GPIO_Pin_3
#define MOTOR2_MS1							GPIO_Pin_4
#define MOTOR2_MS2							GPIO_Pin_5
#define MOTOR2_MS3							GPIO_Pin_6

void gpio_init(void);

#endif
