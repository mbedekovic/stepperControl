/* gpio.h */
#ifndef GPIO_H
#define GPIO_H
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
// GPIO RCC Peripheral defines 
#define MOTOR1_RCC_GPIOx 				RCC_AHB1Periph_GPIOA
#define MOTOR2_RCC_GPIOx				RCC_AHB1Periph_GPIOD
#define MOTOR3_RCC_GPIOx				RCC_AHB1Periph_GPIOC
#define MOTOR4_RCC_GPIOx				RCC_AHB1Periph_GPIOE

// GPIO Peripheral defines
#define MOTOR1_GPIOx						GPIOA
#define MOTOR2_GPIOx						GPIOD
	//motor 3 is a little special due to soldering errors
#define MOTOR3_GPIOx_DIR				GPIOD
#define MOTOR3_GPIOx_STP_MS1		GPIOC
#define MOTOR3_GPIOx_MS23				GPIOA
#define MOTOR4_GPIOx						GPIOE

// Pin numbers for motors
	//Motor 1 
#define MOTOR1_DIR							GPIO_Pin_1
#define MOTOR1_STEP							GPIO_Pin_2
#define MOTOR1_MS1							GPIO_Pin_3
#define MOTOR1_MS2							GPIO_Pin_4
#define MOTOR1_MS3							GPIO_Pin_5

	//Motor 2
#define MOTOR2_DIR							GPIO_Pin_10
#define MOTOR2_STEP							GPIO_Pin_11
#define MOTOR2_MS1							GPIO_Pin_12
#define MOTOR2_MS2							GPIO_Pin_13
#define MOTOR2_MS3							GPIO_Pin_14

	//Motor 3
#define MOTOR3_DIR							GPIO_Pin_0
#define MOTOR3_STEP							GPIO_Pin_10
#define MOTOR3_MS1							GPIO_Pin_11
#define MOTOR3_MS2							GPIO_Pin_14
#define MOTOR3_MS3							GPIO_Pin_15

	//Motor 4
#define MOTOR4_DIR							GPIO_Pin_1
#define MOTOR4_STEP							GPIO_Pin_2
#define MOTOR4_MS1							GPIO_Pin_3
#define MOTOR4_MS2							GPIO_Pin_4
#define MOTOR4_MS3							GPIO_Pin_5


void gpio_init(void);

#endif
