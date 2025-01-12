/*
 * dshot.h
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 */


#ifndef __DSHOT_H__
#define __DSHOT_H__


#include "stm32g0xx_hal.h"
#include <stdbool.h>	
#include <math.h>		// lrintf


/* User Configuration */
// Timer Clock
#define TIMER_CLOCK				64000000	// 64MHz

#define MOTOR_1_TIM             (&htim2)
#define MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_2_TIM             (&htim2)
#define MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_2

#define MOTOR_3_TIM             (&htim2)
#define MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_3

#define MOTOR_4_TIM             (&htim2)
#define MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_4


/* Definition */
#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define MOTOR_BIT_0            	27
#define MOTOR_BIT_1            	53
#define MOTOR_BITLENGTH        	71

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      0
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)


/* Enumeration */
typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshot_type_e;


/* Functions */
void dshot_init(dshot_type_e dshot_type);
void dshot_write(uint16_t* motor_value);


#endif /* __DSHOT_H__ */
