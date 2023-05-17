#include "stm32f3xx_hal.h"


#define SERVO_MAX_US 	2500
#define SERVO_MIN_US 	1000

#define SERVO_MAX_ANGLE 	180
#define SERVO_MIN_ANGLE 	0

#define SERVO_MAX_SPEED 	100


void servo_init(TIM_HandleTypeDef *, uint32_t);
void servo_set_angle(uint8_t);
void servo_set_speed(int);
