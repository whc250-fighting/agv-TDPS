#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/*
 * Motor_Init:      启动 TIM2 两路 PWM 输出，必须在 MX_TIM2_Init 之后调用。
 * Motor_SetSpeed:  设置左右轮速度，范围 -999~999，正值前转，负值后转。
 * Motor_Stop:      拉低 STBY 引脚，TB6612 进入待机，两路输出同时断开。
 */
void Motor_Init(void);
void Motor_SetSpeed(int left_speed, int right_speed);
void Motor_Stop(void);

#endif
