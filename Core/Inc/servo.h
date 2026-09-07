/*
 * servo.h
 *
 *  Created on: Apr 5, 2026
 *      Author: 陈思危
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

void Servo_Init(void);               // 初始化舵机相关定时器及默认角度
void Servo_SetAngle(uint8_t ch, uint8_t angle);  // ch: 0或1, angle: 0~180
void Servo_TimerTick(void);

#endif /* INC_SERVO_H_ */
