/*
 * servo.c
 *
 *  Created on: Apr 5, 2026
 *      Author: 陈思危
 */


#include"servo.h"

// 舵机引脚定义

#define SERVO0_PORT GPIOA
#define SERVO0_PIN  GPIO_PIN_6//供球电机2

#define SERVO1_PORT GPIOB
#define SERVO1_PIN  GPIO_PIN_1//0
#define SERVO2_PORT GPIOB
#define SERVO2_PIN  GPIO_PIN_5//1

#define SERVO0_HIGH() HAL_GPIO_WritePin(SERVO0_PORT, SERVO0_PIN, GPIO_PIN_SET)
#define SERVO0_LOW()  HAL_GPIO_WritePin(SERVO0_PORT, SERVO0_PIN, GPIO_PIN_RESET)
#define SERVO1_HIGH() HAL_GPIO_WritePin(SERVO1_PORT, SERVO1_PIN, GPIO_PIN_SET)
#define SERVO1_LOW()  HAL_GPIO_WritePin(SERVO1_PORT, SERVO1_PIN, GPIO_PIN_RESET)
#define SERVO2_HIGH() HAL_GPIO_WritePin(SERVO2_PORT, SERVO2_PIN, GPIO_PIN_SET)
#define SERVO2_LOW()  HAL_GPIO_WritePin(SERVO2_PORT, SERVO2_PIN, GPIO_PIN_RESET)

#define PWM_CYCLE 400
static uint8_t  servo_angle[3] = {90, 90, 0};
static uint16_t servo_pulse[3] = {30, 30, 10};
static uint16_t pwm_counter = 0;

static void Update_Servo_Pulse(uint8_t index, uint8_t angle)
{
    // 脉冲宽度 500us ~ 2500us 对应计数 10 ~ 50（50us一步）
    uint16_t pulse_cnt = 10 + (angle * 40 / 180);
    servo_pulse[index] = pulse_cnt;
}

void Servo_SetAngle(uint8_t ch, uint8_t angle)
{
	if (ch >= 3U)
	{
	    return;
	}
    if(angle > 180) angle = 180;
    servo_angle[ch] = angle;
    Update_Servo_Pulse(ch, angle);
}

void Servo_Init(void)
{
	Servo_SetAngle(2, 0);//0供球
    Servo_SetAngle(1, 150);
    Servo_SetAngle(0, 90);


}

void Servo_TimerTick(void)
{
    pwm_counter++;
    if(pwm_counter <= servo_pulse[0]) SERVO1_HIGH();
    else SERVO1_LOW();

    if(pwm_counter <= servo_pulse[1]) SERVO2_HIGH();
    else SERVO2_LOW();

    if(pwm_counter <= servo_pulse[2]) SERVO0_HIGH();
    else SERVO0_LOW();

    if(pwm_counter >= PWM_CYCLE) pwm_counter = 0;
}
