/*
 * shooter.c
 *
 *  Created on: May 10, 2026
 *      Author: 陈思危
 */

#include "tim.h"    // TIM3 句柄 htim3
#include "gpio.h"   // 如果供球电机是普通 GPIO

#include "shooter.h"
#include "servo.h"

#define SHOOTER_LEFT_CHANNEL     TIM_CHANNEL_2   // PA7
#define SHOOTER_RIGHT_CHANNEL    TIM_CHANNEL_3   // PB0

#define FEED_HOME_ANGLE          0
#define FEED_PUSH_ANGLE          90//供球舵机的两个角度，home是初始，push是固定的拨球


#define SHOOTER_IDLE_PWM         300//待机维持低转速

#define SHOOTER_MIN_FIRE_PWM     500
#define SHOOTER_MAX_FIRE_PWM     800

#define SHOOTER_SPINUP_MS        1000//提速等待时间(供球舵机等待发射电机)

#define FEED_PUSH_MS             500
#define FEED_BACK_MS             500

static ShooterState_t shooter_state = SHOOTER_DISABLED;//状态机
static uint32_t state_start_ms = 0;//当前状态开始的时间，判断等待，以及其他是否完成
static uint8_t shooter_done = 0;
static uint16_t fire_pwm = SHOOTER_MIN_FIRE_PWM;

static void Shooter_SetWheelPwm(uint16_t pwm)
{
    if (pwm > SHOOTER_MAX_FIRE_PWM) {
        pwm = SHOOTER_MAX_FIRE_PWM;
    }

    __HAL_TIM_SET_COMPARE(&htim3, SHOOTER_LEFT_CHANNEL, pwm);
    __HAL_TIM_SET_COMPARE(&htim3, SHOOTER_RIGHT_CHANNEL, pwm);
}//摩擦轮转速控制

bool Shooter_Init(void)
{
    if (HAL_TIM_PWM_Start(&htim3, SHOOTER_LEFT_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, SHOOTER_RIGHT_CHANNEL) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim3, SHOOTER_LEFT_CHANNEL);
        HAL_TIM_PWM_Stop(&htim3, SHOOTER_RIGHT_CHANNEL);
        shooter_state = SHOOTER_ERROR;
        return false;
    }

    Servo_SetAngle(2, FEED_HOME_ANGLE);
    Shooter_SetWheelPwm(0);

    shooter_state = SHOOTER_DISABLED;
    shooter_done = 0;
    return true;
}

void Shooter_Enable(void)
{
    Servo_SetAngle(2, FEED_HOME_ANGLE);
    Shooter_SetWheelPwm(SHOOTER_IDLE_PWM);
    shooter_state = SHOOTER_IDLE;
    shooter_done = 0;
}

bool Shooter_StartFire(uint8_t force_level)
{
    if (shooter_state != SHOOTER_IDLE && shooter_state != SHOOTER_DONE) {
        return false;
    }

    if (force_level < 1U || force_level > 10U) {
        return false;
    }

    fire_pwm = SHOOTER_MIN_FIRE_PWM +  (force_level - 1) * (SHOOTER_MAX_FIRE_PWM - SHOOTER_MIN_FIRE_PWM) / 9;

    shooter_done = 0;
    shooter_state = SHOOTER_SPINUP;//状态机切换
    return true;
}

void Shooter_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (shooter_state)
    {
        case SHOOTER_DISABLED:
            Shooter_SetWheelPwm(0);
            break;

        case SHOOTER_IDLE:
            Shooter_SetWheelPwm(SHOOTER_IDLE_PWM);
            break;

        case SHOOTER_SPINUP:
            Shooter_SetWheelPwm(fire_pwm);
            state_start_ms = now;
            shooter_state = SHOOTER_WAIT_STABLE;
            break;

        case SHOOTER_WAIT_STABLE:
            if (now - state_start_ms >= SHOOTER_SPINUP_MS) {
                Servo_SetAngle(2, FEED_PUSH_ANGLE);
                state_start_ms = now;
                shooter_state = SHOOTER_FEED_PUSH;
            }
            break;

        case SHOOTER_FEED_PUSH:
            if (now - state_start_ms >= FEED_PUSH_MS) {
                Servo_SetAngle(2, FEED_HOME_ANGLE);
                state_start_ms = now;
                shooter_state = SHOOTER_FEED_BACK;
            }
            break;

        case SHOOTER_FEED_BACK:
            if (now - state_start_ms >= FEED_BACK_MS) {
                Shooter_SetWheelPwm(SHOOTER_IDLE_PWM);
                shooter_done = 1;
                shooter_state = SHOOTER_DONE;
            }
            break;

        case SHOOTER_DONE:
            break;

        case SHOOTER_ERROR:
            Shooter_SetWheelPwm(0);
            Servo_SetAngle(2, FEED_HOME_ANGLE);
            break;

        default:
            Shooter_SetWheelPwm(0);
            Servo_SetAngle(2, FEED_HOME_ANGLE);
            shooter_done = 0;
            shooter_state = SHOOTER_ERROR;
            break;
    }
}
void Shooter_Stop(void)
{
    Shooter_SetWheelPwm(0);
    Servo_SetAngle(2, FEED_HOME_ANGLE);
    shooter_state = SHOOTER_DISABLED;
    shooter_done = 0;
}

uint8_t Shooter_IsBusy(void)
{
    return shooter_state == SHOOTER_SPINUP ||
           shooter_state == SHOOTER_WAIT_STABLE ||
           shooter_state == SHOOTER_FEED_PUSH ||
           shooter_state == SHOOTER_FEED_BACK;
}

uint8_t Shooter_IsDone(void)
{
    return shooter_done;
}

void Shooter_ClearDone(void)
{
    shooter_done = 0;
    if (shooter_state == SHOOTER_DONE) {
        shooter_state = SHOOTER_IDLE;
    }
}


bool Shooter_HasError(void)
{
    return shooter_state == SHOOTER_ERROR;
}
