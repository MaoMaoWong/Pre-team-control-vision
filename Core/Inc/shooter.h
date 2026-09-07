/*
 * shooter.h
 *
 *  Created on: May 10, 2026
 *      Author: 陈思危
 */

#ifndef INC_SHOOTER_H_
#define INC_SHOOTER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SHOOTER_DISABLED = 0,
    SHOOTER_IDLE,
    SHOOTER_SPINUP,
    SHOOTER_WAIT_STABLE,
    SHOOTER_FEED_PUSH,
    SHOOTER_FEED_BACK,
    SHOOTER_DONE,
    SHOOTER_ERROR
} ShooterState_t;//不同状态

bool Shooter_Init(void);
void Shooter_Enable(void);
void Shooter_Task(void);
bool Shooter_StartFire(uint8_t force_level);
void Shooter_Stop(void);

uint8_t Shooter_IsBusy(void);
uint8_t Shooter_IsDone(void);
void Shooter_ClearDone(void);
bool Shooter_HasError(void);//把发射器故障传给robot
#endif /* INC_SHOOTER_H_ */
