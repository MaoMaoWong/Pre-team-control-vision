/*
 * robot.h
 *
 *  Created on: Jun 2, 2026
 *      Author: 陈思危
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include <stdint.h>
#include "protocol.h"

typedef enum {
    ROBOT_IDLE = 0,
    ROBOT_WAIT_GRID,//已经有目标字，没有九宫格数据
    ROBOT_READY,//目标字和棋盘数据都有了，准备执行当前偏旁

	ROBOT_MOVE_TO_FIRE,   // 正在前往发射点
	ROBOT_AIM,//瞄准阶段。以后在这里控制水平/俯仰舵机。
	ROBOT_WAIT_AIM_STABLE,

    ROBOT_SHOOT,//开始发射。这里调用 Shooter_StartFire()发射
    ROBOT_WAIT_SHOOT_DONE,//等待 shooter.c 的发射状态机完成。
	ROBOT_MOVE_TO_START,  // 正在返回起点
    ROBOT_WAIT_RELOAD,//发完一颗球后，等待你回启动区重新放球并确认。
    ROBOT_DONE,//两个目标字都打完了。
    ROBOT_ERROR
} RobotState_t;

void Robot_Init(void);
void Robot_Task(void);

bool Robot_StartMatch(uint8_t word1, uint8_t word2, uint8_t field_side);
bool Robot_UpdateGrid(const GridInfo *grids);//保存两个目标字
void Robot_StopMatch(void);//正常停止比赛
void Robot_EmergencyStop(void);//突发停止
void Robot_ConfirmReload(void);//确认已经装好球
uint8_t Robot_UpdateTargetWords(uint8_t word1, uint8_t word2);

RobotState_t Robot_GetState(void);
#endif /* INC_ROBOT_H_ */
