/*
 * robot.c
 *
 *  Created on: Jun 2, 2026
 *      Author: 陈思危
 */
#include "robot.h"
#include "shooter.h"
#include "chassis.h"
#include <string.h>
#include "servo.h"

#define TARGET_WORD_COUNT       2
#define RADICALS_PER_WORD       3
#define GRID_COUNT              9

static RobotState_t robot_state = ROBOT_IDLE;

static uint8_t target_words[TARGET_WORD_COUNT] = {0};//保存两个目标字

static uint8_t current_word_idx = 0;
static uint8_t current_radical_idx = 0;//当前字和偏旁

static GridInfo grid_info[GRID_COUNT];
static uint8_t grid_ready = 0;
static uint8_t reload_confirmed = 0;

static const uint8_t word_radicals[4][RADICALS_PER_WORD] = {
    {RAD_HE,   RAD_REN,  RAD_SHUI}, // 黍
    {RAD_WANG, RAD_SHAN, RAD_ER},   // 瑞
    {RAD_HE,   RAD_KOU,  RAD_WANG}, // 程
    {RAD_REN,  RAD_YU,   RAD_ER}    // 儒
};

typedef struct {
    uint8_t horizontal_angle;  // 水平舵机角度
    uint8_t pitch_angle;       // 俯仰舵机角度
    uint8_t force_level;       // 发射力度，1～10
    bool calibrated;          // 是否完成实测标定
} AimConfig_t;

/* 未标定时全部为0，禁止自动发射 */
static const AimConfig_t aim_table[3][3] = {0};

static uint8_t current_force_level = 0;


#define ROBOT_GRID_TIMEOUT_MS   5000U
#define ROBOT_MOVE_TIMEOUT_MS  20000U
#define ROBOT_SHOOT_TIMEOUT_MS  5000U
#define ROBOT_AIM_STABLE_MS      500U
#define ROBOT_RELOAD_WAIT_MS     2000U

static uint32_t state_enter_ms;

static void Robot_SetState(RobotState_t next)
{
    robot_state = next;
    state_enter_ms = HAL_GetTick();
    if (next == ROBOT_WAIT_RELOAD)
        {
            reload_confirmed = 0;
            Chassis_Stop();
            Shooter_Stop();
        }
}//状态入口

static void Robot_StopAllActuators(void)
{
    Shooter_Stop();
    Chassis_Stop();
    Servo_SetAngle(0, 90U);
    Servo_SetAngle(1, 150U);
}

static void Robot_EnterError(void)
{
    Robot_StopAllActuators();
    Robot_SetState(ROBOT_ERROR);
}

static int8_t Robot_FindGridByRadical(uint8_t radical_id)
{
    for (uint8_t i = 0; i < GRID_COUNT; i++) {
        if (grid_info[i].radical_id == radical_id) {
            return i;
        }
    }
    return -1;
}

static uint8_t Robot_GetCurrentRadical(void)
{
    uint8_t word = target_words[current_word_idx];

    if (word >= 4) {
        return 0;
    }

    return word_radicals[word][current_radical_idx];
}

static void Robot_GotoNextRadical(void)
{
    current_radical_idx++;

    if (current_radical_idx >= RADICALS_PER_WORD) {
        current_radical_idx = 0;
        current_word_idx++;
    }

    if (current_word_idx >= TARGET_WORD_COUNT) {
        Robot_StopAllActuators();
        Robot_SetState(ROBOT_DONE);
    } else {
    	Robot_SetState(ROBOT_WAIT_RELOAD);
    }
}


void Robot_Init(void)
{
    Robot_SetState(ROBOT_IDLE);
    current_word_idx = 0;
    current_radical_idx = 0;
    grid_ready = 0;
    reload_confirmed = 0;
    memset(grid_info, 0, sizeof(grid_info));
}

bool Robot_StartMatch(uint8_t word1, uint8_t word2, uint8_t field_side)
{
    if (word1 >= 4 || word2 >= 4 || field_side > FIELD_SIDE_RIGHT)
    {
        return false;
    }

    /* 执行过程中，不允许新开始命令重置任务 */
    if (robot_state != ROBOT_IDLE)
    {
        return false;
    }

    if (!Chassis_SetFieldSide((FieldSide_t)field_side))
    {
        return false;
    }

    Robot_StopAllActuators();

    target_words[0] = word1;
    target_words[1] = word2;

    current_word_idx = 0;
    current_radical_idx = 0;
    reload_confirmed = 0;

    /* 每轮比赛重新获取棋盘 */
    grid_ready = 0;
    memset(grid_info, 0, sizeof(grid_info));

    Robot_SetState(ROBOT_WAIT_GRID);

    return true;
}


bool Robot_UpdateGrid(const GridInfo *grids)
{
    if (grids == NULL || robot_state != ROBOT_WAIT_GRID)
    {
        return false;
    }

    uint16_t radical_mask = 0U;

    for (uint8_t i = 0; i < GRID_COUNT; i++)
    {
        uint16_t radical_bit;

        if (grids[i].radical_id > RAD_MU || grids[i].occupied > 2U ||
            grids[i].reserved[0] != 0U || grids[i].reserved[1] != 0U)
        {
            Robot_EnterError();
            return false;
        }

        radical_bit = (uint16_t)(1U << grids[i].radical_id);
        if ((radical_mask & radical_bit) != 0U)
        {
            Robot_EnterError();
            return false;
        }
        radical_mask |= radical_bit;
    }

    memcpy(grid_info, grids, sizeof(grid_info));
    grid_ready = 1;

    Robot_SetState(ROBOT_READY);
    return true;
}


void Robot_Task(void)
{
	uint32_t elapsed = HAL_GetTick() - state_enter_ms;
	uint32_t timeout_ms = 0;

	switch (robot_state)
	{
	    case ROBOT_WAIT_GRID:
	        timeout_ms = ROBOT_GRID_TIMEOUT_MS;
	        break;

	    case ROBOT_MOVE_TO_FIRE:
	    case ROBOT_MOVE_TO_START:
	        timeout_ms = ROBOT_MOVE_TIMEOUT_MS;
	        break;

	    case ROBOT_WAIT_SHOOT_DONE:
	        timeout_ms = ROBOT_SHOOT_TIMEOUT_MS;
	        break;

	    default:
	        break;
	}

	if (timeout_ms != 0U && elapsed >= timeout_ms)
	{
	    Robot_EnterError();
	    return;
	}


    switch (robot_state)
    {
        case ROBOT_IDLE:

        case ROBOT_WAIT_GRID:

            break;

        case ROBOT_DONE:
        case ROBOT_ERROR:
            Robot_StopAllActuators();
            break;

        case ROBOT_READY:
        	if (Chassis_GotoFirePoint())
        	{
        	    Robot_SetState(ROBOT_MOVE_TO_FIRE);
        	}
        	else
        	{
        	    Robot_EnterError();
        	}
            break;

        case ROBOT_MOVE_TO_FIRE:
        {
            ChassisMoveState_t move = Chassis_GetMoveState();

            if (move == CHASSIS_MOVE_ARRIVED)
            {
                Chassis_Stop();
                Shooter_Enable();
                Robot_SetState(ROBOT_AIM);
            }
            else if (move != CHASSIS_MOVE_RUNNING)
            {
                Robot_EnterError();
            }
            break;
        }

        case ROBOT_AIM:
        {
            uint8_t radical = Robot_GetCurrentRadical();
            int8_t grid_idx = Robot_FindGridByRadical(radical);

            if (grid_idx < 0 || grid_idx >= GRID_COUNT)
            {
                Robot_EnterError();
                break;
            }

            uint8_t row = (uint8_t)grid_idx / 3U;
            uint8_t col = (uint8_t)grid_idx % 3U;

            const AimConfig_t *aim = &aim_table[row][col];

            if (!aim->calibrated ||
                aim->horizontal_angle > 180U ||
                aim->pitch_angle > 180U ||
                aim->force_level < 1U ||
                aim->force_level > 10U)
            {
                Robot_EnterError();
                break;
            }

            Chassis_Stop();

            current_force_level = aim->force_level;

            Servo_SetAngle(0, aim->horizontal_angle);
            Servo_SetAngle(1, aim->pitch_angle);

            Robot_SetState(ROBOT_WAIT_AIM_STABLE);
            break;
        }

        case ROBOT_WAIT_AIM_STABLE:
            if ((uint32_t)(HAL_GetTick() - state_enter_ms) >= ROBOT_AIM_STABLE_MS)
            {
                Robot_SetState(ROBOT_SHOOT);
            }
            break;

        case ROBOT_SHOOT:
            if (Shooter_StartFire(current_force_level))
            {
                Robot_SetState(ROBOT_WAIT_SHOOT_DONE);
            }
            else
            {
                Robot_EnterError();
            }
            break;

        case ROBOT_WAIT_SHOOT_DONE:
        	if (Shooter_HasError())
        	    {
        	        Robot_EnterError();
        	        break;
        	    }
            if (Shooter_IsDone()) {
                Shooter_ClearDone();
                /* 发射完成后立即关闭摩擦轮和供球机构 */
                Shooter_Stop();
                if (Chassis_GotoStartPoint())
                {
                  Robot_SetState(ROBOT_MOVE_TO_START);
                }
                 else
                {
                   Robot_EnterError();
                }
            }
            break;

        case ROBOT_MOVE_TO_START:
        {
            ChassisMoveState_t move = Chassis_GetMoveState();

            if (move == CHASSIS_MOVE_ARRIVED)
            {
                Chassis_Stop();
                Robot_GotoNextRadical();
            }
            else if (move != CHASSIS_MOVE_RUNNING)
            {
                Robot_EnterError();
            }
            break;
        }
        case ROBOT_WAIT_RELOAD:
            if (reload_confirmed ||
                (uint32_t)(HAL_GetTick() - state_enter_ms) >= ROBOT_RELOAD_WAIT_MS)
            {
                reload_confirmed = 0;
                Robot_SetState(ROBOT_READY);
            }
            break;

        default:
            Robot_EnterError();
            break;
    }
}

void Robot_EmergencyStop(void)
{
    Robot_EnterError();
}
uint8_t Robot_UpdateTargetWords(uint8_t word1, uint8_t word2)
{
    /* 当前只有4组目标字，编号0～3 */
    if (word1 >= 4 || word2 >= 4)
    {
        return 0;
    }

    /* 防止发射过程中更换目标 */
    if (robot_state != ROBOT_IDLE &&
        robot_state != ROBOT_WAIT_GRID)
    {
        return 0;
    }

    target_words[0] = word1;
    target_words[1] = word2;

    return 1;
}

void Robot_ConfirmReload(void)
{
    if (robot_state == ROBOT_WAIT_RELOAD) {
        reload_confirmed = 1;
    }
}
void Robot_StopMatch(void)
{
    Robot_StopAllActuators();

    grid_ready = 0;
    reload_confirmed = 0;
    current_word_idx = 0;
    current_radical_idx = 0;

    Robot_SetState(ROBOT_IDLE);
}

RobotState_t Robot_GetState(void)
{
    return robot_state;
}
