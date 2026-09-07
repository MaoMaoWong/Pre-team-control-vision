/*
 * chassis.h
 *
 *  Created on: May 10, 2026
 *      Author: 陈思危
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include<stdint.h>
#include"main.h"
#include"gy53.h"

/*============================= 硬件参数配置 =============================*/
// 电机与编码器
#define MOTOR_NUM          4       // 电机总数
#define PWM_MAX           100    // PWM 最大占空比（对应计数器周期 1000-1）
#define ENCODER_RES        2496     // 编码器每转脉冲数（根据实际电机减速比和编码器线数调整）

// 控制周期 (ms)
#define CONTROL_PERIOD_MS  10      // 控制循环周期 10ms

// 运动学参数（麦轮底盘几何尺寸，单位：米）
#define WHEEL_BASE         0.212f    // 轮距（左右轮间距）
#define WHEEL_TRACK        0.179f    // 轴距（前后轮间距）
#define WHEEL_RADIUS       0.0325f // 轮子半径（米），根据实际轮子直径调整

/*============================= 结构体定义 =============================*/
// 运动指令（期望底盘运动）
typedef struct {
    float vx;   // 左右方向速度 (m/s)，向右为正
    float vy;   // 前后方向速度 (m/s)，向前为正
    float w;    // 旋转角速度 (rad/s)，逆时针为正
} MotionCmd;

// PID 参数结构
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_t;

/* 电机数组和 Motor_Set() 统一使用这一套物理顺序。 */
typedef enum {
    MOTOR_LF = 0,
    MOTOR_LB,
    MOTOR_RF,
    MOTOR_RB
} MotorIndex_t;

/*============================= 外部变量声明 =============================*/
// PID 参数（在 chassis.c 中定义）
extern PID_t pid_speed_LF;  // 左前轮速度 PID
extern PID_t pid_speed_RF;  // 右前轮速度 PID
// 后轮无编码器，不需要速度 PID

// 目标速度（来自运动学解算）
extern int16_t speed_tar[4];   // 索引 0:左前, 1:左后, 2:右前, 3:右后

// 实际速度（仅前轮有效）
extern int16_t speed_act[2];   // 0:左前, 1:右前

// 最终 PWM 输出
extern int16_t pwm_out[4];     // 索引 0:左前, 1:左后, 2:右前, 3:右后

// 运动指令（可由遥控器/视觉决策更新）
extern MotionCmd target_motion;

// 陀螺仪相关
extern float yaw_angle;        // 当前偏航角（弧度）
extern float target_yaw;       // 期望偏航角（用于自动纠偏）

// 调试用变量（可选）
extern uint32_t loop_counter;  // 控制循环计数器


/*============================= 函数原型 =============================*/
// 初始化函数
bool PWM_Init(void);            // 初始化 TIM2 四路 PWM
bool Encoder_Init(void);        // 初始化 TIM4(左前) 和 TIM1(右前) 编码器
bool Control_Init(void);        // 总体初始化，依次调用上述初始化

// 运动学与补偿
void Mecanum_Kinematics(float vx, float vy, float w, int16_t *tar_speed);
void Yaw_Compensate(MotionCmd *cmd);   // 根据当前偏航角修正 cmd->w

// PID 相关
float PID_Update(PID_t *pid, float target, float current);
void PID_Reset(PID_t *pid);            // 重置积分和误差

// 速度计算（从编码器脉冲计算实际转速）
int16_t CalcSpeedFromEncoder(int32_t encoder_delta, uint32_t dt_ms);

// 控制循环（在定时器中断中调用）
void Control_Loop(void);        // 周期执行：读编码器→读陀螺仪→运动学解算→PID→输出PWM

// 电机底层控制
void Motor_Set(int motor_idx, short int pwm);
// motor_idx: 0~3 对应 LF, LB, RF, RB
// direction: 0/1 对应正反转（根据驱动板逻辑定义）

// 辅助函数
int16_t Limit_PWM(int16_t pwm);
void Send_Data(void);           // 通过 USB 虚拟串口发送调试数据

//急停或者停止时让PID和轮子转速变成0
void Chassis_Stop(void);
void Chassis_Enable(void);



typedef enum {
    FIELD_SIDE_LEFT = 0,
    FIELD_SIDE_RIGHT
} FieldSide_t;

bool Chassis_SetFieldSide(FieldSide_t side);//确定在左半区还是右半区

/*移动结果*/
typedef enum {
    CHASSIS_MOVE_IDLE = 0,
    CHASSIS_MOVE_RUNNING,
    CHASSIS_MOVE_ARRIVED,
    CHASSIS_MOVE_TIMEOUT,
    CHASSIS_MOVE_TOF_LOST,
    CHASSIS_MOVE_MPU_LOST
} ChassisMoveState_t;

/*发射地与起始点的转换*/

uint8_t Chassis_IsArrived(void);
bool Chassis_GotoFirePoint(void);
bool Chassis_GotoStartPoint(void);
ChassisMoveState_t Chassis_GetMoveState(void);




#endif /* INC_CHASSIS_H_ */
