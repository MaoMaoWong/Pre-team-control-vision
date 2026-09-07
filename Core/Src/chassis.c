/*
 * chassis.c
 *
 *  Created on: May 10, 2026
 *      Author: 陈思危
 */

#include"chassis.h"
#include "main.h"          // 包含 HAL 库定义及 GPIO/PWM 句柄
#include "tim.h"           // 包含 TIM2、TIM4、TIM1 等定时器句柄
#include "usb_device.h"    // 如果需要 USB 虚拟串口调试
#include <math.h>
#include"mpu6500.h"
#include<stdbool.h>

static ChassisMoveState_t fixed_move_state = CHASSIS_MOVE_IDLE;
static int32_t fixed_move_progress = 0;
static int32_t fixed_move_target_y_pulses = 0;

#define FIXED_MOVE_TIMEOUT_MS   15000U
#define TOF_TOLERANCE_MM        60.0f   /* 暂定，之后实测 */
#define TOF_MIN_SPEED            0.04f
#define TOF_KP                    0.0008f
#define Y_TOLERANCE_PULSES      250
#define ARRIVAL_STABLE_MS        200U
#define MPU_FAILURE_LIMIT          3U

static uint32_t fixed_move_start_ms;
static uint32_t arrival_candidate_ms;
static bool arrival_candidate_valid;
static uint8_t mpu_failure_count;
static bool mpu_sample_valid;

/*
 * 正X运动使测距：
 * 增大填+1，减小填-1。
 * ToF朝右测量右墙时应为-1。
 */
static int8_t tof_x_sign = -1;


/*
 * 调整规则：给对应电机正PWM时，
 * 处理后的编码器增量必须为正。
 */
#define ENCODER_LF_SIGN    (+1)
#define ENCODER_RF_SIGN    (+1)

static int32_t YMetersToCounts(float meters)
{
    return (int32_t)lroundf(
        fabsf(meters) * ENCODER_RES /
        (2.0f * 3.1415926f * WHEEL_RADIUS)
    );
}
/* 移动速度 */
#define FIXED_MOVE_SPEED       0.20f
#define FIXED_SLOW_SPEED       0.08f
#define FIXED_SLOW_PULSES      1500L

static uint32_t fixed_move_start_tof_mm = 0;
static uint32_t fixed_move_target_tof_mm = 0;
static bool fixed_move_start_valid = false;


/*============================= 外部变量定义 =============================*/
// PID 参数实例（具体数值需调试确定）
PID_t pid_speed_LF = { .Kp = 0.5f, .Ki = 0.1f, .Kd = 0.0f, .integral = 0, .prev_error = 0 };
PID_t pid_speed_RF = { .Kp = 0.5f, .Ki = 0.1f, .Kd = 0.0f, .integral = 0, .prev_error = 0 };

// 目标速度数组：索引 0~3 对应 左前、左后，右前，右后
int16_t speed_tar[4] = {0, 0, 0, 0};

// 实际速度数组：仅前轮有编码器
int16_t speed_act[2] = {0, 0};   // 0:左前, 1:右前

// 最终 PWM 输出数组
int16_t pwm_out[4] = {0, 0, 0, 0};

// 运动指令（初始为 0）
MotionCmd target_motion = { .vx = 0, .vy = 0, .w = 0 };

// 陀螺仪相关（角度单位为弧度）
float yaw_angle = 0.0f;
float target_yaw = 0.0f;   // 期望偏航角，默认为 0（直行时维持方向）

// 调试计数器
uint32_t loop_counter = 0;

// 前次编码器计数值（用于速度计算）
static int32_t last_encoder_LF = 0, last_encoder_RF = 0;

/*============================= 辅助函数 =============================*/
/**
 * @brief 限制 PWM 值在有效范围内
 */
int16_t Limit_PWM(int16_t pwm) {
    if (pwm > PWM_MAX) return PWM_MAX;
    if (pwm < -PWM_MAX) return -PWM_MAX;
    return pwm;
}

/**
 * @brief 更新 PID 控制器（增量式，适合速度环）
 * @param pid PID 参数结构指针
 * @param target 目标值
 * @param current 当前值
 * @return 控制输出增量
 */
float PID_Update(PID_t *pid, float target, float current) {
    float error = target - current;
    pid->integral += error;
    // 积分限幅（可选）
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;
    return output;
}//pid

/**
 * @brief 重置 PID 积分与误差
 */
void PID_Reset(PID_t *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

/**
 * @brief 根据编码器脉冲差计算实际转速（rpm）
 * @param encoder_delta 两次采样间的编码器脉冲变化（四倍频计数）
 * @param dt_ms 采样间隔（毫秒）
 * @return 实际转速（转/分钟）
 */
int16_t CalcSpeedFromEncoder(int32_t encoder_delta, uint32_t dt_ms) {
    // 转速 = (脉冲数 / 每转脉冲数) / (时间/60) = (脉冲数 * 60 * 1000) / (每转脉冲数 * dt_ms)
    float rpm = (float)encoder_delta * 60.0f * 1000.0f / (ENCODER_RES * dt_ms);
    return (int16_t)rpm;
}

/**
 * @brief 电机底层控制（根据电机索引设置 PWM 和方向）
 * @param motor_idx 0~3: 左前,右前,左后,右后
 * @param pwm 目标 PWM 值（正负表示方向，绝对值 ≤ PWM_MAX）
 * @param direction 额外方向标志（如果 pwm 已经包含方向信息，可忽略此参数）
 *
 * 注意：此函数需要根据实际硬件连接实现具体的 GPIO 和 PWM 设置
 */
void Motor_Set(int motor_idx, short int pwm)
{
    uint16_t abs_pwm;

    abs_pwm = (pwm >= 0) ? pwm : -pwm;

    if(abs_pwm > PWM_MAX)
        abs_pwm = PWM_MAX;

    switch(motor_idx)
    {
        // 左前轮
        case 0:

            // 设置方向
            if(pwm >= 0)
            {
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET);

            }
            else
            {
            	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
            }

            // 设置PWM
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,abs_pwm);
              break;

        // 左后轮
        case 1:

        	 // 设置方向
        	            if(pwm >= 0)
        	            {
        	                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET);

        	            }
        	            else
        	            {
        	            	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);
        	            }

        	            // 设置PWM
        	            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,abs_pwm);
        	              break;

        // 右前轮
        case 2:

        	 // 设置方向
        	            if(pwm >= 0)
        	            {
        	                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_SET);

        	            }
        	            else
        	            {
        	            	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_RESET);
        	            }

        	            // 设置PWM
        	            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,abs_pwm);
        	              break;

        // 右后轮
        case 3:

        	 // 设置方向
        	            if(pwm >= 0)
        	            {
        	                HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET);

        	            }
        	            else
        	            {
        	            	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_RESET);
        	            }

        	            // 设置PWM
        	            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,abs_pwm);
        	              break;
    }
}
/**
 * @brief 发送调试数据（通过 USB 虚拟串口）
 * 可根据需要打印速度、角度、PID 输出等
 */
/*void Send_Data(void) {
    // 示例：打印前轮速度和偏航角
    char buffer[128];
  int len = sprintf(buffer, "LF:%d RF:%d Yaw:%.2f\n", speed_act[0], speed_act[1], yaw_angle);
    // 假设 USB CDC 设备已初始化，使用 CDC_Transmit_FS 发送
    extern USBD_HandleTypeDef hUsbDeviceFS;
     CDC_Transmit_FS((uint8_t*)buffer, len);
    (void)len; // 避免未使用警告
}*/

/*============================= 初始化函数 =============================*/
/**
 * @brief PWM 初始化（TIM2 四路 PWM）
 * 假设 TIM2 已在 CubeMX 中配置好，此处只需启动 PWM 输出
 */
bool PWM_Init(void) {
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        return false;
    }
    return true;
}

/**
 * @brief 编码器初始化（TIM4 和 TIM1）
 */
bool Encoder_Init(void) {
    // 启动编码器模式
	 if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
	     HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
	 {
	     HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
	     HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
	     return false;
	 }


    // 清空计数值
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    // 记录初始值
    last_encoder_LF = 0;
    last_encoder_RF = 0;
    return true;
}



/**
 * @brief 总体控制初始化
 */
bool Control_Init(void) {
    if (!PWM_Init() || !Encoder_Init())
    {
        return false;
    }
    //Gyro_Init();//陀螺仪参数初始化，后期可选择开启
    // 可选：PID 参数重置
    PID_Reset(&pid_speed_LF);
    PID_Reset(&pid_speed_RF);
    return true;
}

static uint8_t chassis_enabled = 0;

void Chassis_Enable(void)
{
    PID_Reset(&pid_speed_LF);
    PID_Reset(&pid_speed_RF);

    target_yaw = yaw_angle;
    chassis_enabled = 1;
}

void Chassis_Stop(void)
{
	fixed_move_state = CHASSIS_MOVE_IDLE;
    arrival_candidate_valid = false;
    chassis_enabled = 0;

    target_motion.vx = 0;
    target_motion.vy = 0;
    target_motion.w  = 0;

    PID_Reset(&pid_speed_LF);
    PID_Reset(&pid_speed_RF);

    for (uint8_t i = 0; i < 4; i++)
    {
        speed_tar[i] = 0;
        pwm_out[i] = 0;
        Motor_Set(i, 0);
    }
}
static int8_t fixed_move_y_sign = 1;//保存本次运动的y轴方向
//启动函数
static bool Chassis_StartFixedMove(float dy,int32_t target_y_pulses,uint32_t target_tof_mm)
{
    if (target_y_pulses <= 0 ||
        !isfinite(dy) ||
        fabsf(dy) < 0.001f ||
        target_tof_mm < 20U ||
        target_tof_mm > 7800U)
    {
        Chassis_Stop();
        fixed_move_state = CHASSIS_MOVE_IDLE;
        return false;
    }

    if (!mpu_sample_valid)
    {
        Chassis_Stop();
        fixed_move_state = CHASSIS_MOVE_MPU_LOST;
        return false;
    }

    GY53_Sample_t tof;

    if (!GY53_GetSample(&tof))
    {
        Chassis_Stop();
        fixed_move_state = CHASSIS_MOVE_TOF_LOST;
        return false;
    }

    Chassis_Stop();

    /* 从本次启动位置开始累计编码器增量 */
    last_encoder_LF =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    last_encoder_RF =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    fixed_move_progress = 0;
    fixed_move_target_y_pulses = target_y_pulses;
    fixed_move_target_tof_mm = target_tof_mm;
    fixed_move_y_sign = (dy >= 0.0f) ? 1 : -1;
    fixed_move_start_ms = HAL_GetTick();
    arrival_candidate_valid = false;

    fixed_move_state = CHASSIS_MOVE_RUNNING;
    Chassis_Enable();

    return true;
}


uint8_t Chassis_IsArrived(void)
{
    return fixed_move_state == CHASSIS_MOVE_ARRIVED;
}


static void Chassis_FinishMove(ChassisMoveState_t result)
{
    Chassis_Stop();
    fixed_move_state = result;
}

/*更新函数*/
static void Chassis_UpdateFixedMove(int32_t wheel_delta_LF,int32_t wheel_delta_RF)
{
    if (fixed_move_state != CHASSIS_MOVE_RUNNING) {
        return;
    }

    uint32_t now = HAL_GetTick();

       if ((uint32_t)(now - fixed_move_start_ms)>= FIXED_MOVE_TIMEOUT_MS) {
           Chassis_FinishMove(CHASSIS_MOVE_TIMEOUT);
           return;
       }

       fixed_move_progress +=((wheel_delta_LF + wheel_delta_RF) / 2)* fixed_move_y_sign;

       GY53_Sample_t tof;

       if (!GY53_GetSample(&tof)) {
           Chassis_FinishMove(CHASSIS_MOVE_TOF_LOST);
           return;
       }

       float x_error =(float)fixed_move_target_tof_mm -(float)tof.distance_mm;

       float y_error =(float)fixed_move_target_y_pulses -(float)fixed_move_progress;

       bool x_arrived =fabsf(x_error) <= TOF_TOLERANCE_MM;

       bool y_arrived =fabsf(y_error) <= Y_TOLERANCE_PULSES;

       if (x_arrived && y_arrived) {
           target_motion.vx = 0.0f;
           target_motion.vy = 0.0f;
           target_motion.w = 0.0f;

           if (!arrival_candidate_valid)
           {
               arrival_candidate_ms = now;
               arrival_candidate_valid = true;
           }
           else if ((uint32_t)(now - arrival_candidate_ms) >= ARRIVAL_STABLE_MS)
           {
               Chassis_FinishMove(CHASSIS_MOVE_ARRIVED);
           }
           return;
       }

       arrival_candidate_valid = false;

       float vx = 0.0f;
       float vy = 0.0f;

       if (!x_arrived) {
           vx = x_error * TOF_KP * tof_x_sign;

           if (fabsf(vx) > FIXED_MOVE_SPEED) {
               vx = copysignf(FIXED_MOVE_SPEED, vx);
           } else if (fabsf(vx) < TOF_MIN_SPEED) {
               vx = copysignf(TOF_MIN_SPEED, vx);
           }
       }

       if (!y_arrived) {
           float speed =
               fabsf(y_error) < FIXED_SLOW_PULSES
               ? FIXED_SLOW_SPEED
               : FIXED_MOVE_SPEED;

           vy = fixed_move_y_sign * copysignf(speed, y_error);
       }

       /* 限制对角运动的总速度 */
       float magnitude = hypotf(vx, vy);

       if (magnitude > FIXED_MOVE_SPEED) {
           float scale = FIXED_MOVE_SPEED / magnitude;
           vx *= scale;
           vy *= scale;
       }

       target_motion.vx = vx;
       target_motion.vy = vy;
       target_motion.w = 0.0f;
   }



typedef struct {
    float fire_dx_m;
    float fire_dy_m;
} FieldConfig_t;

static const FieldConfig_t left_field = {
    .fire_dx_m = +0.70f,      // 发射点在右前方（左半区）
    .fire_dy_m = +1.20f
};

static const FieldConfig_t right_field = {
    .fire_dx_m = -0.70f,      // 发射点在左前方，（右半区）
    .fire_dy_m = +1.20f
};

static const FieldConfig_t *active_field = &right_field;
/*选择当前半区*/
bool Chassis_SetFieldSide(FieldSide_t side)
{
    if (side == FIELD_SIDE_LEFT) {
        active_field = &left_field;
    } else if (side == FIELD_SIDE_RIGHT) {
        active_field = &right_field;
    } else {
        return false;
    }

    return true;
}

bool Chassis_GotoFirePoint(void)
{
	if (fixed_move_state == CHASSIS_MOVE_RUNNING)
	{
	    return false;
	 }

	fixed_move_start_valid = false;
    GY53_Sample_t tof;

    /* 没有有效且新鲜的测距数据，不启动移动 */
    if (!GY53_GetSample(&tof))
    {
        Chassis_FinishMove(CHASSIS_MOVE_TOF_LOST);
        return false;
    }

    int32_t dx_mm =
        (int32_t)lroundf(active_field->fire_dx_m * 1000.0f);

    int32_t target_tof_mm =
        (int32_t)tof.distance_mm + tof_x_sign * dx_mm;

    /* 目标距离必须在传感器使用范围内 */
    if (target_tof_mm < 20 || target_tof_mm > 7800)
    {
    	Chassis_FinishMove(CHASSIS_MOVE_IDLE);
    	return false;
    }

    bool started = Chassis_StartFixedMove(
        active_field->fire_dy_m,
        YMetersToCounts(active_field->fire_dy_m),
        (uint16_t)target_tof_mm
    );

    if (started)
    {
        fixed_move_start_tof_mm = tof.distance_mm;
        fixed_move_start_valid = true;
    }

    return started;
}


/*原路返回*/
bool Chassis_GotoStartPoint(void)
{
    if (fixed_move_state == CHASSIS_MOVE_RUNNING)
    {
        return false;
    }

    if (!fixed_move_start_valid)
    {
        Chassis_FinishMove(CHASSIS_MOVE_IDLE);
        return false;
    }

    return Chassis_StartFixedMove(
        -active_field->fire_dy_m,
        YMetersToCounts(active_field->fire_dy_m),
        fixed_move_start_tof_mm
    );
}

ChassisMoveState_t Chassis_GetMoveState(void)
{
    return fixed_move_state;
}

/*============================= 核心控制函数 =============================*/
/**
 * @brief 麦轮运动学解算
 * @param vx 前后速度（m/s）
 * @param vy 左右速度（m/s）
 * @param w 旋转角速度（rad/s）
 * @param tar_speed 输出数组：四个轮子的目标速度（单位 rpm）
 */
void Mecanum_Kinematics(float vx, float vy, float w, int16_t *tar_speed) {
    // 根据麦轮运动学公式计算每个轮子的线速度（m/s）
    // 设机器人坐标系：x向右为正，y向前为正，w逆时针为正
    // 四个轮子位置：LF(-a, b), RF(a, b), LB(-a, -b), RB(a, -b)
    // 其中 a = WHEEL_BASE/2, b = WHEEL_TRACK/2
    float a = WHEEL_BASE / 2.0f;
    float b = WHEEL_TRACK / 2.0f;
    float k = a + b;
    float v_LF = vy + vx - w * k;
    float v_LB = vy - vx - w * k;
    float v_RF = vy - vx + w * k;
    float v_RB = vy + vx + w * k;
    // 将线速度转换为电机转速（rpm）
    // 线速度 (m/s) = 轮速 (rpm) * 2πr / 60  => 轮速 = (线速度 * 60) / (2πr)
    const float factor = 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
    tar_speed[0] = (int16_t)(v_LF * factor);
    tar_speed[1] = (int16_t)(v_LB * factor);
    tar_speed[2] = (int16_t)(v_RF * factor);
    tar_speed[3] = (int16_t)(v_RB * factor);
}

/**
 * @brief 偏航角补偿
 * @param cmd 运动指令（包含 vx, vy, w）
 * 根据当前偏航角与期望偏航角的偏差，修正角速度 w
 */
void Yaw_Compensate(MotionCmd *cmd) {
    // 仅当需要自动纠偏时使用，例如目标偏航角为 0（直行）
    float yaw_error = target_yaw - yaw_angle;
    while (yaw_error > 3.1415926f) yaw_error -= 2.0f * 3.1415926f;
    while (yaw_error < -3.1415926f) yaw_error += 2.0f * 3.1415926f;
    // 简单的 P 控制补偿
    const float yaw_comp_Kp = 1.0f;   // 需调试
    float w_comp = yaw_comp_Kp * yaw_error;
    // 将补偿叠加到原角速度上，并限幅
    cmd->w += w_comp;
    const float MAX_W = 3.0f; // 最大角速度限制 (rad/s)
    if (cmd->w > MAX_W) cmd->w = MAX_W;
    if (cmd->w < -MAX_W) cmd->w = -MAX_W;
}

static int16_t SpeedTargetToPwm(int16_t target_rpm)
{
    return (int16_t)((float)target_rpm * (float)PWM_MAX / 300.0f);
}

/**
 * @brief 主控制循环（应在定时器中断中周期性调用，周期 CONTROL_PERIOD_MS）
 */
void Control_Loop(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    if (last_time == 0)
        {
            last_time = now;
            return;
        }

    uint32_t dt_ms = now - last_time;
    if (dt_ms == 0) {dt_ms = CONTROL_PERIOD_MS; }// 防止除零

    // 1. 读取编码器计数值并计算实际转速（仅前轮）
    uint16_t enc_LF =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    uint16_t enc_RF =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    int32_t delta_LF =
        (int32_t)enc_LF - (int32_t)last_encoder_LF;

    int32_t delta_RF =
        (int32_t)enc_RF - (int32_t)last_encoder_RF;

    /*处理从65535突变到0的突变错误*/
    /* 处理正向回绕：65535 → 0 */
    if (delta_LF < -32768)
    {
        delta_LF += 65536;
    }

    /* 处理反向回绕：0 → 65535 */
    else if (delta_LF > 32767)
    {
        delta_LF -= 65536;
    }

    if (delta_RF < -32768)
    {
        delta_RF += 65536;
    }
    else if (delta_RF > 32767)
    {
        delta_RF -= 65536;
    }
    //统一编码器增量方向
    int32_t wheel_delta_LF =ENCODER_LF_SIGN * delta_LF;
    int32_t wheel_delta_RF =ENCODER_RF_SIGN * delta_RF;

    last_encoder_LF = enc_LF;
    last_encoder_RF = enc_RF;

    speed_act[0] = CalcSpeedFromEncoder(wheel_delta_LF, dt_ms);
    speed_act[1] = CalcSpeedFromEncoder(wheel_delta_RF, dt_ms);

  // 2. 获取陀螺仪偏航角
    float dt = dt_ms / 1000.0f;
    if (MPU_Update(dt))
    {
        mpu_failure_count = 0;
        mpu_sample_valid = true;
    }
    else
    {
        if (mpu_failure_count < MPU_FAILURE_LIMIT)
        {
            mpu_failure_count++;
        }

        if (mpu_failure_count >= MPU_FAILURE_LIMIT)
        {
            mpu_sample_valid = false;
            if (fixed_move_state == CHASSIS_MOVE_RUNNING)
            {
                Chassis_FinishMove(CHASSIS_MOVE_MPU_LOST);
            }
        }
    }

    Chassis_UpdateFixedMove(wheel_delta_LF, wheel_delta_RF);


    if (!chassis_enabled)
    {
        Motor_Set(0, 0);
        Motor_Set(1, 0);
        Motor_Set(2, 0);
        Motor_Set(3, 0);

        last_time = now;
        return;
    }



   // 3. 复制当前运动指令（避免中断中修改）
    MotionCmd cmd = target_motion;

    // 4. 偏航角补偿（可选，根据需求决定是否启用）
    Yaw_Compensate(&cmd);

    // 5. 运动学解算得到四个轮子的目标速度（rpm）
    Mecanum_Kinematics(cmd.vx, cmd.vy, cmd.w, speed_tar);

    // 6. PID 控制前轮，后轮直接使用目标速度（开环）
    // 注意：PID 输出是速度误差对应的控制量（单位可能是 PWM），需映射到 PWM 范围
    float pid_out_LF = PID_Update(&pid_speed_LF, (float)speed_tar[0], (float)speed_act[0]);
   float pid_out_RF = PID_Update(&pid_speed_RF, (float)speed_tar[2], (float)speed_act[1]);

    // 将 PID 输出转换为 PWM 值（假设速度与 PWM 近似线性，需标定）
    // 此处简单映射：PID 输出范围假设在 [-1000,1000] 对应 PWM
    int16_t pwm_LF = SpeedTargetToPwm(speed_tar[0]) + (int16_t)pid_out_LF;
    int16_t pwm_RF = SpeedTargetToPwm(speed_tar[2]) + (int16_t)pid_out_RF;
    // 后轮直接使用目标速度对应的 PWM（需根据电机特性转换）
    // 简单比例：转速 (rpm) -> PWM (占空比) 需标定，这里假设 1000 PWM 对应 300 rpm
    int16_t pwm_LB = SpeedTargetToPwm(speed_tar[1]);
   int16_t pwm_RB = SpeedTargetToPwm(speed_tar[3]);

    // 限幅
    pwm_out[0] = Limit_PWM(pwm_LF);
    pwm_out[1] = Limit_PWM(pwm_LB);
    pwm_out[2] = Limit_PWM(pwm_RF);
    pwm_out[3] = Limit_PWM(pwm_RB);

    // 7. 输出到电机（实际方向由 PWM 符号决定）
    Motor_Set(0, pwm_out[0]);
    Motor_Set(1, pwm_out[1]);
    Motor_Set(2, pwm_out[2]);
    Motor_Set(3, pwm_out[3]);

    // 8. 发送调试数据（可周期性发送，避免阻塞）
    //static uint32_t last_send = 0;
   // if (now - last_send > 100) { // 每 100ms 发送一次
   //     Send_Data();
   //     last_send = now;
  //  }

    loop_counter++;
    last_time = now;
}
void Send_Data(void) {

}
