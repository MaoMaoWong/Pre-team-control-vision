/*
 * mpu6500.h
 *
 *  Created on: Mar 12, 2026
 *      Author: 陈思危
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_
#include<stdbool.h>
#include <stdint.h>
extern float yaw_angle;
bool mpu_write_byte(uint8_t const reg,uint8_t const data);/***** 写入一个字节的数据 *****/
/***** 读取一个字节的数据 *****/
bool mpu_read_byte(uint8_t const reg, uint8_t *data);

bool MPU6500_Init(void);//mpu6500初始化

bool Get_MPU6500_Accelerometer(short *ax,short *ay,short *az);//得到加速度值（初始值），gx,gy,gz陀螺仪三个轴的初始读数（带符号）

/**
 * @brief       得到陀螺仪值(原始值)
 * @param       gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
 * @retval      无
 */
bool Get_MPU6500_GYRO(short *gx,short *gy,short *gz);

bool MPU_Calibrate(void);
bool MPU_Update(float dt);
#endif /* INC_MPU6500_H_ */
