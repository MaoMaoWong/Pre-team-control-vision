/*
 * mpu6500.c
 *
 *  Created on: Mar 12, 2026
 *      Author: 陈思危
 */


#include"mpu6500.h"
#include"bsp_mpu6500_reg.h"
#include"gpio.h"
#include"spi.h"
#define MPU_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define MPU_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)


#define MPU_SPI_TIMEOUT_MS 5U
/*分别写入与读取一个字节的数据*/
bool mpu_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {
        (uint8_t)(reg & 0x7F),
        data
    };

    MPU_CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi2, tx, sizeof(tx), MPU_SPI_TIMEOUT_MS
    );

    MPU_CS_HIGH();

    return status == HAL_OK;
}

bool mpu_read_byte(uint8_t reg, uint8_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    uint8_t tx[2] = {
        (uint8_t)(reg | 0x80),
        0
    };
    uint8_t rx[2] = {0};

    MPU_CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi2, tx, rx, sizeof(tx), MPU_SPI_TIMEOUT_MS
    );

    MPU_CS_HIGH();

    if (status != HAL_OK)
    {
        return false;
    }

    *data = rx[1];
    return true;
}
/***** mpu6500初始化函数 *****/
bool MPU6500_Init(void)
{
	uint8_t id = 0;

	if (!mpu_read_byte(0x75, &id) || id != 0x70U)
	{
	    return false;
	}
	    if (!mpu_write_byte(0x6B, 0x80))
	    {
	        return false;
	    }
	    HAL_Delay(100);
	    if (!mpu_write_byte(0x6B, 0x01) ||
	        !mpu_write_byte(0x1B, 0x08) ||
	        !mpu_write_byte(0x1C, 0x08) ||
	        !mpu_write_byte(0x1A, 0x04))
	    {
	        return false;
	    }

	    return mpu_read_byte(0x75, &id) && id == 0x70U;
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
bool Get_MPU6500_Accelerometer(short *ax,short *ay,short *az)
{
	uint8_t tx[7] = {0x3B | 0x80, 0, 0, 0, 0, 0, 0};
	uint8_t rx[7] = {0};

	if (ax == NULL || ay == NULL || az == NULL)
	{
	    return false;
	}

	MPU_CS_LOW();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
	    &hspi2, tx, rx, sizeof(tx), MPU_SPI_TIMEOUT_MS);
	MPU_CS_HIGH();

	if (status != HAL_OK)
	{
	    return false;
	}

	*ax = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);
	*ay = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
	*az = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
	return true;
}

/**
 * @brief       得到陀螺仪值(原始值)
 * @param       gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
 * @retval      无
 */
bool Get_MPU6500_GYRO(
    int16_t *gx,
    int16_t *gy,
    int16_t *gz)
{
    if (gx == NULL || gy == NULL || gz == NULL)
    {
        return false;
    }

    uint8_t tx[7] = {0x43 | 0x80, 0, 0, 0, 0, 0, 0};
    uint8_t rx[7] = {0};

    MPU_CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi2, tx, rx, sizeof(tx), MPU_SPI_TIMEOUT_MS
    );

    MPU_CS_HIGH();

    if (status != HAL_OK)
    {
        return false;
    }

    *gx = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);
    *gy = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
    *gz = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

    return true;
}
//得到沿三个轴的分别的角加速度
// 陀螺仪数据从 0x43 开始连续读取


float gyro_z_offset = 0;
bool MPU_Calibrate(void)
{
    short gx, gy, gz;

    long sum = 0;

    for(int i=0;i<1000;i++)
    {
        if (!Get_MPU6500_GYRO(&gx, &gy, &gz))
        {
            return false;
        }

        sum += gz;

        HAL_Delay(2);
    }

    gyro_z_offset = sum / 1000.0f;

    yaw_angle = 0.0f;
    return true;
}//零偏校准

bool MPU_Update(float dt)
{

    short gx, gy, gz;

    if (dt <= 0.0f || dt > 0.1f ||
        !Get_MPU6500_GYRO(&gx, &gy, &gz))
    {
        return false;
     }

    // ±500°/s量程
    // 灵敏度 65.5 LSB/(°/s)

    float gyro_z_dps =	((float)gz-gyro_z_offset)/ 65.5f;

    float gyro_z_rps =	gyro_z_dps * 3.14159f / 180.0f;


    // 积分得到角度
    yaw_angle += gyro_z_rps * dt;

    if (yaw_angle > 3.1415926f)
    {
        yaw_angle -= 2.0f * 3.1415926f;
    }
    else if (yaw_angle < -3.1415926f)
    {
        yaw_angle += 2.0f * 3.1415926f;
    }

    return true;
}//沿z轴角加速度积分得到偏移角
