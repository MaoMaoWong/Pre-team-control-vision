/*
 * gy53.c
 *
 *  Created on: Apr 16, 2026
 *      Author: 陈思危
 */
#include"gy53.h"
#include"usart.h"
#include<string.h>

// 全局变量定义
static volatile uint32_t gy53_distance_mm = 0;
static volatile uint32_t gy53_last_update_ms = 0;
static volatile bool gy53_data_valid = false;

// DMA 接收缓冲区
static uint8_t gy53_dma_buffer[GY53_DMA_BUFFER_SIZE];
static uint8_t gy53_frame[GY53_FRAME_SIZE];
static uint8_t gy53_frame_index=0;
static volatile bool gy53_restart_requested = false;
static uint32_t gy53_last_restart_ms = 0;

static bool GY53_Parse_Frame(const uint8_t frame[GY53_FRAME_SIZE]);

static void GY53_FeedBytes(const uint8_t *data,uint16_t length);
// 启动接收中断（非阻塞，只启动一次即可）
bool GY53_Init(void)
{
	 gy53_frame_index = 0;
	    gy53_restart_requested = false;
	    gy53_data_valid = false;
	    gy53_distance_mm = 0;
	    gy53_last_update_ms = 0;

    // 启动 DMA 循环接收
    return HAL_UART_Receive_DMA(&huart3, gy53_dma_buffer, GY53_DMA_BUFFER_SIZE)==HAL_OK;
}

void GY53_Task(void)
{
    uint32_t now;

    if (!gy53_restart_requested)
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - gy53_last_restart_ms) < 50U)
    {
        return;
    }

    gy53_last_restart_ms = now;
    HAL_UART_AbortReceive(&huart3);
    gy53_frame_index = 0;
    gy53_restart_requested = false;

    if (HAL_UART_Receive_DMA(&huart3, gy53_dma_buffer,
                             GY53_DMA_BUFFER_SIZE) == HAL_OK)
    {
        return;
    }

    gy53_restart_requested = true;
}


// 串口接收完成回调（由 HAL_UART_IRQHandler 调用）
// TOF.c
//增加半满和全满回调
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        GY53_FeedBytes(&gy53_dma_buffer[0], GY53_DMA_BUFFER_SIZE / 2);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        GY53_FeedBytes(&gy53_dma_buffer[GY53_DMA_BUFFER_SIZE/2],GY53_DMA_BUFFER_SIZE / 2 );
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        gy53_data_valid = false;
        gy53_restart_requested = true;
    }
}
// 解析一帧数据
static bool GY53_Parse_Frame(const uint8_t frame[GY53_FRAME_SIZE])
{
    if (frame[0] != 0x57 || frame[1] != 0x00)
    {
        return false;
    }

    uint8_t checksum = 0;

    for (uint8_t i = 0; i < 15; i++)
    {
        checksum += frame[i];
    }

    if (checksum != frame[15])
    {
        return false;
    }

    /* 第11字节表示本次距离是否有效 */
    if (frame[11] != 1)
    {
        return false;
    }

    uint32_t distance_mm =
        ((uint32_t)frame[8]) |
        ((uint32_t)frame[9] << 8) |
        ((uint32_t)frame[10] << 16);

    if (distance_mm < 20 || distance_mm > 7800)
    {
        return false;
    }

    gy53_distance_mm = distance_mm;
    gy53_last_update_ms = HAL_GetTick();
    gy53_data_valid = true;

    return true;
}

/*GY53_FeedBytes() 的作用是：
1. 接收DMA送来的任意一段字节。
2. 从字节流里寻找 0x57 0x00。
3. 找到帧头后收集完整16字节。
4. 再调用 GY53_Parse_Frame() 校验和解析。*/
static void GY53_FeedBytes(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t byte = data[i];

        if (gy53_frame_index == 0)
        {
            if (byte == 0x57)
            {
                gy53_frame[0] = byte;
                gy53_frame_index = 1;
            }
            continue;
        }

        if (gy53_frame_index == 1)
        {
            if (byte == 0x00)
            {
                gy53_frame[1] = byte;
                gy53_frame_index = 2;
            }
            else if (byte == 0x57)
            {
                gy53_frame[0] = byte;
                gy53_frame_index = 1;
            }
            else
            {
                gy53_frame_index = 0;
            }
            continue;
        }

        gy53_frame[gy53_frame_index++] = byte;

        if (gy53_frame_index >= GY53_FRAME_SIZE)
        {
            GY53_Parse_Frame(gy53_frame);
            gy53_frame_index = 0;
        }
    }
}

bool GY53_GetSample(GY53_Sample_t *sample)
{
    if (sample == NULL)
    {
        return false;
    }

    /* 短暂保护读取，保证距离和时间戳来自同一次更新 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now = HAL_GetTick();

    sample->distance_mm = gy53_distance_mm;
    sample->timestamp_ms = gy53_last_update_ms;
    sample->valid = gy53_data_valid;

    __set_PRIMASK(primask);

    sample->fresh = sample->valid && ((uint32_t)(now - sample->timestamp_ms) <= GY53_FRESH_TIMEOUT_MS);

    return sample->valid && sample->fresh;
}

