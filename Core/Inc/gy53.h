/*
 * gy53.h
 *
 *  Created on: Apr 16, 2026
 *      Author: 陈思危
 */

#ifndef INC_GY53_H_
#define INC_GY53_H_

#include <stdbool.h>
#include <stdint.h>

#define GY53_FRAME_SIZE        16U
#define GY53_DMA_BUFFER_SIZE   64U
#define GY53_FRESH_TIMEOUT_MS  200U

typedef struct {
    uint32_t distance_mm;
    uint32_t timestamp_ms;
    bool valid;
    bool fresh;
} GY53_Sample_t;

bool GY53_Init(void);
void GY53_Task(void);
bool GY53_GetSample(GY53_Sample_t *sample);


#endif /* INC_GY53_H_ */
