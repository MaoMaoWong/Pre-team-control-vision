/*
 * protocol.c
 *
 *  Created on: Apr 14, 2026
 *      Author: 陈思危
 */


#include "protocol.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "robot.h"

/*============================================================================
 * 全局变量
 *============================================================================*/
//接通心脏停车
#define HEARTBEAT_PERIOD_MS   500U
#define HEARTBEAT_TIMEOUT_MS 1500U

static uint32_t last_heartbeat_tx_ms;
static uint32_t last_heartbeat_ack_ms;
static bool heartbeat_armed;


RingBuffer rx_ringbuf;
static bool is_connected = false;   /* 电脑端串口是否打开 (DTR) */

/* 协议解析状态机状态 */
typedef enum {
    STATE_SYNC1,
    STATE_SYNC2,
    STATE_CMD,
    STATE_LEN_L,
    STATE_LEN_H,
    STATE_DATA,
    STATE_CRC,
    STATE_TAIL
} ParseState;

static ParseState parse_state = STATE_SYNC1;
static uint16_t   data_len = 0;
static uint16_t   data_idx = 0;
static uint8_t    frame_cmd = 0;
static uint8_t    calc_crc = 0;
static uint8_t    rx_frame_data[256];  /* 暂存 Data 域内容 */

#define TX_QUEUE_DEPTH      8U
#define TX_FRAME_MAX_SIZE   300U

typedef struct {
    uint16_t len;
    uint8_t data[TX_FRAME_MAX_SIZE];
} TxFrame;

static TxFrame tx_queue[TX_QUEUE_DEPTH];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;
static uint8_t tx_count = 0;
static uint8_t tx_in_flight = 0;
static uint32_t tx_drop_count = 0;

/* 设备信息 (常量) */
static const DeviceInfo device_info = {
    .firmware_version = "v1.0.0",
    .protocol_version = 0x03,      /* 协议版本 3.0 */
    .reserved = {0}
};

/*============================================================================
 * 内部函数声明
 *============================================================================*/

static uint8_t calc_crc8(uint8_t *data, uint16_t len);
static void handle_received_frame(uint8_t cmd, uint8_t *data, uint16_t len);
static bool send_frame(uint8_t cmd, uint8_t *data, uint16_t len);
static void protocol_tx_task(void);

/* 环形缓冲区操作 */
static void ringbuf_init(RingBuffer *rb);
static uint16_t ringbuf_available(RingBuffer *rb);
static uint16_t ringbuf_free_space(RingBuffer *rb);
static void ringbuf_append(RingBuffer *rb, uint8_t *data, uint16_t len);
static uint8_t ringbuf_read_byte(RingBuffer *rb);
static void protocol_heartbeat_task(void);

/*============================================================================
 * 环形缓冲区实现
 *============================================================================*/

static void ringbuf_init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->mask = RX_RINGBUF_SIZE - 1;
}

static uint16_t ringbuf_available(RingBuffer *rb) {
    return (rb->head - rb->tail) & rb->mask;
}

static uint16_t ringbuf_free_space(RingBuffer *rb) {
    return (rb->tail - rb->head - 1) & rb->mask;
}

static void ringbuf_append(RingBuffer *rb, uint8_t *data, uint16_t len) {
    uint16_t free = ringbuf_free_space(rb);
    if (len > free) {
        len = free;   /* 缓冲区满则丢弃多余数据 */
    }
    for (uint16_t i = 0; i < len; i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) & rb->mask;
    }
}

static uint8_t ringbuf_read_byte(RingBuffer *rb) {
    uint8_t data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & rb->mask;
    return data;
}

/*============================================================================
 * 工具函数
 *============================================================================*/

static uint8_t calc_crc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

/* 发送一帧数据 (自动添加帧头、长度、CRC、帧尾) */
static bool send_frame(uint8_t cmd, uint8_t *data, uint16_t len) {
    TxFrame *frame;
    uint8_t *buffer;
    uint16_t idx = 0;

    if ((len > (TX_FRAME_MAX_SIZE - 7U)) ||
        ((len > 0U) && (data == NULL)) ||
        (tx_count >= TX_QUEUE_DEPTH)) {
        tx_drop_count++;
        return false;
    }

    frame = &tx_queue[tx_head];
    buffer = frame->data;

    buffer[idx++] = PROTOCOL_FRAME_HEADER0;
    buffer[idx++] = PROTOCOL_FRAME_HEADER1;
    buffer[idx++] = cmd;
    buffer[idx++] = len & 0xFF;
    buffer[idx++] = (len >> 8) & 0xFF;

    if (len > 0 && data != NULL) {
        memcpy(&buffer[idx], data, len);
        idx += len;
    }

    /* 计算 CRC (从 CMD 到 Data 末尾) */
    uint8_t crc = calc_crc8(&buffer[2], 1 + 2 + len);  /* CMD + Length + Data */
    buffer[idx++] = crc;
    buffer[idx++] = PROTOCOL_FRAME_TAIL;

    frame->len = idx;
    tx_head = (uint8_t)((tx_head + 1U) % TX_QUEUE_DEPTH);
    tx_count++;
    return true;
}

/* USB CDC 是异步发送：缓冲区必须保留到 TxState 重新空闲。 */
static void protocol_tx_task(void)
{
    uint8_t result;

    if (tx_in_flight) {
        if (!CDC_TransmitReady_FS()) {
            return;
        }

        tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_DEPTH);
        tx_count--;
        tx_in_flight = 0;
    }

    if ((tx_count == 0U) || !CDC_TransmitReady_FS()) {
        return;
    }

    result = CDC_Transmit_FS(tx_queue[tx_tail].data, tx_queue[tx_tail].len);
    if (result == USBD_OK) {
        tx_in_flight = 1;
    }
}

/*============================================================================
 * 协议初始化
 *============================================================================*/

void protocol_init(void) {
    ringbuf_init(&rx_ringbuf);
    parse_state = STATE_SYNC1;
    data_len = 0;
    data_idx = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
    tx_in_flight = 0;
    tx_drop_count = 0;
    last_heartbeat_tx_ms = HAL_GetTick();
    last_heartbeat_ack_ms = 0;
    heartbeat_armed = false;
    is_connected = false;
}

/*============================================================================
 * 数据喂入 (由USB接收回调调用)
 *============================================================================*/

void protocol_feed_data(uint8_t *data, uint16_t len) {
    ringbuf_append(&rx_ringbuf, data, len);
}

/*============================================================================
 * 主任务：解析环形缓冲区中的数据帧，并调用处理函数
 *============================================================================*/

void protocol_task(void) {
    protocol_tx_task();

    while (ringbuf_available(&rx_ringbuf) > 0) {
        uint8_t byte = ringbuf_read_byte(&rx_ringbuf);//protocol_feed_data在usblib.c那里薅过来的数据

        switch (parse_state) {
            case STATE_SYNC1:
                if (byte == PROTOCOL_FRAME_HEADER0) {
                    parse_state = STATE_SYNC2;
                }
                break;

            case STATE_SYNC2:
                if (byte == PROTOCOL_FRAME_HEADER1)
                {
                    parse_state = STATE_CMD;
                }
                else if (byte == PROTOCOL_FRAME_HEADER0)
                {
                    parse_state = STATE_SYNC2;
                }
                else
                {
                    parse_state = STATE_SYNC1;
                }
                break;

            case STATE_CMD:
                frame_cmd = byte;
                calc_crc = byte;        /* 初始化 CRC */
                parse_state = STATE_LEN_L;
                break;

            case STATE_LEN_L:
                data_len = byte;
                calc_crc ^= byte;
                parse_state = STATE_LEN_H;
                break;

            case STATE_LEN_H:
                data_len |= (byte << 8);
                calc_crc ^= byte;
                /* 长度超过接收缓冲区，拒绝该帧 */
                if (data_len > sizeof(rx_frame_data))
                {
                       protocol_send_error(ERR_FORMAT_ERROR, frame_cmd);

                        data_len = 0;
                        data_idx = 0;
                        calc_crc = 0;
                        parse_state = STATE_SYNC1;

                        break;
                 }
                 if (data_len == 0)
                 {
                    parse_state = STATE_CRC;
                 }
                 else
                 {
                     data_idx = 0;
                     parse_state = STATE_DATA;
                 }
                break;

            case STATE_DATA:
                if (data_idx < sizeof(rx_frame_data)) {
                    rx_frame_data[data_idx++] = byte;
                }
                calc_crc ^= byte;
                if (data_idx >= data_len) {
                    parse_state = STATE_CRC;
                }
                break;

            case STATE_CRC:
                if (byte == calc_crc) {
                    parse_state = STATE_TAIL;
                } else {
                    /* CRC 错误，发送 ERROR 并丢弃本帧 */
                    protocol_send_error(ERR_CRC_FAIL, frame_cmd);
                    parse_state = STATE_SYNC1;
                }
                break;

            case STATE_TAIL:
                if (byte == PROTOCOL_FRAME_TAIL) {
                    /* 成功接收完整一帧 */
                    handle_received_frame(frame_cmd, rx_frame_data, data_len);
                }
                parse_state = STATE_SYNC1;
                break;

            default:
                parse_state = STATE_SYNC1;
                break;
        }
    }
    protocol_heartbeat_task();
    protocol_tx_task();
}

/*============================================================================
 * 命令处理 (根据收到的命令字执行相应动作)
 *============================================================================*/

static void handle_received_frame(uint8_t cmd, uint8_t *data, uint16_t len) {
    switch (cmd) {
        case CMD_GET_DEVICE_INFO:
            if (len == 0U) {
                protocol_send_device_info();
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_MATCH_START:
        {
            if (len != sizeof(MatchStart))
            {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
                break;
            }

            MatchStart ms;
            memcpy(&ms, data, sizeof(ms));

            if (ms.word1 >= 4U || ms.word2 >= 4U ||
                ms.field_side > 1U || ms.reserved != 0U)
            {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
                break;
            }
            /* 在这里确认最近收到过有效ACK */
          bool link_alive =heartbeat_armed && ((uint32_t)(HAL_GetTick() - last_heartbeat_ack_ms) < HEARTBEAT_TIMEOUT_MS);
                if (!link_alive)
                {
                    protocol_send_error(ERR_INVALID_STATE, cmd);
                    break;
                }

                /* ACK检查通过后，才允许开始比赛 */
                if (Robot_StartMatch(ms.word1, ms.word2, ms.field_side))
                {
                    protocol_send_match_start_ack();
                    protocol_send_grid_query();
                }
                else
                {
                    protocol_send_error(ERR_INVALID_STATE, cmd);
                }

            break;
        }

        case CMD_MATCH_STOP:
            if (len == 0U) {
                Robot_StopMatch();
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_GRID_DATA:
            if (len == 9 * sizeof(GridInfo)) {
                if (Robot_GetState() == ROBOT_WAIT_GRID) {
                    GridInfo grids[9];
                    memcpy(grids, data, sizeof(grids));
                    if (!Robot_UpdateGrid(grids)) {
                        protocol_send_error(ERR_FORMAT_ERROR, cmd);
                    }
                } else {
                    protocol_send_error(ERR_INVALID_STATE, cmd);
                }
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_TARGET_WORDS:
            if (len == sizeof(MatchStart)) {
                MatchStart ms;
                memcpy(&ms, data, sizeof(ms));
                if (ms.word1 >= 4U || ms.word2 >= 4U ||
                    ms.field_side > 1U || ms.reserved != 0U ||
                    !Robot_UpdateTargetWords(ms.word1, ms.word2))
                {
                  protocol_send_error(ERR_FORMAT_ERROR, cmd);
                }
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_MOVE_TO:
            if (len == sizeof(MoveTo)) {
                protocol_send_error(ERR_INVALID_STATE, cmd);
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_SHOOT:
            if (len == sizeof(ShootCmd)) {
                protocol_send_error(ERR_INVALID_STATE, cmd);
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_EMERGENCY_STOP:
            if (len == 0U) {
                Robot_EmergencyStop();
            } else {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;

        case CMD_HEARTBEAT_ACK:
            if (len == 0U)
            {
                last_heartbeat_ack_ms = HAL_GetTick();
                heartbeat_armed = true;
                is_connected = true;
            }
            else
            {
                protocol_send_error(ERR_FORMAT_ERROR, cmd);
            }
            break;


        default:
            /* 未知命令 */
            protocol_send_error(ERR_UNKNOWN_CMD, cmd);
            break;
    }
}

/*============================================================================
 * 状态查询
 *============================================================================*/

bool protocol_is_connected(void) {
    return is_connected;
}

void protocol_set_connected(bool connected) {
    is_connected = connected;
}

/*============================================================================
 * 发送函数实现
 *============================================================================*/

void protocol_send_heartbeat(void) {
    send_frame(CMD_HEARTBEAT, NULL, 0);
}

void protocol_send_device_info(void) {
    send_frame(CMD_DEVICE_INFO, (uint8_t*)&device_info, sizeof(DeviceInfo));
}

void protocol_send_match_start_ack(void) {
    send_frame(CMD_MATCH_START_ACK, NULL, 0);
}

void protocol_send_move_status(uint8_t status, int16_t x, int16_t y) {
    MoveStatus ms = {
        .status = status,
        .current_x = x,
        .current_y = y,
        .reserved = 0
    };
    send_frame(CMD_MOVE_STATUS, (uint8_t*)&ms, sizeof(MoveStatus));
}

void protocol_send_position_update(int16_t x, int16_t y, int16_t yaw) {
    PositionUpdate pu = {
        .x = x,
        .y = y,
        .yaw = yaw
    };
    send_frame(CMD_POSITION_UPDATE, (uint8_t*)&pu, sizeof(PositionUpdate));
}

void protocol_send_shoot_result(uint8_t result, uint8_t radical_id) {
    ShootResult sr = {
        .result = result,
        .radical_id = radical_id,
        .reserved = {0}
    };
    send_frame(CMD_SHOOT_RESULT, (uint8_t*)&sr, sizeof(ShootResult));
}

void protocol_send_error(uint8_t error_code, uint8_t original_cmd) {
    ErrorInfo ei = {
        .error_code = error_code,
        .original_cmd = original_cmd,
        .reserved = {0}
    };
    send_frame(CMD_ERROR, (uint8_t*)&ei, sizeof(ErrorInfo));
}

void protocol_send_grid_query(void) {
    send_frame(CMD_GRID_QUERY, NULL, 0);
}


static void protocol_heartbeat_task(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_heartbeat_tx_ms)
        >= HEARTBEAT_PERIOD_MS && CDC_TransmitReady_FS())
    {
        last_heartbeat_tx_ms = now;
        protocol_send_heartbeat();
    }

    bool alive =
        heartbeat_armed &&
        ((uint32_t)(now - last_heartbeat_ack_ms)
         < HEARTBEAT_TIMEOUT_MS);

    is_connected = alive;

    RobotState_t state = Robot_GetState();

    bool match_active =
        state != ROBOT_IDLE &&
        state != ROBOT_DONE &&
        state != ROBOT_ERROR;

    if (match_active && !alive)
    {
        Robot_EmergencyStop();
    }
}
