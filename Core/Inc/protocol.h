/*
 * protocol.h
 *
 *  Created on: Apr 14, 2026
 *      Author: 陈思危
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

/**
 * @file    protocol.h
 * @brief   机器人“汉字智投”视觉-电控通信协议 V3.0 接口
 *
 * 适用于 STM32F103 + USB CDC (usblib.c) 环境。
 * 坐标系约定：
 *   - 地面坐标系（电控导航）：原点为比赛区域左下角，X向右，Y向前，单位 mm。
 *   - 棋盘坐标系（视觉识别）：原点为棋盘左下角，U向右，V向上，单位 mm。
 */
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * 协议常量
 *============================================================================*/

#define PROTOCOL_FRAME_HEADER0      0xAA
#define PROTOCOL_FRAME_HEADER1      0x55
#define PROTOCOL_FRAME_TAIL         0xED

#define RX_RINGBUF_SIZE             1024    /* 环形缓冲区大小，建议2的幂次方 */
//enum是一种数据枚举类型，用来表示开光状态等
/* 命令字定义 */
enum ProtocolCMD {
    CMD_HEARTBEAT           = 0x01,     /* 电控→视觉 */
    CMD_HEARTBEAT_ACK       = 0x02,     /* 视觉→电控 */
    CMD_GET_DEVICE_INFO     = 0x03,     /* 视觉→电控 */
    CMD_DEVICE_INFO         = 0x04,     /* 电控→视觉 */
    CMD_MATCH_START         = 0x10,     /* 视觉→电控 */
    CMD_MATCH_START_ACK     = 0x11,     /* 电控→视觉 */
    CMD_MATCH_STOP          = 0x12,     /* 视觉→电控 */
    CMD_GRID_QUERY          = 0x20,     /* 电控→视觉 */
    CMD_GRID_DATA           = 0x21,     /* 视觉→电控 */
    CMD_TARGET_WORDS        = 0x22,     /* 视觉→电控 */ //目标字id
    CMD_MOVE_TO             = 0x30,     /* 视觉→电控 */
    CMD_MOVE_STATUS         = 0x31,     /* 电控→视觉 */ //移动状态
    CMD_POSITION_UPDATE     = 0x32,     /* 电控→视觉 */
    CMD_SHOOT               = 0x40,     /* 视觉→电控 */
    CMD_SHOOT_RESULT        = 0x41,     /* 电控→视觉 */
    CMD_ERROR               = 0xFE,     /* 双向 */
    CMD_EMERGENCY_STOP      = 0xFF      /* 双向 */
};

/* 错误码 */
enum ErrorCode {
    ERR_UNKNOWN_CMD         = 0x01,
    ERR_CRC_FAIL            = 0x02,
    ERR_FORMAT_ERROR        = 0x03,
	ERR_INVALID_STATE       = 0x04
};

/* 偏旁ID */
enum RadicalID {
    RAD_HE      = 0,    /* 禾 */
    RAD_REN     = 1,    /* 人 */
    RAD_SHUI    = 2,    /* 氺 */
    RAD_ER      = 3,    /* 而 */
    RAD_WANG    = 4,    /* 王 */
    RAD_SHAN    = 5,    /* 山 */
    RAD_YU      = 6,    /* 雨 */
    RAD_KOU     = 7,    /* 口 */
    RAD_MU      = 8     /* 木 */
};

/* 目标汉字ID */
enum TargetWordID {
    WORD_SHU    = 0,    /* 黍 (禾 人 氺) */
    WORD_RUI    = 1,    /* 瑞 (王 山 而) */
    WORD_CHENG  = 2,    /* 程 (禾 口 王) */
    WORD_RU     = 3     /* 儒 (人 雨 而) */
};

/* 移动状态 */
enum MoveStatus {
    MOVE_MOVING     = 0x00,
    MOVE_ARRIVED    = 0x01,
    MOVE_FAILED     = 0x02
};

/* 投掷结果 */
enum ShootResultCode {
    SHOOT_MISS      = 0x00,
    SHOOT_HIT       = 0x01,
    SHOOT_FAULT     = 0x02
};

/*============================================================================
 * 数据结构 (使用紧凑打包)
 *============================================================================*/

#pragma pack(push, 1)

/* 棋盘格子信息 (9个一组，GRID_DATA 使用) */
typedef struct {
    uint8_t  radical_id;    /* 偏旁ID */
    int16_t  center_x;      /* 棋盘坐标系 U (mm) */
    int16_t  center_y;      /* 棋盘坐标系 V (mm) */
    uint8_t  occupied;      /* 占领状态: 0=无, 1=红, 2=蓝 (技能赛忽略) */
    uint8_t  reserved[2];      /* 保留，填0 */
} GridInfo;                 /* 共8字节 */

/* DEVICE_INFO */
typedef struct {
    char     firmware_version[8];
    uint8_t  protocol_version;
    uint8_t  reserved[3];
} DeviceInfo;               /* 共12字节 */

/* MATCH_START / TARGET_WORDS */
typedef struct {
    uint8_t  word1;
    uint8_t  word2;
    uint8_t  field_side;
    uint8_t  reserved;
} MatchStart;               /* 共4字节 */

/* MOVE_TO */
typedef struct {
    int16_t  target_x;
    int16_t  target_y;
    uint8_t  speed_level;   /* 暂保留，填0 */
    uint8_t  reserved[3];
} MoveTo;                   /* 共8字节 */

/* MOVE_STATUS */
typedef struct {
    uint8_t  status;        /* 0x00=移动中,0x01=到达,0x02=失败 */
    int16_t  current_x;
    int16_t  current_y;
    uint8_t  reserved;
} MoveStatus;               /* 共6字节 */

/* POSITION_UPDATE */
typedef struct {
    int16_t  x;             /* 地面坐标系 X (mm) */
    int16_t  y;             /* 地面坐标系 Y (mm) */
    int16_t  yaw;           /* 航向角，单位 0.1° */
} PositionUpdate;           /* 共6字节 */

/* SHOOT */
typedef struct {
    uint8_t  force_level;   /* 力度等级 1-10 */
    uint8_t  reserved[3];
} ShootCmd;                 /* 共4字节 */

/* SHOOT_RESULT */
typedef struct {
    uint8_t  result;        /* 0=未中,1=命中,2=故障 */
    uint8_t  radical_id;    /* 命中偏旁ID (仅当result=1时有效) */
    uint8_t  reserved[2];
} ShootResult;              /* 共4字节 */

/* ERROR */
typedef struct {
    uint8_t  error_code;
    uint8_t  original_cmd;
    uint8_t  reserved[2];
} ErrorInfo;                /* 共4字节 */

#pragma pack(pop)

/*============================================================================
 * 环形缓冲区
 *============================================================================*/

typedef struct {
    uint8_t           buffer[RX_RINGBUF_SIZE];
    volatile uint16_t head;     /* 写指针 (中断上下文修改) */
    volatile uint16_t tail;     /* 读指针 (主循环修改) */
    uint16_t          mask;
} RingBuffer;

extern RingBuffer rx_ringbuf;

/*============================================================================
 * 公开接口
 *============================================================================*/

void protocol_init(void);

/* 数据喂入 (由 USB 接收回调调用) */
void protocol_feed_data(uint8_t *data, uint16_t len);

/* 主循环调用，处理接收帧并执行命令 */
void protocol_task(void);

/* 获取是否已连接（DTR有效） */
bool protocol_is_connected(void);
void protocol_set_connected(bool connected);

/*============================================================================
 * 发送函数 (内部调用 USBLIB_Transmit)
 *============================================================================*/

void protocol_send_heartbeat(void);
void protocol_send_device_info(void);
void protocol_send_match_start_ack(void);
void protocol_send_move_status(uint8_t status, int16_t x, int16_t y);
void protocol_send_position_update(int16_t x, int16_t y, int16_t yaw);
void protocol_send_shoot_result(uint8_t result, uint8_t radical_id);
void protocol_send_error(uint8_t error_code, uint8_t original_cmd);

/* 查询命令发送 */
void protocol_send_grid_query(void);



#endif /* INC_PROTOCOL_H_ */
