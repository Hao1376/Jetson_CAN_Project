#ifndef _STAR_CAN_PROTOCOL_H
#define _STAR_CAN_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define CAN_MAX_DATA_LEN 1024   /* 复帧最大应用数据长度 */
#define CAN_BUS_MAX      2      /* 本工程使用 can0 / can1 两路 */

/* --- 节点地址（CAN ID bit7~3） --- */
typedef enum
{
    NODE_ADDR_OBC      = 0x00, /* 星务计算机 */
    NODE_ADDR_GNSS_SIM = 0x01, /* GNSS 模拟器 */
    NODE_ADDR_AOC      = 0x02, /* 另一台星务计算机 */
    NODE_ADDR_GPU      = 0x05, /* 边缘计算 GPU */
    NODE_ADDR_TMTC_SIM = 0x06, /* 测控应答机模拟器 */
    NODE_ADDR_DTS_SIM  = 0x07, /* 数传载荷模拟器 */
} NodeAddress_t;

/* --- 帧类型（CAN ID bit9~8） --- */
typedef enum
{
    FRAME_TYPE_CMD       = 0x00,
    FRAME_TYPE_BROADCAST = 0x01,
    FRAME_TYPE_BUS_CTRL  = 0x02,
    FRAME_TYPE_POLL      = 0x03,
} FrameType_t;

/* --- 单复帧信息（CAN ID bit1~0） --- */
typedef enum
{
    MULTI_FRAME_SINGLE = 0x00,
    MULTI_FRAME_FIRST  = 0x01,
    MULTI_FRAME_MIDDLE = 0x02,
    MULTI_FRAME_LAST   = 0x03,
} MultiFrameType_t;

/* --- 广播数据类型（应用数据 byte0） --- */
typedef enum
{
    BROADCAST_STAR_TIME       = 0x11,
    BROADCAST_SATELLITE_ORBIT = 0x22,
} BroadcastDataType_t;

/* --- CAN ID 解析结果 --- */
typedef struct
{
    uint8_t bus_owner;   /* bit10 */
    uint8_t frame_type;  /* bit9~8 */
    uint8_t node_addr;   /* bit7~3 */
    uint8_t bus_flag;    /* bit2 */
    uint8_t multi_frame; /* bit1~0 */
} CanIdInfo_t;

typedef enum
{
    Portocol_OK      = 0,
    Portocol_ERROR   = 1,
    Portocol_BUSY    = 2,
    Portocol_TIMEOUT = 3,
} Portocol_Status;

/* 完整应用帧（单帧或复帧拼接完成）回调 */
typedef void (*Can_Protocol_RxCallback_t)(uint32_t can_id, uint8_t frame_type,
                                          const uint8_t *data, uint16_t len);

void Can_Protocol_SetRxCallback(Can_Protocol_RxCallback_t cb);

/* bus: 0 = can0, 1 = can1；复帧状态按总线独立维护 */
void Can_Protocol_RxHandler(int bus, uint32_t can_id,
                            const uint8_t *data, uint8_t DLC);

#endif
