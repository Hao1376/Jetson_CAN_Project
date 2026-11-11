/**
 * @file    Star_Can_Protocol.c
 * @brief   CAN 协议实现
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-07-28
 */

/* 需要 POSIX 时钟接口(clock_gettime/CLOCK_MONOTONIC)，须放在所有头文件之前 */
#define _POSIX_C_SOURCE 199309L

#include "Star_Can_Protocol.h"
#include <stdio.h>
#include <time.h>

#define MULTI_FRAME_TIMEOUT_MS  500   /* 复帧接收超时阈值(ms) */

typedef struct {
    uint8_t  multi_buffer[1024];     // 协议规定最大1024字节
    uint8_t  is_receiving;           // 是否正在接收复帧的标志
    uint16_t total_len;              // 应用数据总长度
    uint16_t received_len;           // 已接收长度
    uint32_t last_tick_ms;           // 复帧最近一帧到达时间(ms)
} RxMultiInfo_t;

static RxMultiInfo_t rx_state = {0};

static Can_Protocol_RxCallback_t s_rx_cb = NULL;
static int s_send_fd = -1;

/* 单调时钟毫秒 */
static uint32_t GetTickMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* 设置上层接收回调 */
void Can_Protocol_SetRxCallback(Can_Protocol_RxCallback_t cb)
{
    s_rx_cb = cb;
}

/* 设置发送句柄（供 SendPollCmd 使用） */
void Can_Protocol_SetSendFd(int fd)
{
    s_send_fd = fd;
}

/* 将完整应用帧交给上层（无回调时默认打印） */
static void NotifyFrame(uint32_t can_id, uint8_t frame_type,
                        const uint8_t *data, uint16_t len)
{
    if (s_rx_cb)
    {
        s_rx_cb(can_id, frame_type, data, len);
        return;
    }
}

/* 复帧接收状态重置 */
static void RxMultiReset(void)
{
    rx_state.is_receiving = 0;
    rx_state.received_len = 0;
    rx_state.total_len = 0;
}

/*
* 函数介绍：多帧接收处理函数
*	参数：can_id 原始CAN ID
*	参数：id 已解析的ID信息（在Can_Protocol_RxHandler解析过）
*	参数：data[] 数据帧
*	参数：DLC 数据长度
*	返回值：Portocol_Status 枚举Portocol_OK/Portocol_ERROR/Portocol_BUSY/Portocol_TIMEOUT
*	备注: 完整复帧会通过回调传给上层
*/
Portocol_Status Can_Protocol_FeedMultiFrame(uint32_t can_id, CanIdInfo_t id,
                                            uint8_t data[], uint8_t DLC)
{
    switch (id.multi_frame)
    {
        case MULTI_FRAME_FIRST:
        {
            if (DLC < 2)
                return Portocol_ERROR;   /* 首帧至少含2字节长度字段 */

            /* 解析总长度 */
            rx_state.total_len = (data[0] << 8) | data[1];
            if (rx_state.total_len > 1024)
                return Portocol_ERROR;

            /* 首帧载荷不能超过声明总长度 */
            if ((DLC - 2) > rx_state.total_len)
                return Portocol_ERROR;

            /* 拷贝首帧数据 */
            memcpy(rx_state.multi_buffer, &data[2], DLC - 2);
            rx_state.received_len = DLC - 2;
            rx_state.is_receiving = 1;
            rx_state.last_tick_ms = GetTickMs();
            break;
        }

        case MULTI_FRAME_MIDDLE:
        {
            if (!rx_state.is_receiving)
                return Portocol_ERROR;

            /* 超时检查：丢帧后自动恢复状态机 */
            if ((GetTickMs() - rx_state.last_tick_ms) > MULTI_FRAME_TIMEOUT_MS)
            {
                RxMultiReset();
                return Portocol_TIMEOUT;
            }

            /* 防止缓冲区溢出 */
            if (rx_state.received_len + DLC > rx_state.total_len)
            {
                RxMultiReset();
                return Portocol_ERROR;
            }
            memcpy(&rx_state.multi_buffer[rx_state.received_len], data, DLC);
            rx_state.received_len += DLC;
            rx_state.last_tick_ms = GetTickMs();
            break;
        }

        case MULTI_FRAME_LAST:
        {
            if (!rx_state.is_receiving)
                return Portocol_ERROR;

            /* 超时检查 */
            if ((GetTickMs() - rx_state.last_tick_ms) > MULTI_FRAME_TIMEOUT_MS)
            {
                RxMultiReset();
                return Portocol_TIMEOUT;
            }

            /* 末帧至少含1字节校验，DLC>=1 防止 DLC-1 下溢 */
            if (DLC < 1)
                return Portocol_ERROR;

            uint8_t data_len = DLC - 1;
            /* 防止缓冲区溢出 */
            if (rx_state.received_len + data_len > rx_state.total_len)
            {
                RxMultiReset();
                return Portocol_ERROR;
            }
            memcpy(&rx_state.multi_buffer[rx_state.received_len], data, data_len);
            rx_state.received_len += data_len;

            /* 累加校验（取低8位） */
            uint8_t calculated_sum = 0;
            for (int i = 0; i < rx_state.received_len; i++)
                calculated_sum += rx_state.multi_buffer[i];

            uint8_t received_sum = data[DLC - 1];
            Portocol_Status status =
                (calculated_sum == received_sum) ? Portocol_OK : Portocol_ERROR;

            if (status == Portocol_OK)
            {
                /* 完整复帧上传上层 */
                NotifyFrame(can_id, id.frame_type,
                            rx_state.multi_buffer, rx_state.received_len);
            }

            RxMultiReset();
            return status;
        }

        default:
            return Portocol_ERROR;
    }
    return Portocol_OK; /* 对于首帧和中间帧，返回OK表示已接收 */
}

/* 单帧接收处理函数 */
static inline Portocol_Status Can_Protocol_FeedSingalFrame(uint32_t can_id,
                                                           CanIdInfo_t id,
                                                           uint8_t data[],
                                                           uint8_t DLC)
{
    /* 完整数据直接传给上层处理 */
    NotifyFrame(can_id, id.frame_type, data, DLC);
    return Portocol_OK;
}

/*
* 函数介绍：Can协议层接收函数
*	参数：can_id ID
*	参数：data[] 数据帧
*	参数：DLC 数据长度
*	返回值：空
*	备注: 解析CAN_ID 转换成CanIdInfo_t 进入单独的单帧、复帧处理函数
*/
void Can_Protocol_RxHandler(uint32_t can_id, uint8_t data[], uint8_t DLC)
{
    CanIdInfo_t id;
    id.bus_owner    = ((can_id >> 10) & 0x01);
    id.frame_type   = ((can_id >> 8)  & 0x03);
    id.node_addr    = ((can_id >> 3)  & 0x1F);
    id.bus_flag     = ((can_id >> 2)  & 0x01);
    id.multi_frame  = ((can_id)       & 0x03);
    printf("can id :%d\n",can_id);
    if (id.multi_frame == MULTI_FRAME_SINGLE)
    {
        Can_Protocol_FeedSingalFrame(can_id, id, data, DLC);
    }
    else
    {
        /* 只要不是单帧，就交给复帧处理函数 */
        Can_Protocol_FeedMultiFrame(can_id, id, data, DLC);
    }
}

/*
* 函数介绍：发送轮询指令
*	参数：target_addr 目标节点地址
*	参数：packet_id 数据包ID
*	参数：bus_sel AB总线选择
*	备注: 需先调用 Can_Protocol_SetSendFd 设置发送句柄
*/
void Can_Protocol_SendPollCmd(uint8_t target_addr, uint8_t packet_id, uint8_t bus_sel)
{
    if (s_send_fd < 0)
    {
        fprintf(stderr, "SendPollCmd: send fd not set\n");
        return;
    }

    /* 按 CanIdInfo_t 位段构造ID：bus_owner|frame_type|node_addr|bus_flag|multi_frame */
    uint32_t can_id = ((0 & 0x01) << 10) |
                      ((FRAME_TYPE_POLL & 0x03) << 8) |
                      ((target_addr & 0x1F) << 3) |
                      ((bus_sel & 0x01) << 2) |
                      (MULTI_FRAME_SINGLE & 0x03);

    BSP_Can_Msg msg;
    memset(&msg, 0, sizeof(msg));
    msg.can_id = can_id;
    msg.DLC = 2;
    msg.data[0] = packet_id;
    msg.data[1] = 0x00;   /* 预留字段，按协议文档补充 */

    Can_Write(s_send_fd, &msg);
}



/*
* 函数介绍：（通用单zhen发送）遥测数据
*	参数：target_addr 目标节点地址
*	参数：packet_id 数据包ID
*	参数：bus_sel AB总线选择
*	备注: 需先调用 Can_Protocol_SetSendFd 设置发送句柄
*/
void Can_Protocol_SendData(uint8_t target_addr, uint8_t bus_sel, uint8_t *data, uint8_t DLC)
{
    if (s_send_fd < 0)
    {
        fprintf(stderr, "SendData: send fd not set\n");
        return;
    }

    /* 参数校验：DLC 不能超过 CAN 单帧最大长度，避免 memcpy 越界写栈 */
    if (DLC > CAN_MAX_DLC)
    {
        fprintf(stderr, "SendData: DLC(%u) > CAN_MAX_DLC(%d)\n", DLC, CAN_MAX_DLC);
        return;
    }
    if ((DLC > 0) && (data == NULL))
    {
        fprintf(stderr, "SendData: data is NULL\n");
        return;
    }

    /* 按 CanIdInfo_t 位段构造ID：bus_owner|frame_type|node_addr|bus_flag|multi_frame */
    uint32_t can_id = ((0 & 0x00) << 10) |
                      ((FRAME_TYPE_BROADCAST & 0x03) << 8) |
                      ((target_addr & 0x1F) << 3) |
                      ((bus_sel & 0x01) << 2) |
                      (MULTI_FRAME_SINGLE & 0x03);

    BSP_Can_Msg msg;
    memset(&msg, 0, sizeof(msg));
    msg.can_id = can_id;
    msg.DLC = DLC;
    if (DLC > 0)
        memcpy(&msg.data, data, DLC);

    Can_Write(s_send_fd, &msg);
}