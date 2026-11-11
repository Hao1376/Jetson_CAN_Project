/* 需要 POSIX 时钟接口(clock_gettime/CLOCK_MONOTONIC)，须放在所有头文件之前 */
#define _POSIX_C_SOURCE 199309L

#include "Star_Can_Protocol.h"
#include <time.h>

#define MULTI_FRAME_TIMEOUT_MS 500u

typedef struct
{
    uint8_t  buffer[CAN_MAX_DATA_LEN];
    uint8_t  is_receiving;
    uint16_t total_len;
    uint16_t received_len;
    uint32_t last_tick_ms;
} RxMultiInfo_t;

/* 每个 CAN 口独立维护复帧状态，避免两路总线互相串包 */
static RxMultiInfo_t rx_state[CAN_BUS_MAX];
static Can_Protocol_RxCallback_t s_rx_cb = NULL;

static uint32_t GetTickMs(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void RxMultiReset(int bus)
{
    rx_state[bus].is_receiving = 0u;
    rx_state[bus].total_len    = 0u;
    rx_state[bus].received_len = 0u;
}

static void ParseId(uint32_t can_id, CanIdInfo_t *id)
{
    id->bus_owner   = (uint8_t)((can_id >> 10) & 0x01u);
    id->frame_type  = (uint8_t)((can_id >> 8)  & 0x03u);
    id->node_addr   = (uint8_t)((can_id >> 3)  & 0x1Fu);
    id->bus_flag    = (uint8_t)((can_id >> 2)  & 0x01u);
    id->multi_frame = (uint8_t)(can_id & 0x03u);
}

static void NotifyFrame(uint32_t can_id, uint8_t frame_type,
                        const uint8_t *data, uint16_t len)
{
    if (s_rx_cb != NULL)
        s_rx_cb(can_id, frame_type, data, len);
}

void Can_Protocol_SetRxCallback(Can_Protocol_RxCallback_t cb)
{
    s_rx_cb = cb;
}

static Portocol_Status FeedMultiFrame(int bus, uint32_t can_id, CanIdInfo_t id,
                                      const uint8_t *data, uint8_t DLC)
{
    RxMultiInfo_t *st = &rx_state[bus];

    switch (id.multi_frame)
    {
        case MULTI_FRAME_FIRST:
        {
            /* 新的首帧总是丢弃上一包未完成的数据 */
            RxMultiReset(bus);

            if (DLC < 2u)
                return Portocol_ERROR;

            st->total_len = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            if ((st->total_len == 0u) || (st->total_len > CAN_MAX_DATA_LEN))
                return Portocol_ERROR;

            if ((uint16_t)(DLC - 2u) > st->total_len)
                return Portocol_ERROR;

            memcpy(st->buffer, &data[2], (size_t)(DLC - 2u));
            st->received_len = (uint16_t)(DLC - 2u);
            st->is_receiving = 1u;
            st->last_tick_ms = GetTickMs();
            return Portocol_OK;
        }

        case MULTI_FRAME_MIDDLE:
        {
            if (st->is_receiving == 0u)
                return Portocol_ERROR;

            if ((GetTickMs() - st->last_tick_ms) > MULTI_FRAME_TIMEOUT_MS)
            {
                RxMultiReset(bus);
                return Portocol_TIMEOUT;
            }

            if ((uint32_t)st->received_len + DLC > st->total_len)
            {
                RxMultiReset(bus);
                return Portocol_ERROR;
            }

            memcpy(&st->buffer[st->received_len], data, DLC);
            st->received_len = (uint16_t)(st->received_len + DLC);
            st->last_tick_ms = GetTickMs();
            return Portocol_OK;
        }

        case MULTI_FRAME_LAST:
        {
            uint8_t data_len;
            uint8_t sum = 0u;
            uint16_t i;

            if (st->is_receiving == 0u)
                return Portocol_ERROR;

            if ((GetTickMs() - st->last_tick_ms) > MULTI_FRAME_TIMEOUT_MS)
            {
                RxMultiReset(bus);
                return Portocol_TIMEOUT;
            }

            if (DLC < 1u)
            {
                RxMultiReset(bus);
                return Portocol_ERROR;
            }

            data_len = (uint8_t)(DLC - 1u);
            if ((uint32_t)st->received_len + data_len > st->total_len)
            {
                RxMultiReset(bus);
                return Portocol_ERROR;
            }

            memcpy(&st->buffer[st->received_len], data, data_len);
            st->received_len = (uint16_t)(st->received_len + data_len);

            for (i = 0u; i < st->received_len; i++)
                sum = (uint8_t)(sum + st->buffer[i]);

            if (sum != data[DLC - 1u])
            {
                RxMultiReset(bus);
                return Portocol_ERROR;
            }

            NotifyFrame(can_id, id.frame_type, st->buffer, st->received_len);
            RxMultiReset(bus);
            return Portocol_OK;
        }

        default:
            RxMultiReset(bus);
            return Portocol_ERROR;
    }
}

static Portocol_Status FeedSingleFrame(uint32_t can_id, CanIdInfo_t id,
                                       const uint8_t *data, uint8_t DLC)
{
    NotifyFrame(can_id, id.frame_type, data, DLC);
    return Portocol_OK;
}

void Can_Protocol_RxHandler(int bus, uint32_t can_id,
                            const uint8_t *data, uint8_t DLC)
{
    CanIdInfo_t id;

    if ((bus < 0) || (bus >= CAN_BUS_MAX) || (data == NULL) || (DLC == 0u))
        return;

    ParseId(can_id, &id);

    if (id.multi_frame == MULTI_FRAME_SINGLE)
        (void)FeedSingleFrame(can_id, id, data, DLC);
    else
        (void)FeedMultiFrame(bus, can_id, id, data, DLC);
}
