/* 需要 POSIX 时钟接口(clock_gettime/CLOCK_MONOTONIC)，须放在所有头文件之前 */
#define _POSIX_C_SOURCE 199309L

#include "Camera_Ctrl_Cmd.h"
#include "Camera_Shot.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#define CAM_CMD_FETCH_RETRY    8u
#define CAM_ORBIT_FRESH_MS     2000u  /* 经纬度超过 2 s 未更新视为失效 */
#define CAM_ORBIT_PAIR_MS      300u   /* 经度、纬度两帧最大允许时间差 */

/* ---------------- 参数表 ---------------- */
const CameraParamInfo_t g_cam_param_table[CAM_PARAM_NUM] = {
    { CAM_PARAM_BRIGHTNESS,   0u, 200u, "亮度",   "brightness" },
    { CAM_PARAM_CONTRAST,     0u, 200u, "对比度", "contrast"   },
    { CAM_PARAM_SATURATION,   0u, 200u, "饱和度", "saturation" },
    { CAM_PARAM_HUE,          0u, 360u, "色调",   "hue"        },
    { CAM_PARAM_GAMMA,        0u, 100u, "伽马",   "gamma"      },
    { CAM_PARAM_GAIN,         0u,  40u, "增益",   "gain"       },
    { CAM_PARAM_SHARPNESS,    0u,  32u, "锐度",   "sharpness"  },
};

/* ---------------- 命令缓存 ---------------- */
Camera_CtrlCmd_t       g_cam_ctrl_cmd     = {0};
volatile uint32_t      g_cam_ctrl_cmd_seq = 0;

/* ---------------- 经纬度缓存（仅本文件可写） ---------------- */
static volatile float    s_orbit_lon_deg   = 0.0f;
static volatile float    s_orbit_lat_deg   = 0.0f;
static volatile uint8_t  s_orbit_lon_valid = 0u;
static volatile uint8_t  s_orbit_lat_valid = 0u;
static volatile uint32_t s_orbit_lon_ms    = 0u;
static volatile uint32_t s_orbit_lat_ms    = 0u;

static uint32_t MonoMs(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u +
                      (uint64_t)(ts.tv_nsec / 1000000));
}

static uint32_t ComposeParam(const uint8_t raw[3])
{
#if (CAM_PARAM_LITTLE_ENDIAN == 0)
    return ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
#else
    return ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
#endif
}

Portocol_Status Camera_CtrlCmd_ParseCmdFrame(const uint8_t *data, uint16_t len,
                                             Camera_CtrlCmd_t *out)
{
    if ((data == NULL) || (out == NULL))
        return Portocol_ERROR;

    if (len < CAM_CMD_LEN)
    {
        fprintf(stderr, "[CAM_CMD] 命令帧长度不足，丢弃：len=%u\n", (unsigned)len);
        return Portocol_ERROR;
    }

    if ((data[0] != CAM_CMD_HEAD0) || (data[1] != CAM_CMD_HEAD1))
    {
        fprintf(stderr, "[CAM_CMD] 命令帧头错误，丢弃：%02X %02X\n",
                data[0], data[1]);
        return Portocol_ERROR;
    }

    out->head0      = data[0];
    out->head1      = data[1];
    out->site       = data[CAM_CMD_IDX_SITE];
    out->shot_count = data[CAM_CMD_IDX_SHOT_CNT];
    out->param_type = data[CAM_CMD_IDX_PARAM_TYPE];

    out->param_raw[0] = data[CAM_CMD_IDX_PARAM_VAL + 0u];
    out->param_raw[1] = data[CAM_CMD_IDX_PARAM_VAL + 1u];
    out->param_raw[2] = data[CAM_CMD_IDX_PARAM_VAL + 2u];
    out->param_value  = ComposeParam(out->param_raw);

    out->param_present = 0u;
    out->param_valid   = 0u;

    if (out->param_type < (uint8_t)CAM_PARAM_NUM)
    {
        const CameraParamInfo_t *info = &g_cam_param_table[out->param_type];

        out->param_present = 1u;
        if ((out->param_value >= info->min) && (out->param_value <= info->max))
        {
            out->param_valid = 1u;
        }
        else
        {
            fprintf(stderr, "[CAM_CMD] 参数越界：%s=%u（%u~%u）\n",
                    info->name, (unsigned)out->param_value,
                    (unsigned)info->min, (unsigned)info->max);
        }
    }
    else if (out->param_type != (uint8_t)CAM_PARAM_NONE)
    {
        fprintf(stderr, "[CAM_CMD] 未知参数类型：%u\n",
                (unsigned)out->param_type);
    }

    return Portocol_OK;
}

static Portocol_Status ParseOrbitFrame(const uint8_t *data, uint16_t len,
                                       uint8_t *sub, float *val, uint8_t *valid)
{
    union
    {
        uint32_t u;
        float    f;
    } cv;

    if ((data == NULL) || (sub == NULL) || (val == NULL) || (valid == NULL))
        return Portocol_ERROR;

    if (len < 7u)
        return Portocol_ERROR;

    if (data[0] != (uint8_t)BROADCAST_SATELLITE_ORBIT)
        return Portocol_ERROR;

    if ((data[1] != 0x01u) && (data[1] != 0x02u))
        return Portocol_ERROR;

    /* 小端 IEEE-754 */
    cv.u = ((uint32_t)data[2])       |
           ((uint32_t)data[3] << 8)  |
           ((uint32_t)data[4] << 16) |
           ((uint32_t)data[5] << 24);

    *sub   = data[1];
    *val   = cv.f;
    *valid = data[6];

    return Portocol_OK;
}

uint8_t Camera_CtrlCmd_GetOrbit(float *lat, float *lon)
{
    uint32_t now_ms;
    uint32_t lon_ms;
    uint32_t lat_ms;
    uint32_t diff_ms;

    if ((lat == NULL) || (lon == NULL))
        return 0u;

    if ((s_orbit_lon_valid == 0u) || (s_orbit_lat_valid == 0u))
        return 0u;

    now_ms = MonoMs();
    lon_ms = s_orbit_lon_ms;
    lat_ms = s_orbit_lat_ms;

    if ((uint32_t)(now_ms - lon_ms) > CAM_ORBIT_FRESH_MS)
        return 0u;
    if ((uint32_t)(now_ms - lat_ms) > CAM_ORBIT_FRESH_MS)
        return 0u;

    diff_ms = (lon_ms > lat_ms) ? (lon_ms - lat_ms) : (lat_ms - lon_ms);
    if (diff_ms > CAM_ORBIT_PAIR_MS)
        return 0u;

    *lat = s_orbit_lat_deg;
    *lon = s_orbit_lon_deg;
    return 1u;
}

static void Camera_CtrlCmd_Store(const Camera_CtrlCmd_t *cmd)
{
    g_cam_ctrl_cmd_seq++;
    __sync_synchronize();
    memcpy(&g_cam_ctrl_cmd, cmd, sizeof(*cmd));
    __sync_synchronize();
    g_cam_ctrl_cmd_seq++;
}

uint8_t Camera_CtrlCmd_Fetch(Camera_CtrlCmd_t *out, uint32_t *seq_out)
{
    uint32_t i;

    if (out == NULL)
        return 0u;

    for (i = 0u; i < CAM_CMD_FETCH_RETRY; i++)
    {
        uint32_t s1 = g_cam_ctrl_cmd_seq;
        uint32_t s2;

        if ((s1 & 1u) != 0u)
            continue;

        __sync_synchronize();
        memcpy(out, (const void *)&g_cam_ctrl_cmd, sizeof(*out));
        __sync_synchronize();

        s2 = g_cam_ctrl_cmd_seq;
        if (s1 == s2)
        {
            if (seq_out != NULL)
                *seq_out = s2;
            return 1u;
        }
    }

    return 0u;
}

void Camera_CtrlCmd_OnCanFrame(uint32_t can_id, uint8_t frame_type,
                               const uint8_t *data, uint16_t len)
{
    uint8_t node_addr = (uint8_t)((can_id >> 3) & 0x1Fu);

    switch (frame_type)
    {
        case FRAME_TYPE_CMD:
        {
            Camera_CtrlCmd_t cmd;

            if ((data == NULL) || (node_addr != (uint8_t)NODE_ADDR_GPU))
                return;

            memset(&cmd, 0, sizeof(cmd));
            if (Camera_CtrlCmd_ParseCmdFrame(data, len, &cmd) != Portocol_OK)
                return;

            Camera_CtrlCmd_Store(&cmd);

            printf("[CAM_CMD] 地点=%u 张数=%u 参数=%s 值=%u %s\n",
                   (unsigned)cmd.site, (unsigned)cmd.shot_count,
                   (cmd.param_present != 0u)
                       ? g_cam_param_table[cmd.param_type].name : "无",
                   (unsigned)cmd.param_value,
                   (cmd.param_present == 0u)
                       ? "无参数" : (cmd.param_valid ? "有效" : "无效"));
            break;
        }

        case FRAME_TYPE_BROADCAST:
        {
            uint8_t sub = 0u;
            uint8_t valid = 0u;
            float   val = 0.0f;

            if ((data == NULL) || (len == 0u))
                return;

            if (data[0] != (uint8_t)BROADCAST_SATELLITE_ORBIT)
                return;

            if (ParseOrbitFrame(data, len, &sub, &val, &valid) != Portocol_OK)
                return;

            if (sub == 0x01u)
            {
                s_orbit_lon_deg   = val;
                s_orbit_lon_valid = valid;
                s_orbit_lon_ms    = MonoMs();
            }
            else
            {
                s_orbit_lat_deg   = val;
                s_orbit_lat_valid = valid;
                s_orbit_lat_ms    = MonoMs();
            }
            break;
        }

        default:
            break;
    }
}

int Camera_CtrlCmd_ApplyParam(const Camera_CtrlCmd_t *cmd)
{
    const CameraParamInfo_t *info;

    if (cmd == NULL)
        return -1;

    if (cmd->param_present == 0u)
        return 0;

    if ((cmd->param_valid == 0u) || (cmd->param_type >= (uint8_t)CAM_PARAM_NUM))
    {
        fprintf(stderr, "[CAM_PARAM] 参数无效：类型=%u，值=%u\n",
                (unsigned)cmd->param_type, (unsigned)cmd->param_value);
        return -1;
    }

    info = &g_cam_param_table[cmd->param_type];
    printf("[CAM_PARAM] 下发参数：%s=%u\n",
           info->name, (unsigned)cmd->param_value);

    return Camera_Shot_ApplyParam(info->key, cmd->param_value);
}
