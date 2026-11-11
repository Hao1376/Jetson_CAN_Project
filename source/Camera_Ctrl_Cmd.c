/**
 * @file    Camera_Ctrl_Cmd.c
 * @brief   相机控制帧 / UTC 广播帧解析与缓存
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 */

#include "Camera_Ctrl_Cmd.h"
#include <stdio.h>
#include <string.h>

#define CAM_CMD_FETCH_RETRY  8u   /* seqlock 读取最大重试次数 */

/* ---------------- 参数信息表（下标 == CameraParamType_t） ---------------- */
const CameraParamInfo_t g_cam_param_table[CAM_PARAM_NUM] = {
    { CAM_PARAM_BRIGHTNESS,   0u, 200u, "brightness" },
    { CAM_PARAM_CONTRAST,     0u, 200u, "contrast"   },
    { CAM_PARAM_SATURATION,   0u, 200u, "saturation" },
    { CAM_PARAM_HUE,          0u, 360u, "hue"        },
    { CAM_PARAM_GAMMA,        0u, 100u, "gamma"      },
    { CAM_PARAM_GAIN,         0u,  40u, "gain"       },
    { CAM_PARAM_SHARPNESS,    0u,  32u, "sharpness"  },
};

/* ---------------- 全局缓存 ---------------- */
Camera_CtrlCmd_t       g_cam_ctrl_cmd     = {0};
volatile uint32_t      g_cam_ctrl_cmd_seq = 0;

volatile uint32_t      g_utc_time         = 0;
volatile uint8_t       g_utc_time_valid   = 0;
volatile uint32_t      g_utc_time_seq     = 0;

/* 最近一次收到的星上经纬度（0x22 广播） */
volatile float         g_orbit_lon_deg    = 0.0f;
volatile float         g_orbit_lat_deg    = 0.0f;
volatile uint8_t       g_orbit_lon_valid  = 0u;
volatile uint8_t       g_orbit_lat_valid  = 0u;
volatile uint32_t      g_orbit_seq        = 0u;

/* 组合 3 字节参数为 32bit 值 */
static uint32_t Camera_CtrlCmd_ComposeParam(const uint8_t raw[3])
{
#if (CAM_PARAM_LITTLE_ENDIAN == 0)
    /* 大端：raw[0] 为最高字节 */
    return ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2];
#else
    /* 小端：raw[0] 为最低字节 */
    return ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[0];
#endif
}

/*
* 函数介绍：查询参数类型的取值范围
*	参数：type 参数类型
*	参数：min/max 输出范围（可为 NULL）
*	返回值：Portocol_OK / Portocol_ERROR
*/
Portocol_Status Camera_CtrlCmd_ParamRange(uint8_t type,
                                          uint32_t *min, uint32_t *max)
{
    if (type >= (uint8_t)CAM_PARAM_NUM)
        return Portocol_ERROR;

    if (min) *min = g_cam_param_table[type].min;
    if (max) *max = g_cam_param_table[type].max;
    return Portocol_OK;
}

/*
* 函数介绍：解析 8 字节命令帧
*	参数：data 数据域
*	参数：len  数据长度（建议按 DLC 传入，不足 8 直接判错）
*	参数：out  解析结果输出
*	返回值：Portocol_OK 成功；Portocol_ERROR 帧头/长度非法
*	备注: 参数类型 0x07 表示本帧不带参数（仅地点），param_present = 0；
*	      其它非法类型或数值超范围不算解析失败，只是 param_valid = 0，
*	      交由业务层决定是否采纳
*/
Portocol_Status Camera_CtrlCmd_ParseCmdFrame(const uint8_t *data, uint16_t len,
                                             Camera_CtrlCmd_t *out)
{
    if (data == NULL || out == NULL)
        return Portocol_ERROR;

    /* 单帧固定 8 字节，长度不足直接丢弃 */
    if (len < CAM_CMD_LEN)
    {
        fprintf(stderr, "[CAM_CMD] drop: len=%u < %u\n", (unsigned)len, CAM_CMD_LEN);
        return Portocol_ERROR;
    }

    /* 帧头校验：6BH 01H */
    if (data[0] != CAM_CMD_HEAD0 || data[1] != CAM_CMD_HEAD1)
    {
        fprintf(stderr, "[CAM_CMD] drop: bad head %02X %02X\n", data[0], data[1]);
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
    out->param_value  = Camera_CtrlCmd_ComposeParam(out->param_raw);

    /* 参数存在性 + 类型/取值范围校验 */
    out->param_present = 0u;
    out->param_valid   = 0u;

    if (out->param_type < (uint8_t)CAM_PARAM_NUM)
    {
        /* 0x00~0x06：本帧带参数 */
        const CameraParamInfo_t *info = &g_cam_param_table[out->param_type];

        out->param_present = 1u;
        if (out->param_value >= info->min && out->param_value <= info->max)
            out->param_valid = 1u;
        else
            fprintf(stderr, "[CAM_CMD] param out of range: %s=%u (%u~%u)\n",
                    info->name, (unsigned)out->param_value,
                    (unsigned)info->min, (unsigned)info->max);
    }
    else if (out->param_type == (uint8_t)CAM_PARAM_NONE)
    {
        /* 0x07：本帧不设置参数，仅地点 */
    }
    else
    {
        fprintf(stderr, "[CAM_CMD] unknown param type: %u\n",
                (unsigned)out->param_type);
    }

    return Portocol_OK;
}

/*
* 函数介绍：解析广播帧中的 UTC 星上时间
*	参数：data 数据域（byte0 = 0x11，byte1~4 = UTC 大端 32bit）
*	参数：len  数据长度
*	参数：utc_out UTC 秒输出
*	返回值：Portocol_OK / Portocol_ERROR
*/
Portocol_Status Camera_CtrlCmd_ParseBroadcastFrame(const uint8_t *data,
                                                   uint16_t len,
                                                   uint32_t *utc_out)
{
    if (data == NULL || utc_out == NULL)
        return Portocol_ERROR;

    /* 有效信息 5 字节：[0] 类型 + [1..4] UTC */
    if (len < 5u)
    {
        fprintf(stderr, "[CAM_UTC] drop: len=%u < 5\n", (unsigned)len);
        return Portocol_ERROR;
    }

    if (data[0] != (uint8_t)BROADCAST_STAR_TIME)
    {
        /* 非星上时间广播（如经纬度 0x22）由 Camera_CtrlCmd_ParseOrbitFrame 处理 */
        return Portocol_ERROR;
    }

    *utc_out = ((uint32_t)data[1] << 24) |
               ((uint32_t)data[2] << 16) |
               ((uint32_t)data[3] << 8)  |
               ((uint32_t)data[4]);

    return Portocol_OK;
}

/*
* 函数介绍：解析经纬度广播帧（0x22）
*	参数：data  数据域（byte0 = 0x22，byte1 = 0x01/0x02，byte2~5 = float32 小端）
*	参数：len   数据长度
*	参数：sub   输出：0x01 经度 / 0x02 纬度
*	参数：val   输出：经纬度（度）
*	参数：valid 输出：有效位
*	返回值：Portocol_OK / Portocol_ERROR
*	备注: float32 按小端 IEEE-754 解释。Jetson(ARM64) 与 f413/liantiao(Cortex-M)
*	      均为小端，按字节拼成 32bit 后直接当 float 用即可。
*/
Portocol_Status Camera_CtrlCmd_ParseOrbitFrame(const uint8_t *data, uint16_t len,
                                               uint8_t *sub, float *val,
                                               uint8_t *valid)
{
    union
    {
        uint32_t u;
        float    f;
    } cv;

    if (data == NULL || sub == NULL || val == NULL || valid == NULL)
        return Portocol_ERROR;

    if (len < 7u)
        return Portocol_ERROR;

    if (data[0] != (uint8_t)BROADCAST_SATELLITE_ORBIT)
        return Portocol_ERROR;

    if (data[1] != 0x01u && data[1] != 0x02u)
        return Portocol_ERROR;

    /* 小端 IEEE-754：byte2 为最低字节 */
    cv.u = ((uint32_t)data[2])       |
           ((uint32_t)data[3] << 8)  |
           ((uint32_t)data[4] << 16) |
           ((uint32_t)data[5] << 24);

    *sub   = data[1];
    *val   = cv.f;
    *valid = data[6];

    return Portocol_OK;
}

/* 写入命令帧（seqlock：写前序号变奇数，写后变偶数） */
static void Camera_CtrlCmd_Store(const Camera_CtrlCmd_t *cmd)
{
    g_cam_ctrl_cmd_seq++;               /* -> 奇数，标记正在写入 */
    __sync_synchronize();
    memcpy(&g_cam_ctrl_cmd, cmd, sizeof(*cmd));
    __sync_synchronize();
    g_cam_ctrl_cmd_seq++;               /* -> 偶数，写入完成 */
}

/*
* 函数介绍：读取最近一条命令帧（序号保护）
*	参数：out     命令帧输出
*	参数：seq_out 读取到的命令序号（可为 NULL）
*	返回值：1 成功；0 无有效数据或重试失败
*/
uint8_t Camera_CtrlCmd_Fetch(Camera_CtrlCmd_t *out, uint32_t *seq_out)
{
    if (out == NULL)
        return 0u;

    for (uint32_t i = 0u; i < CAM_CMD_FETCH_RETRY; i++)
    {
        uint32_t s1 = g_cam_ctrl_cmd_seq;
        if (s1 & 1u)                /* 正在写入，重试 */
            continue;

        __sync_synchronize();
        memcpy(out, (const void *)&g_cam_ctrl_cmd, sizeof(*out));
        __sync_synchronize();

        uint32_t s2 = g_cam_ctrl_cmd_seq;
        if (s1 == s2)
        {
            if (seq_out) *seq_out = s2;
            return 1u;
        }
    }

    return 0u;
}

/*
* 函数介绍：读取最近一次 UTC
*	参数：utc_out UTC 秒输出（可为 NULL）
*	返回值：1 = 已有有效 UTC；0 = 尚未收到
*/
uint8_t Camera_CtrlCmd_GetUtc(uint32_t *utc_out)
{
    uint8_t valid = g_utc_time_valid;
    if (utc_out) *utc_out = g_utc_time;
    return valid;
}

/*
* 函数介绍：CAN 上层回调统一入口
*	参数：can_id     原始 CAN ID
*	参数：frame_type 帧类型（can_id bit9-8）
*	参数：data       数据域
*	参数：len        数据长度
*	返回值：无
*	备注: 命令帧 node_addr = NODE_ADDR_GPU(0x05)；
*	      0x11 广播 node_addr = NODE_ADDR_OBC(0x00)，0x22 转发后 = NODE_ADDR_GPU(0x05)，
*	      所以按 frame_type 分发，不能只用 node_addr 过滤。
*/
void Camera_CtrlCmd_OnCanFrame(uint32_t can_id, uint8_t frame_type,
                               const uint8_t *data, uint16_t len)
{
    uint8_t node_addr = (uint8_t)((can_id >> 3) & 0x1F);

    switch (frame_type)
    {
        /* ---------- 命令帧：上位机下发拍照/参数控制 ---------- */
        case FRAME_TYPE_CMD:
        {
            Camera_CtrlCmd_t cmd;

            /* 只接收发给本节点(GPU)的指令 */
            if (node_addr != (uint8_t)NODE_ADDR_GPU)
                return;

            memset(&cmd, 0, sizeof(cmd));
            if (Camera_CtrlCmd_ParseCmdFrame(data, len, &cmd) != Portocol_OK)
                return;

            /* 缓存到全局变量 */
            Camera_CtrlCmd_Store(&cmd);

            printf("[CAM_CMD] site=%u shot=%u param=%s val=%u(0x%02X%02X%02X) %s\n",
                   (unsigned)cmd.site, (unsigned)cmd.shot_count,
                   (cmd.param_present != 0u)
                       ? g_cam_param_table[cmd.param_type].name : "none",
                   (unsigned)cmd.param_value,
                   cmd.param_raw[0], cmd.param_raw[1], cmd.param_raw[2],
                   (cmd.param_present == 0u)
                       ? "NONE" : (cmd.param_valid ? "OK" : "INVALID"));
            break;
        }

        /* ---------- 广播帧：星上时间(0x11) / 经纬度(0x22) ---------- */
        case FRAME_TYPE_BROADCAST:
        {
            uint32_t utc = 0u;

            /* 星上经纬度广播（0x22）：经度、纬度各一帧 */
            if ((data != NULL) && (len >= 1u) &&
                (data[0] == (uint8_t)BROADCAST_SATELLITE_ORBIT))
            {
                uint8_t sub   = 0u;
                uint8_t valid = 0u;
                float   val   = 0.0f;

                if (Camera_CtrlCmd_ParseOrbitFrame(data, len, &sub, &val, &valid)
                    != Portocol_OK)
                    return;

                if (sub == 0x01u)
                {
                    g_orbit_lon_deg   = val;
                    g_orbit_lon_valid = valid;
                    printf("[CAM_ORBIT] lon=%.6f valid=%u\n",
                           (double)val, (unsigned)valid);
                }
                else
                {
                    g_orbit_lat_deg   = val;
                    g_orbit_lat_valid = valid;
                    printf("[CAM_ORBIT] lat=%.6f valid=%u\n",
                           (double)val, (unsigned)valid);
                }
                g_orbit_seq++;
                break;
            }

            if (Camera_CtrlCmd_ParseBroadcastFrame(data, len, &utc) != Portocol_OK)
                return;

            /* 存入全局变量 */
            g_utc_time       = utc;
            g_utc_time_valid = 1u;
            g_utc_time_seq++;

            printf("[CAM_UTC] utc=%u (0x%08X)\n", (unsigned)utc, (unsigned)utc);
            break;
        }

        default:
            /* 轮询/总线控制等帧类型，相机侧暂不处理 */
            break;
    }
}

/*
* 函数介绍：把命令帧里的参数立刻下发到相机（当前为空实现）
*	参数：cmd 一条完整命令帧（已解析、已校验）
*	返回值：0 成功；-1 参数非法/下发失败
*	备注: 只在“没在连拍”时被 Camera_Task 调用；连拍中会先缓存
*	      具体实现方法见本文件末尾说明
*/
int Camera_CtrlCmd_ApplyParam(const Camera_CtrlCmd_t *cmd)
{
    if (cmd == NULL)
        return -1;

    if (cmd->param_present == 0u)
    {
        printf("[CAM_PARAM] no parameter (type=0x%02X), skip\n",
               (unsigned)cmd->param_type);
        return 0;
    }

    if (cmd->param_valid == 0u)
    {
        fprintf(stderr, "[CAM_PARAM] skip: type=%u value=%u invalid\n",
                (unsigned)cmd->param_type, (unsigned)cmd->param_value);
        return -1;
    }

    /* ================== 下发动作留空 ================== */
    /* TODO: 按 cmd->param_type 映射 V4L2 控件并把 cmd->param_value 换算后
     *       VIDIOC_S_CTRL 下发，具体方法见文件末尾【参数下发（V4L2）】 */
    printf("[CAM_PARAM] apply %s = %u (尚未下发到相机)\n",
           g_cam_param_table[cmd->param_type].name,
           (unsigned)cmd->param_value);

    return 0;
}

/* ======================================================================
 *  Camera_CtrlCmd_ApplyParam() 实现方法说明
 * ======================================================================
 *
 * 【参数下发（V4L2）】
 *   - 参数类型 -> V4L2 控件映射（打开 /dev/videoX 后 VIDIOC_S_CTRL）：
 *         0x00 亮度   -> V4L2_CID_BRIGHTNESS
 *         0x01 对比度 -> V4L2_CID_CONTRAST
 *         0x02 饱和度 -> V4L2_CID_SATURATION
 *         0x03 色调   -> V4L2_CID_HUE
 *         0x04 伽马   -> V4L2_CID_GAMMA
 *         0x05 增益   -> V4L2_CID_GAIN
 *         0x06 锐度   -> V4L2_CID_SHARPNESS
 *   - 注意：协议取值范围(0~200/0~360/…)与 V4L2 控件范围(通常 -64~64 等)
 *     并不一致，需先 VIDIOC_QUERYCTRL 取 qctrl.minimum/maximum，
 *     再做线性映射：
 *         v4l2_val = qmin + (param - pmin) * (qmax - qmin) / (pmax - pmin)
 *   - 参数范围校验直接用 g_cam_param_table[]，非法(cmd->param_valid == 0)
 *     时只记日志不执行，避免把相机设成异常状态。
 *
 * 【与 Camera_Task 的分工】
 *   - 本模块(s: Camera_Ctrl_Cmd)：协议解析 + 全局缓存 + 参数下发动作
 *   - Camera_Task：经纬度围栏 / 连拍调度 / 决定参数何时下发
 *   - 正在连拍时收到的参数由 Camera_Task 缓存，本次连拍结束后再调用
 *     本函数，避免同一组照片中途改参数导致曝光不一致。
 * ====================================================================== */
