#ifndef _CAMERA_CTRL_CMD_H
#define _CAMERA_CTRL_CMD_H

#include <stdint.h>
#include <Star_Can_Protocol.h>

/*
 * 命令帧（FRAME_TYPE_CMD，目标节点 GPU，单帧 8 字节）
 *
 *   byte0  : 0x6B
 *   byte1  : 0x01
 *   byte2  : 地点  0x00 空 / 0x01 福州 / 0x02 南京 / 0x03 北京
 *   byte3  : 拍照次数
 *   byte4  : 参数类型  0x00 亮度 / 0x01 对比度 / 0x02 饱和度
 *                      0x03 色调 / 0x04 伽马 / 0x05 增益 / 0x06 锐度
 *                      0x07 空（本帧不设置参数）
 *   byte5~7: 参数值（3 字节，默认大端）
 *
 * 三种组合：
 *   地点 + 参数  -> 参数立即下发，再等经纬度进入地点范围拍照
 *   仅参数       -> 只下发参数，不装填拍照任务
 *   仅地点       -> 不设置参数，等进入地点范围拍照
 *
 * 广播帧（FRAME_TYPE_BROADCAST，8 字节）
 *   0x22 经纬度：byte1 = 0x01 经度 / 0x02 纬度，byte2~5 = float32 小端，
 *               byte6 = 有效位；收到后由 Camera_CtrlCmd_GetOrbit() 统一读取。
 *
 * 参数下发不直接打开相机设备，而是通过 Camera_Shot 转发给 Python 相机服务。
 */

#define CAM_CMD_LEN             8u
#define CAM_CMD_HEAD0           0x6Bu
#define CAM_CMD_HEAD1           0x01u
#define CAM_CMD_IDX_SITE        2u
#define CAM_CMD_IDX_SHOT_CNT    3u
#define CAM_CMD_IDX_PARAM_TYPE  4u
#define CAM_CMD_IDX_PARAM_VAL   5u

#define CAM_PARAM_NONE          0x07u

/* 3 字节参数值字节序：0 = 大端（默认），1 = 小端 */
#ifndef CAM_PARAM_LITTLE_ENDIAN
#define CAM_PARAM_LITTLE_ENDIAN 0
#endif

typedef enum
{
    CAM_PARAM_BRIGHTNESS = 0x00,
    CAM_PARAM_CONTRAST   = 0x01,
    CAM_PARAM_SATURATION = 0x02,
    CAM_PARAM_HUE        = 0x03,
    CAM_PARAM_GAMMA      = 0x04,
    CAM_PARAM_GAIN       = 0x05,
    CAM_PARAM_SHARPNESS  = 0x06,
    CAM_PARAM_NUM        = 0x07
} CameraParamType_t;

typedef struct
{
    uint8_t  type;
    uint32_t min;
    uint32_t max;
    const char *name;   /* 中文名，用于日志 */
    const char *key;    /* 协议键名，发给 Python 相机服务 */
} CameraParamInfo_t;

extern const CameraParamInfo_t g_cam_param_table[CAM_PARAM_NUM];

typedef struct
{
    uint8_t  head0;
    uint8_t  head1;
    uint8_t  site;
    uint8_t  shot_count;
    uint8_t  param_type;
    uint8_t  param_raw[3];
    uint32_t param_value;
    uint8_t  param_present; /* 参数类型 0x00~0x06 = 1 */
    uint8_t  param_valid;   /* 参数存在且在范围内 = 1 */
} Camera_CtrlCmd_t;

/* 最近一条合法命令，写入用序号保护，读取请用 Camera_CtrlCmd_Fetch() */
extern Camera_CtrlCmd_t  g_cam_ctrl_cmd;
extern volatile uint32_t g_cam_ctrl_cmd_seq;

/* 解析 8 字节命令帧 */
Portocol_Status Camera_CtrlCmd_ParseCmdFrame(const uint8_t *data, uint16_t len,
                                             Camera_CtrlCmd_t *out);

/* 读取最近一条命令（序号保护） */
uint8_t Camera_CtrlCmd_Fetch(Camera_CtrlCmd_t *out, uint32_t *seq_out);

/* 读取当前有效经纬度；新鲜度/配对检查不通过时返回 0 */
uint8_t Camera_CtrlCmd_GetOrbit(float *lat, float *lon);

/* 把命令中的参数交给 Python 相机服务下发 */
int Camera_CtrlCmd_ApplyParam(const Camera_CtrlCmd_t *cmd);

/* CAN 上层统一入口，按 frame_type 分发命令帧/广播帧 */
void Camera_CtrlCmd_OnCanFrame(uint32_t can_id, uint8_t frame_type,
                               const uint8_t *data, uint16_t len);

#endif
