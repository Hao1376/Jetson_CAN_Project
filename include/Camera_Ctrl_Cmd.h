/**
 * @file    Camera_Ctrl_Cmd.h
 * @brief   相机控制帧（命令帧）与广播帧（UTC 星上时间）的解析与缓存
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 *
 * @note    本模块只做“接收 -> 解析 -> 缓存 + 参数下发动作”。
 *          经纬度围栏/连拍调度在 Camera_Task.c 里，二者分工见 Camera_Task.h。
 *
 * ---------------------------------------------------------------------------
 * 一、命令帧（FRAME_TYPE_CMD，target_addr = NODE_ADDR_GPU，单帧 DLC = 8）
 * ---------------------------------------------------------------------------
 *   byte0  : 0x6B               固定帧头 1
 *   byte1  : 0x01               固定帧头 2
 *   byte2  : 地点               0x00 空 / 0x01 福州 / 0x02 南京 / 0x03 北京
 *   byte3  : 拍照次数           进入地点经纬度范围后的连拍张数
 *   byte4  : 参数类型           0x00 亮度 / 0x01 对比度 / 0x02 饱和度
 *                               0x03 色调 / 0x04 伽马 / 0x05 增益 / 0x06 锐度
 *                               0x07 空（本帧不设置参数）
 *   byte5~7: 参数值             3 字节，默认大端（高字节在前）
 *
 *   例：6B 01 02 03 00 00 00 C8
 *       => 地点=南京, 拍照次数=3, 参数类型=亮度, 参数值=200
 *
 *   三种组合：
 *     1) 地点 + 参数：参数立即下发，然后等待经纬度进入地点范围再拍照
 *     2) 仅参数（地点=0）：参数立即下发，不装填拍照任务
 *     3) 仅地点（参数类型=0x07）：不设置参数，等待进入地点范围再拍照
 *
 * ---------------------------------------------------------------------------
 * 二、广播帧（FRAME_TYPE_BROADCAST，DLC = 8）
 * ---------------------------------------------------------------------------
 *  1) 星上时间（0x11）：
 *     byte0  : 0x11               BROADCAST_STAR_TIME
 *     byte1~4: UTC 秒             大端 32bit（上位机按 utc>>24 ... utc&0xFF 填充）
 *     byte5~7: 填充 0
 *
 *  2) 星上经纬度（0x22，由智能体星务从 CAN1 转发到 CAN2）：
 *     byte0  : 0x22               BROADCAST_SATELLITE_ORBIT
 *     byte1  : 0x01 经度 / 0x02 纬度
 *     byte2~5: float32 小端（IEEE-754，单位：度）
 *     byte6  : 有效位（1 = 有效）
 *     byte7  : 填充 0
 *     经度、纬度各一帧，收到后分别写入 g_orbit_lon_deg / g_orbit_lat_deg。
 *
 * ---------------------------------------------------------------------------
 * 三、注意
 * ---------------------------------------------------------------------------
 *   命令帧的 node_addr 字段是 NODE_ADDR_GPU(0x05)；0x11 星上时间广播的
 *   node_addr 是 NODE_ADDR_OBC(0x00)，而 0x22 经纬度由智能体星务转发后
 *   node_addr 是 NODE_ADDR_GPU(0x05)。因此分发必须依据 can_id 的
 *   frame_type 字段（bit9-8），不能只用 node_addr 过滤。
 */

#ifndef _CAMERA_CTRL_CMD_H
#define _CAMERA_CTRL_CMD_H

#include <stdint.h>
#include <Star_Can_Protocol.h>

/* ---------------- 命令帧格式定义 ---------------- */
#define CAM_CMD_LEN             8u        /* 命令帧固定长度 */
#define CAM_CMD_HEAD0           0x6Bu     /* 帧头 1 */
#define CAM_CMD_HEAD1           0x01u     /* 帧头 2 */
#define CAM_CMD_IDX_SITE        2u        /* 地点 */
#define CAM_CMD_IDX_SHOT_CNT    3u        /* 拍照次数 */
#define CAM_CMD_IDX_PARAM_TYPE  4u        /* 参数类型 */
#define CAM_CMD_IDX_PARAM_VAL   5u        /* 参数值起始下标（3 字节） */

/* 参数类型 0x07 表示本帧不设置参数（只有地点） */
#define CAM_PARAM_NONE          0x07u

/* 3 字节参数值的字节序：0 = 大端(高字节在前，默认)；1 = 小端 */
#ifndef CAM_PARAM_LITTLE_ENDIAN
#define CAM_PARAM_LITTLE_ENDIAN 0
#endif

/* ---------------- 参数类型 ---------------- */
typedef enum {
    CAM_PARAM_BRIGHTNESS = 0x00,   /* 亮度   0 ~ 200   */
    CAM_PARAM_CONTRAST   = 0x01,   /* 对比度 0 ~ 200   */
    CAM_PARAM_SATURATION = 0x02,   /* 饱和度 0 ~ 200   */
    CAM_PARAM_HUE        = 0x03,   /* 色调   0 ~ 360   */
    CAM_PARAM_GAMMA      = 0x04,   /* 伽马   0 ~ 100   */
    CAM_PARAM_GAIN       = 0x05,   /* 增益   0 ~ 40    */
    CAM_PARAM_SHARPNESS  = 0x06,   /* 锐度   0 ~ 32    */
    CAM_PARAM_NUM        = 0x07
} CameraParamType_t;

/* ---------------- 参数信息表（类型 / 取值范围 / 名称） ---------------- */
typedef struct {
    uint8_t     type;   /* CameraParamType_t */
    uint32_t    min;    /* 最小值 */
    uint32_t    max;    /* 最大值 */
    const char *name;   /* 名称（调试打印用） */
} CameraParamInfo_t;

extern const CameraParamInfo_t g_cam_param_table[CAM_PARAM_NUM];

/* ---------------- 解析后的命令帧 ---------------- */
typedef struct {
    uint8_t  head0;         /* 0x6B */
    uint8_t  head1;         /* 0x01 */
    uint8_t  site;          /* 地点编号 */
    uint8_t  shot_count;    /* 拍照次数 */
    uint8_t  param_type;    /* 参数类型 CameraParamType_t */
    uint8_t  param_raw[3];  /* 参数原始 3 字节（保真保存） */
    uint32_t param_value;   /* 参数值（按 CAM_PARAM_LITTLE_ENDIAN 组合） */
    uint8_t  param_present; /* 本帧带参数（参数类型 0x00~0x06）= 1 */
    uint8_t  param_valid;   /* 参数存在、类型合法且数值在范围内 = 1 */
} Camera_CtrlCmd_t;

/* ---------------- 全局缓存（供其它线程读取） ---------------- */

/* 最近一条合法命令帧；写入时用序号保护，读取请用 Camera_CtrlCmd_Fetch() */
extern Camera_CtrlCmd_t       g_cam_ctrl_cmd;
/* 命令帧序号：写入过程为奇数，写完为偶数，兼作“新命令”标志 */
extern volatile uint32_t      g_cam_ctrl_cmd_seq;
/* 最近一次收到的 UTC（星上时间，秒，UTC） */
extern volatile uint32_t      g_utc_time;
/* UTC 是否已收到过至少一帧有效值 */
extern volatile uint8_t       g_utc_time_valid;
/* UTC 更新次数（每收到一帧 UTC 广播 +1） */
extern volatile uint32_t      g_utc_time_seq;

/* 最近一次收到的星上经纬度（0x22 广播，经度/纬度各一帧） */
extern volatile float         g_orbit_lon_deg;   /* 经度（度），东经为正 */
extern volatile float         g_orbit_lat_deg;   /* 纬度（度），北纬为正 */
extern volatile uint8_t       g_orbit_lon_valid; /* 经度帧有效位 */
extern volatile uint8_t       g_orbit_lat_valid; /* 纬度帧有效位 */
extern volatile uint32_t      g_orbit_seq;       /* 每收到一帧经纬度 +1 */

/* ---------------- 接口 ---------------- */

/*
 * 函数介绍：CAN 上层回调统一入口（按 frame_type 分发到命令帧 / 广播帧）
 *  参数：can_id     原始 CAN ID
 *  参数：frame_type 帧类型（来自 can_id bit9-8）
 *  参数：data       数据域
 *  参数：len        数据长度
 *  返回值：无
 */
void Camera_CtrlCmd_OnCanFrame(uint32_t can_id, uint8_t frame_type,
                               const uint8_t *data, uint16_t len);

/*
 * 函数介绍：解析 8 字节命令帧
 *  参数：data 数据域   参数：len 数据长度
 *  参数：out  解析结果输出
 *  返回值：Portocol_OK 成功；Portocol_ERROR 帧头/长度/内容非法
 */
Portocol_Status Camera_CtrlCmd_ParseCmdFrame(const uint8_t *data, uint16_t len,
                                             Camera_CtrlCmd_t *out);

/*
 * 函数介绍：解析广播帧中的 UTC 星上时间
 *  参数：data 数据域   参数：len 数据长度
 *  参数：utc_out UTC 秒输出
 *  返回值：Portocol_OK 成功；Portocol_ERROR 非法
 */
Portocol_Status Camera_CtrlCmd_ParseBroadcastFrame(const uint8_t *data,
                                                   uint16_t len,
                                                   uint32_t *utc_out);

/*
 * 函数介绍：解析经纬度广播帧（0x22）
 *  参数：data  数据域
 *  参数：len   数据长度（>= 7）
 *  参数：sub   输出：0x01 经度 / 0x02 纬度
 *  参数：val   输出：float32 经纬度（度）
 *  参数：valid 输出：有效位
 *  返回值：Portocol_OK / Portocol_ERROR
 */
Portocol_Status Camera_CtrlCmd_ParseOrbitFrame(const uint8_t *data, uint16_t len,
                                               uint8_t *sub, float *val,
                                               uint8_t *valid);

/*
 * 函数介绍：读取最近一次 UTC（线程安全）
 *  参数：utc_out UTC 秒输出（可为 NULL）
 *  返回值：1 = 已有有效 UTC；0 = 尚未收到
 */
uint8_t Camera_CtrlCmd_GetUtc(uint32_t *utc_out);

/*
 * 函数介绍：读取最近一条命令帧（序号保护，避免读到写一半的数据）
 *  参数：out     命令帧输出
 *  参数：seq_out 读取到的命令序号（可为 NULL，用于判断是否为新命令）
 *  返回值：1 = 读取成功；0 = 无有效数据或重试失败
 */
uint8_t Camera_CtrlCmd_Fetch(Camera_CtrlCmd_t *out, uint32_t *seq_out);

/*
 * 函数介绍：查询参数类型的取值范围
 *  参数：type 参数类型   参数：min/max 输出范围（可为 NULL）
 *  返回值：Portocol_OK 合法；Portocol_ERROR 类型非法
 */
Portocol_Status Camera_CtrlCmd_ParamRange(uint8_t type,
                                          uint32_t *min, uint32_t *max);

/*
 * 函数介绍：把命令帧里的参数立刻下发到相机（V4L2）
 *  参数：cmd 一条完整的命令帧（已解析、已校验）
 *  返回值：0 = 成功（或空实现直接成功）；-1 = 参数非法/失败
 *  备注: 当前为空实现；是否“立刻调用”由 Camera_Task 调度决定
 *        （正在连拍时 Task 层会缓存，拍完再调用本函数）
 */
int Camera_CtrlCmd_ApplyParam(const Camera_CtrlCmd_t *cmd);

#endif /* _CAMERA_CTRL_CMD_H */
