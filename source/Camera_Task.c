/**
 * @file    Camera_Task.c
 * @brief   拍照任务调度：经纬度地理围栏 + 到点连拍 + 参数择机下发
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 */

/* 需要 POSIX 时钟接口(clock_gettime/CLOCK_MONOTONIC)，须放在所有头文件之前 */
#define _POSIX_C_SOURCE 199309L

#include "Camera_Task.h"
#include "Camera_Shot.h"
#include <stdio.h>
#include <time.h>

/* 等待进入范围时，当前位置日志的最小间隔(ms) */
#define CAM_TASK_POS_LOG_MS   10000u

/* ======================================================================
 *  地点经纬度范围表
 *  ----------------------------------------------------------------------
 *  地点编号 0 表示“空”，不装填任务；1/2/3 分别对应福州/南京/北京。
 *  下面的经纬度是城市大致包围盒，按实际任务需要直接改这里即可。
 *  判断条件：lat_min <= lat <= lat_max 且 lon_min <= lon <= lon_max。
 * ====================================================================== */
typedef struct
{
    uint8_t     site;    /* 地点编号（命令帧 byte2） */
    const char *name;    /* 地点名称（日志用） */
    float       lat_min; /* 纬度下限（度，北纬为正） */
    float       lat_max; /* 纬度上限 */
    float       lon_min; /* 经度下限（度，东经为正） */
    float       lon_max; /* 经度上限 */
} SiteRange_t;

static const SiteRange_t s_site_ranges[] =
{
    { 1u, "福州", 25.6f, 26.5f, 118.8f, 119.8f },
    { 2u, "南京", 31.6f, 32.5f, 118.3f, 119.3f },
    { 3u, "北京", 39.4f, 40.4f, 115.8f, 117.0f },
};

#define SITE_RANGE_NUM  (sizeof(s_site_ranges) / sizeof(s_site_ranges[0]))

/* ---------------- 运行态 ---------------- */
static Camera_TaskState_t s_state        = CAM_TASK_IDLE;
static uint8_t  s_site                   = 0u;
static uint8_t  s_shots                  = 0u;   /* 本次任务总张数 */
static uint8_t  s_shot_index             = 0u;   /* 已拍到第几张 */
static uint32_t s_last_shot_ms           = 0u;   /* 上次触发拍照的单调毫秒 */
static uint8_t  s_shot_inflight          = 0u;   /* 已发出请求、还在等 Python 回包 */
static uint32_t s_last_pos_log_ms        = 0u;   /* 位置日志节流 */

/* 缓存的命令（连拍中收到的，拍完再执行） */
static Camera_CtrlCmd_t s_pending_cmd;
static uint8_t          s_has_pending    = 0u;

/* ---------------- 对外暴露的运行态 ---------------- */
volatile uint8_t  g_cam_task_state      = CAM_TASK_IDLE;
volatile uint8_t  g_cam_task_site       = 0u;
volatile uint8_t  g_cam_task_shots      = 0u;

/* 单调时钟毫秒 */
static uint32_t MonoMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000));
}

/* 查地点经纬度范围 */
static const SiteRange_t *Task_FindSite(uint8_t site)
{
    for (uint8_t i = 0u; i < (uint8_t)SITE_RANGE_NUM; i++)
    {
        if (s_site_ranges[i].site == site)
            return &s_site_ranges[i];
    }
    return NULL;
}

/* 取当前有效经纬度：由 Camera_Ctrl_Cmd 统一做新鲜度/配对检查 */
static uint8_t Task_GetPosition(float *lat, float *lon)
{
    return Camera_CtrlCmd_GetOrbit(lat, lon);
}

/* 判断经纬度是否落在地点范围内 */
static uint8_t Task_InRange(const SiteRange_t *range, float lat, float lon)
{
    if (range == NULL)
        return 0u;

    return ((lat >= range->lat_min) && (lat <= range->lat_max) &&
            (lon >= range->lon_min) && (lon <= range->lon_max)) ? 1u : 0u;
}

/* 任务状态复位 */
static void Task_Reset(void)
{
    s_state           = CAM_TASK_IDLE;
    s_site            = 0u;
    s_shots           = 0u;
    s_shot_index      = 0u;
    s_shot_inflight   = 0u;
    s_last_pos_log_ms = 0u;

    g_cam_task_state = CAM_TASK_IDLE;
    g_cam_task_site  = 0u;
    g_cam_task_shots = 0u;
}

/* ======================================================================
 *  命令处理
 * ====================================================================== */

/*
* 函数介绍：按命令帧装填地点任务（只负责地点，不处理参数）
*	参数：cmd 命令帧
*	返回值：无
*	备注: 地点 0 不会走到这里；同一个地点重复下发只更新张数。
*/
static void Task_ArmFromCmd(const Camera_CtrlCmd_t *cmd)
{
    const SiteRange_t *range = Task_FindSite(cmd->site);
    uint8_t shots;

    if (range == NULL)
    {
        fprintf(stderr, "[CAM_TASK] 地点 %u 不在范围表中，忽略拍照任务\n",
                (unsigned)cmd->site);
        return;
    }

    shots = (cmd->shot_count != 0u) ? cmd->shot_count : 1u;

    /* 同一地点重复下发：只更新张数，不重置等待状态 */
    if ((s_state == CAM_TASK_ARMED) && (s_site == cmd->site))
    {
        s_shots          = shots;
        g_cam_task_shots = shots;
        printf("[CAM_TASK] 地点 %u（%s）重复下发，仅更新张数为 %u，继续等待进入范围\n",
               (unsigned)s_site, range->name, (unsigned)shots);
        return;
    }

    s_site            = cmd->site;
    s_shots           = shots;
    s_shot_index      = 0u;
    s_state           = CAM_TASK_ARMED;
    s_last_pos_log_ms = 0u;

    g_cam_task_state = CAM_TASK_ARMED;
    g_cam_task_site  = s_site;
    g_cam_task_shots = s_shots;

    printf("[CAM_TASK] 装填任务：地点 %u（%s）范围 纬度[%.4f,%.4f] 经度[%.4f,%.4f]，"
           "张数 %u，等待进入范围\n",
           (unsigned)s_site, range->name,
           (double)range->lat_min, (double)range->lat_max,
           (double)range->lon_min, (double)range->lon_max,
           (unsigned)s_shots);
}

/*
* 函数介绍：执行一条命令：参数立即下发 + 地点任务装填
*	参数：cmd 命令帧
*	返回值：无
*	备注: 对应三种组合：
*	      地点+参数 -> 下发参数 + 装填任务
*	      仅参数     -> 只下发参数
*	      仅地点     -> 只装填任务（param_type = 0x07）
*/
static void Task_ApplyCmd(const Camera_CtrlCmd_t *cmd)
{
    if (cmd->param_present != 0u)
    {
        (void)Camera_CtrlCmd_ApplyParam(cmd);
    }
    else
    {
        printf("[CAM_TASK] 本帧无参数（类型=0x%02X），跳过参数下发\n",
               (unsigned)cmd->param_type);
    }

    if (cmd->site != 0u)
    {
        Task_ArmFromCmd(cmd);
    }
    else
    {
        printf("[CAM_TASK] 本帧无地点（地点编号=0），只执行参数\n");
    }
}

/* ======================================================================
 *  对外接口
 * ====================================================================== */

/*
* 函数介绍：初始化
*	返回值：无
*/
void Camera_Task_Init(void)
{
    Task_Reset();
    s_has_pending    = 0u;
    s_last_pos_log_ms = 0u;

    /* 拍照通道（/tmp/cam_ctrl.sock）复位 */
    Camera_Shot_Init();

    printf("[CAM_TASK] 初始化：经纬度围栏任务，地点表 %u 条，控制套接字=%s，照片目录=%s\n",
           (unsigned)SITE_RANGE_NUM, CAM_SHOT_SOCK_PATH, CAM_SHOT_FRAME_DIR);

    for (uint8_t i = 0u; i < (uint8_t)SITE_RANGE_NUM; i++)
    {
        printf("[CAM_TASK]   地点 %u（%s）：纬度[%.4f,%.4f] 经度[%.4f,%.4f]\n",
               (unsigned)s_site_ranges[i].site, s_site_ranges[i].name,
               (double)s_site_ranges[i].lat_min, (double)s_site_ranges[i].lat_max,
               (double)s_site_ranges[i].lon_min, (double)s_site_ranges[i].lon_max);
    }
}

/*
* 函数介绍：收到命令帧时调用
*	参数：cmd 已解析的命令帧
*	返回值：无
*	备注: 参数部分——不在连拍 -> 立刻下发；正在连拍 -> 缓存，拍完再下发
*	      任务部分——不在连拍 -> 立刻装填；正在连拍 -> 随参数一起延后
*/
void Camera_Task_OnCtrlCmd(const Camera_CtrlCmd_t *cmd)
{
    if (cmd == NULL)
        return;

    if (s_state == CAM_TASK_SHOOTING)
    {
        /* 连拍中：整帧缓存，避免中途改参数/打断任务 */
        s_pending_cmd = *cmd;
        s_has_pending = 1u;
        printf("[CAM_TASK] 连拍中（%u/%u），本帧已缓存，拍完再执行\n",
               (unsigned)s_shot_index, (unsigned)s_shots);
        return;
    }

    Task_ApplyCmd(cmd);
}

/* ======================================================================
 *  周期轮询
 * ====================================================================== */

/* 连拍结束后处理缓存的命令 */
static void Task_AfterShooting(void)
{
    if (s_has_pending == 0u)
        return;

    s_has_pending = 0u;
    printf("[CAM_TASK] 连拍结束，执行缓存的命令\n");
    Task_ApplyCmd(&s_pending_cmd);
}

/* 本次连拍结束：回 IDLE 并处理连拍期间缓存的命令 */
static void Task_FinishShooting(void)
{
    uint8_t site  = s_site;
    uint8_t shots = s_shots;

    printf("[CAM_TASK] 地点 %u 连拍完成（%u 张）\n", (unsigned)site, (unsigned)shots);
    Task_Reset();
    Task_AfterShooting();
}

/*
* 函数介绍：周期轮询（建议 20~50ms 一次）
*	返回值：无
*	备注: ARMED 状态判断经纬度是否进入地点范围，进入后开始连拍；
*	      SHOOTING 状态按间隔逐张触发，拍完回 IDLE
*/
void Camera_Task_Poll(void)
{
    switch (s_state)
    {
        case CAM_TASK_ARMED:
        {
            float lat = 0.0f;
            float lon = 0.0f;
            const SiteRange_t *range = Task_FindSite(s_site);

            if (range == NULL)
            {
                Task_Reset();
                break;
            }

            /* 还没收到有效经纬度：继续等 */
            if (Task_GetPosition(&lat, &lon) == 0u)
                break;

            /* 还没进入范围：低频打印当前位置，便于现场判断 */
            if (Task_InRange(range, lat, lon) == 0u)
            {
                uint32_t now_ms = MonoMs();

                if ((s_last_pos_log_ms == 0u) ||
                    ((uint32_t)(now_ms - s_last_pos_log_ms) >= CAM_TASK_POS_LOG_MS))
                {
                    s_last_pos_log_ms = now_ms;
                    printf("[CAM_TASK] 等待地点 %u（%s）：当前 纬度=%.6f 经度=%.6f\n",
                           (unsigned)s_site, range->name,
                           (double)lat, (double)lon);
                }
                break;
            }

            /* 经纬度进入范围：开始连拍 */
            printf("[CAM_TASK] 地点 %u（%s）进入范围：纬度=%.6f 经度=%.6f，开始连拍 %u 张\n",
                   (unsigned)s_site, range->name,
                   (double)lat, (double)lon, (unsigned)s_shots);

            s_state           = CAM_TASK_SHOOTING;
            s_shot_index      = 0u;
            s_last_shot_ms    = MonoMs() - CAM_TASK_SHOT_INTERVAL_MS; /* 立即拍第一张 */
            g_cam_task_state  = CAM_TASK_SHOOTING;
            break;
        }

        case CAM_TASK_SHOOTING:
        {
            /* 上一张已经发出、还在等 Python 回包（含 TFTP 上传）：先不推进 */
            if (s_shot_inflight != 0u)
            {
                int r = Camera_Shot_Poll();
                if (r == 0)
                    break;              /* 还没回包，下个周期再看 */

                s_shot_inflight = 0u;
                s_shot_index++;         /* 本张已结束（成功或失败） */
                s_last_shot_ms = MonoMs();

                if (s_shot_index >= s_shots)
                {
                    Task_FinishShooting();
                    break;
                }

                if (r < 0)
                    printf("[CAM_TASK] 地点 %u 第 %u/%u 张失败，继续下一张\n",
                           (unsigned)s_site, (unsigned)s_shot_index,
                           (unsigned)s_shots);
                break;
            }

            /* 两张之间的间隔节流 */
            if ((uint32_t)(MonoMs() - s_last_shot_ms) < CAM_TASK_SHOT_INTERVAL_MS)
                break;

            /* 发起下一张（非阻塞：connect + 发 capture，回包在下一轮 Poll 收） */
            if (Camera_Task_TriggerShot(s_site, s_shot_index, s_shots) == 0)
            {
                s_shot_inflight = 1u;
                break;
            }

            /* 发起就失败（Python 没起来 / socket 异常）：跳过这一张 */
            fprintf(stderr, "[CAM_TASK] 地点 %u 第 %u/%u 张发起失败\n",
                    (unsigned)s_site, (unsigned)(s_shot_index + 1u),
                    (unsigned)s_shots);
            s_shot_index++;
            s_last_shot_ms = MonoMs();
            if (s_shot_index >= s_shots)
                Task_FinishShooting();
            break;
        }

        case CAM_TASK_IDLE:
        default:
            break;
    }
}

/* 是否正在拍照 */
uint8_t Camera_Task_IsBusy(void)
{
    return (s_state == CAM_TASK_SHOOTING) ? 1u : 0u;
}

/* ======================================================================
 * 按快门：非阻塞发起一次拍照
 * ======================================================================
 *  Camera_Shot_Request() 只做 connect("/tmp/cam_ctrl.sock") + 发 "capture\n"
 *  然后立刻返回，不会等拍照/TFTP 结果（那要十几秒，会卡死 20ms 调度周期）。
 *  结果由 Camera_Shot_Poll() 在 Camera_Task_Poll() 里非阻塞收取。
 *
 *  TFTP 上传在 Python 端完成（Camera_tftp.py -> Send_With_TFTP ->
 *  tftp -m binary <TFTP_SERVER> -c put <本地图> <远程名>），
 *  Python 是「上传成功后才回报 img_xxx.jpg」，所以 C 端收到文件名即成功，
 *  随后删除 Frame_File 下的本地副本；回 "ERROR" 则保留本地文件便于重传。
 * ====================================================================== */
int Camera_Task_TriggerShot(uint8_t site, uint8_t index, uint8_t total)
{
    return Camera_Shot_Request(site, index, total);
}
