/**
 * @file    Camera_Task.h
 * @brief   拍照任务调度：经纬度地理围栏 + 到点连拍 + 参数择机下发
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 *
 * ---------------------------------------------------------------------------
 * 运行逻辑
 * ---------------------------------------------------------------------------
 *  1. 地点编号 -> 经纬度范围表（在 Camera_Task.c 的 s_site_ranges 中维护）
 *       0x00 空 / 0x01 福州 / 0x02 南京 / 0x03 北京
 *
 *  2. 收到命令帧（6B 01 地点 张数 参数类型 参数值3B）后，按三种情况处理：
 *       a. 地点 != 0 且 参数类型 0x00~0x06
 *            -> 参数立即下发，然后装填地点任务，等待进入经纬度范围
 *       b. 地点 == 0 且 参数类型 0x00~0x06
 *            -> 只立即下发参数，不装填拍照任务
 *       c. 地点 != 0 且 参数类型 0x07（空）
 *            -> 不设置参数，只装填地点任务，等待进入经纬度范围
 *
 *  3. 装填任务后，每个轮询周期通过 Camera_CtrlCmd_GetOrbit() 读取经纬度：
 *       数据新鲜且落入该地点范围 -> 进入 SHOOTING，开始连拍
 *
 *  4. 拍完 -> 回 IDLE；连拍期间收到的命令会缓存，本次拍完再执行
 *
 * ---------------------------------------------------------------------------
 * 状态机
 * ---------------------------------------------------------------------------
 *   IDLE ──收到带地点的命令──> ARMED ──经纬度进入范围──> SHOOTING ──拍完──> IDLE
 *
 * ---------------------------------------------------------------------------
 * 说明
 * ---------------------------------------------------------------------------
 *  - “按快门”走 Camera_Task_TriggerShot() -> Camera_Shot_Request()，
 *    非阻塞地给 /tmp/cam_ctrl.sock 发 "CAPTURE ...\n"，结果由
 *    Camera_Shot_Poll() 在下个调度周期收（拍照 + TFTP 上传在 Python 端完成）。
 *  - 参数下发通过 Camera_Shot 交给 Python 相机服务执行。
 *  - 所有函数都在同一个业务线程里调用，无需再加锁。
 */

#ifndef _CAMERA_TASK_H
#define _CAMERA_TASK_H

#include <stdint.h>
#include <Camera_Ctrl_Cmd.h>

/* 连拍两张之间的间隔(ms) */
#define CAM_TASK_SHOT_INTERVAL_MS   200u

/* ---------------- 任务状态 ---------------- */
typedef enum {
    CAM_TASK_IDLE     = 0,   /* 空闲：没有等待中的地点任务，也没有连拍 */
    CAM_TASK_ARMED    = 1,   /* 已装填地点任务，等待经纬度进入范围 */
    CAM_TASK_SHOOTING = 2    /* 正在连拍 */
} Camera_TaskState_t;

/* ---------------- 运行态（调试/显示用，只读） ---------------- */
extern volatile uint8_t  g_cam_task_state;      /* Camera_TaskState_t */
extern volatile uint8_t  g_cam_task_site;       /* 当前任务地点 */
extern volatile uint8_t  g_cam_task_shots;      /* 当前任务张数 */

/* ---------------- 接口 ---------------- */

/*
 * 函数介绍：初始化（清状态、打印地点范围表）
 *  返回值：无
 */
void Camera_Task_Init(void);

/*
 * 函数介绍：收到一条命令帧时调用
 *  参数：cmd 已解析的命令帧
 *  返回值：无
 *  备注：参数立即下发；地点非 0 时装填经纬度围栏任务；
 *        正在连拍时整帧缓存，拍完再执行。
 */
void Camera_Task_OnCtrlCmd(const Camera_CtrlCmd_t *cmd);

/*
 * 函数介绍：周期轮询（建议 20~50ms 调一次）
 *  返回值：无
 *  备注：ARMED 时判断经纬度是否进入地点范围，进入后开始连拍；
 *        SHOOTING 时按间隔逐张触发，拍完回 IDLE。
 */
void Camera_Task_Poll(void);

/*
 * 函数介绍：是否正在执行拍照
 *  返回值：1 正在连拍；0 其它
 */
uint8_t Camera_Task_IsBusy(void);

/*
 * 函数介绍：按一次快门（非阻塞发起，不等结果）
 *  参数：site  地点编号
 *  参数：index 本次是连拍中的第几张（从 0 开始）
 *  参数：total 本次连拍总张数
 *  返回值：0 已发出请求；-1 失败（如 Python 服务未启动）
 *  备注: 内部调用 Camera_Shot_Request()；结果由 Camera_Shot_Poll() 收
 */
int Camera_Task_TriggerShot(uint8_t site, uint8_t index, uint8_t total);

#endif
