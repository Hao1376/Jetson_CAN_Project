/**
 * @file    Camera_Shot.h
 * @brief   按下快门事件的上报/回执通道（对接 Camera_tftp.py 的 /tmp/cam_ctrl.sock）
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 *
 * ---------------------------------------------------------------------------
 * 分工（与 Jetson_Camera_Project/Camera_tftp.py 一致）
 * ---------------------------------------------------------------------------
 *   C 端（本模块） ：发 "capture\n" -> 等回包 -> 删本地文件
 *   Python 端       ：拍一张 -> 存 Frame_File -> TFTP 上传 -> 回报文件名/ERROR
 *
 *   TFTP 上传本身在 Python 端的 Send_With_TFTP() 里完成：
 *       tftp -m binary <TFTP_SERVER> -c put <本地路径> <远程文件名>
 *   而且 Python 是「先上传完再回报 C」，所以 C 端收到 img_xxx.jpg
 *   就代表已经上传成功了，直接删本地文件即可，不会与上传产生竞态。
 *
 *   如果以后改成「C 端自己做 TFTP」，只需把 Camera_Shot_Poll() 里
 *   收到 img_ 回包后的 remove() 那段替换成调用 tftp 的流程即可，
 *   本模块的异步状态机不用动。
 *
 * ---------------------------------------------------------------------------
 * 为什么必须异步
 * ---------------------------------------------------------------------------
 *   Camera_Task 的调度线程每 20ms 轮询一次，拍照（含 TFTP 上传）最长要
 *   十几秒。所以这里把一次拍照拆成：
 *        Request()  —— 非阻塞发起（connect + send，瞬时完成）
 *        Poll()     —— 每周期非阻塞查一次回包
 *   这样调度线程不会被卡住，经纬度围栏判断/参数下发照常工作。
 */

#ifndef _CAMERA_SHOT_H
#define _CAMERA_SHOT_H

#include <stdint.h>

/* 以下两项必须与 Camera_tftp.py 中的 CTRL_SOCKET / FRAME_FILE 完全一致 */
#define CAM_SHOT_SOCK_PATH          "/tmp/cam_ctrl.sock"
#define CAM_SHOT_FRAME_DIR          "/home/Jetson/Desktop/project/Cam/Frame_File"

/* 等 Python 回包的超时(ms)：Python 内 TFTP 超时 10s，这里留余量 */
#define CAM_SHOT_REPLY_TIMEOUT_MS   20000u

/* 命令字（与 Python Handle_Command 对应） */
#define CAM_SHOT_CMD_CAPTURE        "capture\n"
#define CAM_SHOT_CMD_BALANCE_WHITE  "baipingheng:"   /* 需要拼数字，如 baipingheng:1920 */

/*
 * 函数介绍：初始化（丢弃可能残留的 socket 状态）
 *  返回值：无
 */
void Camera_Shot_Init(void);

/*
 * 函数介绍：发起一次拍照（非阻塞，立即返回）
 *  参数：site  地点编号
 *  参数：index 本次是连拍中的第几张（从 0 开始）
 *  参数：total 本次连拍总张数
 *  返回值：0 已发出请求；-1 失败（未连接/发送失败/上一次还没结束）
 */
int Camera_Shot_Request(uint8_t site, uint8_t index, uint8_t total);

/*
 * 函数介绍：查询本次拍照的结果（非阻塞，每个轮询周期调用一次）
 *  返回值：0  还在等回包
 *          1  成功（收到 img_xxx.jpg，Python 已完成 TFTP，本地文件已删）
 *         -1  失败（超时 / ERROR / 回包异常 / socket 断开）
 */
int Camera_Shot_Poll(void);

/*
 * 函数介绍：是否还有一次拍照没结束
 *  返回值：1 进行中；0 空闲
 */
uint8_t Camera_Shot_IsBusy(void);

#endif /* _CAMERA_SHOT_H */
