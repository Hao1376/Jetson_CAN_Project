/**
 * @file    Camera_Shot.h
 * @brief   与 Python 相机服务通信的 Unix socket 客户端
 *
 * 分工：
 *   C 端      ：根据经纬度围栏决定拍照时机，发 PARAM / CAPTURE 命令并等回执
 *   Python 端 ：独占 /dev/videoX，负责设置参数、拍照、存文件、TFTP 上传
 *
 * 协议（按行，UTF-8/ASCII，换行结束）：
 *   C -> Python:
 *       PARAM <key> <value>\n
 *       CAPTURE <site> <index> <total>\n
 *       PING\n
 *
 *   Python -> C:
 *       OK\n                      PARAM/PING 成功
 *       OK <filename>\n           CAPTURE 成功，文件已上传
 *       ERROR <reason>\n          任意命令失败
 *
 * 拍照是异步的：Request() 只负责发命令，Poll() 每 20ms 非阻塞查一次回包，
 * 避免 TFTP 上传十几秒卡住经纬度围栏和调度线程。
 */

#ifndef _CAMERA_SHOT_H
#define _CAMERA_SHOT_H

#include <stdint.h>

/* 以下两项必须与 Python 相机服务中的配置一致 */
#define CAM_SHOT_SOCK_PATH          "/tmp/cam_ctrl.sock"
#define CAM_SHOT_FRAME_DIR          "/home/Jetson/Desktop/project/Cam/Frame_File"

/* 等 Python 回包的超时(ms)：TFTP 超时 10s，这里留余量 */
#define CAM_SHOT_REPLY_TIMEOUT_MS   20000u

/* 命令字 */
#define CAM_SHOT_CMD_CAPTURE        "CAPTURE"
#define CAM_SHOT_CMD_PARAM          "PARAM"

void Camera_Shot_Init(void);

/*
 * 发起一次拍照（非阻塞，立即返回）
 *   site  地点编号
 *   index 本次连拍中的第几张（从 0 开始）
 *   total 本次连拍总张数
 * 返回：0 已发出；-1 失败（未连接/发送失败/上一次未结束）
 */
int Camera_Shot_Request(uint8_t site, uint8_t index, uint8_t total);

/*
 * 查询本次拍照结果（非阻塞，每个轮询周期调用一次）
 * 返回：0 等待中；1 成功；-1 失败/超时/socket 断开
 */
int Camera_Shot_Poll(void);

/*
 * 同步下发一个相机参数给 Python 服务
 *   key   协议键名：brightness / contrast / saturation / hue / gamma / gain / sharpness
 *   value 协议值（范围由 Camera_Ctrl_Cmd 的参数表约束）
 * 返回：0 Python 回 OK；-1 失败/超时
 */
int Camera_Shot_ApplyParam(const char *key, uint32_t value);

/* 1 表示还有一次拍照没结束；0 空闲 */
uint8_t Camera_Shot_IsBusy(void);

#endif /* _CAMERA_SHOT_H */
