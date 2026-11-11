/**
 * @file    Camera_Shot.c
 * @brief   拍照请求/回执通道：Unix domain socket 异步状态机
 * @author  张皓
 * @email   haozhang1018@gmail.com
 * @date    2026-11-10
 *
 * 流程：
 *   Request(): socket -> connect(/tmp/cam_ctrl.sock) -> send("capture\n")
 *              -> 转非阻塞 -> WAIT_REPLY
 *   Poll():    poll(0ms) 查可读
 *              - 收到 "img_xxx.jpg" -> Python 已完成 TFTP 上传 -> 删本地文件 -> 成功
 *              - 收到 "ERROR"       -> 失败（拍照失败或 TFTP 失败）
 *              - 超时 20s           -> 失败
 */

/* 需要 POSIX 接口(clock_gettime/poll/fcntl)与 MSG_NOSIGNAL，须放在所有头文件之前 */
#define _GNU_SOURCE

#include "Camera_Shot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>

typedef enum {
    SHOT_IDLE = 0,
    SHOT_WAIT_REPLY = 1
} ShotState_t;

static ShotState_t s_state      = SHOT_IDLE;
static int         s_fd         = -1;
static uint32_t    s_start_ms   = 0u;
static uint8_t     s_site       = 0u;
static uint8_t     s_index      = 0u;
static uint8_t     s_total      = 0u;

/* 单调时钟毫秒 */
static uint32_t Shot_MonoMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000));
}

/* 关闭连接并回到空闲态 */
static void Shot_Close(void)
{
    if (s_fd >= 0)
    {
        close(s_fd);
        s_fd = -1;
    }
    s_state = SHOT_IDLE;
}

/*
* 函数介绍：初始化（丢弃残留状态）
*	返回值：无
*/
void Camera_Shot_Init(void)
{
    Shot_Close();
}

/*
* 函数介绍：发起一次拍照（非阻塞）
*	参数：site 地点 参数：index 第几张(0起) 参数：total 总张数
*	返回值：0 已发出；-1 失败
*/
int Camera_Shot_Request(uint8_t site, uint8_t index, uint8_t total)
{
    if (s_state != SHOT_IDLE)
    {
        fprintf(stderr, "[CAM_SHOT] 上一次拍照还没结束，忽略本次请求\n");
        return -1;
    }

    /* 1. 连接控制 socket（Python 端必须已启动，否则此处立即失败，不会阻塞） */
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
    {
        perror("[CAM_SHOT] 创建 socket 失败");
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    (void)snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", CAM_SHOT_SOCK_PATH);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        fprintf(stderr, "[CAM_SHOT] 连接 %s 失败：%s（Camera_tftp.py 未启动？）\n",
                CAM_SHOT_SOCK_PATH, strerror(errno));
        close(fd);
        return -1;
    }

    /* 2. 发送拍照命令 */
    const char *cmd = CAM_SHOT_CMD_CAPTURE;
    size_t cmd_len  = strlen(cmd);
    ssize_t n = send(fd, cmd, cmd_len, MSG_NOSIGNAL);
    if (n != (ssize_t)cmd_len)
    {
        fprintf(stderr, "[CAM_SHOT] 发送拍照命令失败：%s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* 3. 转非阻塞，后续只在 Poll() 里查回包，避免卡住调度线程 */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    s_fd       = fd;
    s_state    = SHOT_WAIT_REPLY;
    s_site     = site;
    s_index    = index;
    s_total    = total;
    s_start_ms = Shot_MonoMs();

    printf("[CAM_SHOT] 已请求拍照：地点 %u 第 %u/%u 张，等待回包...\n",
           (unsigned)s_site, (unsigned)(s_index + 1u), (unsigned)s_total);
    return 0;
}

/*
* 函数介绍：查询本次拍照结果（非阻塞）
*	返回值：0 进行中；1 成功；-1 失败
*/
int Camera_Shot_Poll(void)
{
    if (s_state != SHOT_WAIT_REPLY)
        return 0;

    struct pollfd pfd;
    pfd.fd      = s_fd;
    pfd.events  = POLLIN;
    pfd.revents = 0;

    int pr = poll(&pfd, 1, 0);
    if (pr < 0)
    {
        if (errno == EINTR)
            return 0;
        perror("[CAM_SHOT] poll 失败");
        Shot_Close();
        return -1;
    }

    if (pr == 0)
    {
        /* 还没数据：检查超时 */
        if ((uint32_t)(Shot_MonoMs() - s_start_ms) >= CAM_SHOT_REPLY_TIMEOUT_MS)
        {
            fprintf(stderr, "[CAM_SHOT] 等待回包超时（%u ms）：地点 %u 第 %u/%u 张\n",
                    (unsigned)CAM_SHOT_REPLY_TIMEOUT_MS,
                    (unsigned)s_site, (unsigned)(s_index + 1u), (unsigned)s_total);
            Shot_Close();
            return -1;
        }
        return 0;
    }

    /* 对端关闭/出错（没有 POLLIN），直接判失败 */
    if ((pfd.revents & POLLIN) == 0)
    {
        fprintf(stderr, "[CAM_SHOT] socket 异常（revents=0x%X），对端可能已退出\n",
                (unsigned)pfd.revents);
        Shot_Close();
        return -1;
    }

    /* 数据已就绪，读回包（Python 一条 sendall，一次读完即可） */
    char buf[256];
    ssize_t n = recv(s_fd, buf, sizeof(buf) - 1u, 0);
    if (n <= 0)
    {
        fprintf(stderr, "[CAM_SHOT] 接收回包失败/对端关闭：%s\n", strerror(errno));
        Shot_Close();
        return -1;
    }
    buf[n] = '\0';

    /* 去掉行尾换行 */
    char *nl = strchr(buf, '\n');
    if (nl) *nl = '\0';

    int ret;
    if (strncmp(buf, "img_", 4u) == 0)
    {
        /*
         * Python 端是「先 TFTP 上传成功、再回报文件名」，
         * 所以走到这里代表照片已经发出去了，清掉本地副本即可。
         */
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", CAM_SHOT_FRAME_DIR, buf);
        if (remove(path) == 0)
            printf("[CAM_SHOT] 地点 %u 第 %u/%u 张完成：%s 已上传并删除本地文件\n",
                   (unsigned)s_site, (unsigned)(s_index + 1u),
                   (unsigned)s_total, buf);
        else
            fprintf(stderr, "[CAM_SHOT] 删除本地文件失败：%s（%s）\n",
                    path, strerror(errno));
        ret = 1;
    }
    else if (strcmp(buf, "ERROR") == 0)
    {
        fprintf(stderr, "[CAM_SHOT] Python 回报 ERROR：地点 %u 第 %u/%u 张"
                        "（拍照失败或 TFTP 上传失败，本地文件已保留）\n",
                (unsigned)s_site, (unsigned)(s_index + 1u), (unsigned)s_total);
        ret = -1;
    }
    else
    {
        fprintf(stderr, "[CAM_SHOT] 未知回包：\"%s\"\n", buf);
        ret = -1;
    }

    Shot_Close();
    return ret;
}

/* 是否还有一次拍照没结束 */
uint8_t Camera_Shot_IsBusy(void)
{
    return (s_state == SHOT_WAIT_REPLY) ? 1u : 0u;
}
