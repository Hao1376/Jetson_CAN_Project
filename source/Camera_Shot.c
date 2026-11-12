/**
 * @file    Camera_Shot.c
 * @brief   与 Python 相机服务通信：参数下发 + 异步拍照回执
 *
 * 协议：
 *   PARAM <key> <value>\n      -> OK / ERROR ...
 *   CAPTURE <site> <index> <total>\n -> OK <filename> / ERROR ...
 *
 * 拍照是异步的：Request() 只发命令，Poll() 每 20ms 查一次回包，
 * 避免 Python 拍照 + TFTP 上传期间卡住经纬度围栏和调度线程。
 */

/* 需要 POSIX 接口(clock_gettime/poll/fcntl)与 MSG_NOSIGNAL，须放在所有头文件之前 */
#define _GNU_SOURCE

#include "Camera_Shot.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>

#define CAM_SHOT_PARAM_TIMEOUT_MS 2000u

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

static uint32_t Shot_MonoMs(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u +
                      (uint64_t)(ts.tv_nsec / 1000000));
}

static void Shot_Close(void)
{
    if (s_fd >= 0)
    {
        close(s_fd);
        s_fd = -1;
    }
    s_state = SHOT_IDLE;
}

static int Shot_Connect(void)
{
    struct sockaddr_un addr;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (fd < 0)
    {
        perror("[CAM_SHOT] 创建 socket 失败");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    (void)snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", CAM_SHOT_SOCK_PATH);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        fprintf(stderr, "[CAM_SHOT] 连接 %s 失败：%s（Python 相机服务未启动？）\n",
                CAM_SHOT_SOCK_PATH, strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

void Camera_Shot_Init(void)
{
    Shot_Close();
}

int Camera_Shot_Request(uint8_t site, uint8_t index, uint8_t total)
{
    char cmd[128];
    int n;
    int fd;
    int flags;

    if (s_state != SHOT_IDLE)
    {
        fprintf(stderr, "[CAM_SHOT] 上一次拍照还没结束，忽略本次请求\n");
        return -1;
    }

    fd = Shot_Connect();
    if (fd < 0)
        return -1;

    n = snprintf(cmd, sizeof(cmd), "%s %u %u %u\n",
                 CAM_SHOT_CMD_CAPTURE, (unsigned)site,
                 (unsigned)index, (unsigned)total);
    if ((n <= 0) || ((size_t)n >= sizeof(cmd)))
    {
        close(fd);
        return -1;
    }

    if (send(fd, cmd, (size_t)n, MSG_NOSIGNAL) != n)
    {
        fprintf(stderr, "[CAM_SHOT] 发送拍照命令失败：%s\n", strerror(errno));
        close(fd);
        return -1;
    }

    flags = fcntl(fd, F_GETFL, 0);
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

static int Shot_HandleSuccess(const char *filename)
{
    char path[512];

    if ((filename == NULL) || (*filename == '\0'))
    {
        printf("[CAM_SHOT] 地点 %u 第 %u/%u 张完成（Python 未返回文件名）\n",
               (unsigned)s_site, (unsigned)(s_index + 1u), (unsigned)s_total);
        return 1;
    }

    snprintf(path, sizeof(path), "%s/%s", CAM_SHOT_FRAME_DIR, filename);
    if (remove(path) == 0)
    {
        printf("[CAM_SHOT] 地点 %u 第 %u/%u 张完成：%s 已上传并删除本地文件\n",
               (unsigned)s_site, (unsigned)(s_index + 1u),
               (unsigned)s_total, filename);
    }
    else
    {
        fprintf(stderr, "[CAM_SHOT] 删除本地文件失败：%s（%s）\n",
                path, strerror(errno));
    }

    return 1;
}

int Camera_Shot_Poll(void)
{
    struct pollfd pfd;
    char buf[256];
    char *line_end;
    const char *filename = NULL;
    ssize_t n;
    int pr;
    int ret = -1;

    if (s_state != SHOT_WAIT_REPLY)
        return 0;

    pfd.fd      = s_fd;
    pfd.events  = POLLIN;
    pfd.revents = 0;

    pr = poll(&pfd, 1, 0);
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

    if ((pfd.revents & POLLIN) == 0)
    {
        fprintf(stderr, "[CAM_SHOT] socket 异常（revents=0x%X），对端可能已退出\n",
                (unsigned)pfd.revents);
        Shot_Close();
        return -1;
    }

    n = recv(s_fd, buf, sizeof(buf) - 1u, 0);
    if (n <= 0)
    {
        fprintf(stderr, "[CAM_SHOT] 接收回包失败/对端关闭：%s\n", strerror(errno));
        Shot_Close();
        return -1;
    }
    buf[n] = '\0';

    line_end = strpbrk(buf, "\r\n");
    if (line_end != NULL)
        *line_end = '\0';

    if (strncmp(buf, "OK", 2u) == 0)
    {
        filename = buf + 2;
        while (*filename == ' ')
            filename++;
        ret = Shot_HandleSuccess(filename);
    }
    else if (strncmp(buf, "img_", 4u) == 0)
    {
        /* 兼容旧版 Python 回包：img_xxx.jpg */
        ret = Shot_HandleSuccess(buf);
    }
    else if (strncmp(buf, "ERROR", 5u) == 0)
    {
        fprintf(stderr, "[CAM_SHOT] Python 回报错误：地点 %u 第 %u/%u 张"
                        "（拍照或 TFTP 上传失败，本地文件已保留）：%s\n",
                (unsigned)s_site, (unsigned)(s_index + 1u),
                (unsigned)s_total, buf);
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

int Camera_Shot_ApplyParam(const char *key, uint32_t value)
{
    struct pollfd pfd;
    char cmd[128];
    char buf[128];
    ssize_t n;
    int fd;
    int pr;
    int len;

    if ((key == NULL) || (*key == '\0'))
        return -1;

    fd = Shot_Connect();
    if (fd < 0)
        return -1;

    len = snprintf(cmd, sizeof(cmd), "%s %s %u\n",
                   CAM_SHOT_CMD_PARAM, key, (unsigned)value);
    if ((len <= 0) || ((size_t)len >= sizeof(cmd)))
    {
        close(fd);
        return -1;
    }

    if (send(fd, cmd, (size_t)len, MSG_NOSIGNAL) != len)
    {
        fprintf(stderr, "[CAM_PARAM] 发送参数命令失败：%s\n", strerror(errno));
        close(fd);
        return -1;
    }

    pfd.fd      = fd;
    pfd.events  = POLLIN;
    pfd.revents = 0;

    pr = poll(&pfd, 1, (int)CAM_SHOT_PARAM_TIMEOUT_MS);
    if (pr <= 0)
    {
        fprintf(stderr, "[CAM_PARAM] 等待 Python 回包%s：%s=%u\n",
                (pr == 0) ? "超时" : "失败", key, (unsigned)value);
        close(fd);
        return -1;
    }

    n = recv(fd, buf, sizeof(buf) - 1u, 0);
    if (n <= 0)
    {
        fprintf(stderr, "[CAM_PARAM] 接收回包失败：%s\n", strerror(errno));
        close(fd);
        return -1;
    }
    buf[n] = '\0';

    {
        char *line_end = strpbrk(buf, "\r\n");
        if (line_end != NULL)
            *line_end = '\0';
    }

    close(fd);

    if (strncmp(buf, "OK", 2u) == 0)
    {
        printf("[CAM_PARAM] Python 已确认：%s=%u\n", key, (unsigned)value);
        return 0;
    }

    fprintf(stderr, "[CAM_PARAM] Python 返回错误：%s=%u，%s\n",
            key, (unsigned)value, buf);
    return -1;
}

uint8_t Camera_Shot_IsBusy(void)
{
    return (s_state == SHOT_WAIT_REPLY) ? 1u : 0u;
}
