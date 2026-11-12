/* 需要 GNU/POSIX 扩展(usleep / epoll)，须放在所有头文件之前 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <Bsp_Can_Linux.h>
#include <Log_Udp.h>
#include <Star_Can_Protocol.h>
#include <Camera_Ctrl_Cmd.h>
#include <Camera_Task.h>

typedef struct
{
    int fd0;
    int fd1;
} CanFds_t;

static CanFds_t s_can_fds;

static void MyCanRxCallback(uint32_t can_id, uint8_t frame_type,
                            const uint8_t *data, uint16_t len)
{
    Camera_CtrlCmd_OnCanFrame(can_id, frame_type, data, len);
}

/* 接收线程：同一个 epoll 管理 can0/can1，按 fd 区分总线 */
static void *Can_Rx_Thread(void *arg)
{
    CanFds_t *fds = (CanFds_t *)arg;
    struct epoll_event events[8];
    struct can_frame frame;

    while (1)
    {
        int n = epoll_wait(can_epfd, events, 8, 1000);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            perror("[主程序] epoll_wait 失败");
            break;
        }

        for (int i = 0; i < n; i++)
        {
            int fd;
            int bus;

            if ((events[i].events & EPOLLIN) == 0u)
                continue;

            fd = events[i].data.fd;
            if (fd == fds->fd0)
                bus = 0;
            else if (fd == fds->fd1)
                bus = 1;
            else
                continue;

            if (Can_Read_One(fd, &frame) == 1)
                Can_Protocol_RxHandler(bus, frame.can_id,
                                       frame.data, frame.can_dlc);
        }
    }

    return NULL;
}

/* 业务线程：消费命令缓存，推进经纬度围栏和连拍状态机 */
static void *Camera_Ctrl_Thread(void *arg)
{
    uint32_t last_cmd_seq = 0u;
    (void)arg;

    while (1)
    {
        Camera_CtrlCmd_t cmd;
        uint32_t seq = 0u;

        if (Camera_CtrlCmd_Fetch(&cmd, &seq) && (seq != last_cmd_seq))
        {
            last_cmd_seq = seq;
            Camera_Task_OnCtrlCmd(&cmd);
        }

        Camera_Task_Poll();
        usleep(20 * 1000);
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t rx_tid;
    pthread_t cam_tid;
    const char *log_host;
    const char *log_port;

    (void)argc;
    (void)argv;

    /*
     * 日志 tee 到 UDP：
     *   LOG_UDP_HOST 未设置 -> 默认 192.168.0.100（有线默认）
     *   Wi-Fi 热点下请设置成地面站实际 IP，例如 192.168.43.100
     *   LOG_UDP_HOST=""    -> 禁用 UDP
     *   LOG_UDP_PORT 未设置 -> 9999
     */
    log_host = getenv("LOG_UDP_HOST");
    if (log_host == NULL)
        log_host = "192.168.0.100";

    log_port = getenv("LOG_UDP_PORT");
    (void)Log_Udp_Init(log_host,
                       (uint16_t)((log_port != NULL) ? atoi(log_port) : 9999));

    /* 先初始化业务层，再启动线程，避免早到的命令被跳过 */
    Camera_Task_Init();
    Can_Protocol_SetRxCallback(MyCanRxCallback);

    s_can_fds.fd0 = Can_Init(&g_can_cfg[0]);
    s_can_fds.fd1 = Can_Init(&g_can_cfg[1]);
    if ((s_can_fds.fd0 < 0) || (s_can_fds.fd1 < 0))
    {
        fprintf(stderr, "[CAN] 初始化失败：can0=%d，can1=%d\n",
                s_can_fds.fd0, s_can_fds.fd1);
        return -1;
    }

    if ((Can_Read_Register(s_can_fds.fd0) < 0) ||
        (Can_Read_Register(s_can_fds.fd1) < 0))
    {
        fprintf(stderr, "[CAN] 注册 epoll 失败\n");
        return -1;
    }

    printf("[主程序] 初始化完成：can0 fd=%d，can1 fd=%d\n",
           s_can_fds.fd0, s_can_fds.fd1);

    if (pthread_create(&rx_tid, NULL, Can_Rx_Thread, &s_can_fds) != 0)
    {
        perror("[主程序] 创建 CAN 接收线程失败");
        return -1;
    }

    if (pthread_create(&cam_tid, NULL, Camera_Ctrl_Thread, NULL) != 0)
    {
        perror("[主程序] 创建相机业务线程失败");
        return -1;
    }

    pthread_join(rx_tid, NULL);
    return 0;
}
