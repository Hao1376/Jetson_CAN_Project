/* 需要 GNU 时钟接口(CLOCK_BOOTTIME)，须放在所有头文件之前 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <Star_Can_Protocol.h>
#include <Camera_Ctrl_Cmd.h>
#include <Camera_Task.h>
#include <time.h>

/*
* 上层回调：完整应用帧（单帧或复帧拼接完成）上传
*	备注: 不在这里做业务，只按帧类型分发：
*	      命令帧(CMD)      -> 解析 8 字节控制帧，存全局 g_cam_ctrl_cmd
*	      广播帧(BROADCAST) -> 解析 UTC(0x11) 和经纬度(0x22)，存全局
*/
static void MyCanRxCallback(uint32_t can_id, uint8_t frame_type, const uint8_t *data, uint16_t len)
{
    Camera_CtrlCmd_OnCanFrame(can_id, frame_type, data, len);
}

/* 接收线程主循环 */
static void *Can_Rx_Thread(void *arg)
{
    struct epoll_event events[8];
    struct can_frame frame;
    (void)arg;

    while (1)
    {
        int n = epoll_wait(can_epfd, events, 8, 1000);  /* 1s 超时 */
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            perror("epoll_wait");
            break;
        }
        for (int i = 0; i < n; i++)
        {
            if (!(events[i].events & EPOLLIN))
                continue;
            int fd = events[i].data.fd;
            if (Can_Read_One(fd, &frame) == 1)
            {
                Can_Protocol_RxHandler(frame.can_id, frame.data, frame.can_dlc);
            }
        }
    }
    return NULL;
}

/*
* 业务处理线程：参数下发 / 经纬度围栏 / 连拍调度
*	备注: 接收线程只做解析+写全局，业务动作全部在本线程做，
*	      避免在 CAN 回调里阻塞（拍照要几十毫秒到几秒）
*/
static void *Camera_Ctrl_Thread(void *arg)
{
    uint32_t last_cmd_seq = g_cam_ctrl_cmd_seq;   /* 已处理到的命令序号 */
    (void)arg;

    while (1)
    {
        Camera_CtrlCmd_t cmd;
        uint32_t seq = 0u;

        /* 1. 有新命令帧 -> 参数择机下发 + 按地点装填经纬度围栏任务 */
        if (Camera_CtrlCmd_Fetch(&cmd, &seq) && seq != last_cmd_seq)
        {
            last_cmd_seq = seq;
            Camera_Task_OnCtrlCmd(&cmd);
        }

        /* 2. 推进经纬度围栏判断 / 连拍状态机 */
        Camera_Task_Poll();

        usleep(20 * 1000);   /* 20ms 轮询周期 */
    }
    return NULL;
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /* 挂载CAN设备与初始化，波特率内部已经设置过 */
    int fd0 = Can_Init(&g_can_cfg[0]);   /* can0 */
    int fd1 = Can_Init(&g_can_cfg[1]);   /* can1 */
    if (fd0 < 0 || fd1 < 0)
    {
        fprintf(stderr, "Can_Init failed: fd0=%d fd1=%d\n", fd0, fd1);
        return -1;
    }

    /* 注册epoll */
    if (Can_Read_Register(fd0) < 0 || Can_Read_Register(fd1) < 0)
    {
        fprintf(stderr, "Can_Read_Register failed\n");
        return -1;
    }

    printf("init OK: fd0=%d fd1=%d\n", fd0, fd1);
    fflush(stdout);

    /* 注册上层接收回调（单帧/复帧完整数据都会到这里） */
    Can_Protocol_SetRxCallback(MyCanRxCallback);
    Can_Protocol_SetSendFd(fd0);
    /* 创建接收线程 */
    pthread_t tid;
    if (pthread_create(&tid, NULL, Can_Rx_Thread, NULL) != 0)
    {
        perror("pthread_create");
        return -1;
    }

    /* 初始化拍照任务调度（经纬度围栏 / 连拍状态机） */
    Camera_Task_Init();

    /* 创建业务处理线程（消费 g_cam_ctrl_cmd / g_orbit_lat_deg / g_orbit_lon_deg） */
    pthread_t cam_tid;
    if (pthread_create(&cam_tid, NULL, Camera_Ctrl_Thread, NULL) != 0)
    {
        perror("pthread_create camera");
        return -1;
    }

    /* 阻塞等待接收线程退出，避免 main 返回导致进程退出 */
    pthread_join(tid, NULL);
    return 0;
}
