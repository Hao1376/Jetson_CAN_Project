#include "Bsp_Can_Linux.h"
#include <stdio.h>

LinuxCanConfig g_can_cfg[2] = {
    { .ifname = "can0", .bitrate = 500000 },
    { .ifname = "can1", .bitrate = 500000 },
};

int can_epfd = -1;

int Can_Init(const LinuxCanConfig *cfg)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int fd;

    if (cfg == NULL)
        return -1;

    /* 先停止接口，避免接口已 up 时 can_set_bitrate 返回 EBUSY */
    (void)can_do_stop(cfg->ifname);

    if (can_set_bitrate(cfg->ifname, cfg->bitrate) < 0)
    {
        perror("[CAN] 设置波特率失败");
        return -1;
    }

    /* bus-off 后 100 ms 自动重启；驱动不支持时仅告警，不阻止启动 */
    if (can_set_restart_ms(cfg->ifname, 100) < 0)
        perror("[CAN] 设置重启时间失败");

    if (can_do_start(cfg->ifname) < 0)
    {
        perror("[CAN] 启动接口失败");
        return -1;
    }

    fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
    {
        perror("[CAN] 创建 SocketCAN 套接字失败");
        return -1;
    }

    /* 不接收自己发出的帧 */
    {
        int recv_own = 0;
        (void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                         &recv_own, sizeof(recv_own));
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, cfg->ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("[CAN] 获取接口索引失败");
        close(fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("[CAN] 绑定接口失败");
        close(fd);
        return -1;
    }

    return fd;
}

int Can_Read_Register(int fd)
{
    struct epoll_event ev;

    if (can_epfd < 0)
    {
        can_epfd = epoll_create1(EPOLL_CLOEXEC);
        if (can_epfd < 0)
        {
            perror("[CAN] 创建 epoll 失败");
            return -1;
        }
    }

    memset(&ev, 0, sizeof(ev));
    ev.events   = EPOLLIN;
    ev.data.fd  = fd;

    if (epoll_ctl(can_epfd, EPOLL_CTL_ADD, fd, &ev) < 0)
    {
        perror("[CAN] 注册 epoll 失败");
        return -1;
    }

    return 0;
}

int Can_Read_One(int fd, struct can_frame *frame)
{
    ssize_t n;

    if (frame == NULL)
        return -1;

    n = read(fd, frame, sizeof(*frame));
    if (n < 0)
    {
        if (errno == EINTR)
            return 0;
        perror("[CAN] 读取帧失败");
        return -1;
    }

    /* 半帧视为无效，直接丢弃 */
    if (n < (ssize_t)sizeof(*frame))
        return 0;

    return 1;
}
