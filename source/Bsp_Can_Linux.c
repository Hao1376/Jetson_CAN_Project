#include <Bsp_Can_Linux.h>
#include <stdio.h>



LinuxCanConfig g_can_cfg[2] = {
    { .ifname = "can0", .bitrate = 500000 },
    { .ifname = "can1", .bitrate = 500000 },
};

int can_epfd = -1;   /* 所有CAN共用同一 epoll 实例 */

int Can_Init(const LinuxCanConfig *cfg)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int fd;

    /* 1. Set Baud */
    if (can_set_bitrate(cfg->ifname, cfg->bitrate) < 0)
    {
        perror("Set_bitrate error");
        return -1;
    }

    /* 2. Start Pin */
    if (can_do_start(cfg->ifname) < 0)
    {
        perror("can_do_start");
        return -1;
    }
    /* 3. identify the mode */


    /* 3. Create Socket */
    fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
    {
        perror("socket error");
        return -1;
    }

    // close receive self
    int recv_own = 0;
    setsockopt(fd, SOL_CAN_RAW,CAN_RAW_RECV_OWN_MSGS,&recv_own,sizeof(recv_own));

    /* 4. Select Pin and write in ifr_name */
    strncpy(ifr.ifr_name, cfg->ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';   /* 确保以 '\0' 结尾 */
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }

    /* 5. bind */
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(fd);
        return -1;
    }

    return fd;
}

int Can_Write(int fd, BSP_Can_Msg *Msg)
{
    struct can_frame Frame;

    if ((Msg == NULL) || (Msg->DLC > 8))
    {
        return -1;
    }

    memset(&Frame, 0, sizeof(Frame));
    Frame.can_id = Msg->can_id;
    Frame.can_dlc = Msg->DLC;
    memcpy(Frame.data, Msg->data, Frame.can_dlc);

    ssize_t n = write(fd, &Frame, sizeof(Frame));
    if (n != (ssize_t)sizeof(Frame))
    {
        perror("Write Error");
        return -1;
    }
    return 0;
}

int Can_Read_Register(int fd)
{
    if (can_epfd < 0)
    {
        can_epfd = epoll_create1(EPOLL_CLOEXEC);
        if (can_epfd < 0)
        {
            perror("epoll_create1");
            return -1;
        }
    }

    struct epoll_event ev = {
        .events = EPOLLIN,
        .data.fd = fd
    };

    if (epoll_ctl(can_epfd, EPOLL_CTL_ADD, fd, &ev) < 0)
    {
        perror("epoll_ctl");
        return -1;
    }

    return 0;
}

int Can_Read_One(int fd, struct can_frame *frame)
{
    ssize_t n = read(fd, frame, sizeof(struct can_frame));
    if (n < 0)
    {
        if (errno == EINTR)
            return 0;       /* 信号打断，重试 */
        perror("read");
        return -1;
    }
    if (n < (ssize_t)sizeof(struct can_frame))  /* 半帧，非法丢弃 */
    {
        return 0;
    }

    return 1;
}
