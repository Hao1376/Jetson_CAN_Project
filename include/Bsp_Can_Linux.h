#ifndef _BSP_CAN_LINUX_H_
#define _BSP_CAN_LINUX_H_

#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <libsocketcan.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <errno.h>

/* 所有 CAN 口共用同一个 epoll 实例 */
extern int can_epfd;

typedef struct
{
    const char *ifname;
    uint32_t    bitrate;
} LinuxCanConfig;

extern LinuxCanConfig g_can_cfg[2];

/* 配置并启动一个 SocketCAN 接口，返回绑定的 socket fd；失败返回 -1 */
int Can_Init(const LinuxCanConfig *cfg);

/* 把 fd 注册到共用 epoll */
int Can_Read_Register(int fd);

/* 非阻塞读一帧：1 成功，0 无数据/被打断，-1 出错 */
int Can_Read_One(int fd, struct can_frame *frame);

#endif
