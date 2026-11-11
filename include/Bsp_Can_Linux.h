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

#define CAN_FRAME_DLC_MAX 8
extern int can_epfd; //
typedef struct {
    const char *ifname;
    uint32_t    bitrate;
} LinuxCanConfig;

typedef struct{
    unsigned char data[8];
    unsigned char DLC;
    unsigned int can_id;
}BSP_Can_Msg;

extern LinuxCanConfig g_can_cfg[2];


extern int Can_Init(const LinuxCanConfig *cfg);
extern int Can_Read_Register(int fd);
extern int Can_Write(int fd,BSP_Can_Msg* Msg);
extern int Can_Read_One(int fd, struct can_frame *frame);
#endif
