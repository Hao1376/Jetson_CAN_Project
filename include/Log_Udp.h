#ifndef _LOG_UDP_H
#define _LOG_UDP_H

#include <stdint.h>

/*
 * 把 stdout/stderr 的日志同时转发到 UDP。
 * host 为空或 NULL 时禁用；port 为 0 时使用默认 9999。
 * 本地控制台仍会打印（tee），UDP 发送失败不影响程序运行。
 */
int Log_Udp_Init(const char *host, uint16_t port);

#endif
