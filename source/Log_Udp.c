/**
 * @file    Log_Udp.c
 * @brief   把 stdout/stderr 日志 tee 到 UDP
 *
 * 实现方式：
 *   1. dup 保存原始 stdout/stderr；
 *   2. 创建 pipe，把 stdout/stderr 重定向到 pipe 写端；
 *   3. 后台线程按行读取 pipe，打印到原始 stdout，并 sendto 到 UDP。
 *
 * 这样现有所有 printf/fprintf/perror 都不用改。
 */

#define _GNU_SOURCE

#include "Log_Udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/socket.h>
#include <netdb.h>

static int s_udp_fd    = -1;
static int s_pipe_rd   = -1;
static int s_pipe_wr   = -1;
static int s_orig_out  = -1;
static int s_orig_err  = -1;
static pthread_t s_thread;
static int s_running   = 0;
static struct sockaddr_in s_dst;

static void Send_Line(const char *line, size_t len)
{
    char out[4200];
    struct timespec ts;
    struct tm tm_now;
    int prefix_len;
    size_t copy_len;

    if ((line == NULL) || (len == 0u))
        return;

    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm_now);

    prefix_len = snprintf(out, sizeof(out),
                          "[C %04d-%02d-%02d %02d:%02d:%02d.%03ld] ",
                          tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
                          tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec,
                          ts.tv_nsec / 1000000L);
    if (prefix_len < 0)
        prefix_len = 0;
    if ((size_t)prefix_len >= sizeof(out))
        prefix_len = (int)sizeof(out) - 1;

    copy_len = len;
    if (copy_len > (sizeof(out) - 1u - (size_t)prefix_len))
        copy_len = sizeof(out) - 1u - (size_t)prefix_len;

    memcpy(&out[prefix_len], line, copy_len);
    out[(size_t)prefix_len + copy_len] = '\n';

    /* 本地控制台仍然打印 */
    if (s_orig_out >= 0)
    {
        (void)write(s_orig_out, out, (size_t)prefix_len + copy_len + 1u);
    }

    /* UDP 发送失败直接丢弃，不影响业务 */
    if (s_udp_fd >= 0)
    {
        (void)sendto(s_udp_fd, out, (size_t)prefix_len + copy_len + 1u,
                     MSG_DONTWAIT, (struct sockaddr *)&s_dst, sizeof(s_dst));
    }
}

static void *Log_Reader_Thread(void *arg)
{
    char buf[4096];
    char line[4096];
    size_t line_len = 0u;
    ssize_t n;

    (void)arg;

    while (1)
    {
        n = read(s_pipe_rd, buf, sizeof(buf));
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }
        if (n == 0)
            break;

        for (ssize_t i = 0; i < n; i++)
        {
            char c = buf[i];

            if ((c == '\n') || (line_len >= sizeof(line) - 1u))
            {
                line[line_len] = '\0';
                Send_Line(line, line_len);
                line_len = 0u;
            }
            else
            {
                line[line_len++] = c;
            }
        }
    }

    if (line_len > 0u)
    {
        line[line_len] = '\0';
        Send_Line(line, line_len);
    }

    return NULL;
}

int Log_Udp_Init(const char *host, uint16_t port)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    char port_str[16];
    int pipefd[2];
    int rc;

    if ((host == NULL) || (*host == '\0') || (s_running != 0))
        return 0;

    if (port == 0u)
        port = 9999u;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);

    rc = getaddrinfo(host, port_str, &hints, &res);
    if (rc != 0)
    {
        fprintf(stderr, "[LOG] 解析日志服务器 %s 失败：%s\n", host, gai_strerror(rc));
        return -1;
    }

    s_udp_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s_udp_fd < 0)
    {
        perror("[LOG] 创建 UDP socket 失败");
        freeaddrinfo(res);
        return -1;
    }

    memcpy(&s_dst, res->ai_addr, sizeof(s_dst));
    freeaddrinfo(res);

    s_orig_out = dup(STDOUT_FILENO);
    s_orig_err = dup(STDERR_FILENO);
    if ((s_orig_out < 0) || (s_orig_err < 0))
    {
        perror("[LOG] 保存原始 stdout/stderr 失败");
        goto fail;
    }

    if (pipe(pipefd) < 0)
    {
        perror("[LOG] 创建日志 pipe 失败");
        goto fail;
    }
    s_pipe_rd = pipefd[0];
    s_pipe_wr = pipefd[1];

    if (pthread_create(&s_thread, NULL, Log_Reader_Thread, NULL) != 0)
    {
        perror("[LOG] 创建日志线程失败");
        goto fail;
    }

    if ((dup2(s_pipe_wr, STDOUT_FILENO) < 0) ||
        (dup2(s_pipe_wr, STDERR_FILENO) < 0))
    {
        perror("[LOG] 重定向 stdout/stderr 失败");
        goto fail;
    }

    close(s_pipe_wr);
    s_pipe_wr = -1;

    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    s_running = 1;
    printf("[LOG] UDP 日志已启用：%s:%u\n", host, (unsigned)port);
    return 0;

fail:
    if (s_pipe_wr >= 0) close(s_pipe_wr);
    if (s_pipe_rd >= 0) close(s_pipe_rd);
    if (s_udp_fd >= 0) close(s_udp_fd);
    if (s_orig_out >= 0) close(s_orig_out);
    if (s_orig_err >= 0) close(s_orig_err);
    s_pipe_wr = s_pipe_rd = s_udp_fd = s_orig_out = s_orig_err = -1;
    return -1;
}
