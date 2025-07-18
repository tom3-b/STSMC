#include <sched.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <cmath>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <malloc.h>
#include "Nano.h"
#include "slave.h"

int run = 1;
pthread_t nano_thread;
/*Nano udp 通信*/
static int sockfd;
struct sockaddr_in server_addr, client_addr; // 服务器和客户端地址结构
socklen_t addr_len = sizeof(client_addr);    // 地址结构长度
static Nano_request_st nano_request;         // 待发送的数据结构体
static Nano_responce_st nano_response;       // struct Nano_responce_st nano_response;// 接收的数据结构体

void signal_handler(int sig)
{
    run = 0;
}

/*Nano cycle thread*/
void *nano_cyclic(void *arg)
{
    /*连接 XDDP 通信*/
    char *devname, buf[60];
    int fd, ret;

    if (asprintf(&devname,
                 "/proc/xenomai/registry/rtipc/xddp/%s",
                 XDDP_PORT_LABEL) < 0)
    {
        printf("socket set label fail\r\n");
        return NULL;
    }

    fd = open(devname, O_RDWR | O_NONBLOCK);
    free(devname);
    if (fd < 0)
    {
        printf("socket open fail\r\n");
        return NULL;
    }

    shared_buffer_t to_rt;
    double fz = 0;
    double a = 0.653;
    /*连接 XDDP 通信结束*/

    /*初始化udp通信*/
    /*连接Nano传感器socket初始化*/
    // 创建UDP套接字
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        printf("套接字创建失败\r\n"); // 输出错误信息
    }
    // 配置目标设备地址
    memset(&server_addr, 0, sizeof(server_addr));           // 清零地址结构
    server_addr.sin_family = AF_INET;                       // 设置地址族为IPv4
    server_addr.sin_port = htons(49152);                    // 设置端口号（主机字节序转网络字节序）
    server_addr.sin_addr.s_addr = inet_addr("192.168.2.1"); // 设置IP地址
    /*启动nano发送*/
    // 准备要发送的数据
    nano_request.command_header = htons(0x1234);
    nano_request.command = htons(0x0002);
    nano_request.sample_count = 0;
    if (sendto(sockfd, &nano_request, sizeof(nano_request), 0, // 发送数据结构体
               (struct sockaddr *)&server_addr,                // 目标地址
               sizeof(server_addr)) < 0)
    {                               // 地址长度
        printf("数据发送失败\r\n"); // 发送失败时输出错误
        close(sockfd);
        return NULL; // 关闭套接字
    }
    printf("数据已发送到 192.168.2.1:49152\r\n"); // 打印发送成功信息
    ssize_t recv_len;
    uint32_t cycle_count = 0;
    while (run)
    { // 无限循环
        if (!run)
        {
            break;
        }
        // 清空缓冲区
        memset(&nano_response, 0, sizeof(nano_response)); // 每次接收前清零缓冲区
        // 接收响应
        recv_len = recvfrom(sockfd, &nano_response, sizeof(nano_response), MSG_WAITALL, // 接收数据到缓冲区
                            (struct sockaddr *)&client_addr,                            // 存储发送方地址
                            &addr_len);                                                 // 地址长度
        if (recv_len < 0)
        {                               // 检查接收是否出错
            printf("数据接收失败\r\n"); // 输出错误信息
            // break;
            continue; // 继续循环，尝试再次接收
        }
        nano_response.rdt_sequence = ntohl(nano_response.rdt_sequence);
        nano_response.status = ntohl(nano_response.status);
        nano_response.Fx = (int32_t)ntohl((uint32_t)nano_response.Fx);
        nano_response.Fy = (int32_t)ntohl((uint32_t)nano_response.Fy);
        nano_response.Fz = (int32_t)ntohl((uint32_t)nano_response.Fz);
        nano_response.Tx = (int32_t)ntohl((uint32_t)nano_response.Tx);
        nano_response.Ty = (int32_t)ntohl((uint32_t)nano_response.Ty);
        nano_response.Tz = (int32_t)ntohl((uint32_t)nano_response.Tz);
        cycle_count++;
        // 处理接收到的数据
        memset(&to_rt, 0, sizeof(to_rt));
        to_rt.flag = 1;
        to_rt.f[0] = (double)nano_response.Fx / 1000000.0;
        to_rt.f[1] = (double)nano_response.Fy / 1000000.0;
        to_rt.f[2] = ((double)nano_response.Fz / 1000000.0) * a + (1 - a) * fz; // 低通滤波 发送频率1000 截止频率300
        to_rt.f[3] = (double)nano_response.Tx / 1000000.0;
        to_rt.f[4] = (double)nano_response.Ty / 1000000.0;
        to_rt.f[5] = (double)nano_response.Tz / 1000000.0;
        fz = to_rt.f[2];
        if ((cycle_count % 1000) == 0)
        {
            printf("fx fy fz = %f  %f  %f\r\n", to_rt.f[0], to_rt.f[1], to_rt.f[2]);
        }
        if (cycle_count >= 10 * 1000)
        {
            /* Get the next message from realtime_thread2. */
            ret = read(fd, buf, sizeof(buf));
            // xddp 通信
            /* Echo the message back to realtime_thread. */
            ret = write(fd, &to_rt, sizeof(to_rt));
            if (ret <= 0)
                printf("write fail\n");
        }
    }
    // 准备结束nano发送的数据
    nano_request.command_header = htons(0x1234);
    nano_request.command = 0x0000;
    nano_request.sample_count = 0;
    // 发送数据到目标设备
    if (sendto(sockfd, &nano_request, sizeof(nano_request), 0, // 发送数据结构体
               (struct sockaddr *)&server_addr,                // 目标地址
               sizeof(server_addr)) < 0)
    {                               // 地址长度
        printf("数据发送失败\r\n"); // 发送失败时输出错误
    }
    close(fd);
    close(sockfd); // 释放套接字资源
    printf("stoping thread %s\r\n", __func__);
    return NULL;
}

int main(int argc, char *argv[])
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    /* Create cyclic RT-thread */
    struct sched_param param = {.sched_priority = 40};
    pthread_attr_t regattr;
    pthread_attr_init(&regattr);
    pthread_attr_setdetachstate(&regattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&regattr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&regattr, SCHED_OTHER);
    pthread_attr_setschedparam(&regattr, &param);
    int ret = pthread_create(&nano_thread, &regattr, &nano_cyclic, NULL);
    if (ret)
    {
        fprintf(stderr, "%s: pthread_create cyclic task failed\n",
                strerror(-ret));
        return 1;
    }
    // 主线程等待特定信号
    pthread_join(nano_thread, NULL);
    printf("End of Program\n");
    return 0;
}