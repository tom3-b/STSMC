#include <stdio.h>      // 提供标准输入输出功能（printf, perror等）
#include <stdlib.h>     // 提供内存分配和程序退出功能（exit, EXIT_FAILURE等）
#include <string.h>     // 提供字符串操作功能（memset, strcpy等）
#include <unistd.h>     // 提供UNIX标准函数（close等）
#include <arpa/inet.h>  // 提供IP地址转换功能（inet_addr, htons等）
#include <sys/socket.h> // 提供套接字API（socket, sendto, recvfrom等）
#include <signal.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include "Nano.h"

NanoUDP nano_udp_st;
bool running = true;
void fail(const char *reason)
{
    perror(reason);
    exit(EXIT_FAILURE);
}

void *regular_thread(void *arg)
{
    char *devname;
    char buf[] = "sleep";
    int fd, ret;
    if (asprintf(&devname, "/dev/rtp%d", 0) < 0)
        fail("asprintf");

    fd = open(devname, O_RDWR);
    free(devname);
    if (fd < 0)
        fail("open");

    for (;;)
    {
        /* Get the next message from realtime_thread. */
        // ret = read(fd, buf, sizeof(buf));
        // if (ret <= 0)
        // 	fail("read");
        /* Echo the message back to realtime_thread. */
        ret = write(fd, buf, 5);
        if (ret <= 0)
            fail("write");
        sleep(1);
    }

    return NULL;
}

void *regular2_thread(void *arg)
{
    // int sockfd;                                  // 套接字文件描述符
    // struct sockaddr_in server_addr, client_addr; // 服务器和客户端地址结构
    // socklen_t addr_len = sizeof(client_addr);    // 地址结构长度

    // // 创建UDP套接字
    // if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    // {
    //     perror("套接字创建失败"); // 输出错误信息
    //     exit(EXIT_FAILURE);       // 终止程序
    // }

    // // 配置目标设备地址
    // memset(&server_addr, 0, sizeof(server_addr));           // 清零地址结构
    // server_addr.sin_family = AF_INET;                       // 设置地址族为IPv4
    // server_addr.sin_port = htons(49152);                    // 设置端口号（主机字节序转网络字节序）
    // server_addr.sin_addr.s_addr = inet_addr("192.168.2.1"); // 设置IP地址
    nano_udp_st.init();
    // 准备要发送的数据
    nano_udp_st.nano_request.command_header = htons(0x1234);
    nano_udp_st.nano_request.command = htons(0x0002);
    nano_udp_st.nano_request.sample_count = 0;
    // nano_request.command_header = htons(0x1234);
    // nano_request.command = htons(0x0002);
    // nano_request.sample_count = 0;

    // 发送数据到目标设备
    // if (sendto(sockfd, &nano_request, sizeof(nano_request), 0, // 发送数据结构体
    //            (struct sockaddr *)&server_addr,                // 目标地址
    //            sizeof(server_addr)) < 0)
    // {                           // 地址长度
    //     perror("数据发送失败"); // 发送失败时输出错误
    //     close(sockfd);          // 关闭套接字
    //     exit(EXIT_FAILURE);     // 终止程序
    // }
    if (sendto(nano_udp_st.sockfd, &nano_udp_st.nano_request, sizeof(nano_udp_st.nano_request), 0, // 发送数据结构体
               (struct sockaddr *)&nano_udp_st.server_addr,                // 目标地址
               sizeof(nano_udp_st.server_addr)) < 0)
    {                           // 地址长度
        perror("数据发送失败"); // 发送失败时输出错误
        close(nano_udp_st.sockfd);          // 关闭套接字
        exit(EXIT_FAILURE);     // 终止程序
    }

    printf("数据已发送到 192.168.2.1:49152\n"); // 打印发送成功信息
    int num = 50;
    // 循环接收反馈数据
    double fx, fy, fz, tx, ty, tz;
    while (num >= 0)
    { // 无限循环
        // 清空缓冲区
        memset(&nano_udp_st.nano_response, 0, sizeof(nano_udp_st.nano_response)); // 每次接收前清零缓冲区
        // 接收响应
        ssize_t recv_len = recvfrom(nano_udp_st.sockfd, &nano_udp_st.nano_response, sizeof(nano_udp_st.nano_response), MSG_WAITALL, // 接收数据到缓冲区
                                    (struct sockaddr *)&nano_udp_st.client_addr,                            // 存储发送方地址
                                    &nano_udp_st.addr_len);                                                 // 地址长度
        if (recv_len < 0)
        {                           // 检查接收是否出错
            perror("数据接收失败"); // 输出错误信息
            // break;
            continue;               // 继续循环，尝试再次接收
        }
        nano_udp_st.nano_response.rdt_sequence = ntohl(nano_udp_st.nano_response.rdt_sequence);
        nano_udp_st.nano_response.status = ntohl(nano_udp_st.nano_response.status);
        nano_udp_st.nano_response.Fx = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Fx);
        nano_udp_st.nano_response.Fy = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Fy);
        nano_udp_st.nano_response.Fz = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Fz);
        nano_udp_st.nano_response.Tx = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Tx);
        nano_udp_st.nano_response.Ty = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Ty);
        nano_udp_st.nano_response.Tz = (int32_t)ntohl((uint32_t)nano_udp_st.nano_response.Tz);
        // 处理接收到的数据
        printf("接收到反馈ID: %d, 状态: %d\r\n", // 打印接收到的数据
               nano_udp_st.nano_response.rdt_sequence, nano_udp_st.nano_response.status);
        nano_udp_st.fx = (double)nano_udp_st.nano_response.Fx / 1000000.0;
        nano_udp_st.fy = (double)nano_udp_st.nano_response.Fy / 1000000.0;
        nano_udp_st.fz = (double)nano_udp_st.nano_response.Fz / 1000000.0;
        nano_udp_st.tx = (double)nano_udp_st.nano_response.Tx / 1000000.0;
        nano_udp_st.ty = (double)nano_udp_st.nano_response.Ty / 1000000.0;
        nano_udp_st.tz = (double)nano_udp_st.nano_response.Tz / 1000000.0;
        printf("FX=%f,FY=%f,FZ=%f,TX=%f,TY=%f,TZ=%f\r\n",
             nano_udp_st.fx, nano_udp_st.fy, nano_udp_st.fz, nano_udp_st.tx, nano_udp_st.ty, nano_udp_st.tz);
        num--;
    }
    // 准备要发送的数据
    nano_udp_st.nano_request.command_header = htons(0x1234);
    nano_udp_st.nano_request.command = 0x0000;
    nano_udp_st.nano_request.sample_count = 0;

    // 发送数据到目标设备
    if (sendto(nano_udp_st.sockfd, &nano_udp_st.nano_request, sizeof(nano_udp_st.nano_request), 0, // 发送数据结构体
               (struct sockaddr *)&nano_udp_st.server_addr,                // 目标地址
               sizeof(nano_udp_st.server_addr)) < 0)
    {                           // 地址长度
        perror("数据发送失败"); // 发送失败时输出错误
        close(nano_udp_st.sockfd);          // 关闭套接字
        exit(EXIT_FAILURE);     // 终止程序
    }
    // 关闭套接字（由于无限循环，此处代码不会执行，除非添加break条件）
    close(nano_udp_st.sockfd); // 释放套接字资源
    return NULL;
}
// Signal handler for Ctrl+C (SIGINT)
void handle_sigint(int sig)
{
    printf("\nReceived Ctrl+C. Stopping thread...\n");
    running = false;
}

int main()
{
    pthread_t thread_id;
    int ret;

    // Set up signal handler for Ctrl+C
    struct sigaction sa;
    sa.sa_handler = handle_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);

    // Create the thread
    ret = pthread_create(&thread_id, NULL, regular_thread, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "Error creating thread: %d\n", ret);
        return 1;
    }

    // Wait for the thread to finish
    pthread_join(thread_id, NULL);

    printf("Main program exiting.\n");
    return 0;
}