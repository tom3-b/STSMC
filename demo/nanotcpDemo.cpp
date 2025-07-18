#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "Nano.h"

int main()
{
    int sockfd;
    struct sockaddr_in server_addr;
    Nano_TCP_request_st req_data;
    Nano_TCP_response_st resp_data;
    Nano_TCP_ft_request_st req_ft;
    Nano_TCP_ft_response_st resp_ft;
    // 创建socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("Socket creation failed");
        return -1;
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(49151);
    server_addr.sin_addr.s_addr = inet_addr("192.168.2.1");

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Connection failed");
        close(sockfd);
        return -1;
    }

    printf("Connected to server 192.168.2.1:49151\n");

    // 准备要发送的数据
    memset(&req_data, 0, sizeof(req_data));
    req_data.command = 1;

    // 发送结构体数据
    ssize_t bytes_sent = send(sockfd, &req_data, sizeof(Nano_TCP_request_st), 0);
    if (bytes_sent < 0)
    {
        perror("Send failed");
        close(sockfd);
        return -1;
    }
    printf("Sent %zd bytes to server\n", bytes_sent);
    memset(&resp_data, 0, sizeof(resp_data));
    // 接收服务器响应
    ssize_t bytes_received = recv(sockfd, &resp_data, sizeof(Nano_TCP_response_st), 0);
    if (bytes_received < 0)
    {
        perror("Receive failed");
        close(sockfd);
        return -1;
    }
    resp_data.countsPerForce = htonl(resp_data.countsPerForce);
    resp_data.countsPerTorque = htonl(resp_data.countsPerTorque);
    resp_data.scaleFactors[0] = htons(resp_data.scaleFactors[0]);
    resp_data.scaleFactors[1] = htons(resp_data.scaleFactors[1]);
    resp_data.scaleFactors[2] = htons(resp_data.scaleFactors[2]);
    resp_data.scaleFactors[3] = htons(resp_data.scaleFactors[3]);
    resp_data.scaleFactors[4] = htons(resp_data.scaleFactors[4]);
    resp_data.scaleFactors[5] = htons(resp_data.scaleFactors[5]);
    printf("Received %zd bytes from server\n", bytes_received);
    printf("标定数据%d,%d;\r\nscaled factor:%d,%d,%d,%d,%d,%d\r\n ", resp_data.countsPerForce, resp_data.countsPerTorque, resp_data.scaleFactors[0],
           resp_data.scaleFactors[1],
           resp_data.scaleFactors[2],
           resp_data.scaleFactors[3],
           resp_data.scaleFactors[4],
           resp_data.scaleFactors[5]);
    printf("unit %d,%d\r\n", resp_data.forceUnits, resp_data.torqueUnits);
    // 准备要发送的数据
    memset(&req_ft, 0, sizeof(req_ft));
    int num = 100;
    while (1)
    {
        /* code */
        // 发送结构体数据
        bytes_sent = send(sockfd, &req_ft, sizeof(req_ft), 0);
        if (bytes_sent < 0)
        {
            perror("Send failed");
            close(sockfd);
            return -1;
        }
        // printf("Sent %zd bytes to server\n", bytes_sent);
        memset(&resp_ft, 0, sizeof(resp_ft));
        // 接收服务器响应
        bytes_received = recv(sockfd, &resp_ft, sizeof(resp_ft), 0);
        if (bytes_received < 0)
        {
            perror("Receive failed");
            close(sockfd);
            return -1;
        }
        resp_ft.ForceX = htons(resp_ft.ForceX);
        resp_ft.ForceY = htons(resp_ft.ForceY);
        if (num == 0)
        {
            printf("forcex= %f,forcey=%f\r\n", resp_ft.ForceX *519/1000000.0,
                   resp_ft.ForceX *519/1000000.0);

            num = 1000;
        }

        num--;
    }
    // 关闭socket
    close(sockfd);
    printf("Connection closed\n");
    return 0;
}
