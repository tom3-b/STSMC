#include <iostream>
#include <stdint.h>
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
#include <cmath>
#pragma pack(1)
#define XDDP_PORT 0 /* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
#define XDDP_PORT_LABEL  "xddp-demo"
typedef struct
{
    double f[6];
    int flag;
} shared_buffer_t;
/*数据保存结构体*/
#define MAX_COLS 13
#define MAX_ROWS (1000 * 120)
typedef struct
{
    double data[MAX_ROWS][MAX_COLS]; // 假设最大1000行
    int rows;
    int cols;
} DataMatrix;
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })
typedef struct
{
    uint16_t command_header;
    uint16_t command;
    uint32_t sample_count;
} Nano_request_st;

typedef struct
{
    uint32_t rdt_sequence;
    uint32_t ft_sequence;
    uint32_t status;
    int32_t Fx;
    int32_t Fy;
    int32_t Fz;
    int32_t Tx;
    int32_t Ty;
    int32_t Tz;
} Nano_responce_st;

typedef struct
{
    uint8_t command;      /* Must be READCALINFO (1). */
    uint8_t reserved[19]; /* Should be all 0s. */
} Nano_TCP_request_st;

typedef struct
{
    uint16_t header;          /* always 0x1234. */
    uint8_t forceUnits;       /* Force Units. */
    uint8_t torqueUnits;      /* Torque Units. */
    uint32_t countsPerForce;  /* Calibration Counts per force unit. */
    uint32_t countsPerTorque; /* Calibration Counts per torque unit. */
    uint16_t scaleFactors[6]; /* Further scaling for 16‑bit counts. */
} Nano_TCP_response_st;

typedef struct
{
    uint8_t command;      /* Must be READFT (0) . */
    uint8_t reserved[15]; /* Should be all 0s. */
    uint16_t MCEnable;    /* Bitmap of MCs to enable. */
    uint16_t sysCommands; /* Bitmap of system commands. */
} Nano_TCP_ft_request_st;

typedef struct
{
    uint16_t header; /* always 0x1234. */
    uint16_t status; /* Upper 16 bits of status code. */
    int16_t ForceX;  /* 16‑bit Force X counts. */
    int16_t ForceY;  /* 16‑bit Force Y counts. */
    int16_t ForceZ;  /* 16‑bit Force Z counts. */
    int16_t TorqueX; /* 16‑bit Torque X counts. */
    int16_t TorqueY; /* 16‑bit Torque Y counts. */
    int16_t TorqueZ; /* 16‑bit Torque Z counts. */
} Nano_TCP_ft_response_st;

class NanoUDP
{
public:
    NanoUDP(/* args */);
    ~NanoUDP();
    int init();
    /************变量定义*************************/
    Nano_request_st nano_request;                // 待发送的数据结构体
    Nano_responce_st nano_response;              // struct Nano_responce_st nano_response;// 接收的数据结构体
    int sockfd;                                  // 套接字文件描述符
    struct sockaddr_in server_addr, client_addr; // 服务器和客户端地址结构
    socklen_t addr_len = sizeof(client_addr);    // 地址结构长度
    double fx, fy, fz, tx, ty, tz;               // 力传感器读数
};


double fd_force_sfun(double sfun_time);
int saveToFile(const DataMatrix *matrix);