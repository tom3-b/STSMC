#include "Nano.h"

NanoUDP::NanoUDP(/* args */)
{
}

NanoUDP::~NanoUDP()
{
}

int NanoUDP::init()
{
    // 创建UDP套接字
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("套接字创建失败"); // 输出错误信息
        exit(EXIT_FAILURE);       // 终止程序
    }

    // 配置目标设备地址
    memset(&server_addr, 0, sizeof(server_addr));           // 清零地址结构
    server_addr.sin_family = AF_INET;                       // 设置地址族为IPv4
    server_addr.sin_port = htons(49152);                    // 设置端口号（主机字节序转网络字节序）
    server_addr.sin_addr.s_addr = inet_addr("192.168.2.1"); // 设置IP地址
    return 0;
}

double fd_force_sfun(double sfun_time)
{
    // % 五次多项式S曲线
    // % t: 时间向量
    // % t_start: 开始时间
    // % t_end: 结束时间
    double A = -0.4;
    double t_start = 0;
    double t_end = 2;
    double y = 0;
    double t = sfun_time;
    // % 对于t < t_start的部分
    if (t <= t_start)
    {
        y = 0;
    }
    // % 对于t > t_end的部分
    if (t >= t_end)
    {
        y = 1;
    }

    // % 对于t_start <= t <= t_end的部分
    if ((t >= t_start) && (t <= t_end))
    {
        // % 归一化时间到[0,1]区间
        double tau = (t - t_start) / (t_end - t_start);
        // % 五次多项式: s(t) = 6t^5 - 15t^4 + 10t^3
        y = 6 * pow(tau, 5) - 15 * pow(tau, 4) + 10 * pow(tau, 3);
    }
    // 正弦
    double A_s = 0.06;
    double f = 1;
    double delt = 0;
    // if (t >= t_end)
    // {
    //     delt = A_s * sin(2 * M_PI * f * (t - t_end));
    // }

    y = A * y + delt;
    return y;
}

// 保存数据到文件
int saveToFile(const DataMatrix *matrix)
{
    time_t current_time;
    struct tm *time_info;
    char filename[100];
    // 获取当前时间
    current_time = time(NULL);
    time_info = localtime(&current_time);
    // 格式化时间为文件名 (例如: 2024-01-15_14-30-25.txt)
    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.txt", time_info);
    FILE *file = fopen(filename, "w");
    if (file == NULL)
    {
        printf("无法创建文件: %s\n", filename);
        return 0;
    }

    for (int i = 0; i < matrix->rows; i++)
    {
        for (int j = 0; j < matrix->cols; j++)
        {
            fprintf(file, "%.6f", matrix->data[i][j]);
            if (j < matrix->cols - 1)
            {
                fprintf(file, " ");
            }
        }
        fprintf(file, "\n");
    }

    fclose(file);
    printf("数据已保存到: %s\n", filename);
    return 1;
}