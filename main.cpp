/*****************************************************************************
 *
 *  Copyright (C)      2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh.de>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************/
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
#include <rtdm/ipc.h>
#ifndef XENOMAI_API_V3
#include <rtdm/rtdm.h>
#include <rtdk.h>
#endif
#include "ecrt.h"
#include "slave.h"
#include "Nano.h"

#pragma pack(1)

DataMatrix data_recoder;
/*多线程共享数据结构*/

int s;
std::once_flag smc_init_flag;
int xddp_create_flag = 0;

shared_buffer_t *shared_buf;
/*Nano udp 通信*/
static int sockfd;
struct sockaddr_in server_addr, client_addr; // 服务器和客户端地址结构
socklen_t addr_len = sizeof(client_addr);    // 地址结构长度
static Nano_request_st nano_request;         // 待发送的数据结构体
static Nano_responce_st nano_response;       // struct Nano_responce_st nano_response;// 接收的数据结构体
/**/
// #define rt_printf(X, Y)
#define NSEC_PER_SEC 1000000000
typedef long long RTIME;

static pthread_t cyclic_thread;
static pthread_t nano_thread;
static unsigned int cycle_ns = 1000000; /* 1 ms */
static volatile int run = 1;

Motor_var voice_mot_var;
int smc_enable = 0;
/****************************************自适应控制算法*************************************/
double alefa = 60;
double gama = 0.2;
double bta = 0.0001;
double md = 10, bd = 1000;
double d = 0;
double f = 0, df = 0, f_prev = 0;
double u=0;
int cycle_counter = 0;
int counter = 0;
double smc_time = 0;
int32_t x_init = 0;
double fd = 0;
double fe = 0;
/*x dx*/
double x = 0;
double dx = 0;
double ddx = 0;


/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;

/****************************************************************************/

// EtherCAT distributed clock variables

#define DC_FILTER_CNT 1024
#define SYNC_MASTER_TO_REF 1
static uint64_t dc_start_time_ns = 0LL;
static uint64_t dc_time_ns = 0;
#if SYNC_MASTER_TO_REF
static uint8_t dc_started = 0;
static int32_t dc_diff_ns = 0;
static int32_t prev_dc_diff_ns = 0;
static int64_t dc_diff_total_ns = 0LL;
static int64_t dc_delta_total_ns = 0LL;
static int dc_filter_idx = 0;
static int64_t dc_adjust_ns;
#endif
static int64_t system_time_base = 0LL;
static uint64_t wakeup_time = 0LL;
static uint64_t overruns = 0LL;

/****************************************************************************/

// process data

#define Elmo_Pos 0, 0
#define Elmo_GOLDEN 0x0000009a, 0x00030924

pdo_entry elmo_pdo_entry;
// process data
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {Elmo_Pos, Elmo_GOLDEN, 0x60b0, 0x00, &elmo_pdo_entry.position_offset, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x607a, 0x00, &elmo_pdo_entry.tar_position, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6040, 0x00, &elmo_pdo_entry.control_wd, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6060, 0x00, &elmo_pdo_entry.mode_of_operation_6060, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6064, 0x00, &elmo_pdo_entry.act_position, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x606c, 0x00, &elmo_pdo_entry.act_volecty, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6077, 0x00, &elmo_pdo_entry.act_torque, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6041, 0x00, &elmo_pdo_entry.state, NULL},
    {Elmo_Pos, Elmo_GOLDEN, 0x6061, 0x00, &elmo_pdo_entry.mode_of_operation_6061, NULL},
    {}};

/****************************************************************************/

/* Slave 1, "EL2004"
 * Vendor ID:       0x00000002
 * Product code:    0x07d43052
 * Revision number: 0x00100000
 */
ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    /* RxPdo 0x1607 */
    {0x607a, 0x00, 32},
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x60b0, 0x00, 32},

    /* TxPDO 0x1a07 */
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1607, 4, slave_1_pdo_entries + 0},
    {0x1a07, 5, slave_1_pdo_entries + 4},
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_DEFAULT},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DEFAULT},
    {0xFF}};
/*30/(s+0.2008)*ef*/
// 使用状态空间方法计算系统响应（更高效的实现）
std::vector<double> calculateResponseStateSpace(const std::vector<double> &ef_t)
{
    // 系统参数
    const double A = 30.0;       // 增益
    const double alpha = 0.2008; // 系统极点
    const double dt = 0.001;     // 采样时间间隔
    int n = ef_t.size();
    std::vector<double> output(n, 0);

    // 初始状态
    double x = 0.0;

    // 使用状态空间方法计算输出
    for (int i = 0; i < n; i++)
    {
        // 更新状态
        x = x * std::exp(-alpha * dt) + A * (1.0 - std::exp(-alpha * dt)) * ef_t[i];
        output[i] = x;
    }

    return output;
}
/*滑膜控制函数
 */
void SMC_CONTROL()
{
    // 任务循环
    /*fd*/
    fd = fd_force_sfun(smc_time);
    /*fe*/
    fe = shared_buf->f[2]; // fz
    f = fd - fe;
    df = (f - f_prev) / 0.001;
    d = d + bta * (-u + f - d) * 0.001;
    /* u
    function y = sfun(d,f,df)
    alefa=80;
    gama=0.2;
    y = -d+1.0*f-alefa*f-gama*df;
    end
    */
    u = -d + 1.0 * f - alefa * f - gama * df;
    ddx = (u - f - bd * dx) / md;
    dx = dx + ddx * 0.001;
    x = x + dx * 0.001;
    f_prev = f;
    smc_time += 0.001;
}
/*****************************************************************************
 * Realtime task
 ****************************************************************************/

/** Get the time in ns for the current cpu, adjusted by system_time_base.
 *
 * \attention Rather than calling rt_get_time_ns() directly, all application
 * time calls should use this method instead.
 *
 * \ret The time in ns.
 */
uint64_t system_time_ns(void)
{
    // RTIME time = rt_get_time_ns();
    struct timespec time_st;
    RTIME time;
    clock_gettime(CLOCK_MONOTONIC, &time_st);
    time = time_st.tv_sec * NSEC_PER_SEC + time_st.tv_nsec;
    if (system_time_base > time)
    {
        rt_printf("%s() error: system_time_base greater than"
                  " system time (system_time_base: %lld, time: %llu\n",
                  __func__, system_time_base, time);
        return time;
    }
    else
    {
        return time - system_time_base;
    }
}

/****************************************************************************/

/** Convert system time to RTAI time in counts (via the system_time_base).
 */
RTIME system2count(
    uint64_t time)
{
    RTIME ret;

    if ((system_time_base < 0) &&
        ((uint64_t)(-system_time_base) > time))
    {
        rt_printf("%s() error: system_time_base less than"
                  " system time (system_time_base: %lld, time: %llu\n",
                  __func__, system_time_base, time);
        ret = time;
    }
    else
    {
        ret = time + system_time_base;
    }
    return ret;
    // return nano2count(ret);
}

/****************************************************************************/

/** Synchronise the distributed clocks
 */
void sync_distributed_clocks(void)
{
#if SYNC_MASTER_TO_REF
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;
#endif

    dc_time_ns = system_time_ns();

#if SYNC_MASTER_TO_REF
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);
    dc_diff_ns = (uint32_t)prev_app_time - ref_time;
#else
    // sync reference clock to master
    ecrt_master_sync_reference_clock_to(master, dc_time_ns);
#endif

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master);
}

/****************************************************************************/

/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

/****************************************************************************/

/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void update_master_clock(void)
{
#if SYNC_MASTER_TO_REF
    // calc drift (via un-normalised time diff)
    int32_t delta = dc_diff_ns - prev_dc_diff_ns;
    prev_dc_diff_ns = dc_diff_ns;

    // normalise the time diff
    dc_diff_ns =
        ((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);

    // only update if primary master
    if (dc_started)
    {

        // add to totals
        dc_diff_total_ns += dc_diff_ns;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT)
        {
            // add rounded delta average
            dc_adjust_ns +=
                ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
            if (dc_adjust_ns < -1000)
            {
                dc_adjust_ns = -1000;
            }
            if (dc_adjust_ns > 1000)
            {
                dc_adjust_ns = 1000;
            }

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sign(dc_diff_ns);
    }
    else
    {
        dc_started = (dc_diff_ns != 0);

        if (dc_started)
        {
            // output first diff
            rt_printf("First master diff: %d.\n", dc_diff_ns);

            // record the time of this initial cycle
            dc_start_time_ns = dc_time_ns;
        }
    }
#endif
}

/****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
    {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state)
    {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states)
    {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up)
    {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

/** Wait for the next period
 */
void wait_period(void)
{
    while (1)
    {
        RTIME wakeup_count = system2count(wakeup_time);
        // RTIME current_count = rt_get_time();
        struct timespec time;
        clock_gettime(CLOCK_MONOTONIC, &time);
        RTIME current_count = time.tv_nsec + time.tv_sec * NSEC_PER_SEC;
        if ((wakeup_count < current_count) || (wakeup_count > current_count + (50 * cycle_ns)))
        {
            rt_printf("%s(): unexpected wake time!wakeupcount=%lld,currentcount=%lld\n", __func__, wakeup_count, current_count);
        }
        time.tv_nsec = wakeup_count;
        time.tv_sec = 0;
        while (time.tv_nsec >= NSEC_PER_SEC)
        {
            time.tv_nsec -= NSEC_PER_SEC;
            time.tv_sec++;
        }

        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, NULL);
        if (ret != 0)
        {
            if (ret == EINTR)
            {
                rt_printf("%s break\r\n", __func__);
            }
            else
            {
                // 其他错误
                rt_printf("clock_nanosleep other error\r\n");
            }
        }

        // done if we got to here
        break;
    }

    // set master time in nano-seconds
    ecrt_master_application_time(master, wakeup_time);
    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;
}

/****************************************************************************/
void cycle_cmd()
{
    memset(&voice_mot_var, 0, sizeof(voice_mot_var));
    voice_mot_var.state = EC_READ_U16(domain1_pd + elmo_pdo_entry.state);
    voice_mot_var.act_position = EC_READ_S32(domain1_pd + elmo_pdo_entry.act_position);
    voice_mot_var.act_volecty = EC_READ_S32(domain1_pd + elmo_pdo_entry.act_volecty);
    voice_mot_var.act_torque = EC_READ_S16(domain1_pd + elmo_pdo_entry.act_torque);
    voice_mot_var.mode_of_operation_6061 = EC_READ_S8(domain1_pd + elmo_pdo_entry.mode_of_operation_6061);
    voice_mot_var.position_offset = EC_READ_S32(domain1_pd + elmo_pdo_entry.position_offset);

    data_recoder.data[counter][0] = (double)voice_mot_var.act_position;
    data_recoder.data[counter][1] = (double)voice_mot_var.act_volecty;
    data_recoder.data[counter][2] = (double)voice_mot_var.act_torque;
    data_recoder.data[counter][3] = (double)voice_mot_var.state;
    data_recoder.data[counter][4] = u;
    data_recoder.data[counter][5] = x;
    data_recoder.data[counter][6] = dx;
    data_recoder.data[counter][7] = ddx;
    data_recoder.data[counter][8] = fd;
    data_recoder.data[counter][9] = fe;
    data_recoder.data[counter][10] = d;
    counter++;
    /*使能电机*/
    if (voice_mot_var.state == 0x0250)
    {
        EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd, 0x06);
    }
    if (voice_mot_var.state == 0x0231)
    {
        EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd, 0x07);
    }
    if (voice_mot_var.state == 0x0233)
    {
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.act_position, voice_mot_var.act_position);
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.position_offset, 0);
        EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd, 0x010f);
    }
    if ((voice_mot_var.state & 0x00FF) == 0x0018)
    {
        EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd, 0b0000000010000000);
    }
    /*读取非实时线程nano传感器数据*/
    // 使用MSG_DONTWAIT标志进行非阻塞读取
    /* Get packets relayed by the regular thread */
    int ret = recvfrom(s, shared_buf, sizeof(shared_buffer_t), MSG_DONTWAIT, NULL, 0);

    if ((cycle_counter % 1000) == 0)
    {
        if (ret > 0)
        {
            rt_printf(" fz = %f\r\n", shared_buf->f[2]);
            rt_printf("flag %d", shared_buf->flag);
        }
    }

    smc_enable = (shared_buf->flag == 1) && (cycle_counter >= 8000);

    // if ((voice_mot_var.state & 0xFF) == 0x37 && (cycle_counter >= 4000))
    // {

    //     std::call_once(smc_init_flag, []()
    //                    {
    //     x_init = voice_mot_var.act_position;
    //     smc_time = 0;
    //     EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd,0x0F); });
    //     EC_WRITE_S32(domain1_pd + elmo_pdo_entry.tar_position, x_init);
    //     EC_WRITE_S32(domain1_pd + elmo_pdo_entry.position_offset, 0);
    // }
    // if ((voice_mot_var.state & 0xFF) == 0x37 && (cycle_counter >= 6000))
    // {
    //     u = 0.2 * sin(f * 2 * M_PI * smc_time);
    //     u = u * 100000;
    //     u_p = (int32_t)u;
    //     EC_WRITE_S32(domain1_pd + elmo_pdo_entry.position_offset, u_p);
    //     EC_WRITE_S32(domain1_pd + elmo_pdo_entry.tar_position, x_init);
    //     smc_time += 0.001;
    // }
    if (!smc_enable && ((voice_mot_var.state & 0x00FF) == 0x37)) // enable op and quick stop active
    {
        // 初始化算法时间和当前位置
        // 使能后执行一次
        std::call_once(smc_init_flag, []()
                       {
        x_init = voice_mot_var.act_position;
        smc_time = 0; });
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.tar_position, x_init);
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.position_offset, 0);
        EC_WRITE_U16(domain1_pd + elmo_pdo_entry.control_wd, 0x0F);
    }
    if (smc_enable && ((voice_mot_var.state & 0x00FF) == 0x37))
    {
        SMC_CONTROL();
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.position_offset, (int32_t)(x * 1000 * 100000));
        EC_WRITE_S32(domain1_pd + elmo_pdo_entry.tar_position, x_init);
    }
}
/****************************************************************************/
void *my_cyclic(void *arg)
{
    /*xddp*/
    rt_printf("in cycle \r\n");
    struct rtipc_port_label plabel;
    struct sockaddr_ipc saddr;
    int ret;

    /*
     * Get a datagram socket to bind to the RT endpoint. Each
     * endpoint is represented by a port number within the XDDP
     * protocol namespace.
     */
    s = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    if (s < 0)
    {
        rt_printf("socket set fail\r\n");
        return NULL;
    }

    /*
     * Set a port label. This name will be registered when
     * binding, in addition to the port number (if given).
     */
    strcpy(plabel.label, XDDP_PORT_LABEL);
    ret = setsockopt(s, SOL_XDDP, XDDP_LABEL,
                     &plabel, sizeof(plabel));
    if (ret)
    {
        rt_printf("socket set label fail\r\n");
        return NULL;
    }

    /*
     * Bind the socket to the port, to setup a proxy to channel
     * traffic to/from the Linux domain. Assign that port a label,
     * so that peers may use a descriptive information to locate
     * it. For instance, the pseudo-device matching our RT
     * endpoint will appear as
     * /proc/xenomai/registry/rtipc/xddp/<XDDP_PORT_LABEL> in the
     * Linux domain, once the socket is bound.
     *
     * saddr.sipc_port specifies the port number to use. If -1 is
     * passed, the XDDP driver will auto-select an idle port.
     */
    memset(&saddr, 0, sizeof(saddr));
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = -1;
    ret = bind(s, (struct sockaddr *)&saddr, sizeof(saddr));
    if (ret)
    {
        rt_printf("socket bind fail\r\n");
        return NULL;
    }
    /*xddp end*/
    rt_printf("start cycle\r\n");
    wakeup_time = system_time_ns() + 10 * cycle_ns;
    struct timespec time_end;
    int64_t time_end_us;
    while (run)
    {
        // wait for next period (using adjustable system time)
        wait_period();
        cycle_counter++;
        if (!run)
        {
            break;
        }
        // receive EtherCAT
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        rt_check_domain_state();
        if (!(cycle_counter % 1000))
        {
            rt_check_master_state();
        }
        /*主控制任务*/
        cycle_cmd();
        // queue process data
        ecrt_domain_queue(domain1);

        // sync distributed clock just before master_send to set
        // most accurate master clock time
        sync_distributed_clocks();

        // send EtherCAT data
        ecrt_master_send(master);

        // update the master clock
        // Note: called after ecrt_master_send() to reduce time
        // jitter in the sync_distributed_clocks() call
        update_master_clock();
        // 运行时间测试
        if (!(cycle_counter % 1000))
        {
            clock_gettime(CLOCK_MONOTONIC, &time_end);
            time_end_us = wakeup_time - time_end.tv_sec * NSEC_PER_SEC - time_end.tv_nsec + system_time_base;
            rt_printf("time diff %d us\r\n", time_end_us / 1000);
        }

        /*
         * Send a datagram to the NRT endpoint via the proxy.
         * We may pass a NULL destination address, since a
         * bound socket is assigned a default destination
         * address matching the binding address (unless
         * connect(2) was issued before bind(2), in which case
         * the former would prevail).
         */

        // if (ret != 0)
        // rt_printf("sendto num error\r\n");
    }
    close(s);
    return NULL;
}

/*****************************************************************************
 * Signal handler
 ****************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/*****************************************************************************
 * Main function
 ****************************************************************************/

int main(int argc, char *argv[])
{
    shared_buf = (shared_buffer_t *)malloc(sizeof(shared_buffer_t));
    if (shared_buf == NULL)
    {
        perror("malloc failed");
        exit(1);
    }
    // 可选：初始化内容
    memset(shared_buf, 0, sizeof(shared_buffer_t));

    /*数据保存结构体初始化*/
    memset(&data_recoder, 0, sizeof(data_recoder));
    data_recoder.cols = MAX_COLS;
    data_recoder.rows = MAX_ROWS;
    int ret;

    ec_slave_config_t *sc;

    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        return -1;
    }

    printf("Creating slave configurations...\n");

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, Elmo_Pos, Elmo_GOLDEN);
    if (!sc)
    {
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, slave_1_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    ret = ecrt_slave_config_sdo8(sc, 0x6060, 0, 8); // CSP模式
    if (ret < 0)
    {
        rt_printf("CST 模式 set fail\r\n");
    }
    ret = ecrt_slave_config_sdo16(sc, 0x1c32, 1, 2);
    if (ret < 0)
    {
        rt_printf("0x1C32 set fail\r\n");
    }
    ret = ecrt_slave_config_sdo16(sc, 0x1c33, 1, 2);
    if (ret < 0)
    {
        rt_printf("0x1C33 set fail\r\n");
    }
    ret = ecrt_slave_config_dc(sc, 0x0300, cycle_ns, cycle_ns / 4, 0, 0); // 1ms
    if (ret < 0)
    {
        rt_printf("0x0300 set fail\r\n");
    }
    // ret = ecrt_slave_config_sdo16(sc, 0x6072, 0, 6000);
    // if (ret < 0)
    // {
    //     rt_printf("max torque set fail\r\n");
    // }
    // ret = ecrt_slave_config_sdo32(sc, 0x6080, 0, 100000 * 100);
    // if (ret < 0)
    // {
    //     rt_printf("max speed set fail\r\n");
    // }
    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock.
     */
    dc_start_time_ns = system_time_ns();
    dc_time_ns = dc_start_time_ns;

    ret = ecrt_master_select_reference_clock(master, sc);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }
    rt_printf("creat thread\r\n");
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    /* Create cyclic RT-thread */
    struct sched_param param = {.sched_priority = 90};
    pthread_attr_t rtattr;
    pthread_attr_init(&rtattr);
    pthread_attr_setdetachstate(&rtattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&rtattr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&rtattr, SCHED_FIFO);
    pthread_attr_setschedparam(&rtattr, &param);
    ret = pthread_create(&cyclic_thread, &rtattr, &my_cyclic, NULL);
    if (ret)
    {
        fprintf(stderr, "%s: pthread_create cyclic task failed\n",
                strerror(-ret));
        return 1;
    }
    // 主线程等待特定信号
    pthread_join(cyclic_thread, NULL);
    printf("End of Program\n");
    ecrt_release_master(master);
    saveToFile(&data_recoder);
    return 0;
}

/****************************************************************************/