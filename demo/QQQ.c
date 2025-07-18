RTIME time = rt_get_time_ns();
    if (system_time_base > time) {
        rt_printk("%s() error: system_time_base greater than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);
        return time;
    }
    else {
        return time - system_time_base;
    }

RTIME system2count(
        uint64_t time
        )
{
    RTIME ret;

    if ((system_time_base < 0) &&
            ((uint64_t) (-system_time_base) > time)) {
        rt_printk("%s() error: system_time_base less than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);
        ret = time;
    }
    else {
        ret = time + system_time_base;
    }

    return nano2count(ret);
}

int main(int argc, char *argv[])
{
    
    dc_start_time_ns = system_time_ns();//1:0 
    dc_time_ns = dc_start_time_ns;//1.:0

    int cycle_counter = 0;//1:0

    // set first wake time in a few cycles
    wakeup_time = system_time_ns() + 10 * cycle_ns;//1:10ms

    while (run) {
        // wait for next period (using adjustable system time)
    RTIME wakeup_count = system2count(wakeup_time);//1:10ms  //2:11ms //3:12ms
    RTIME current_count = rt_get_time();//1:0 //2:11ms  //3:12ms
    rt_sleep_until(wakeup_count)
    // set master time in nano-seconds
    ecrt_master_application_time(master, wakeup_time);
    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;//1:11ms  //2:12ms  //3:13ms
    cycle_counter++;
        // sync distributed clock just before master_send to set
        // most accurate master clock time
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;//1:0  //2:10ms //3:11ms
    dc_time_ns = system_time_ns();//1:10ms  //2:11ms  //3:12ms
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);//1:ref1  2:ref2   //3:ref3
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;//1:0-ref1  //2:10ms-ref2   //3:11ms-ref3
    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master);
    // send EtherCAT data
    ecrt_master_send(master);

        // update the master clock
        // Note: called after ecrt_master_send() to reduce time
        // jitter in the sync_distributed_clocks() call
    // calc drift (via un-normalised time diff)
    int32_t delta = dc_diff_ns - prev_dc_diff_ns;//1:0-ref1  //2:10ms-ref2+ref1  //3:1ms-ref3+ref2
    prev_dc_diff_ns = dc_diff_ns;//1:0-ref1  //2:10ms-ref2 //3:11ms-ref3
    // normalise the time diff
    dc_diff_ns =((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);
    // only update if primary master
    if (dc_started) {
        // add to totals
        dc_diff_total_ns += dc_diff_ns;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT) {
            // add rounded delta average
            dc_adjust_ns +=
                ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
            if (dc_adjust_ns < -1000) {
                dc_adjust_ns = -1000;
            }
            if (dc_adjust_ns > 1000) {
                dc_adjust_ns =  1000;
            }

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sign(dc_diff_ns);//1:1us  //2:2us
    }
    else {
        dc_started = (dc_diff_ns != 0);//1:1

        if (dc_started) {
            // output first diff
            rt_printk("First master diff: %d.\n", dc_diff_ns);

            // record the time of this initial cycle
            dc_start_time_ns = dc_time_ns;//1:10ms
        }
    }

    }

    
}