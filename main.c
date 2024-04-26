/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C)      2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
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
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include <errno.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include "main.h"
#include <getopt.h>

static pthread_t cyclic_thread;
static volatile int run = 1;
static volatile int close_signal = 0;

static ec_master_t *master = NULL;
ec_domain_t *domain = NULL;
ec_domain_state_t domain_state;
uint8_t *domain_pd = NULL;
static Slave_Data ec_slave0;
Servo_Run_Mode_t servo_mode = mode_CSP;

const static ec_pdo_entry_reg_t domain0_regs[] = {
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6040, 0x00, &ec_slave0.ctrl_word, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6060, 0x00, &ec_slave0.mode_sel, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x607a, 0x00, &ec_slave0.tar_pos, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x60FF, 0x00, &ec_slave0.tar_vel, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6071, 0x00, &ec_slave0.tar_torque, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x607F, 0x00, &ec_slave0.tar_max_profile_vel, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6041, 0x00, &ec_slave0.status_word, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6061, 0x00, &ec_slave0.mode_display, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6064, 0x00, &ec_slave0.pos_act, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x606c, 0x00, &ec_slave0.vel_act, NULL},
    {SOLO_Slave00_Pos, SOLO_SLAVE00_ID, 0x6077, 0x00, &ec_slave0.torque_act, NULL},
    {}
};

static void data_init(void)
{
    memset(&ec_slave0,0,sizeof(Slave_Data));
    ec_slave0.sc = NULL;
}

static void ec_slave_config_init(ec_slave_config_t *sc)
{
    /* Clear RxPDO */
    ecrt_slave_config_sdo8(sc, 0x1C12, 0, 0); /*clear sm pdo 0x1c12*/
    ecrt_slave_config_sdo8(sc, 0x1600, 0, 0); /*clear sm pdo 0x1600*/
    /* Define RxPDO */
    ecrt_slave_config_sdo32(sc, 0x1600, 1, 0x60400010); /*0x6040:0/16bits, control word*/
    ecrt_slave_config_sdo32(sc, 0x1600, 2, 0x60600008); /*0x6060:0/8bits, modes of operation*/
    ecrt_slave_config_sdo32(sc, 0x1600, 3, 0x607A0020); /*0x607A:0/32bits, Target postion*/
    ecrt_slave_config_sdo32(sc, 0x1600, 4, 0x60FF0020); /*0x60FF:0/32bits, Target velocity*/
    ecrt_slave_config_sdo32(sc, 0x1600, 5, 0x60710010); /*0x6071:0/32bits, Target torque*/
    ecrt_slave_config_sdo32(sc, 0x1600, 6, 0x607F0020); /*0x607F:0/32bits, max profile velocity*/
    ecrt_slave_config_sdo8(sc, 0x1600, 0, 6); /*set number of RxPDO*/
    ecrt_slave_config_sdo16(sc, 0x1C12, 1, 0x1600 ); /*list all RxPdo in 0x1600*/
    ecrt_slave_config_sdo8(sc, 0x1C12, 0, 1 ); /*set number of RxPDO*/
    /* Clear TxPDO */
    ecrt_slave_config_sdo8(sc, 0x1C13, 0, 0); /*clear sm pdo 0x1c12*/
    ecrt_slave_config_sdo8(sc, 0x1A00, 0, 0); /*clear sm pdo 0x1A00*/
    /* Define TxPDO */
    ecrt_slave_config_sdo32(sc, 0x1A00, 1, 0x60410010); /*0x6041:0/16bits, status word*/
    ecrt_slave_config_sdo32(sc, 0x1A00, 2, 0x60610008); /*0x6061:0/8bits, modes of operation display*/
    ecrt_slave_config_sdo32(sc, 0x1A00, 3, 0x60640020); /*0x6064:0/32bits, Position actual value*/
    ecrt_slave_config_sdo32(sc, 0x1A00, 4, 0x606C0020); /*0x606C:0/32bits, velocity actual value*/
    ecrt_slave_config_sdo32(sc, 0x1A00, 5, 0x60770010); /*0x606C:0/32bits, torque actual value*/
    ecrt_slave_config_sdo8(sc, 0x1A00, 0, 5); /*set number of TxPDO*/
    ecrt_slave_config_sdo16(sc, 0x1C13, 1, 0x1A00 ); /*list all TxPdo in 0x1C13*/
    ecrt_slave_config_sdo8(sc, 0x1C13, 0, 1 ); /*set number of TxPDO*/
}

/*****************************************************************************
 * Realtime task
 ****************************************************************************/
static void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain, &ds);
    if (ds.working_counter != domain_state.working_counter)
        printf("Domain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state.wc_state)
        printf("Domain: State %u.\n", ds.wc_state);
    domain_state = ds;
}

static void rt_check_master_state(void)
{
	static ec_master_state_t master_state = {};
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

static void ec_readmotordata(Ctl_Data* ec_motor,Slave_Data ec_slave)
{
    ec_motor->statusWord   = EC_READ_U16(domain_pd + ec_slave.status_word);
    ec_motor->actualMode   = EC_READ_S8(domain_pd + ec_slave.mode_display);
    ec_motor->actualPos    = EC_READ_S32(domain_pd + ec_slave.pos_act);
    ec_motor->actualVel	   = EC_READ_S32(domain_pd + ec_slave.vel_act);
    ec_motor->actualTorque = EC_READ_S16(domain_pd + ec_slave.torque_act);

    if ((ec_motor->statusWord & 0x4f) == 0x40)
        ec_motor->controlWord = 0x6;
    else if ((ec_motor->statusWord & 0x6f) == 0x21){
        ec_motor->controlWord = 0x7;
        ec_motor->targetPos = ec_motor->actualPos;
    }else if ((ec_motor->statusWord & 0x27f)==0x233){
        ec_motor->controlWord = 0xf;
    }
    else if ((ec_motor->statusWord & 0x27f)==0x237)
        ec_motor->controlWord = 0x1f;
    else
        ec_motor->controlWord = 0x80;
}

static void ec_writemotordata(Ctl_Data* ec_motor,Slave_Data ec_slave)
{
    switch(servo_mode)
    {
        case mode_CSP:
            EC_WRITE_S32(domain_pd+ec_slave.tar_pos, ec_motor->targetPos);
            EC_WRITE_U32(domain_pd+ec_slave.tar_max_profile_vel, ec_motor->targetMaxProfileVel);
            EC_WRITE_U8(domain_pd+ec_slave.mode_sel, 0x08);
        break;
        case mode_CSV:
            EC_WRITE_S32(domain_pd+ec_slave.tar_vel, ec_motor->targetVel);
            EC_WRITE_U32(domain_pd+ec_slave.tar_max_profile_vel, ec_motor->targetMaxProfileVel);
            EC_WRITE_U8(domain_pd+ec_slave.mode_sel, 0x09);
        break;
        case mode_CST:
            EC_WRITE_S16(domain_pd+ec_slave.tar_torque, ec_motor->targetTorque);
            EC_WRITE_U32(domain_pd+ec_slave.tar_max_profile_vel, ec_motor->targetMaxProfileVel);
            EC_WRITE_U8(domain_pd+ec_slave.mode_sel, 0x0A);
        break;
    }
    EC_WRITE_U16(domain_pd+ec_slave.ctrl_word, ec_motor->controlWord );
}

void *my_thread(void *arg)
{
    struct timespec next_period;
    unsigned int cycle_counter = 0;
    struct timespec dc_period;
    Ctl_Data ec_motor0;

#ifdef MEASURE_TIMING
    unsigned int last_cycle_counter = 0;
	struct timespec startTime = {0, 0};
	struct timespec endTime = {0, 0};
	struct timespec	lastStartTime ={0, 0};
	struct timespec rece_startTime = {0, 0};
	struct timespec rece_endTime = {0, 0};
	struct timespec pro_endTime = {0, 0};
	struct timespec send_startTime = {0, 0};
	struct timespec send_endTime = {0, 0};

    int64_t period_ns = 0, exec_ns = 0,
            period_min_ns = 1000000, period_max_ns = 0,
            exec_min_ns = 1000000, exec_max_ns = 0;
    int64_t latency_ns = 0;
    int64_t latency_min_ns = 1000000, latency_max_ns = -1000000;
    int64_t total_exec_ns=0;
    int64_t total_cycle_ns=0;
    int64_t avg_cycle_time = 0;
    int64_t min_cycle_time = 1000000;
    int64_t max_cycle_time = -1000000;
    int64_t min_jitter_time = 1000000;
    int64_t max_jitter_time = -1000000;
#endif

    struct sched_param param = {.sched_priority = 99};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    memset(&ec_motor0,0,sizeof(Ctl_Data));
	clock_gettime(CLOCK_MONOTONIC, &next_period);

#ifdef MEASURE_TIMING
	lastStartTime = next_period;
	endTime = next_period;
#endif

    while ((run != 0 )||(close_signal != 0)) {
        next_period.tv_nsec += CYCLE_US * 1000;
        while (next_period.tv_nsec >= NSEC_PER_SEC) {
			next_period.tv_nsec -= NSEC_PER_SEC;
			next_period.tv_sec++;
	    }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        latency_ns = DIFF_NS(next_period, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) 
            latency_max_ns = latency_ns;
        if (latency_ns < latency_min_ns)
            latency_min_ns = latency_ns;
        if (period_ns > period_max_ns)
            period_max_ns = period_ns;
        if (period_ns < period_min_ns)
            period_min_ns = period_ns;
        if (exec_ns > exec_max_ns)
            exec_max_ns = exec_ns;
        if (exec_ns < exec_min_ns)
            exec_min_ns = exec_ns;
        total_exec_ns += exec_ns;
#endif
        cycle_counter++;
	    if(close_signal > 0)
		    close_signal--;
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &rece_startTime);
#endif
        ecrt_master_receive(master);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &rece_endTime);
#endif
        ecrt_domain_process(domain);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &pro_endTime);
#endif
        if ((!(cycle_counter % CYCLE_COUNTER_PERSEC))&&(close_signal==0)) 
        {
	        rt_check_master_state();
            if(min_cycle_time > exec_min_ns)
                min_cycle_time = exec_min_ns;
            if(max_cycle_time < exec_max_ns){
                max_cycle_time = exec_max_ns;
            }
            if(min_jitter_time > latency_min_ns)
                min_jitter_time = latency_min_ns;
            if(max_jitter_time < latency_max_ns){
                max_jitter_time = latency_max_ns;
            }
            last_cycle_counter = cycle_counter;
            period_max_ns = -1000000;
            period_min_ns = 1000000;
            exec_max_ns = -1000000;
            exec_min_ns = 1000000;
            latency_max_ns = -1000000;
            latency_min_ns = 1000000;
            total_cycle_ns += total_exec_ns;
            total_exec_ns = 0;
        }
        rt_check_domain_state();

        ec_readmotordata(&ec_motor0,ec_slave0);
        if (ec_motor0.controlWord==0x1f)
        {
            switch(servo_mode)
            {
                case mode_CSP:
                    if(cycle_counter > 5000)
                        ec_motor0.targetPos = ec_motor0.actualPos + 0x1000;
                    ec_motor0.targetMaxProfileVel = 0x800000;
                break;
                case mode_CSV:
                    ec_motor0.targetVel = 0x10000;
                    ec_motor0.targetMaxProfileVel = 0x800000;
                break;
                case mode_CST:
                    ec_motor0.targetTorque = 100;
                    ec_motor0.targetMaxProfileVel = 0x800000;
                break;
            }
        }
        ec_writemotordata(&ec_motor0,ec_slave0);
        printf("count,%d, mode:%02x, CW,%04x, pos,%08x, vel,%08x, target pos:%08x, torque,%x \n", 
            cycle_counter,
            ec_motor0.actualMode,
            ec_motor0.controlWord,
            ec_motor0.actualPos,
            ec_motor0.actualVel,
            ec_motor0.targetPos,
            ec_motor0.actualTorque);

    	clock_gettime(CLOCK_MONOTONIC, &dc_period);
        ecrt_master_application_time(master, TIMESPEC2NS(dc_period));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &send_startTime);
#endif
        ecrt_master_send(master);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &send_endTime);
#endif
#ifdef MEASURE_TIMING
    	clock_gettime(CLOCK_MONOTONIC, &endTime);
#endif
    }

#ifdef MEASURE_TIMING
	if(last_cycle_counter)
		avg_cycle_time = total_cycle_ns/last_cycle_counter;
    printf("*********************************************\n");
    printf("average cycle time  %10.3f\n", (float)avg_cycle_time/1000);
    printf("cycle counter       %10d\n", last_cycle_counter);
    printf("cycle time          %10.3f ... %10.3f\n", (float)min_cycle_time/1000, (float)max_cycle_time/1000);
    printf("jitter time         %10.3f ... %10.3f\n", (float)min_jitter_time/1000, (float)max_jitter_time/1000);
    printf("*********************************************\n");
#endif
    return NULL;
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/
void signal_handler(int sig)
{
    run = 0;
    close_signal = 1000000/CYCLE_US; /* 1s */
}

static void getOptions(int argc, char **argv)
{
    int index;
    static struct option longOptions[] = {
		/* name		      has_arg			flag	val */
		{"mode_position", no_argument,		NULL,	'p'},
		{"mode_velocity", no_argument,		NULL,	'v'},
		{"mode_torque",   no_argument,		NULL,	't'},
		{"help",          no_argument,		NULL,	'h'},
		{}
    };
    do{
		index = getopt_long(argc, argv, "pvth", longOptions, NULL);
		switch(index){
		case 'p':
			servo_mode = mode_CSP;
            printf("CSP \n");
		break;
		case 'v':
            servo_mode = mode_CSV;
            printf("CSV \n");
		break;
		case 't':
            servo_mode = mode_CST;
            printf("CST \n");
		break;
		case 'h':
			printf("Global options:\n");
			printf("  --mode_position -p  CSP.\n");
            printf("  --mode_velocity -v  CSV.\n");
			printf("  --mode_torque   -t  CST.\n");		
			printf("  --help          -h  Show this help.\n");
			printf("default:  CSP \n");
			exit(0);
		break;
		default:
		break;
		}
    }while(index != -1);
}

/****************************************************************************
 * Main function
 ***************************************************************************/
int main(int argc, char *argv[])
{
    struct timespec dc_period;
    int ret;

	getOptions(argc,argv);
    data_init();
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)
        return -1;

    printf("Creating domain ...\n");
    domain = ecrt_master_create_domain(master);
    if (!domain)
        return -1;

     /* Create configuration for bus coupler */
    ec_slave0.sc = ecrt_master_slave_config(master, SOLO_Slave00_Pos, SOLO_SLAVE00_ID);
    if (!ec_slave0.sc) {
        printf("Slave0 sc is NULL \n");
        return -1;
    }

    printf("Creating slave configurations...\n");
    ec_slave_config_init(ec_slave0.sc);

    if (ecrt_slave_config_pdos(ec_slave0.sc, EC_END, slave_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.  0 \n");
        return -1;
    }

    ecrt_master_set_send_interval(master, CYCLE_US);
    if (ecrt_domain_reg_pdo_entry_list(domain, domain0_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    switch(servo_mode)
    {
        case mode_CSP:
            ecrt_slave_config_sdo8(ec_slave0.sc, 0x6060, 0x00, 0x08);
        break;
        case mode_CSV:
            ecrt_slave_config_sdo8(ec_slave0.sc, 0x6060, 0x00, 0x09);
        break;
        case mode_CST:
            ecrt_slave_config_sdo8(ec_slave0.sc, 0x6060, 0x00, 0x0A);
        break;
    }
//    ecrt_slave_config_sdo16(ec_slave0.sc, 0x2002, 0x03, 1);

    /*Configuring DC signal*/
    ecrt_slave_config_dc(ec_slave0.sc, 0x0300, PERIOD_NS, PERIOD_NS/2, 0, 0);

    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock
    */
    clock_gettime(CLOCK_MONOTONIC, &dc_period);

    /* Attention: The initial application time is also used for phase
     * calcuation for the SYNC0/1 interrupts. Please be sure to call it at
     * the correct phase to the realtime cycle
    */
    ecrt_master_application_time(master, TIMESPEC2NS(dc_period));

    ret = ecrt_master_select_reference_clock(master, ec_slave0.sc);
    if (ret < 0) {
        fprintf(stderr, "Failed to select reference clock: %s\n",strerror(-ret));
        return ret;
    }
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain_pd = ecrt_domain_data(domain))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    /* Create cyclic RT-thread */
    pthread_attr_t thattr;
    pthread_attr_init(&thattr);
    pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_JOINABLE);
    if (pthread_create(&cyclic_thread, &thattr, &my_thread, NULL)) {
        fprintf(stderr, "pthread_create cyclic task failed\n");
		return 1;
    }
    while (run || close_signal != 0)
	    sched_yield();

    pthread_join(cyclic_thread, NULL);
    ecrt_slave_config_sdo16(ec_slave0.sc, 0x6040, 0x00, 0x100);
    printf("End of Program\n");
    ecrt_release_master(master);
    return 0;
}

/****************************************************************************/
