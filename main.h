#ifndef __ETHERCAT_6AIXS_MAIN_H__
#define __ETHERCAT_6AIXS_MAIN_H__

#include <stdio.h>
#include "ecrt.h"

#define MEASURE_TIMING

#define CYCLE_US    			1000
#define PERIOD_NS   			(CYCLE_US*1000)
#define NSEC_PER_SEC    		(1000000000L)
#define TIMESPEC2NS(T)  		((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define DIFF_NS(A,B)    		(((B).tv_sec - (A).tv_sec)*NSEC_PER_SEC + ((B).tv_nsec)-(A).tv_nsec)
#define CYCLE_COUNTER_PERSEC    (NSEC_PER_SEC/PERIOD_NS)

#define MAX_DECODER_COUNT		0x100000
#define MAX_VELOCITY_MOTOR		0x66665f92
#define SERVO_AXIS_SIZE			20.0

#define SOLO_Slave00_Pos  	0, 0
//#define SOLO_SLAVE00_ID   	0x00100000, 0x000c0108

#define SOLO_SLAVE00_ID   	0x0000066F, 0x60380004 //panasonic
/* Master 0, Slave 0, "MADLN05BE"
 * Vendor ID:       0x0000066f
 * Product code:    0x60380004
 * Revision number: 0x00010000
 */


typedef enum{
    mode_CSP = 0,
    mode_CSV = 1,
    mode_CST = 2
}Servo_Run_Mode_t;

typedef struct{
    ec_slave_config_t *sc;
    unsigned int ctrl_word;
    unsigned int mode_sel;
    unsigned int tar_pos;
    unsigned int tar_vel;
    unsigned int tar_torque;
    unsigned int tar_max_profile_vel;

    unsigned int status_word;
    unsigned int mode_display;
    unsigned int pos_act;
    unsigned int vel_act;
	unsigned int torque_act;
}Slave_Data;

typedef struct{
    unsigned short controlWord;
    signed char    tarqueMode;
    signed int     targetPos;
    signed int     targetVel;
    signed short   targetTorque;
    unsigned int   targetMaxProfileVel;

    unsigned short statusWord;
    signed char    actualMode;
    signed int     actualPos;
    signed int     actualVel;
    signed short   actualTorque;
}Ctl_Data;

ec_pdo_entry_info_t slave_pdo_entries[] = {
	{0x6040, 0x00, 16},
	{0x6060, 0x00, 8},
	{0x607A, 0x00, 32},
	{0x60FF, 0x00, 32},
	{0x6071, 0x00, 16},
	{0x607F, 0x00, 32},
	{0x6041, 0x00, 16},
	{0x6061, 0x00, 8},
	{0x6064, 0x00, 32},
	{0x606C, 0x00, 32},
	{0x6077, 0x00, 16},
};

ec_pdo_info_t slave_pdos[] = {
   {0x1600, 6, slave_pdo_entries + 0},
   {0x1a00, 5, slave_pdo_entries + 6},
};

ec_sync_info_t slave_syncs[] = {
   {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
   {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
   {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
   {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
   {0xff}
};

#endif

