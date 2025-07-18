#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#pragma pack(1)
typedef struct
{
    unsigned int act_position;
    unsigned int act_volecty;
    unsigned int act_torque;
    unsigned int state;
    unsigned int control_wd;
    unsigned int tar_position;
    unsigned int tar_volecty;
    unsigned int tar_torque;
    unsigned int position_offset;
    unsigned int mode_of_operation_6060;
    unsigned int mode_of_operation_6061;
    unsigned int digit_inputs;
    unsigned int digit_outputs;
} pdo_entry;

typedef struct
{
    int32_t act_position;
    int32_t act_volecty;
    int16_t act_torque;
    u_int16_t state;
    u_int16_t control_wd;
    int32_t tar_position;
    int32_t tar_volecty;
    int16_t tar_torque;
    int32_t position_offset;
    int8_t mode_of_operation_6060;
    int8_t mode_of_operation_6061;
    u_int32_t digit_inputs;
    u_int32_t digit_outputs;
} Motor_var;


