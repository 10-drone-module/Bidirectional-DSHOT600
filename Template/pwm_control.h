/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _PWM_CONTROL_H_
#define _PWM_CONTROL_H_

#include "stdint.h"

#define MOTOR_NUM 4

typedef struct edge_data_
{
	uint32_t edge[4][21];
	uint8_t edge_cnt[4];
} edge_data_str;

typedef struct edge_dispose_
{
	uint8_t start_flag ;
	uint8_t signal_cnt;
	uint8_t hight_low_status;
	uint8_t old_hight_low_status;
	uint8_t vessel_number;
} edge_dispose_str;

typedef struct motor_speed_str_s
{
    uint16_t speed[4];
} motor_speed_str;

enum PWM_INDEX_NUMBER
{
    PWM_1 = 0,
    PWM_2 = 1,
    PWM_3 = 2,
    PWM_4 = 3,
    PWM_MAX = 4,
};

void test_dshot600_1ms(void);

#endif
