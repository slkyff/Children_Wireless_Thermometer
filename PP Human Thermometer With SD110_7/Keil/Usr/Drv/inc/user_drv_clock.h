#ifndef USER_DRV_CLOCK_H
#define USER_DRV_CLOCK_H

typedef enum
{
	START_STATE_CLOCK = 1,//1S Time_out
	STOP_STATE_CLOCK //4h Time_out
}state_clock_type;

extern void clock_start(state_clock_type en_clock_type);
extern void clock_res_init(void);
extern unsigned char clock_set(unsigned int target_sec);
extern void clock_get(unsigned int* p_cur_sec);

#endif

