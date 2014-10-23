#ifndef USER_DRV_PWR_CTRL_H
#define USER_DRV_PWR_CTRL_H

typedef enum
{
	USER_DRV_PWR_CTRL_SYS_OFFSTATE,
	USER_DRV_PWR_CTRL_SYS_ONSTATE
}user_drv_pwr_ctrl_sys_state_t;

extern void pwr_ctrl_res_init(void);

#endif

