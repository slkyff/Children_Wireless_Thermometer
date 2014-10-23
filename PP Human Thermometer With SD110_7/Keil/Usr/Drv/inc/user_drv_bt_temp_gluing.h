
#ifndef USER_DRV_BT_TEMP_GLUING_H
#define USER_DRV_BT_TEMP_GLUING_H

extern void bt_temp_glue_record_time(void);
extern void bt_temp_glue_add_once_temp(unsigned short add_val, unsigned char curr_tx_cnt);
extern void bt_temp_glue_process_get_curr_temp_action(void);
extern void bt_temp_glue_check_after_get_curr_temp(void);

#endif

