#ifndef USER_DRV_BT_COMM_H
#define USER_DRV_BT_COMM_H

#define DEVICE_NAME                          "SCNT11000001"                               /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME							 "FIRSTDEV1"
#define MANUFACTURER_NAME                    "Sybercare"  			                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                            "SC-HTS"                                     /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                      0x1122334455                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                        0x667788                                   /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */
#define HW_REVISION							 "0.0.1"
#define FW_REVISION							 "7.0.0"									//S110 Revision
#define SW_REVISION							 "0.1.1"

extern void adc_bat_measure_start(void);
extern void adc_inter_board_measure_start(unsigned int start_moment);
extern void reset_global_flag_when_disconneted_or_shut(void);
extern void user_bt_comm_res_init(void);
extern void user_bt_comm_connect_timeout_start(void);
extern void user_bt_comm_connect_timeout_stop(void);

#endif

