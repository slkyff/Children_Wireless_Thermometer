
#ifndef USER_DRV_OFFLINE_STORAGE_H
#define USER_DRV_OFFLINE_STORAGE_H

enum
{
	EN_USER_CONSOLE_FACTORY_INFO_MODE_CAN_SET,
	EN_USER_CONSOLE_FACTORY_INFO_MODE_CANNT_SET,
	EN_USER_CONSOLE_FACTORY_INFO_MODE_RECAN_SET
};

#define FACTORY_INFO_REGION_SIZE        40

//设置厂商信息区字段
#define FACTORY_INFO_OP_CODE_SET_FACTORY_MODE	0x01
#define FACTORY_INFO_OP_CODE_SET_DEFAULT_MODE	(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x01)
#define FACTORY_INFO_OP_CODE_SET_BOOT_DELAY  	(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x02)
#define FACTORY_INFO_OP_CODE_SET_SHUT_DELAY  	(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x03)
#define FACTORY_INFO_OP_CODE_SET_UNSTABLE_CLT_INTERVAL (FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x04)
#define FACTORY_INFO_OP_CODE_SET_STABLE_CLT_INTERVAL (FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x05)
#define FACTORY_INFO_OP_CODE_SET_REPORT_INTERVAL	   (FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x06)
#define FACTORY_INFO_OP_CODE_SET_OUT_OFF_BODY_TIMEOUT  (FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x07)
#define FACTORY_INFO_OP_CODE_SET_AD1_OFFSET		(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x08)
#define FACTORY_INFO_OP_CODE_SET_AD2_OFFSET		(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x09)
#define FACTORY_INFO_OP_CODE_SET_DEVICE_NAME	(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x0A)
#define FACTORY_INFO_OP_CODE_SET_ALL_CONFIG		(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE + 0x0B)		//一次设置所有的配置

//获取厂商信息区字段
#define FACTORY_INFO_OP_CODE_GET_FACTORY_MODE	0x10
#define FACTORY_INFO_OP_CODE_GET_DEFAULT_MODE	(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x10)
#define FACTORY_INFO_OP_CODE_GET_BOOT_DELAY  	(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x20)
#define FACTORY_INFO_OP_CODE_GET_SHUT_DELAY  	(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x30)
#define FACTORY_INFO_OP_CODE_GET_UNSTABLE_CLT_INTERVAL   (FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x40)
#define FACTORY_INFO_OP_CODE_GET_STABLE_CLT_INTERVAL   (FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x50)
#define FACTORY_INFO_OP_CODE_GET_REPORT_INTERVAL	   (FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x60)
#define FACTORY_INFO_OP_CODE_GET_OUT_OFF_BODY_TIMEOUT  (FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x70)
#define FACTORY_INFO_OP_CODE_GET_AD1_OFFSET		(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x80)
#define FACTORY_INFO_OP_CODE_GET_AD2_OFFSET		(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0x90)
#define FACTORY_INFO_OP_CODE_GET_DEVICE_NAME	(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0xA0)
#define FACTORY_INFO_OP_CODE_GET_ALL_CONFIG     (FACTORY_INFO_OP_CODE_GET_FACTORY_MODE + 0xB0)		//一次获取所有的配置


extern uint32_t user_drv_offline_storage_init(void);

extern uint32_t user_drv_factory_info_init(void);

extern void user_drv_factory_info_flag_set(uint8_t flag);

extern uint32_t factory_info_set(uint8_t op_code, uint8_t* p_buf, uint8_t size);

extern uint32_t factory_info_get(uint8_t op_code, uint8_t* p_buf, uint8_t* p_size);

extern uint32_t factory_mode_exec(void);


#endif

