
#include <string.h>
#include <stdbool.h>
#include "nrf51.h"
#include "pstorage.h"
#include "user_drv_offline_storage.h"
#include "user_drv_console.h"

#include "app_trace.h"
#include "app_error.h"

//#define OL_DISABLE_LOGS

#ifndef OL_DISABLE_LOGS
#define OL_LOG  app_trace_log  /**< Used for logging details. */
#define OL_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define OL_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define OL_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //OL_DISABLE_LOGS
#define OL_DUMP(...)           /**< Disables dumping of octet streams. */
#define OL_LOG(...)            /**< Disables detailed logs. */
#define OL_ERR(...)            /**< Disables error logs. */
#define OL_TRC(...)            /**< Disables traces. */
#endif 

#define OFFLINE_STORAGE_BLOCK_SIZE		256	
#define OFFLINE_STORAGE_BLOCK_COUNT 	((PSTORAGE_USER_DATA_PAGE_CNT - 2) * PSTORAGE_FLASH_PAGE_SIZE) / OFFLINE_STORAGE_BLOCK_SIZE
#define FACTORY_INFO_BLOCK_SIZE			PSTORAGE_MIN_BLOCK_SIZE
#define FACTORY_INFO_BLOCK_COUNT		20

#define FACTORY_INFO_MODE_BLOCK_INDEX   0
#define FACTORY_INFO_DEFAULT1_BLOCK_INDEX 1
#define FACTORY_INFO_DEFAULT2_BLOCK_INDEX 2
#define FACTORY_INFO_OFFSET_BLOCK_INDEX 3
#define FACTORY_INFO_DEVICE_BLOCK_INDEX 4

static pstorage_handle_t      m_offline_storage_handle;                                     
static pstorage_handle_t 	  m_factory_info_storage_handle;

struct st_factory_info_default
{
	uint32_t boot_delay;
	uint32_t shut_delay;
	uint32_t unstable_clt_interval;
	uint32_t stable_clt_interval;
	uint32_t report_interval;
	uint32_t out_of_body_shut_timeout;
};
struct st_factory_info_default stFactoryInfoDefault = {300, 1000, 4000, 15000, 120000, 1800000};
int32_t ad_code_offset[2] = {0x00};
uint32_t is_default_have_set = 0x0000;
bool is_to_set_default_param = false;								//准备去设置默认参数
uint8_t device_default_name[8] ={'0','0','0','0','0','0','0','0'};

//离线数据存储管理

static void offline_pstorage_cb_handler(pstorage_handle_t * p_handle,
	                                   uint8_t             op_code,
	                                   uint32_t            result,
	                                   uint8_t           * p_data,
	                                   uint32_t            data_len)
{
	
}

uint32_t user_drv_offline_storage_init(void)
{
	pstorage_module_param_t param;
    uint32_t	            err_code;

	param.block_count = OFFLINE_STORAGE_BLOCK_COUNT;
	param.block_size  = OFFLINE_STORAGE_BLOCK_SIZE;
	param.cb		  = offline_pstorage_cb_handler;

	err_code = pstorage_register(&param, &m_offline_storage_handle);

	OL_TRC("[OL]: NVM_STAT: OFFLINE_START_ADDR  0X%08X\r\n", m_offline_storage_handle.block_id);

	return err_code;
	
}


//厂商信息存储管理
static uint8_t m_factory_info_mode_set_flag = EN_USER_CONSOLE_FACTORY_INFO_MODE_CAN_SET;

static void factory_info_default_set_value(uint32_t* p_flag)
{//将默认属性字段赋值
	pstorage_handle_t factory_info_default_handle;

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT1_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_store(&factory_info_default_handle, (uint8_t*)&stFactoryInfoDefault, 16, 0));

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT2_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_store(&factory_info_default_handle, (uint8_t*)&stFactoryInfoDefault.report_interval, 8, 0));

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_OFFSET_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_store(&factory_info_default_handle, (uint8_t*)&ad_code_offset[0], sizeof(ad_code_offset), 0));

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEVICE_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_store(&factory_info_default_handle, (uint8_t*)&device_default_name[0], sizeof(device_default_name), 0));

	pstorage_handle_t factory_info_mode_handle;

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_MODE_BLOCK_INDEX,&factory_info_mode_handle));
	APP_ERROR_CHECK(pstorage_store(&factory_info_mode_handle, (uint8_t*)p_flag, sizeof(uint32_t), 4));	
}

uint32_t factory_info_default_trace_value(bool is_trace)
{//打印默认属性字段值
	pstorage_handle_t factory_info_default_handle;
	struct st_factory_info_default st_tmp_factory_info_default;
	uint16_t ad_offset[2];

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT1_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_load((uint8_t*)&st_tmp_factory_info_default, &factory_info_default_handle, 16, 0));
	
	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT2_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_load((uint8_t*)&st_tmp_factory_info_default.report_interval, &factory_info_default_handle, 8, 0));

	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_OFFSET_BLOCK_INDEX,&factory_info_default_handle));
	APP_ERROR_CHECK(pstorage_load((uint8_t*)&ad_offset[0], &factory_info_default_handle, sizeof(ad_offset), 0));
	
	if(is_trace != false)
	{
		OL_TRC("[OL]: BOOT_DELAY:            %dms\r\n", st_tmp_factory_info_default.boot_delay);
		OL_TRC("[OL]: SHUT_DELAY:            %dms\r\n", st_tmp_factory_info_default.shut_delay);
		OL_TRC("[OL]: UNSTABLE_CLT_INTERVAL: %dms\r\n", st_tmp_factory_info_default.unstable_clt_interval);
		OL_TRC("[OL]: STABLE_CLT_INTERVAL:   %dms\r\n", st_tmp_factory_info_default.stable_clt_interval);
		OL_TRC("[OL]: REPORT_INTERVAL:       %dms\r\n", st_tmp_factory_info_default.report_interval);
		OL_TRC("[OL]: OUT_OF_BODY_SHUT:      %dms\r\n", st_tmp_factory_info_default.out_of_body_shut_timeout);

		OL_TRC("[OL]: AD_LEFT:               %d\r\n", ad_offset[0]);
		OL_TRC("[OL]: AD_RIGHT:              %d\r\n", ad_offset[1]);
	}
	
	return NRF_SUCCESS;
}

static void factory_info_pstorage_cb_handler(pstorage_handle_t * p_handle,
                                   uint8_t             op_code,
                                   uint32_t            result,
                                   uint8_t           * p_data,
                                   uint32_t            data_len)
{
	if((m_factory_info_mode_set_flag == EN_USER_CONSOLE_FACTORY_INFO_MODE_CANNT_SET) || (m_factory_info_mode_set_flag == EN_USER_CONSOLE_FACTORY_INFO_MODE_RECAN_SET))
	{		
		uint32_t count;
		pstorage_access_status_get(&count);
		if(count == 0)
		{
			NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时。
		}
	}
	if(is_to_set_default_param == true)
	{
		uint32_t count;
		pstorage_access_status_get(&count);
		if(count == 0)
		{
			is_to_set_default_param = false;
			NVIC_SystemReset();			//设置好默认参数则进行复位动作	
		}
	}
}

uint32_t user_drv_factory_info_init(void)
{
	pstorage_module_param_t param;
    uint32_t	            err_code;

	param.block_count = FACTORY_INFO_BLOCK_COUNT;
	param.block_size  = FACTORY_INFO_BLOCK_SIZE;
	param.cb		  = factory_info_pstorage_cb_handler;

	err_code = pstorage_register(&param, &m_factory_info_storage_handle);

	OL_TRC("[OL]: NVM_STAT: FACTORY_START_ADDR  0X%08X\r\n", m_factory_info_storage_handle.block_id);

	return err_code;
}

void user_drv_factory_info_flag_set(uint8_t flag)
{
	m_factory_info_mode_set_flag = flag;
}

static void offline_factory_info_write(uint8_t index, uint8_t offset, uint8_t* p_buf, uint8_t size)
{
	pstorage_handle_t factory_info_handle;

	if(size != FACTORY_INFO_REGION_SIZE)
	{//进行单次设置配置
		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_MODE_BLOCK_INDEX + index,&factory_info_handle));
		APP_ERROR_CHECK(pstorage_update(&factory_info_handle, p_buf, size, offset));	
	}
	else
	{//一次性设置所有配置
		
	}
}

static void offline_factory_info_read(uint32_t index, uint8_t offset, uint8_t* p_buf, uint8_t size)
{
	pstorage_handle_t factory_info_handle;
	
	if(size != FACTORY_INFO_REGION_SIZE)
	{
		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_MODE_BLOCK_INDEX + index, &factory_info_handle));
		APP_ERROR_CHECK(pstorage_load(p_buf, &factory_info_handle, size, offset));
	}
	else
	{		
		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT1_BLOCK_INDEX, &factory_info_handle));
		APP_ERROR_CHECK(pstorage_load(p_buf, &factory_info_handle, 16, 0));

		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEFAULT2_BLOCK_INDEX, &factory_info_handle));
		APP_ERROR_CHECK(pstorage_load(p_buf + 16, &factory_info_handle, 8, 0));

		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_OFFSET_BLOCK_INDEX, &factory_info_handle));
		APP_ERROR_CHECK(pstorage_load(p_buf + 24, &factory_info_handle, 8, 0));

		APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_DEVICE_BLOCK_INDEX, &factory_info_handle));
		APP_ERROR_CHECK(pstorage_load(p_buf + 32, &factory_info_handle, 8, 0));		
	}
}

uint32_t factory_info_set(uint8_t op_code, uint8_t* p_buf, uint8_t size)
{//外部使用时注意:op_code需要在范围内,p_buf不能为空，size应该为合适的大小

	uint32_t err_code = NRF_SUCCESS;
	uint8_t index=0,offset=0,specify_size=0;
	
	if(p_buf == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	
	switch(op_code)
	{
		case FACTORY_INFO_OP_CODE_SET_FACTORY_MODE:	
			{
				specify_size = 0x04;
				index = 0;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_DEFAULT_MODE:
			{
				specify_size = 0x04;
				index = 0;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_BOOT_DELAY:
			{
				specify_size = 0x04;
				index = 1;
				offset = 0;				
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_SHUT_DELAY:
			{
				specify_size = 0x04;
				index = 1;
				offset = 4;				
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_UNSTABLE_CLT_INTERVAL:
			{
				specify_size = 0x04;
				index = 1;
				offset = 8;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_STABLE_CLT_INTERVAL:
			{
				specify_size = 0x04;
				index = 1;
				offset = 12;
			}
			break;	
		case FACTORY_INFO_OP_CODE_SET_REPORT_INTERVAL:
			{
				specify_size = 0x04;
				index = 2;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_OUT_OFF_BODY_TIMEOUT:
			{
				specify_size = 0x04;
				index = 2;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_AD1_OFFSET:
			{
				specify_size = 0x04;
				index = 3;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_AD2_OFFSET:
			{
				specify_size = 0x04;
				index = 3;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_DEVICE_NAME:
			{
				specify_size = 0x08;
				index = 4;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_SET_ALL_CONFIG:
			{
				specify_size = FACTORY_INFO_REGION_SIZE;
				index = 0;
				offset = 0;
			}
			break;
		default :
			{
				err_code = NRF_ERROR_INVALID_PARAM;
			}
			break;
	}

	if(specify_size == size)
	{
		offline_factory_info_write(index, offset, p_buf, size);			//进行flash写入操作
	}
	else
	{
		err_code = NRF_ERROR_INVALID_PARAM;
	}
	return err_code;
}

uint32_t factory_info_get(uint8_t op_code, uint8_t* p_buf, uint8_t* p_size)
{
	uint32_t err_code = NRF_SUCCESS;
	uint8_t index=0,size=0,offset=0;

	if(p_buf == NULL || p_size == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	
	switch(op_code)
	{
		case FACTORY_INFO_OP_CODE_GET_FACTORY_MODE:	
			{
				index = 0;
				size  = 4;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_DEFAULT_MODE:
			{
				index = 0;
				size  = 4;
				offset = 4;

			}
			break;
		case FACTORY_INFO_OP_CODE_GET_BOOT_DELAY:
			{
				index = 1;
				size  = 4;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_SHUT_DELAY:
			{
				index = 1;
				size  = 4;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_UNSTABLE_CLT_INTERVAL:
			{
				index = 1;
				size  = 4;
				offset = 8;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_STABLE_CLT_INTERVAL:
			{
				index = 1;
				size  = 4;
				offset = 12;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_REPORT_INTERVAL:
			{
				index = 2;
				size  = 4;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_OUT_OFF_BODY_TIMEOUT:
			{
				index = 2;
				size  = 4;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_AD1_OFFSET:
			{
				index = 3;
				size  = 4;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_AD2_OFFSET:
			{
				index = 3;
				size  = 4;
				offset = 4;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_DEVICE_NAME:
			{
				index = 4;
				size  = 8;
				offset = 0;
			}
			break;
		case FACTORY_INFO_OP_CODE_GET_ALL_CONFIG:
			{
				index = 0;
				size  = FACTORY_INFO_REGION_SIZE;
				offset = 0;
			}
			break;
		default :
			{
				err_code = NRF_ERROR_INVALID_PARAM;
			}
			break;
	}

	offline_factory_info_read(index,  offset, p_buf, size);		//获取厂商设备信息区数据
	*p_size = size;

	return err_code;

}

uint32_t factory_mode_exec(void)
{
	pstorage_handle_t factory_info_mode_handle;
	
	APP_ERROR_CHECK(pstorage_block_identifier_get(&m_factory_info_storage_handle, FACTORY_INFO_MODE_BLOCK_INDEX,&factory_info_mode_handle));
	APP_ERROR_CHECK(pstorage_load((uint8_t*)&is_default_have_set, &factory_info_mode_handle, sizeof(is_default_have_set), 4));
	
	if(is_default_have_set == 0xFFFFFFFF)
	{
		is_default_have_set = 0x00000001;						//设置过默认值了
		is_to_set_default_param = true;
		factory_info_default_set_value(&is_default_have_set);	
	}
	else if(is_default_have_set == 0x00000001)
	{
		struct st_factory_info_default st_tmp_info_default;
		uint16_t ad_offset[2];
		
		APP_ERROR_CHECK(factory_info_default_trace_value(true));	
	}
	else
	{
		return NRF_ERROR_NOT_SUPPORTED;
	}

	user_console_res_init();
	
	return NRF_SUCCESS;
}

