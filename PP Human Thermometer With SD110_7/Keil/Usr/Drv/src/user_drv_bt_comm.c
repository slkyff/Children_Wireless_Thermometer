
#include "user_drv_bt_comm.h"
#include "user_drv_self_def_service.h"
#include "user_drv_clock.h"
#include "user_drv_ad7171op.h"
#include "user_drv_offline_storage.h"

#include "sc_ht_bt_comm_def.h"
#include "sc_ht_common_def.h"

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "softdevice_handler.h"
#include "ble_hci.h"

#include "ble_conn_params.h"

#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"

#include "user_app_evt.h"

#include "app_timer.h"


#define  CONNECT_TIMEOUT_TICKS				APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)		//10s timeout when ble4.0 connetted before receive update time request 

#define TEMP_TYPE_AS_CHARACTERISTIC 		0

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_BAT_MESURE_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_BAT_MESURE_PRE_SCALING_COMPENSATION)
#define ADC_BAT_MESURE_REF_VOLTAGE_IN_MILLIVOLTS     1200                               /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_BAT_MESURE_PRE_SCALING_COMPENSATION      3                                  /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    0	                                           /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define TEMP_DATA_BUF_LEN					20
#define UPDATE_TIME_RSP_LEN					USER_COMM_PKT_CMDID_UPDATE_TIME_RSP_SIZE
#define SET_CONFIG_RSP_LEN					4
#define PARAMETER_LEN						8
#define UPDATE_TIME_REQ_LEN					8
#define SET_CONFIG_REQ_LEN					10
#define GET_CURR_TEMP_LEN					4
#define SELF_DEF_BUF_SIZE					20

typedef enum
{
	ADC_MEASURE_BAT,
	ADC_MEASURE_BOARDINTER_NTC
}adc_mesure_type;


static uint8_t 								 m_adc_mesure_type;							/**< mesure data type. */
static app_timer_id_t						 m_timer_connect_timeout;					/**< timer id for connect timeout. */ 

static uint8_t m_realtime_tempdata_buf[TEMP_DATA_BUF_LEN];
static uint8_t m_history_tempdata_buf[TEMP_DATA_BUF_LEN];
static uint8_t m_update_time_rsp_buf[UPDATE_TIME_RSP_LEN];
static uint8_t m_set_config_rsp_buf[SET_CONFIG_RSP_LEN];
static uint8_t m_parameter_buf[PARAMETER_LEN];
static hts_idc_char_dat_t m_realtime_tempdata_struct;
static hts_idc_char_dat_t m_history_tempdata_struct;
static hts_idc_char_dat_t m_update_time_rsp_struct;
static hts_idc_char_dat_t m_set_config_rsp_struct;
static hts_para_char_dat_t m_parameter_struct;

static uint8_t m_set_config_req_buf[SET_CONFIG_REQ_LEN];
static uint8_t m_update_time_req_buf[UPDATE_TIME_REQ_LEN];
static uint8_t m_get_current_temp_buf[GET_CURR_TEMP_LEN];
static hts_write_char_dat_t m_set_config_req_struct;
static hts_write_char_dat_t m_update_time_req_struct;
static hts_write_char_dat_t m_get_current_temp_struct;

static uint8_t m_self_def_send_buf[SELF_DEF_BUF_SIZE];
static uint8_t m_self_def_rcv_buf[SELF_DEF_BUF_SIZE];
static sds_idc_char_dat_t m_self_def_send_struct;
static sds_write_char_dat_t m_self_def_rcv_struct;

ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
ble_hts_t                             m_hts;                                     /**< Structure used to identify the health thermometer service. */
ble_sds_t							  m_sds;
ble_dfu_t							  m_dfus;

bool                                  m_hts_meas_realtime_ind_conf_pending = false; /**< Flag to keep track of when an indication confirmation is pending. */
bool 								  m_hts_meas_history_ind_conf_pending = false;
bool 								  m_hts_meas_update_time_rsp_ind_conf_pending = false;
bool 								  m_hts_meas_set_config_rsp_ind_conf_pengding = false;
bool 								  m_is_pwrup_first_clt_battery  = false;

static bool							  m_is_update_time_set = false;				//indicate the wireless thermemoter has received update time request instruction

bool b_is_history_en = false;
bool b_is_realtime_en = false;

uint32_t realtime_temp_send(ble_hts_t* p_hts, uint8_t data_type, uint8_t* pbuf, uint16_t buf_len);


/**@brief Function for handling the ADC interrupt.
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
static uint32_t BoardNTC_StartCltMoment = 0;

double rntc = 0.0;
float delt_log = 0.0;
float f_environmental_temp = 0.0;

void ADC_IRQHandler(void)
{
    if (NRF_ADC->EVENTS_END != 0)
    {
		if(ADC_MEASURE_BAT == m_adc_mesure_type)
		{
			uint8_t     adc_result;
	        uint16_t    batt_lvl_in_milli_volts;
	        uint8_t     percentage_batt_lvl;
	        uint32_t    err_code;

	        NRF_ADC->EVENTS_END     = 0;
	        adc_result              = NRF_ADC->RESULT;
	        NRF_ADC->TASKS_STOP     = 1;

	        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
	                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
	        percentage_batt_lvl     = battery_level_in_percent(batt_lvl_in_milli_volts);

	        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);				//向对端发送电池电量
	        if (
	            (err_code != NRF_SUCCESS)
	            &&
	            (err_code != NRF_ERROR_INVALID_STATE)
	            &&
	            (err_code != BLE_ERROR_NO_TX_BUFFERS)
	            &&
	            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
	        )
	        {
	            APP_ERROR_HANDLER(err_code);
	        }	
		}
		else if(ADC_MEASURE_BOARDINTER_NTC == m_adc_mesure_type)
		{
			uint16_t adc_result;

			NRF_ADC->EVENTS_END     = 0;
	        adc_result              = NRF_ADC->RESULT;
	        NRF_ADC->TASKS_STOP     = 1;
		
			rntc = (double)(3 * adc_result) / (2048 - 3 * adc_result);
			
			delt_log = (11.512925 - log(rntc * 100000));
			
			f_environmental_temp = (double)(1267137.5) / (double)(4250 - (double)(298.15 * delt_log)) - 273.15;

			uint16_t environmental_temp;
			environmental_temp = (uint16_t)(f_environmental_temp * 100);
			
			uint8_t buf[6];
			memcpy(&buf[0], &BoardNTC_StartCltMoment, sizeof(BoardNTC_StartCltMoment));
			memcpy(&buf[sizeof(BoardNTC_StartCltMoment)], &environmental_temp, sizeof(environmental_temp));
			
			APP_ERROR_CHECK(realtime_temp_send(&m_hts, ad7171_get_cur_temp_state(), &buf[0], sizeof(buf)));		

			extern void ad7171_close_analog_power(void);
			ad7171_close_analog_power();							//关闭模拟电路供电
		}
    }
}

/**@brief Function for making the ADC start a battery level conversion.
 */
void adc_bat_measure_start(void)
{
    uint32_t err_code;

    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;

	m_adc_mesure_type = ADC_MEASURE_BAT;
}

void adc_inter_board_measure_start(uint32_t start_moment)
{
	uint32_t err_code;

	// Cofigure ADC
	while(ADC_BUSY_BUSY_Busy == (NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk))
	{

	}	
	NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
	NRF_ADC->CONFIG = (ADC_CONFIG_REFSEL_SupplyOneHalfPrescaling << ADC_CONFIG_REFSEL_Pos) | \
					  (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) | \
					  (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) | \
					  (ADC_CONFIG_PSEL_AnalogInput6 << ADC_CONFIG_PSEL_Pos) |\
					  (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

	NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;
	
	// Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;

	m_adc_mesure_type = ADC_MEASURE_BOARDINTER_NTC;				//测量板载NTC的值
	BoardNTC_StartCltMoment = start_moment;
}

void reset_global_flag_when_disconneted_or_shut(void)
{
	m_hts_meas_update_time_rsp_ind_conf_pending = false;
	m_hts_meas_set_config_rsp_ind_conf_pengding = false;
	m_hts_meas_history_ind_conf_pending			= false;
	m_hts_meas_realtime_ind_conf_pending		= false;
	
	m_is_pwrup_first_clt_battery  				= false;

	m_is_update_time_set						= false;

	b_is_realtime_en 							= false;
	b_is_history_en								= false;
}

static uint32_t update_time_response(ble_hts_t* p_hts, uint8_t flag)
{
	uint32_t	err_code;	

	if(p_hts->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		uint8_t update_time_rsp_buf[USER_COMM_PKT_CMDID_UPDATE_TIME_RSP_SIZE];
		uint16_t	len;
		uint16_t	hvx_len;

		ble_gatts_hvx_params_t hvx_params;

		memset(&update_time_rsp_buf[0], 0, sizeof(update_time_rsp_buf));
		memset(&hvx_params, 0, sizeof(hvx_params));

		update_time_rsp_buf[0] = USER_COMM_PKT_HEADER_FLAG;
		update_time_rsp_buf[USER_COMM_PKT_CMDID_UPDATE_TIME_RSP_SIZE - 1] = USER_COMM_PKT_TAIL_FLAG;
		update_time_rsp_buf[1] = (USER_COMM_PKT_CMDID_SET_CONFIG_RSP << USER_COMM_PKT_TYPE_FIELD_POS) | USER_COMM_PKT_TYPE_CMD;
		update_time_rsp_buf[2] = 1;
		update_time_rsp_buf[3] = flag;						

		len = sizeof(update_time_rsp_buf);
		hvx_len = len;
		
		hvx_params.handle	=	p_hts->update_time_response_handles.value_handle;
		hvx_params.offset	=	0;
		hvx_params.p_data	=	&update_time_rsp_buf[0];
		hvx_params.p_len	=	&hvx_len;
		hvx_params.type		=	BLE_GATT_HVX_INDICATION;

		err_code = sd_ble_gatts_hvx(p_hts->conn_handle, &hvx_params);

		if(err_code == NRF_SUCCESS && hvx_len != len)
		{
			err_code = NRF_ERROR_DATA_SIZE;
		}	
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;	
	}
	
	return err_code;
}

static void update_time_request_process(uint8_t* p_buf, uint8_t len)
{
	APP_ERROR_CHECK_BOOL(p_buf != NULL);
	APP_ERROR_CHECK_BOOL(len != 0);
	
	uint8_t encode_rcv_buf[20];
	memcpy(&encode_rcv_buf[0], p_buf, len);	
	
	if(len == USER_COMM_PKT_CMDID_UPDATE_TIME_REQ_SIZE)
	{
		if(encode_rcv_buf[0] == USER_COMM_PKT_HEADER_FLAG && encode_rcv_buf[USER_COMM_PKT_CMDID_UPDATE_TIME_REQ_SIZE - 1] == USER_COMM_PKT_TAIL_FLAG)
		{
			if((encode_rcv_buf[1] & 0x03) == USER_COMM_PKT_TYPE_CMD && (encode_rcv_buf[1] >> USER_COMM_PKT_TYPE_FIELD_POS) == USER_COMM_PKT_CMDID_UPDATE_TIME_REQ)
			{
				if(encode_rcv_buf[2] == 4)
				{
					uint32_t update_clock_value = 0x00;
					uint8_t buf[4] = {0};
					memcpy(&buf[0], &encode_rcv_buf[3], 4);
					update_clock_value = *(uint32_t*)&buf[0];
					uint8_t ret_flag = clock_set(update_clock_value);				//设置时钟

					m_is_update_time_set = true;
					
					//返回update time response
					uint32_t err_code = 0x00;
					if(false == m_hts_meas_update_time_rsp_ind_conf_pending)
					{
						err_code = update_time_response(&m_hts, ret_flag);			//发送update time response
					}
					if(err_code == NRF_SUCCESS)
					{
						m_hts_meas_update_time_rsp_ind_conf_pending = true;	
					}
					else 
					{
						APP_ERROR_CHECK(err_code);	
					}
				}
			}
		}
	}
}

static void set_config_request_process(uint8_t* p_buf, uint8_t len)
{
	APP_ERROR_CHECK_BOOL(p_buf != NULL);
	APP_ERROR_CHECK_BOOL(len != 0);

}

static void get_curr_temp_process(uint8_t* p_buf, uint8_t len)
{
	APP_ERROR_CHECK_BOOL(p_buf != NULL);
	APP_ERROR_CHECK_BOOL(len != 0);

	bt_temp_glue_process_get_curr_temp_action();
}

static uint8_t sds_encode_rcv_buf[20];			//可能用于设备名向flash的写入，所以定义一个全局缓冲区
static void self_define_config_process(uint8_t* p_buf, uint8_t len)
{
	APP_ERROR_CHECK_BOOL(p_buf != NULL);
	APP_ERROR_CHECK_BOOL(len != 0);

	
	memcpy(&sds_encode_rcv_buf[0], p_buf, len);	

	if(len == 4 + 8)
	{
		if(sds_encode_rcv_buf[0] == 'S' && sds_encode_rcv_buf[1] == 'N')
		{//设置设备名称
			APP_ERROR_CHECK(factory_info_set(FACTORY_INFO_OP_CODE_SET_DEVICE_NAME, &sds_encode_rcv_buf[4], len - 4));
		}
	}	
}

uint8_t g_buf[20] = {0x00};
uint8_t update_time_rcv_cnt = 0;
uint8_t set_config_rcv_cnt = 0;
uint8_t get_curr_temp_rcv_cnt = 0;

static void hts_rcv_data_handler(ble_hts_t* p_hts, uint8_t* p_data, uint16_t length, uint16_t datasrc_handle)
{
	
	memcpy(&g_buf[0], p_data, length);
	
	if(datasrc_handle == p_hts->update_time_request_handles.value_handle)
	{
		update_time_request_process(p_data, length);							//设置时间同步请求
	}
	else if(datasrc_handle == p_hts->set_config_request_handles.value_handle)
	{
		set_config_request_process(p_data, length);								//设置配置请求
	}	
	else if(datasrc_handle == p_hts->get_curr_temp_handles.value_handle)
	{
		get_curr_temp_process(p_data, length);									//获取立即温度数据
	}
	else
	{
		//TODO:
	}
}

static void sds_rcv_data_handler(ble_sds_t* p_sds,uint8_t* p_data, uint16_t length, uint16_t datasrc_handle)
{
	memcpy(&g_buf[0], p_data, length);
	
	if(datasrc_handle == p_sds->self_def_rcv_handles.value_handle)
	{
		self_define_config_process(p_data, length);								//自定义数据格式配置处理
	}
	else
	{
		//TODO:
	}
}

uint32_t realtime_temp_send(ble_hts_t* p_hts, uint8_t data_type, uint8_t* pbuf, uint16_t buf_len)
{
	uint8_t err_code = 0x00; 

	APP_ERROR_CHECK_BOOL(buf_len != 0);

	if(m_hts_meas_realtime_ind_conf_pending == false && b_is_realtime_en == true)
	{
		if(p_hts->conn_handle != BLE_CONN_HANDLE_INVALID)
		{			
			if(b_is_realtime_en == true)
			{
				m_hts_meas_realtime_ind_conf_pending = true;
			
				uint8_t realtime_temp_buf[TEMP_DATA_BUF_LEN];
				uint16_t	len;
				uint16_t	hvx_len;

				ble_gatts_hvx_params_t hvx_params;

				memset(&realtime_temp_buf[0], 0, sizeof(realtime_temp_buf));
				memset(&hvx_params, 0, sizeof(hvx_params));

				realtime_temp_buf[0] = USER_COMM_PKT_HEADER_FLAG;
				len = 4 + USER_COMM_PKT_RSV_SIZE + buf_len; 
				realtime_temp_buf[len - 1] = USER_COMM_PKT_TAIL_FLAG;
				realtime_temp_buf[1] = (data_type << 2) | USER_COMM_PKT_TYPE_DATA;
				realtime_temp_buf[2] = buf_len;
				memcpy(&realtime_temp_buf[3], pbuf, buf_len);

				hvx_len = len;

				hvx_params.handle = p_hts->meas_realtime_handles.value_handle;
				hvx_params.offset = 0;
				hvx_params.p_data = &realtime_temp_buf[0];
				hvx_params.p_len  = &hvx_len;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;

				err_code = sd_ble_gatts_hvx(p_hts->conn_handle, &hvx_params);

				if(err_code == NRF_SUCCESS && hvx_len != len)
				{
					err_code = NRF_ERROR_DATA_SIZE;
				}	
			}
			else
			{

			}			
		}
		else
		{
//			err_code = NRF_ERROR_INVALID_STATE;	
		}
	}

	return err_code;
}

uint32_t history_temp_send(ble_hts_t* p_hts, uint8_t* pbuf, uint16_t buf_len)
{
	uint8_t err_code = 0x00; 

	APP_ERROR_CHECK_BOOL(buf_len != 0);
	APP_ERROR_CHECK_BOOL((TEMP_DATA_BUF_LEN - (buf_len + 4 + USER_COMM_PKT_RSV_SIZE)) >= buf_len);

	if(m_hts_meas_history_ind_conf_pending == false && b_is_history_en == true)
	{	
		if(p_hts->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			if(b_is_history_en == true)
			{
				m_hts_meas_history_ind_conf_pending = true;
			
				uint8_t history_temp_buf[TEMP_DATA_BUF_LEN];
				uint16_t	len;
				uint16_t	hvx_len;

				ble_gatts_hvx_params_t hvx_params;

				memset(&history_temp_buf[0], 0, sizeof(history_temp_buf));
				memset(&hvx_params, 0, sizeof(hvx_params));

				history_temp_buf[0] = USER_COMM_PKT_HEADER_FLAG;
				len = 4 + USER_COMM_PKT_RSV_SIZE + buf_len; 
				history_temp_buf[len - 1] = USER_COMM_PKT_TAIL_FLAG;
				history_temp_buf[1] = (USER_COMM_PKT_DATAID_HISTORY_TEMP << 2) | USER_COMM_PKT_TYPE_DATA;
				history_temp_buf[2] = buf_len;
				memcpy(&history_temp_buf[3], pbuf, buf_len);

				hvx_len = len;

				hvx_params.handle = p_hts->meas_history_handles.value_handle;
				hvx_params.offset = 0;
				hvx_params.p_data = &history_temp_buf[0];
				hvx_params.p_len  = &hvx_len;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;

				err_code = sd_ble_gatts_hvx(p_hts->conn_handle, &hvx_params);

				if(err_code == NRF_SUCCESS && hvx_len != len)
				{
					err_code = NRF_ERROR_DATA_SIZE;
				}		
			}
			else
			{

			}
		}
		else
		{
//			err_code = NRF_ERROR_INVALID_STATE;	
		}
		
	}

	return err_code;
}

/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in]   p_hts   Health Thermometer Service structure.
 * @param[in]   p_evt   Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case HTS_REALTIME_EVT_INDICATION_ENABLED:
			{
				b_is_realtime_en = true;
			}
            break;

		case HTS_REALTIME_EVT_INDICATION_DISABLE:
			{
				b_is_realtime_en = false;
			}
			break;

        case HTS_REALTIME_EVT_INDICATION_CONFIRMED:
			{
           		m_hts_meas_realtime_ind_conf_pending = false;
				bt_temp_glue_check_after_get_curr_temp();				
			}
            break;
		case HTS_HISTORY_EVT_INDICATION_ENABLED:
			{
				b_is_history_en = true;
			}
			break;

		case HTS_HISTORY_EVT_INDICATION_DISABLE:
			{
				b_is_history_en = false;
			}
			break;
			
		case HTS_HISTORY_EVT_INDICATION_CONFIRMED:
			m_hts_meas_history_ind_conf_pending = false;
			break;
		case HTS_UPDATE_TIME_RSP_EVT_INDICATION_ENABLED:

			break;
		case HTS_UPDATE_TIME_RSP_EVT_INDICATION_CONFIRMED:
			m_hts_meas_update_time_rsp_ind_conf_pending = false;
			break;
		case HTS_SET_CONFIG_RSP_EVT_INDICATION_ENABLED:

			break;
		case HTS_SET_CONFIG_RSP_EVT_INDICATION_CONFIRMED:
			m_hts_meas_set_config_rsp_ind_conf_pengding = false;
			break;
		
        default:
            // No implementation needed.
            break;
    }
}

static void on_sds_evt(ble_sds_t * p_hts, ble_sds_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case SELF_DEF_SEND_EVT_INDICATION_ENABLED:

            break;

        case SELF_DEF_SEND_EVT_INDICATION_CONFIRMED:
      

            break;

        default:
            // No implementation needed.
            break;
    }
}

extern void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t *p_evt);

static void sc_wt_tms_init(void)
{//sybercare wireless thermometer temperature measure service init
	
	uint32_t		 err_code;
	ble_hts_init_t	 hts_init;
	
	// Initialize Health Thermometer Service
	memset(&hts_init, 0, sizeof(hts_init));

	hts_init.evt_handler				 = on_hts_evt;
	hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
	hts_init.temp_type					 = BLE_HTS_TEMP_TYPE_BODY;
	hts_init.p_realtime_char_buf		 = &m_realtime_tempdata_struct;
	hts_init.p_history_char_buf			 = &m_history_tempdata_struct;
	hts_init.p_update_time_rsp_char_buf	 = &m_update_time_rsp_struct;
	hts_init.p_set_config_rsp_char_buf	 = &m_set_config_rsp_struct;
	hts_init.p_parameter_char_buf		 = &m_parameter_struct;
	hts_init.p_update_time_req_char_buf	 = &m_update_time_req_struct;
	hts_init.p_set_config_req_char_buf	 = &m_set_config_req_struct;
	hts_init.p_get_current_temp_char_buf = &m_get_current_temp_struct;	
	
	hts_init.data_handler				 = hts_rcv_data_handler;

	// Here the sec level for the Health Thermometer Service can be changed/increased.
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_realtime_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_realtime_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_realtime_attr_md.write_perm);	//设置Realtime charactersitic 权限
	hts_init.p_realtime_char_buf->len	=	(uint8_t)sizeof(m_realtime_tempdata_buf);
	hts_init.p_realtime_char_buf->p_buf	=	&m_realtime_tempdata_buf[0];					//设置realtime用户缓存空间

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_history_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_history_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_history_attr_md.write_perm);		//设置History charactersitic 权限
	hts_init.p_history_char_buf->len	=	(uint8_t)sizeof(m_history_tempdata_buf);
	hts_init.p_history_char_buf->p_buf	=	&m_history_tempdata_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.set_config_response_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.set_config_response_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.set_config_response_attr_md.read_perm);		//设置set_config_response charactersitic权限
	hts_init.p_set_config_rsp_char_buf->len		=	(uint8_t)sizeof(m_set_config_rsp_buf);
	hts_init.p_set_config_rsp_char_buf->p_buf	=	&m_set_config_rsp_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.update_time_response_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.update_time_response_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.update_time_response_attr_md.read_perm);		//设置update time response charactersitic权限
	hts_init.p_update_time_rsp_char_buf->len	=	(uint8_t)sizeof(m_update_time_rsp_buf);
	hts_init.p_update_time_rsp_char_buf->p_buf  = 	&m_update_time_rsp_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_temp_type_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_temp_type_attr_md.write_perm);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_parameter_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_parameter_attr_md.write_perm);	//设置hts meas parameter charactersitic权限
	hts_init.p_parameter_char_buf->len			= (uint8_t)sizeof(m_parameter_buf);
	hts_init.p_parameter_char_buf->p_buf		= &m_parameter_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.update_time_request_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.update_time_request_attr_md.read_perm);		//设置update time request characteristic权限
	hts_init.p_update_time_req_char_buf->len 	= (uint8_t)sizeof(m_update_time_req_buf);	
	hts_init.p_update_time_req_char_buf->p_buf  = &m_update_time_req_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.set_config_request_attr_md.write_perm);		
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.set_config_request_attr_md.read_perm);			//设置set config request characteristic权限
	hts_init.p_set_config_req_char_buf->len 	= (uint8_t)sizeof(m_set_config_req_buf);
	hts_init.p_set_config_req_char_buf->p_buf  = &m_set_config_req_buf[0];

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.get_current_temp_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.get_current_temp_attr_md.read_perm);			//设置get current temp characteristic权限
	hts_init.p_get_current_temp_char_buf->len 	= (uint8_t)sizeof(m_get_current_temp_buf);
	hts_init.p_get_current_temp_char_buf->p_buf  = &m_get_current_temp_buf[0];
	
	err_code = ble_hts_init(&m_hts, &hts_init);
	APP_ERROR_CHECK(err_code);
}

static void sc_wt_bas_init(void)
{//sybercare wireless thermometer battery service init
	uint32_t err_code;
	ble_bas_init_t   bas_init;

	// Initialize Battery Service Struct
	memset(&bas_init, 0, sizeof(bas_init));

	// Here the sec level for the Battery Service can be changed/increased.
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

	bas_init.initial_batt_level = 100;
	bas_init.p_report_ref 		= NULL;
	bas_init.support_notification = true;
	bas_init.evt_handler		= on_bas_evt;

	err_code = ble_bas_init(&m_bas, &bas_init);		
	APP_ERROR_CHECK(err_code);
	
}

static void sc_wt_dis_init(void)
{//sybercare wireless thermometer device information service init
	uint32_t err_code;

	ble_dis_init_t dis_init;
	ble_dis_sys_id_t sys_id;

	// Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, 	   HW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,		   FW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,		   SW_REVISION);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
	
}

static void sc_wt_sds_init(void)
{//sybercare wireless thermometer selfdef service init
	uint32_t		 err_code;
	ble_sds_init_t	 sds_init;
	
	// Initialize Health Thermometer Service
	memset(&sds_init, 0, sizeof(sds_init));

	sds_init.evt_handler				 = on_sds_evt;
	sds_init.p_self_def_send_char_buf 	 = &m_self_def_send_struct;
	sds_init.p_self_def_rcv_char_buf	 = &m_self_def_rcv_struct;

	
	sds_init.data_handler				 = sds_rcv_data_handler;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sds_init.sds_self_def_send_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sds_init.sds_self_def_send_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sds_init.sds_self_def_send_attr_md.write_perm);	//设置selfdef send charactersitic 权限
	sds_init.p_self_def_send_char_buf->len	=	(uint8_t)sizeof(m_self_def_send_buf);
	sds_init.p_self_def_send_char_buf->p_buf =	&m_self_def_send_buf[0];					

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sds_init.sds_self_def_rcv_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sds_init.sds_self_def_rcv_attr_md.read_perm);		//设置update time request characteristic权限
	sds_init.p_self_def_rcv_char_buf->len 	= (uint8_t)sizeof(m_self_def_rcv_buf);	
	sds_init.p_self_def_rcv_char_buf->p_buf  = &m_self_def_rcv_buf[0];

	err_code = ble_sds_init(&m_sds, &sds_init);\
	APP_ERROR_CHECK(err_code);
	
}

static void reset_prepare(void)
{
    uint32_t err_code;
    
    if (user_get_current_con_handle() != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(user_get_current_con_handle(), BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, then the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }
    
    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}

static void sc_wt_dfu_init(void)
{
	uint32_t err_code = 0x00;
	ble_dfu_init_t   dfus_init;
	
	// Initialize the Device Firmware Update Service.
	memset(&dfus_init, 0, sizeof(dfus_init));
	dfus_init.evt_handler    = dfu_app_on_dfu_evt;
	dfus_init.error_handler  = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
	err_code = ble_dfu_init(&m_dfus, &dfus_init);
	APP_ERROR_CHECK(err_code);

	dfu_app_reset_prepare_set(reset_prepare);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 */
static void services_init(void)
{

	sc_wt_tms_init();
	sc_wt_bas_init();
	sc_wt_dis_init();
	sc_wt_sds_init();
	sc_wt_dfu_init();
	
}

static void connect_timeout_handler(void* p_context)
{	
	if(m_is_update_time_set != true)
	{//在超时时间段内没有收到时间同步请求，则主动断开连接
//		extern uint16_t user_get_current_con_handle(void);
//	
//		APP_ERROR_CHECK(sd_ble_gap_disconnect(user_get_current_con_handle(),
//                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
	}
}

void user_bt_comm_res_init(void)
{
	APP_ERROR_CHECK(app_timer_create(&m_timer_connect_timeout, APP_TIMER_MODE_SINGLE_SHOT, connect_timeout_handler));

	services_init();
}

void user_bt_comm_connect_timeout_start(void)
{
	APP_ERROR_CHECK(app_timer_start(m_timer_connect_timeout, CONNECT_TIMEOUT_TICKS, NULL));
}

void user_bt_comm_connect_timeout_stop(void)
{
	APP_ERROR_CHECK(app_timer_stop(m_timer_connect_timeout));
}


