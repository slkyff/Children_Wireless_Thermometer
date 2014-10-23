
#include <stdlib.h>
#include <math.h>
#include "user_drv_ntc_temp_table.h"
#include "user_drv_ad7171op.h"
#include "sc_ht_bt_comm_def.h"
#include "user_drv_bt_temp_gluing.h"
#include "spi_master.h"
#include "app_timer.h"
#include "math.h"
#include "sc_ht_perpin_def.h"
#include "sc_ht_common_def.h"
#include "nordic_common.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "app_trace.h"

#define PP_SC_PHY

#ifdef EVB_SC_PHY
//#define EAD_DISABLE_LOGS        /**< Enable this macro to disable any logs from this module. */
#ifndef EAD_DISABLE_LOGS
#define EAD_LOG  app_trace_log  /**< Used for logging details. */
#define EAD_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define EAD_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define EAD_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //EAD_DISABLE_LOGS
#define EAD_DUMP(...)           /**< Disables dumping of octet streams. */
#define EAD_LOG(...)            /**< Disables detailed logs. */
#define EAD_ERR(...)            /**< Disables error logs. */
#define EAD_TRC(...)            /**< Disables traces. */
#endif 


#define AD7171_PWRUP_DELAY						APP_TIMER_TICKS(25, APP_TIMER_PRESCALER) /**< Power up delay(ticks),25ms. */
#define AD7171_SAMPLE_INTERVAL					APP_TIMER_TICKS(6, APP_TIMER_PRESCALER)	 /**< Sample interval(ticks),6ms. */
#define ACQUISITE_SAMPLE_CNT					5	//定义一次采集	ACQUISITE_SAMPLE_CNT 次采样		
#define SPI_TX_RX_MSG_LENGTH 					3

#define STABLE_WINDOW_AVERAGE_SIZE				2	//稳态窗口滑动大小
#define UNSTABLE_WINDOW_AVERAGE_SIZE			5	//非稳态窗口滑动大小
#define STABLE_DATA								1
#define UNSTABLE_DATA							2

#define BASE_K_VAL 				273.15				//开尔文温度基量
#define ET_B_VAL				3398.77046			//NTC特性B值
#define STANDARD_TEMP_RESISTANCE 32.222				//STANDARD_TEMP温度对应的特征阻值
#define STANDARD_TEMP				37				//特征温度
#define REFERENCE_RESISTANCE		39				//参考电阻
#define CAL_VAL						0				//校准电阻值

app_timer_id_t m_timer_ad7171_pwrup_delay;			//上电延迟定时器
app_timer_id_t m_timer_ad7171_sample_interval;		//采样间隔定时器
app_timer_id_t m_timer_ad7171_acquise_interval;		//采集间隔定时器

bool m_is_spi_on = false;

uint8_t m_ad7171_effective_sample_cnt = 0;			//有效采样计数
uint8_t m_ad7171_true_sample_cnt = 0;				//正确采样计数
uint32_t m_ad7171_acquisite_interval = APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER);		//采集间隔	

uint16_t buf_len = SPI_TX_RX_MSG_LENGTH;
uint8_t rx_buffer[SPI_TX_RX_MSG_LENGTH]; 				//Receive buffer to get data from SPI slave.
uint8_t tx_buffer[SPI_TX_RX_MSG_LENGTH]; 				//Transmit buffer to send data from SPI master with sample data.

uint8_t pwrup_timer_has_create = 0;
uint8_t acquisite_timer_has_create = 0;
static bool m_is_on_acquisite = false;

typedef struct _st_ad7171_readval
{
	uint16_t ad7171_cltval;

	union 
	{
		uint8_t statusval;
		struct 
		{
			uint8_t pat_flag : 3;
			uint8_t id_flag : 2;
			uint8_t err_flag : 1;
			uint8_t rsv : 1;
			uint8_t ready_flag : 1;
		}statusbs;
	}status;
}st_ad7171_readval_t;

st_ad7171_readval_t st_ad7171_readval;				//ad7171读取数据的结构
	
uint32_t g_sum_readcode = 0x00;						//采样ACQUISITE_SAMPLE_CNT得到的ADCode值
uint16_t g_once_readcode = 0x00;					//一次采集的ADCode值，为ACQUISITE_SAMPLE_CNT的平均			
uint16_t ad7171_readval_debug_buf[ACQUISITE_SAMPLE_CNT];	//调试看中间采集值

uint16_t g_average_window_size = 0x00;
typedef struct _st_average_window_t
{
	uint16_t cur_data_pos;							//当前数据索引位置
	uint16_t window_size;							//窗口大小
	uint8_t  data_type;								//数据类型
}st_average_window_t;
st_average_window_t g_st_average_window;			//滑动窗口参数记录
uint8_t average_para_has_init_flag = 0x00;
uint16_t g_once_window_readcode = 0x00;				//窗口滑动平均的AD读取的值
float g_window_resistance = 0.0;					//窗口滑动平均得到的电阻值
float g_window_temperature = 0.0;					//窗口滑动平均得到的温度值

uint16_t stable_array_buf[STABLE_WINDOW_AVERAGE_SIZE];		//稳态数据缓冲
uint16_t unstable_array_buf[UNSTABLE_WINDOW_AVERAGE_SIZE];	//非稳态数据缓冲

uint8_t m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP;	//温度处理的状态

spi_master_config_t g_st_SpiMasterConfig = {SPI_FREQUENCY_FREQUENCY_M1, AD7171_CLK_PIN_NUM, 
											AD7171_DOUT_PIN_NUM, SPI_PIN_DISCONNECTED, SPI_PIN_DISCONNECTED,
											APP_IRQ_PRIORITY_LOW, SPI_CONFIG_ORDER_MsbFirst, SPI_CONFIG_CPOL_ActiveLow,
											SPI_CONFIG_CPHA_Trailing};						//Spi Master配置结构

void spi_master_event_handler(spi_master_evt_t spi_master_evt);

void ad7171_sample_interval_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	if(0 == nrf_gpio_pin_read(AD7171_DOUT_PIN_NUM))
	{//数据有效,开始一次采样

		//Spi读取一次数据
		uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, tx_buffer, buf_len, rx_buffer, buf_len);

		APP_ERROR_CHECK(err_code);
				
		m_ad7171_effective_sample_cnt ++;

		if(m_ad7171_effective_sample_cnt == (ACQUISITE_SAMPLE_CNT + 1))
		{
			m_ad7171_effective_sample_cnt = 0;			//采样完指定次数的ad7171数据
		}
	}
}

void ad7171_pwrup_delay_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);
	//25ms上电延时后，开始一次采集ACQUISITE_SAMPLE_CNT次采样
	
	m_ad7171_effective_sample_cnt = 0;			//清零有效采样计数
	g_sum_readcode 	=	0x00;					//ACQUISITE_SAMPLE_CNT 读取的Ad采集的值清零
	
	uint32_t err_code;
	
	if(acquisite_timer_has_create == 0)
	{//仅创建一次定时器
		acquisite_timer_has_create = 1;
		err_code = app_timer_create(&m_timer_ad7171_sample_interval, APP_TIMER_MODE_REPEATED, ad7171_sample_interval_timeout_handler);
		APP_ERROR_CHECK(err_code);	
	}
	
	err_code = app_timer_start(m_timer_ad7171_sample_interval, AD7171_SAMPLE_INTERVAL, NULL);//开始ad7171的采样	
}

void ad7171_init(void)
{
	nrf_gpio_pin_clear(SENSE_PWR_CTRL_PIN_NUM);
	nrf_gpio_cfg_output(SENSE_PWR_CTRL_PIN_NUM);
	nrf_gpio_pin_clear(SENSE_PWR_CTRL_PIN_NUM);						//打开mosfet供电

	if(!m_is_spi_on)
	{
		spi_master_open(SPI_MASTER_0, &g_st_SpiMasterConfig);			//打开Spi Master

		spi_master_evt_handler_reg(SPI_MASTER_0, spi_master_event_handler);	//注册Spi处理回调

		m_is_spi_on = true;
	}
			
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
	nrf_gpio_cfg_output(AD7171_PD_PIN_NUM);							
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
	nrf_delay_us(10);
	nrf_gpio_pin_set(AD7171_PD_PIN_NUM);							//使能ad7171,禁止PowerDown

	uint32_t err_code;
	if(pwrup_timer_has_create == 0)
	{//仅创建一次定时器
		pwrup_timer_has_create = 1;
		
		err_code = app_timer_create(&m_timer_ad7171_pwrup_delay, APP_TIMER_MODE_SINGLE_SHOT, ad7171_pwrup_delay_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
	}

	err_code = app_timer_start(m_timer_ad7171_pwrup_delay, AD7171_PWRUP_DELAY, NULL);	//开始延时定时器
}

void ad7171_deinit(void)
{
	if(m_is_spi_on)
	{
		spi_master_close(SPI_MASTER_0); 							//关闭Spi Master

		m_is_spi_on = false;
	}
	
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);							//禁止ad7171,使能PowerDown
	nrf_gpio_pin_set(SENSE_PWR_CTRL_PIN_NUM);						//关闭mosfet供电


	nrf_gpio_cfg_output(AD7171_CLK_PIN_NUM);
	nrf_gpio_cfg_output(AD7171_DOUT_PIN_NUM);
	nrf_gpio_cfg_output(AD7171_PD_PIN_NUM);
	
	nrf_gpio_cfg_output(AD7171_PD_PIN_NUM);
	nrf_gpio_cfg_output(SENSE_PWR_CTRL_PIN_NUM);
	
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);						
	nrf_gpio_pin_set(SENSE_PWR_CTRL_PIN_NUM);					
	
	nrf_gpio_pin_clear(AD7171_CLK_PIN_NUM);
	nrf_gpio_pin_set(AD7171_DOUT_PIN_NUM);							//此处调试时需要注意,2014.08.25
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
	
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);							//禁止ad7171,使能PowerDown
	nrf_gpio_pin_set(SENSE_PWR_CTRL_PIN_NUM);						//关闭mosfet供电
	
}

void ad7171_acquise_interval_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	extern void bt_temp_glue_record_time(void);
	bt_temp_glue_record_time();
	ad7171_init();													//ad7171初始化采集一次
}

void calu_average(uint16_t* pval, uint16_t size, uint16_t* paverage)
{//注意pval不能为空，size不能为零，为节约时间不做入函数判断。
	uint32_t sum = 0;
	
	for(uint16_t i = 0;i < size;i ++)
	{
		sum += (*(pval + i));	
	}

	*paverage = (uint16_t)(sum / size);
}


float ftmpval_ln = 0.0;
float fDenominator = 0.0;
float fMolecular = 0.0;

void calu_temperature(uint16_t adcode, float* pTemperature)			//根据Adcode值换算温度值,用公式还是查表的方式，可以替换此函数
{
	g_window_resistance = (float)(((long)(adcode - 32768) * REFERENCE_RESISTANCE)) / 32768;

	ftmpval_ln = log(g_window_resistance / (STANDARD_TEMP_RESISTANCE + CAL_VAL));
	
	fDenominator = (float)(STANDARD_TEMP + BASE_K_VAL);
	
	fMolecular = (STANDARD_TEMP + BASE_K_VAL) * ftmpval_ln + ET_B_VAL;

	*pTemperature = (fDenominator / fMolecular) * ET_B_VAL - BASE_K_VAL;	
}

void convert_temperature(uint16_t ad_val)
{
	if(g_average_window_size == STABLE_WINDOW_AVERAGE_SIZE)
	{//稳态窗口滑动
		if(g_st_average_window.data_type != STABLE_DATA)
		{
			g_st_average_window.data_type = STABLE_DATA;
			g_st_average_window.cur_data_pos = 0x00;
			g_st_average_window.window_size = 0x00;
		}
		//计算出NTC电阻值和温度值
		if(g_st_average_window.window_size != STABLE_WINDOW_AVERAGE_SIZE)
		{//窗口未满，则有几个点平均几次
			stable_array_buf[g_st_average_window.cur_data_pos] = ad_val;
			g_st_average_window.cur_data_pos = (g_st_average_window.cur_data_pos + 1)%STABLE_WINDOW_AVERAGE_SIZE;
			g_st_average_window.window_size ++;

			calu_average(&stable_array_buf[0], g_st_average_window.window_size, &g_once_window_readcode);//计算滑动窗口AD读取值
		}
		else 
		{//窗口满了，则整个窗口平均
			stable_array_buf[g_st_average_window.cur_data_pos] = ad_val;
			g_st_average_window.cur_data_pos = (g_st_average_window.cur_data_pos + 1)%STABLE_WINDOW_AVERAGE_SIZE;

			calu_average(&stable_array_buf[0], g_st_average_window.window_size, &g_once_window_readcode);//计算滑动窗口AD读取值	
		}
	}
	else if(g_average_window_size == UNSTABLE_WINDOW_AVERAGE_SIZE)
	{//非稳态窗口滑动
		if(g_st_average_window.data_type != UNSTABLE_DATA)
		{
			g_st_average_window.data_type = UNSTABLE_DATA;
			g_st_average_window.cur_data_pos = 0x00;
			g_st_average_window.window_size	= 0x00;
		}
		if(g_st_average_window.window_size != UNSTABLE_WINDOW_AVERAGE_SIZE)
		{
			unstable_array_buf[g_st_average_window.cur_data_pos] = ad_val;
			g_st_average_window.cur_data_pos = (g_st_average_window.cur_data_pos + 1)%UNSTABLE_WINDOW_AVERAGE_SIZE;
			g_st_average_window.window_size ++;

			calu_average(&unstable_array_buf[0], g_st_average_window.window_size, &g_once_window_readcode);//计算滑动窗口AD读取值
		}	
		else
		{
			unstable_array_buf[g_st_average_window.cur_data_pos] = ad_val;
			g_st_average_window.cur_data_pos = (g_st_average_window.cur_data_pos + 1)%UNSTABLE_WINDOW_AVERAGE_SIZE;

			calu_average(&unstable_array_buf[0], g_st_average_window.window_size, &g_once_window_readcode);//计算滑动窗口AD读取值	
		}
	}
	else
	{

	}
}

void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
	switch (spi_master_evt.evt_type)
	{
		case SPI_MASTER_EVT_TRANSFER_COMPLETED:
			//Data transmission is ended successful. 'rx_buffer' has data received from SPI slave.
			//transmission_completed = true;
			if(m_ad7171_effective_sample_cnt != 0)
			{
				uint16_t tmp = 0x00;

				tmp = (rx_buffer[0] << 8) | rx_buffer[1];

				st_ad7171_readval.status.statusval = rx_buffer[2];
				st_ad7171_readval.ad7171_cltval = tmp;
				
				ad7171_readval_debug_buf[m_ad7171_effective_sample_cnt - 1] = tmp;

				if(st_ad7171_readval.status.statusbs.err_flag != 1)
				{//抛弃错误的采样值
					g_sum_readcode += tmp;	
					m_ad7171_true_sample_cnt ++;
				}
			}
			else
			{
				//计算一次采样的温度值			
				uint32_t err_code = app_timer_stop(m_timer_ad7171_sample_interval);	//关闭采样定时器

				APP_ERROR_CHECK(err_code);	

				if(m_ad7171_true_sample_cnt != 0)
				{//计算采样到正确计数个值的平均
					g_once_readcode = (uint16_t)(g_sum_readcode / m_ad7171_true_sample_cnt);	//获取平均值
					m_ad7171_true_sample_cnt = 0;
				}
				else
				{
					g_once_readcode = 0;
				}

				if(g_once_readcode != 0)
				{
					convert_temperature(g_once_readcode);
					
					calu_temperature(g_once_window_readcode, &g_window_temperature);	//计算经滑动窗口的温度值

					uint16_t temp_val_uint = 0x00;
					uint8_t  tx_cnt = 0x00;

					temp_val_uint = (uint16_t)(g_window_temperature * 100);

					if(temp_val_uint <= 3400)
					{
						m_temp_proc_state = USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP;
					}
					else 
					{
						static uint8_t clt_cnt = 0;
						clt_cnt ++;
						if(clt_cnt < 150 && m_temp_proc_state != USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP)
						{
							m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP;
						}
						else if(clt_cnt == 150)
						{
							m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP;						
						}
						else if(clt_cnt > 150 && clt_cnt < 210 && m_temp_proc_state != USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP)
						{
							m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP;
						}
						else if(clt_cnt == 210)
						{
							clt_cnt = 0;
							m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP;
						}
					}
					
					tx_cnt = (m_temp_proc_state == USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP) ? 1 : (m_temp_proc_state == USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP ? 1 : 6);
					extern void bt_temp_glue_add_once_temp(uint16_t add_val, uint8_t curr_tx_cnt);
					bt_temp_glue_add_once_temp(temp_val_uint, tx_cnt);					//将采集的温度值加入缓存

//					EAD_TRC("GET_SAMPLE_TEMP!\n");
				}
				
				ad7171_deinit();
			}
			
		break;
		default:
			//No implementation needed.
		break;
	}
}

void ad7171_para_init(void)
{
	if(average_para_has_init_flag == 0x00)
	{//初始化窗口滑动平均参数
		average_para_has_init_flag = 0x01;

		g_st_average_window.cur_data_pos = 0x00;
		g_st_average_window.window_size = 0x00;	
		g_st_average_window.data_type 	= STABLE_DATA;	//默认非稳态数据状态
		
		g_average_window_size = (g_st_average_window.data_type == UNSTABLE_DATA) ? UNSTABLE_WINDOW_AVERAGE_SIZE : STABLE_WINDOW_AVERAGE_SIZE;
	}
}

void ad7171_res_init(void)
{
	ad7171_deinit();
	
	uint32_t err_code = app_timer_create(&m_timer_ad7171_acquise_interval, APP_TIMER_MODE_REPEATED, ad7171_acquise_interval_timeout_handler);
	APP_ERROR_CHECK(err_code);										//创建采集定时器
}

void ad7171_start_acquisite(uint32_t acquisite_interval,uint8_t ad_mode)
{
	if(m_is_on_acquisite == false)
	{
		m_is_on_acquisite = true;
	
		m_ad7171_acquisite_interval = acquisite_interval;
	
		uint32_t err_code = app_timer_start(m_timer_ad7171_acquise_interval, m_ad7171_acquisite_interval, NULL);
		APP_ERROR_CHECK(err_code);										//开始采集定时器

		EAD_TRC("[EAD_TRC]: TRC: %s\r\n", "EAD_ACQ_START...");

	}
}

void ad7171_stop_acquisite(void)
{
	if(m_is_on_acquisite == true)
	{
		m_is_on_acquisite = false;
	
		m_ad7171_acquisite_interval = 0x00;
	
		uint32_t err_code = app_timer_stop(m_timer_ad7171_acquise_interval); //停止采集定时器
		APP_ERROR_CHECK(err_code);

		EAD_TRC("[EAD_TRC]: TRC: %s\r\n", "EAD_ACQ_STOP!");
	}
}

uint32_t ad7171_acquisite_interval_get(void)
{
	return m_ad7171_acquisite_interval;	
}

uint8_t ad7171_get_cur_temp_state(void)
{
	return m_temp_proc_state;
}
#endif

#ifdef PP_SC_PHY
//#define EAD_DISABLE_LOGS        /**< Enable this macro to disable any logs from this module. */
#ifndef EAD_DISABLE_LOGS
#define EAD_LOG  app_trace_log  /**< Used for logging details. */
#define EAD_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define EAD_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define EAD_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //EAD_DISABLE_LOGS
#define EAD_DUMP(...)           /**< Disables dumping of octet streams. */
#define EAD_LOG(...)            /**< Disables detailed logs. */
#define EAD_ERR(...)            /**< Disables error logs. */
#define EAD_TRC(...)            /**< Disables traces. */
#endif 

#define AD7171_SWITCH_EN						nrf_gpio_cfg_output(NTC_SWITCH_PIN_NUM)
#define AD7171_SWITCH_DIS						nrf_gpio_cfg_input(NTC_SWITCH_PIN_NUM, NRF_GPIO_PIN_PULLDOWN)
#define AD7171_LEFT_SWITCH						nrf_gpio_pin_clear(NTC_SWITCH_PIN_NUM)		//FSA4157 A->B0
#define AD7171_RIGHT_SWITCH						nrf_gpio_pin_set(NTC_SWITCH_PIN_NUM)		//FSA4157 A->B1
//#define AD7171_LEFT_SWITCH						nrf_gpio_pin_set(NTC_SWITCH_PIN_NUM)		//FSA4157 A->B0
//#define AD7171_RIGHT_SWITCH						nrf_gpio_pin_set(NTC_SWITCH_PIN_NUM)		//FSA4157 A->B1

#define AD7171_PWRUP_DELAY						APP_TIMER_TICKS(60, APP_TIMER_PRESCALER) /**< Power up delay(ticks),40ms. */
#define AD7171_SAMPLE_INTERVAL					APP_TIMER_TICKS(6, APP_TIMER_PRESCALER)	 /**< Sample interval(ticks),6ms. */

#define NORMAL_MODE_ADLEFT_ACQUISITE_CNT		5
#define NORMAL_MODE_ADRIGHT_ACQUISITE_CNT		5
#define NORMAL_MODE_ACQUISITE_SAMPLE_CNT		(NORMAL_MODE_ADLEFT_ACQUISITE_CNT + NORMAL_MODE_ADRIGHT_ACQUISITE_CNT)	//定义一次采集	ACQUISITE_SAMPLE_CNT 次采样		

#define CALI_MODE_ADLEFT_ACQUISITE_SAMPLE_CNT	15
#define CALI_MODE_ADRIGHT_ACQUISITE_SAMPLE_CNT	15
#define CALI_MODE_ACQUISITE_SAMPLE_CNT			(CALI_MODE_ADLEFT_ACQUISITE_SAMPLE_CNT + CALI_MODE_ADRIGHT_ACQUISITE_SAMPLE_CNT)

#define SPI_TX_RX_MSG_LENGTH 					3	//SPI一次传输的字节数

#define STABLE_WINDOW_AVERAGE_SIZE				1	//稳态窗口滑动大小
#define UNSTABLE_WINDOW_AVERAGE_SIZE			1	//非稳态窗口滑动大小
#define STABLE_DATA								1
#define UNSTABLE_DATA							2

#define BASE_K_VAL 				273.15				//开尔文温度基量
#define ET_B_VAL				3944				//NTC特性B值

//#define STANDARD_TEMP_RESISTANCE 29.910				//STANDARD_TEMP温度对应的特征阻值,H档
#define STANDARD_TEMP_RESISTANCE 29.964				//STANDARD_TEMP温度对应的特征阻值,I档
//#define STANDARD_TEMP_RESISTANCE 30.018				//STANDARD_TEMP温度对应的特征阻值,J档

#define STANDARD_TEMP				37				//特征温度
#define REFERENCE_RESISTANCE		39				//参考电阻
#define CAL_VAL						0				//校准电阻值

enum 
{
	AD7171_CLT_NULL,
	AD7171_CLT_NTC1P,
	AD7171_CLT_NTC2P,
	AD7171_CLT_ON_ME
};

app_timer_id_t m_timer_ad7171_pwrup_delay;			//上电延迟定时器
app_timer_id_t m_timer_ad7171_sample_interval;		//采样间隔定时器
app_timer_id_t m_timer_ad7171_acquise_interval;		//采集间隔定时器

bool m_is_spi_on = false;

uint8_t m_ad7171_effective_sample_cnt = 0;			//有效采样计数
uint32_t m_ad7171_acquisite_interval = APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER);		//采集间隔	

uint16_t buf_len = SPI_TX_RX_MSG_LENGTH;
uint8_t rx_buffer[SPI_TX_RX_MSG_LENGTH]; 				//Receive buffer to get data from SPI slave.
uint8_t tx_buffer[SPI_TX_RX_MSG_LENGTH]; 				//Transmit buffer to send data from SPI master with sample data.

uint8_t pwrup_timer_has_create = 0;
uint8_t acquisite_timer_has_create = 0;
static bool m_is_on_acquisite = false;

typedef struct _st_ad7171_readval
{
	uint16_t ad7171_cltval;

	union 
	{
		uint8_t statusval;
		struct 
		{
			uint8_t pat_flag : 3;
			uint8_t id_flag : 2;
			uint8_t err_flag : 1;
			uint8_t rsv : 1;
			uint8_t ready_flag : 1;
		}statusbs;
	}status;
}st_ad7171_readval_t;

st_ad7171_readval_t st_ad7171_readval;				//ad7171读取数据的结构
	
uint16_t ad7171_readval_debug_buf[CALI_MODE_ACQUISITE_SAMPLE_CNT];//调试看中间采集值

typedef struct _st_average_window_t
{
	uint16_t cur_data_pos;							//当前数据索引位置
	uint16_t window_size;							//窗口大小
	uint8_t  data_type;								//数据类型
}st_average_window_t;
static st_average_window_t m_st_ntcleft_average_window;			
static st_average_window_t m_st_ntc_right_average_window;

uint8_t average_para_has_init_flag = 0x00;

typedef struct
{
	uint8_t  cur_ad_pos_on_me;						//当前ad在此NTC上采集
	uint32_t sum_readcode;							//ad读取code值之和
	uint16_t sum_average_readcode;					//ad读取code值之和平均
	uint16_t window_average_readcode;				//ad读取code值窗口滑动平均
	float window_average_resistance;				//窗口滑动平均换算的电阻值
	float window_average_temperature;				//窗口滑动平均换算的温度值
	uint16_t uiWindow_average_temperature;			//窗口滑动平均查表换算得到的温度值
	float ftempval_ln;
	float fDenominator;
	float fMolecular;
	uint16_t window_average_size;					//窗口平均窗口长度
	st_average_window_t* pst_average_window;		//滑动窗口参数记录
	uint16_t stable_array_buf[STABLE_WINDOW_AVERAGE_SIZE];		//稳态数据缓冲
	uint16_t unstable_array_buf[UNSTABLE_WINDOW_AVERAGE_SIZE];	//非稳态数据缓冲
	uint16_t offset_readcode;
}st_ad7171_proc;

st_ad7171_proc st_ad7171_left_proc;					//左边AD7171处理结构数据
st_ad7171_proc st_ad7171_right_proc;				//右边AD7171处理结构数据


uint8_t m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP;	//温度处理的状态

static uint8_t m_ad7171_mode = 0x00;

spi_master_config_t g_st_SpiMasterConfig = {SPI_FREQUENCY_FREQUENCY_M1, AD7171_CLK_PIN_NUM, 
											AD7171_DOUT_PIN_NUM, SPI_PIN_DISCONNECTED, SPI_PIN_DISCONNECTED,
											APP_IRQ_PRIORITY_LOW, SPI_CONFIG_ORDER_MsbFirst, SPI_CONFIG_CPOL_ActiveLow,
											SPI_CONFIG_CPHA_Trailing};						//Spi Master配置结构

void spi_master_event_handler(spi_master_evt_t spi_master_evt);
static void ad7171_sample_interval_timeout_handler(void *p_context);

static void ad7171_pwrup_delay_timeout_handler(void *p_context)
{
	//25ms上电延时后，开始一次采集ACQUISITE_SAMPLE_CNT次采样
	if(p_context == NULL)
	{
		EAD_ERR("[EAD]: Null Pointer Error!\r\n");
	}
		
	uint32_t err_code;
	
	if(acquisite_timer_has_create == 0)
	{//仅创建一次定时器
		acquisite_timer_has_create = 1;
		err_code = app_timer_create(&m_timer_ad7171_sample_interval, APP_TIMER_MODE_REPEATED, ad7171_sample_interval_timeout_handler);
		APP_ERROR_CHECK(err_code);	
	}
	
	err_code = app_timer_start(m_timer_ad7171_sample_interval, AD7171_SAMPLE_INTERVAL, p_context);//开始ad7171的采样	
}

static void ad7171_init(void *p_context, uint8_t ad_pos)
{
	if(p_context == NULL)
	{
		EAD_ERR("[EAD]: Null Pointer Error!\r\n");
	}

	nrf_gpio_pin_clear(SENSE_PWR_CTRL_PIN_NUM);						
	nrf_gpio_cfg_output(SENSE_PWR_CTRL_PIN_NUM);
	nrf_gpio_pin_clear(SENSE_PWR_CTRL_PIN_NUM);						//打开mosfet供电

	if(!m_is_spi_on)
	{
		spi_master_open(SPI_MASTER_0, &g_st_SpiMasterConfig);		//打开Spi Master

		spi_master_evt_handler_reg(SPI_MASTER_0, spi_master_event_handler);	//注册Spi处理回调

		m_is_spi_on = true;
	}
			
	nrf_gpio_cfg_output(AD7171_PD_PIN_NUM);							
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
	nrf_delay_us(10);
	nrf_gpio_pin_set(AD7171_PD_PIN_NUM);							//使能ad7171,禁止PowerDown

	AD7171_SWITCH_EN;
	if(ad_pos == AD7171_CLT_NTC1P)
	{
		AD7171_LEFT_SWITCH; 										//默认先采集左边NTC				
	}
	else if(ad_pos == AD7171_CLT_NTC2P)
	{
		AD7171_RIGHT_SWITCH;
	}
	else
	{
		EAD_ERR("[EAD]: Ad Collect Mode Error");
	}

	uint32_t err_code;
	if(pwrup_timer_has_create == 0)
	{//仅创建一次定时器
		pwrup_timer_has_create = 1;
		
		err_code = app_timer_create(&m_timer_ad7171_pwrup_delay, APP_TIMER_MODE_SINGLE_SHOT, ad7171_pwrup_delay_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
	}

	err_code = app_timer_start(m_timer_ad7171_pwrup_delay, AD7171_PWRUP_DELAY, p_context);	//开始延时定时器
}

void ad7171_close_analog_power(void)
{
	nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
	nrf_gpio_pin_set(SENSE_PWR_CTRL_PIN_NUM);						//关闭mosfet供电
}

static void ad7171_deinit(uint8_t ad_pos, bool is_close_power)
{
	if(m_is_spi_on)
	{
		spi_master_close(SPI_MASTER_0); 							//关闭Spi Master

		m_is_spi_on = false;
	}

	if(ad_pos == AD7171_CLT_NTC2P)
	{
		st_ad7171_right_proc.sum_readcode = 0;
		st_ad7171_left_proc.sum_readcode = 0;
	}
		
	nrf_gpio_cfg_output(AD7171_DOUT_PIN_NUM);
	nrf_gpio_pin_clear(AD7171_DOUT_PIN_NUM);
	
	nrf_gpio_cfg_input(AD7171_DOUT_PIN_NUM, NRF_GPIO_PIN_PULLDOWN);
	
	nrf_gpio_cfg_output(AD7171_CLK_PIN_NUM);
	nrf_gpio_cfg_output(AD7171_PD_PIN_NUM);
	nrf_gpio_cfg_output(SENSE_PWR_CTRL_PIN_NUM);

	nrf_gpio_pin_clear(AD7171_CLK_PIN_NUM);

	if(is_close_power == true)
	{
		nrf_gpio_pin_clear(AD7171_PD_PIN_NUM);
		nrf_gpio_pin_set(SENSE_PWR_CTRL_PIN_NUM);						//关闭mosfet供电
	}
		
	AD7171_SWITCH_DIS;	
}

static void ad7171_sample_interval_timeout_handler(void *p_context)
{
	uint8_t ad_mode = 0x00;
	uint8_t acquiste_cnt = 0x00;

	ad_mode = *(uint8_t*)p_context;

	if(0 == nrf_gpio_pin_read(AD7171_DOUT_PIN_NUM))
	{//数据有效,开始一次采样

		//Spi读取一次数据
		uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, tx_buffer, buf_len, rx_buffer, buf_len);

		APP_ERROR_CHECK(err_code);
				
		m_ad7171_effective_sample_cnt ++;

		if(ad_mode == EN_AD7171_NORMAL_CLT_MODE)
		{
			acquiste_cnt = NORMAL_MODE_ACQUISITE_SAMPLE_CNT;
		}
		else if(ad_mode == EN_AD7171_CALI_CONSTANT_RES_MODE)
		{
			acquiste_cnt = CALI_MODE_ACQUISITE_SAMPLE_CNT;
		}
		else
		{
			EAD_ERR("[EAD]: External ADC Mode Error!\r\n");
		}
		if(m_ad7171_effective_sample_cnt == (acquiste_cnt + 2))
		{
			m_ad7171_effective_sample_cnt = 0;			//采样完指定次数的ad7171数据
		}
		if(m_ad7171_effective_sample_cnt == ((m_ad7171_mode == EN_AD7171_NORMAL_CLT_MODE ? (NORMAL_MODE_ADLEFT_ACQUISITE_CNT) : (CALI_MODE_ADLEFT_ACQUISITE_SAMPLE_CNT)) + 1))
		{
			APP_ERROR_CHECK(app_timer_stop(m_timer_ad7171_sample_interval));	//关闭采样定时器	
			ad7171_deinit(AD7171_CLT_NTC1P, true);
			ad7171_init(p_context, AD7171_CLT_NTC2P);	//采集NTC2
		}
	}
}

static void ad7171_acquise_interval_timeout_handler(void *p_context)
{
	if(p_context == NULL)
	{
		EAD_ERR("[EAD]: Null Pointer Error!\r\n");
	}

	bt_temp_glue_record_time();							//每次采集记录下当前时间

	ad7171_init(p_context, AD7171_CLT_NTC1P);
}

void calu_average(uint16_t* pval, uint16_t size, uint16_t* paverage)
{//注意pval不能为空，size不能为零，为节约时间不做入函数判断。
	uint32_t sum = 0;
	
	for(uint16_t i = 0;i < size;i ++)
	{
		sum += (*(pval + i));	
	}

	*paverage = (uint16_t)(sum / size);
}

/**@brief Function for binary search close target num.
 *
 * @details 
 *
 * @param[in]  pindex:目标数在指定数组中最接近的位置
 *			   pbuf : 待搜索的缓冲区,待搜索的缓冲区长度不能超过256
 *			   tosearch: 搜索的目标值
 *                    
 */
static void binary_find_close(uint16_t* pindex, uint16_t* pbuf, uint16_t len,uint16_t tosearch)		
{
	uint16_t low = 0;
	uint16_t high = len - 1;
	uint16_t idx = 0;

	while(low <= high)
	{
		uint16_t mid = 0;
		mid = (low + high) / 2;

		if((tosearch == *(pbuf + mid)) || (low == mid))
		{
			idx = mid;
			break;
		}
		else if(tosearch < *(pbuf + mid))
		{
			low = mid;
		}
		else if(tosearch > *(pbuf + mid))
		{
			high = mid;
		}
	}

	*pindex = idx;	
}

int offset = 0x00;
int delt = 0x00;
uint16_t search_index = 0x00;

uint8_t calu_temperature(st_ad7171_proc* p_st_ad_proc)					//根据Adcode值换算温度值,用公式还是查表的方式，可以替换此函数
{
	uint8_t tempType = 0x00;
	
	//公式法
//	p_st_ad_proc->window_average_resistance = (float)(((long)(p_st_ad_proc->window_average_readcode	- 32768) * REFERENCE_RESISTANCE)) / 32768;

//	p_st_ad_proc->ftempval_ln = log(p_st_ad_proc->window_average_resistance / (STANDARD_TEMP_RESISTANCE + CAL_VAL));

//	p_st_ad_proc->fDenominator = (float)(STANDARD_TEMP + BASE_K_VAL);

//	p_st_ad_proc->fMolecular = (STANDARD_TEMP + BASE_K_VAL) * p_st_ad_proc->ftempval_ln + ET_B_VAL;

//	p_st_ad_proc->window_average_temperature = (p_st_ad_proc->fDenominator / p_st_ad_proc->fMolecular) * ET_B_VAL - BASE_K_VAL;

	//查表法
	
	p_st_ad_proc->window_average_resistance = (float)(((long)(p_st_ad_proc->window_average_readcode	- 32768) * REFERENCE_RESISTANCE)) / 32768;
	
	if(p_st_ad_proc->window_average_readcode > ExternalNTC_MidAccuracy_AdCode[0])
	{//<32℃,两个NTC都没有温度采集时使用板载内部ADC进行温度的测量
		tempType = 0x01;
		p_st_ad_proc->uiWindow_average_temperature = 3100;
	}
	else if(p_st_ad_proc->window_average_readcode > ExternalNTC_HighAccuracy_AdCode[0])
	{//32~34℃之间
		
		binary_find_close(&search_index, (uint16_t*)&ExternalNTC_MidAccuracy_AdCode[0], (uint16_t)sizeof(ExternalNTC_MidAccuracy_AdCode), p_st_ad_proc->window_average_readcode);

		offset = (int)ExternalNTC_MidAccuracy_AdCode[search_index] - (int)p_st_ad_proc->window_average_readcode;

		delt = (int)((ExternalNTC_MidAccuracy_AdCode[search_index] - ExternalNTC_MidAccuracy_AdCode[search_index + 1]) / 5);
					
		offset = (int)(offset / delt);													//将差值细分,线性插值

		p_st_ad_proc->uiWindow_average_temperature = 3200 + search_index * 5 + offset;	//达到最终温度		
	}
	else if(p_st_ad_proc->window_average_readcode > ExternalNTC_LowAccuracy_AdCode[0])
	{//34~40℃之间
		binary_find_close(&search_index, (uint16_t*)&ExternalNTC_HighAccuracy_AdCode[0], (uint16_t)sizeof(ExternalNTC_HighAccuracy_AdCode), p_st_ad_proc->window_average_readcode);
		p_st_ad_proc->uiWindow_average_temperature = 3400 + search_index;		
	}
	else if(p_st_ad_proc->window_average_readcode >= ExternalNTC_LowAccuracy_AdCode[(sizeof(ExternalNTC_LowAccuracy_AdCode) - 1)])
	{//40~60℃之间
		binary_find_close(&search_index, (uint16_t*)&ExternalNTC_LowAccuracy_AdCode[0], (uint16_t)sizeof(ExternalNTC_LowAccuracy_AdCode), p_st_ad_proc->window_average_readcode);
		offset =(int)ExternalNTC_LowAccuracy_AdCode[search_index] - (int)p_st_ad_proc->window_average_readcode;

		delt = (int)((ExternalNTC_LowAccuracy_AdCode[search_index] - ExternalNTC_LowAccuracy_AdCode[search_index + 1]) / 10);
					
		offset = (int)(offset / delt);													//将差值细分,线性插值

		p_st_ad_proc->uiWindow_average_temperature = 4000 + search_index * 10 + offset;	//达到最终温度		
	}
	else
	{
		p_st_ad_proc->uiWindow_average_temperature = 6000;
	}
	
	return tempType;	
}

uint32_t average_window_adcode(st_ad7171_proc* p_st_ad_proc)
{
	uint8_t window_average_size = 0x00;
	uint16_t* pbuf = NULL;

	if(p_st_ad_proc->window_average_size == STABLE_WINDOW_AVERAGE_SIZE)
	{
		if(p_st_ad_proc->pst_average_window->data_type != STABLE_DATA)
		{
			p_st_ad_proc->pst_average_window->data_type = STABLE_DATA;
			p_st_ad_proc->pst_average_window->cur_data_pos = 0x00;
			p_st_ad_proc->pst_average_window->window_size = 0x00;
		}
  		if(p_st_ad_proc->pst_average_window->window_size != STABLE_WINDOW_AVERAGE_SIZE)
		{//窗口未满，则有几个点平均几次
			p_st_ad_proc->pst_average_window->window_size ++;
		}
		window_average_size = STABLE_WINDOW_AVERAGE_SIZE;
		pbuf = &p_st_ad_proc->stable_array_buf[0];
	}
	else if(p_st_ad_proc->window_average_size == UNSTABLE_WINDOW_AVERAGE_SIZE)
	{
		if(p_st_ad_proc->pst_average_window->data_type != UNSTABLE_DATA)\
		{
			p_st_ad_proc->pst_average_window->data_type = UNSTABLE_DATA;
			p_st_ad_proc->pst_average_window->cur_data_pos = 0x00;
			p_st_ad_proc->pst_average_window->window_size = 0x00;
		}
		if(p_st_ad_proc->pst_average_window->window_size != UNSTABLE_WINDOW_AVERAGE_SIZE)
		{
			p_st_ad_proc->pst_average_window->window_size ++;
		}
		window_average_size = UNSTABLE_WINDOW_AVERAGE_SIZE;
		pbuf = &p_st_ad_proc->unstable_array_buf[0];
	}
	else
	{
		return NRF_ERROR_INVALID_LENGTH;
	}

	pbuf[p_st_ad_proc->pst_average_window->cur_data_pos] = p_st_ad_proc->sum_average_readcode;
	p_st_ad_proc->pst_average_window->cur_data_pos = (p_st_ad_proc->pst_average_window->cur_data_pos + 1)%window_average_size;
	calu_average(&pbuf[0], p_st_ad_proc->pst_average_window->window_size, &p_st_ad_proc->window_average_readcode);			//计算窗口滑动平均值

	return NRF_SUCCESS;
}

void EAD_Trace_AdValue(uint16_t left, uint16_t right)
{
	char str[6];
	itoa(left, &str[0]);
	EAD_TRC("%s  ", str);
	itoa(right, &str[0]);
	EAD_TRC("%s  \r\n", str);	
}

uint16_t clt_cnt = 0;
void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
	switch (spi_master_evt.evt_type)
	{
		case SPI_MASTER_EVT_TRANSFER_COMPLETED:
			//Data transmission is ended successful. 'rx_buffer' has data received from SPI slave.
			//transmission_completed = true;
			if(m_ad7171_effective_sample_cnt != 0)
			{
				uint16_t tmp = 0x00;

				tmp = (rx_buffer[0] << 8) | rx_buffer[1];

				st_ad7171_readval.status.statusval = rx_buffer[2];
				st_ad7171_readval.ad7171_cltval = tmp;
				
				ad7171_readval_debug_buf[m_ad7171_effective_sample_cnt - 1] = tmp;

				if(m_ad7171_mode == EN_AD7171_NORMAL_CLT_MODE)
				{
					if(m_ad7171_effective_sample_cnt <= NORMAL_MODE_ADLEFT_ACQUISITE_CNT)
					{
						st_ad7171_left_proc.sum_readcode += (tmp);
					}						
					else
					{
						st_ad7171_right_proc.sum_readcode += (tmp);
					}
				}
				else if(m_ad7171_mode == EN_AD7171_CALI_CONSTANT_RES_MODE)
				{
					if(m_ad7171_effective_sample_cnt <= CALI_MODE_ADLEFT_ACQUISITE_SAMPLE_CNT)
					{
						st_ad7171_left_proc.sum_readcode += tmp;
					}
					else
					{
						st_ad7171_right_proc.sum_readcode += tmp;
					}
				}
				else 
				{
					EAD_ERR("[EAD]: External ADC Mode Error!\r\n");
				}
			}
			else
			{
				//计算一次采样的温度值			
				uint32_t err_code = app_timer_stop(m_timer_ad7171_sample_interval);	//关闭采样定时器

				APP_ERROR_CHECK(err_code);	

				//换算AD采集得到的数字量为温度
				uint8_t left_cnt = 0;
				uint8_t right_cnt = 0;

				left_cnt = ((m_ad7171_mode == EN_AD7171_NORMAL_CLT_MODE) ? NORMAL_MODE_ADLEFT_ACQUISITE_CNT: CALI_MODE_ADLEFT_ACQUISITE_SAMPLE_CNT);
				right_cnt = ((m_ad7171_mode == EN_AD7171_NORMAL_CLT_MODE)?  NORMAL_MODE_ADRIGHT_ACQUISITE_CNT: CALI_MODE_ADRIGHT_ACQUISITE_SAMPLE_CNT);
				
				st_ad7171_left_proc.sum_average_readcode = (uint16_t)(st_ad7171_left_proc.sum_readcode / left_cnt);
				st_ad7171_right_proc.sum_average_readcode = (uint16_t)(st_ad7171_right_proc.sum_readcode / right_cnt);
				
				average_window_adcode(&st_ad7171_left_proc);
				average_window_adcode(&st_ad7171_right_proc);						//窗口滑动计算adcode值

				if(st_ad7171_left_proc.window_average_readcode != 0xFFFF)
				{
					st_ad7171_left_proc.window_average_readcode += 29;				//测试时手动将固定电阻的差值补上，实际时需要经过校准过程
				}
				if(st_ad7171_right_proc.window_average_readcode != 0xFFFF)
				{
					st_ad7171_right_proc.window_average_readcode += 29;
				}
				
				uint8_t tmp_flag;
				tmp_flag = 0;

				bool close_power_flag = true;										//指示是否关闭电源
				
				if((0x01 == calu_temperature(&st_ad7171_left_proc))) //计算经滑动窗口adcode对应温度值
				{//两个NTC都是无效的采集值时,启动板载NTC内部ADC采集
					tmp_flag = 1;						
				}
				
				if((0x01 == calu_temperature(&st_ad7171_right_proc)) && (tmp_flag == 1))
				{
					adc_inter_board_measure_start(bt_temp_glue_get_record_time());	//开始板载NTC内部ADC采集
					m_temp_proc_state = USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP;
					close_power_flag = false;										//不要关闭电源
				}
				else
				{
					uint16_t temp_left_val_uint = 0x00;
					uint16_t temp_right_val_uint = 0x00;
					uint8_t  tx_cnt = 0x00;

					//公式法使用
	//				temp_left_val_uint = (uint16_t)(st_ad7171_left_proc.window_average_temperature * 100);
	//				temp_right_val_uint = (uint16_t)(st_ad7171_right_proc.window_average_temperature * 100);

					//查表法适用
					temp_left_val_uint = st_ad7171_left_proc.uiWindow_average_temperature;
					temp_right_val_uint = st_ad7171_right_proc.uiWindow_average_temperature;

	//				EAD_Trace_AdValue(temp_left_val_uint, temp_right_val_uint);			//打印两个NTC经换算得到的温度值
					
					if(temp_left_val_uint <= 3450)
					{//32~34.5℃为实时非稳态数据
						m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP;
					}
					else if(temp_left_val_uint <= 4300)
					{//34.5~43℃为实时稳态数据
						m_temp_proc_state = USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP;
					}
					else if(temp_left_val_uint <= 6000)
					{//43~60℃为无效数据
						m_temp_proc_state = USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP;
					}
					
//					tx_cnt = (m_temp_proc_state == USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP) ? 1 : (m_temp_proc_state == USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP ? 1 : 6);
					tx_cnt = 1;
//					bt_temp_glue_add_once_temp(temp_left_val_uint, tx_cnt);					//将采集的温度值加入缓存
					bt_temp_glue_add_once_temp(temp_right_val_uint, tx_cnt);
				}

				//AD解除初始化	
				ad7171_deinit(AD7171_CLT_NTC2P, close_power_flag);
			}
			
		break;
		default:
			//No implementation needed.
		break;
	}
}

void ad7171_para_init(void)
{
	if(average_para_has_init_flag == 0x00)
	{//初始化窗口滑动平均参数
		average_para_has_init_flag = 0x01;

		st_ad7171_left_proc.pst_average_window = &m_st_ntcleft_average_window;
		st_ad7171_right_proc.pst_average_window = &m_st_ntc_right_average_window;

		st_ad7171_left_proc.pst_average_window->cur_data_pos = 0x00;
		st_ad7171_left_proc.pst_average_window->data_type = UNSTABLE_DATA; //默认非稳态数据状态
		st_ad7171_left_proc.pst_average_window->window_size = 0x00;

		st_ad7171_right_proc.pst_average_window->cur_data_pos = 0x00;
		st_ad7171_right_proc.pst_average_window->data_type = UNSTABLE_DATA;
		st_ad7171_right_proc.pst_average_window->window_size = 0x00;
		
		st_ad7171_left_proc.window_average_size = (st_ad7171_left_proc.pst_average_window->data_type == UNSTABLE_DATA) ? UNSTABLE_WINDOW_AVERAGE_SIZE : STABLE_WINDOW_AVERAGE_SIZE;
		st_ad7171_right_proc.window_average_size = (st_ad7171_right_proc.pst_average_window->data_type == UNSTABLE_DATA) ? UNSTABLE_WINDOW_AVERAGE_SIZE : STABLE_WINDOW_AVERAGE_SIZE;
	}
}

void ad7171_res_init(void)
{
	ad7171_deinit(AD7171_CLT_NTC1P, true);
	
	uint32_t err_code = app_timer_create(&m_timer_ad7171_acquise_interval, APP_TIMER_MODE_REPEATED, ad7171_acquise_interval_timeout_handler);
	APP_ERROR_CHECK(err_code);										//创建采集定时器
}

void ad7171_start_acquisite(uint32_t acquisite_interval,uint8_t* ad_mode)
{
	if(m_is_on_acquisite == false)
	{
		m_is_on_acquisite = true;

		m_ad7171_mode = *ad_mode;
	
		m_ad7171_acquisite_interval = acquisite_interval;
	
		uint32_t err_code = app_timer_start(m_timer_ad7171_acquise_interval, m_ad7171_acquisite_interval, (void*)ad_mode);
		APP_ERROR_CHECK(err_code);										//开始采集定时器

		EAD_TRC("[EAD]: TRC: %s\r\n", "EAD_ACQ_START...");
	}
}

void ad7171_stop_acquisite(void)
{
	if(m_is_on_acquisite == true)
	{
		m_is_on_acquisite = false;
	
		m_ad7171_acquisite_interval = 0x00;
	
		uint32_t err_code = app_timer_stop(m_timer_ad7171_acquise_interval); //停止采集定时器
		APP_ERROR_CHECK(err_code);

		EAD_TRC("[EAD]: TRC: %s\r\n", "EAD_ACQ_STOP!");
	}
}

uint32_t ad7171_acquisite_interval_get(void)
{
	return m_ad7171_acquisite_interval;	
}

uint8_t ad7171_get_cur_temp_state(void)
{
	return m_temp_proc_state;
}


#endif

