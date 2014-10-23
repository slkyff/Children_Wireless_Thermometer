
#include "user_drv_console.h"
#include "app_uart.h"
#include "app_timer.h"
#include <string.h>
#include "app_trace.h"
#include "sc_ht_common_def.h"
#include "sc_ht_perpin_def.h"
#include "user_drv_clock.h"

#include "user_drv_offline_storage.h"

#define USER_CONSOLE_BUFSIZE		16
#define USER_CONSOLE_TX_BUFSIZE   	44
#define COMM_PKT_CMD				1
#define COMM_PKT_DATA				2

#define COMM_TIME_SET_REQUEST       0x01
#define COMM_UART_ENABLE			0x08
#define COMM_UART_DISABLE			0x09
#define COMM_FACTORY_MODE_SET_REQUEST    0x0A						//串口控制台设置单项设置功能字段的命令值
#define COMM_FACTORY_MODE_GET_ALLINFO_REQUEST   0x0B				//串口控制台设置所有设置功能字段的命令值
#define COMM_FACTORY_MODE_LEAVE_FACTORY_MODE_REQUEST 0x0C			//串口控制台离开工厂模式命令值
#define COMM_FACTORY_MODE_SET_DEVICE_NAME_REQUEST 0x0D				//串口控制台设置设备名称命令值
#define COMM_FACTORY_MODE_AD1_CAIL_MODE_REQUEST 0x0E				//串口控制台AD1校准命令值
#define COMM_FACTORY_MODE_AD2_CAIL_MODE_REQUEST 0x0F				//串口控制台AD2校准命令值
#define COMM_FACTORY_MODE_GET_ALLINFO_RESPONSE  0x20				//获取所有厂商信息请求
#define COMM_FACTORY_MODE_HEARTBEAT_RESPONSE	0x10				//心跳请求
#define COMM_FACTORY_MODE_AD1_CAIL_MODE_RESPONSE 0x30				//AD1校准回馈值
#define COMM_FACTORY_MODE_AD2_CAIL_MODE_RESPONSE 0x31				//AD2校准回馈值

#define BEAT_FACTORY_MODE_TICK		APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

uint8_t user_console_txbuf[USER_CONSOLE_TX_BUFSIZE];
uint8_t user_console_rxbuf[USER_CONSOLE_BUFSIZE];
uint8_t* p_ind = &user_console_rxbuf[0];
static uint8_t m_console_uart_runing = false;

app_timer_id_t m_factory_mode_beat_timer;
	
static uint8_t is_uart_enable_flag = 0x02;

void user_console_res_deinit(void);

uint8_t user_console_get_flag(void)
{
	return is_uart_enable_flag;
}

static void user_console_set_flag(uint8_t flag)
{
	is_uart_enable_flag = flag;
}

app_uart_comm_params_t user_console_comm_params = {RX_PIN_NO,
												   TX_PIN_NO,
												   RTS_PIN_NO,
												   CTS_PIN_NO,
												   APP_UART_FLOW_CONTROL_DISABLED,
												   false,
												   38400,
												  };
static void user_console_up_data(uint8_t op_code, uint8_t cmd_code)
{
	uint8_t read_buf[FACTORY_INFO_REGION_SIZE];
	uint8_t read_cnt;
	uint8_t uart_tx_buf[FACTORY_INFO_REGION_SIZE + 4];
	uint32_t err_code = 0x00;
	
	memset(&read_buf[0], '0', sizeof(read_buf));
	
	err_code = factory_info_get(op_code , &read_buf[0], &read_cnt);

	if(err_code != NRF_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
	}
	else 
	{
		uart_tx_buf[0] = 0x7C;
		uart_tx_buf[1] = (cmd_code << 2) | 0x01;
		uart_tx_buf[2] = FACTORY_INFO_REGION_SIZE;
		memcpy(&uart_tx_buf[3], &read_buf[0], FACTORY_INFO_REGION_SIZE);
		uart_tx_buf[FACTORY_INFO_REGION_SIZE + 4 - 1] = 0x7D;

		m_console_uart_runing = true;
		app_trace_dump_var(&uart_tx_buf[0], sizeof(uart_tx_buf));					//发送所有的厂商信息
		m_console_uart_runing = false;
	}						
}

static void console_beat_timeout_handler(void* p_context)
{
	if(!m_console_uart_runing)
	{
		user_console_up_data(FACTORY_INFO_OP_CODE_GET_DEVICE_NAME, COMM_FACTORY_MODE_HEARTBEAT_RESPONSE);		//心跳包发送无线温度计的设备名		
	}
}

uint32_t user_console_tx_private_dat(uint8_t cmd_code, uint8_t* p_buf, uint8_t size)
{
	uint8_t uart_tx_buf[40];

	if(size == 0 || size > sizeof(uart_tx_buf))
	{
		return NRF_ERROR_INVALID_LENGTH;
	}
	
	uart_tx_buf[0] = 0x7C;
	uart_tx_buf[1] = (cmd_code << 2) | 0x01;
	uart_tx_buf[2] = size;
	memcpy(&uart_tx_buf[3], p_buf, size);
	uart_tx_buf[size + 4 -1 ] = 0x7D;

	m_console_uart_runing = true;
	app_trace_dump_var(&uart_tx_buf[0], size + 4);				
	m_console_uart_runing = false;

	return NRF_SUCCESS;	
}

int32_t ad1_offset = 0; 
int32_t ad2_offset = 0;			//AD校准打桩
	
static char str[10] = {0x00};
void user_console_evt_handler(app_uart_evt_t* p_evt)
{
	if(APP_UART_DATA_READY == p_evt->evt_type)
	{
		app_uart_get(p_ind++);
		if(p_ind == &user_console_rxbuf[0] + USER_CONSOLE_BUFSIZE)
		{
			p_ind = &user_console_rxbuf[0];
			if((user_console_rxbuf[0] == 0x7C) && (user_console_rxbuf[USER_CONSOLE_BUFSIZE - 1] == 0x7D))
			{
				uint8_t pkt_filed = 0x00;
				uint8_t pkt_id = 0x00;
				uint8_t pkt_ava_size = 0x00;

				pkt_filed = (user_console_rxbuf[1] & 0x03);
				pkt_id = (user_console_rxbuf[1] >> 2);
				pkt_ava_size = (user_console_rxbuf[2]);

				memcpy(&str[0], &user_console_rxbuf[3], pkt_ava_size);
				str[pkt_ava_size + 1] = '\0';
				
				if(pkt_filed == COMM_PKT_CMD)
				{
					switch(pkt_id)
					{
						case COMM_TIME_SET_REQUEST:
							{								
								uint32_t set_sec = *(uint32_t*)&str[0];
								
								clock_set(set_sec);
								
							}
						break;
						case COMM_UART_DISABLE:
							{
								user_console_set_flag(1);
							}
						break;
						case COMM_UART_ENABLE:
							{
								user_console_set_flag(2);
							}
						break;
						case COMM_FACTORY_MODE_SET_REQUEST:
							{//设置厂商信息								
								APP_ERROR_CHECK(factory_info_set(str[pkt_ava_size - 1], (uint8_t*)&str[0], pkt_ava_size - 1));	//厂商信息设置
							}
						break;
						case COMM_FACTORY_MODE_GET_ALLINFO_REQUEST:
							{//读取厂商信息区
								user_console_up_data(FACTORY_INFO_OP_CODE_GET_ALL_CONFIG, COMM_FACTORY_MODE_GET_ALLINFO_RESPONSE);
							}
							break;
						case COMM_FACTORY_MODE_LEAVE_FACTORY_MODE_REQUEST:
							{//离开工程模式
								user_console_res_deinit();
							}
							break;
						case COMM_FACTORY_MODE_SET_DEVICE_NAME_REQUEST:
							{//设置设备名
								APP_ERROR_CHECK(factory_info_set(str[pkt_ava_size - 1], (uint8_t*)&str[0], pkt_ava_size - 1));	//设置设备名
							}
							break;
						case COMM_FACTORY_MODE_AD1_CAIL_MODE_REQUEST:
							{//进行AD1校准
								//启动AD1进行一定条件的采集
								ad1_offset = -3;
								APP_ERROR_CHECK(factory_info_set(FACTORY_INFO_OP_CODE_SET_AD1_OFFSET, (uint8_t*)&ad1_offset, sizeof(ad1_offset)));
										
								//发送采集到的误差值,打桩代码
								user_console_tx_private_dat(COMM_FACTORY_MODE_AD1_CAIL_MODE_RESPONSE, (uint8_t*)&ad1_offset, sizeof(ad1_offset));
							}
							break;
						case COMM_FACTORY_MODE_AD2_CAIL_MODE_REQUEST:
							{//进行AD2校准
								//启动AD2进行一定条件的采集
								ad2_offset = 3;
								APP_ERROR_CHECK(factory_info_set(FACTORY_INFO_OP_CODE_SET_AD2_OFFSET, (uint8_t*)&ad2_offset, sizeof(ad2_offset)));

								//发送采集的误差值,打桩代码
								
								user_console_tx_private_dat(COMM_FACTORY_MODE_AD2_CAIL_MODE_RESPONSE, (uint8_t*)&ad2_offset, sizeof(ad2_offset));
							}
							break;
						default:
							break;	
					}
				}
				else if(pkt_filed == COMM_PKT_DATA)
				{

				}
				else
				{
					
				}
			}			
		}
	}
}

uint32_t factory_info_have_set;
void user_console_res_init(void)
{
	uint8_t get_cnt;	

	APP_ERROR_CHECK(factory_info_get(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE, (uint8_t*)&factory_info_have_set, &get_cnt));

	if(factory_info_have_set == 0xFFFFFFFF)
	{
		uint32_t err_code = 0x00;

		APP_UART_FIFO_INIT(&user_console_comm_params, USER_CONSOLE_BUFSIZE, USER_CONSOLE_TX_BUFSIZE, user_console_evt_handler, APP_IRQ_PRIORITY_LOW, err_code);
		APP_ERROR_CHECK(err_code);

//		APP_ERROR_CHECK(app_timer_create(&m_factory_mode_beat_timer, APP_TIMER_MODE_REPEATED, console_beat_timeout_handler));	
//		APP_ERROR_CHECK(app_timer_start(m_factory_mode_beat_timer, BEAT_FACTORY_MODE_TICK, NULL));
	}	
}

void user_console_res_deinit(void)
{
	uint8_t get_cnt;	

	APP_ERROR_CHECK(factory_info_get(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE, (uint8_t*)&factory_info_have_set, &get_cnt));

	if(factory_info_have_set == 0xFFFFFFFF)
	{
		APP_ERROR_CHECK(app_timer_stop(m_factory_mode_beat_timer));

		factory_info_have_set = 0x00000001;
		APP_ERROR_CHECK(factory_info_set(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE, (uint8_t*)&factory_info_have_set, get_cnt));
		user_drv_factory_info_flag_set(EN_USER_CONSOLE_FACTORY_INFO_MODE_CANNT_SET);
	}
}

void user_console_reenable_init_set(void)
{
	uint32_t factory_info_have_set;
	uint8_t get_cnt;	

	APP_ERROR_CHECK(factory_info_get(FACTORY_INFO_OP_CODE_GET_FACTORY_MODE, (uint8_t*)&factory_info_have_set, &get_cnt));

	if(factory_info_have_set == 0x00000001)
	{
		factory_info_have_set = 0xFFFFFFFF;
		APP_ERROR_CHECK(factory_info_set(FACTORY_INFO_OP_CODE_SET_FACTORY_MODE, (uint8_t*)&factory_info_have_set, get_cnt));
		user_drv_factory_info_flag_set(EN_USER_CONSOLE_FACTORY_INFO_MODE_RECAN_SET);
	}	
}

