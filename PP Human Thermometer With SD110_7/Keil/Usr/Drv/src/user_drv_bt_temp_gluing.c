
#include "user_drv_bt_temp_gluing.h"
#include "user_drv_clock.h"
#include "user_drv_ad7171op.h"
#include "sc_ht_bt_comm_def.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"

#include "ble_hts.h"

#define SECOND_BYTE_SIZE			4				//秒数所占的字节数
#define TEMP_POINT_NUM				6				//一包蓝牙传输包最大温度点个数

typedef struct
{
	uint32_t current_pkt_seconds;
	uint16_t temperature_pkt_buf[TEMP_POINT_NUM];
}st_temperature_pkt_t;								//蓝牙发送温度数据格式

st_temperature_pkt_t st_temperature_pkt;
uint8_t curr_temperature_pos = 0x00;

extern ble_hts_t m_hts;
extern uint32_t realtime_temp_send(ble_hts_t * p_hts, uint8_t data_type, uint8_t * pbuf, uint16_t buf_len);		//实时数据发送

void bt_temp_glue_record_time(void)
{
	if(curr_temperature_pos == 0x00)
	{
		clock_get(&st_temperature_pkt.current_pkt_seconds);	//获取当前clock的时间
	}
}

uint32_t bt_temp_glue_get_record_time(void)
{
	return st_temperature_pkt.current_pkt_seconds;			//返回采集的时间点
}

void TriggerBtSend(uint8_t cnt)
{
	uint8_t tmp_buf[16];

	memcpy(&tmp_buf[0], (uint8_t*)&st_temperature_pkt.current_pkt_seconds, sizeof(st_temperature_pkt.current_pkt_seconds));

	memcpy(&tmp_buf[4], (uint8_t*)&st_temperature_pkt.temperature_pkt_buf[0], cnt*sizeof(uint16_t));

	APP_ERROR_CHECK(realtime_temp_send(&m_hts, ad7171_get_cur_temp_state(), &tmp_buf[0], (cnt*sizeof(uint16_t) + sizeof(st_temperature_pkt.current_pkt_seconds))));
		
}

////由稳态到非稳态的跳变，会丢弃到一个非稳态的采集温度数据
void bt_temp_glue_add_once_temp(uint16_t add_val, uint8_t curr_tx_cnt)
{
	APP_ERROR_CHECK_BOOL((curr_tx_cnt != 0) && (curr_tx_cnt <= TEMP_POINT_NUM));

	st_temperature_pkt.temperature_pkt_buf[curr_temperature_pos++] = add_val;
	
	if(curr_temperature_pos == curr_tx_cnt)
	{//当前温度点的位置满,则触发发送
		TriggerBtSend(curr_tx_cnt);
		
		curr_temperature_pos = 0;
	}
	else if(curr_tx_cnt == 1)
	{
		TriggerBtSend(curr_temperature_pos - 1);
		
		curr_temperature_pos = 0;
	}
	
}

uint8_t bt_temp_glue_get_curr_temp_pos(void)
{
	return curr_temperature_pos;
}

void bt_temp_glue_minus_curr_temp_pos(void)
{
	curr_temperature_pos --;
}

uint32_t bt_temp_glue_calu_curr_pos_time(void)
{
	uint32_t tick_val = 0x00;

	tick_val = ad7171_acquisite_interval_get();					//需要调试验证一下				

	return 	st_temperature_pkt.current_pkt_seconds + (curr_temperature_pos * tick_val);
}

uint8_t m_is_get_curr_temp_flag = 0x00;
void bt_temp_glue_check_after_get_curr_temp(void)
{
	if(m_is_get_curr_temp_flag == 0x01)
	{
		m_is_get_curr_temp_flag = 0x00;

		
	}
}

void bt_temp_glue_process_get_curr_temp_action(void)
{
	if(m_is_get_curr_temp_flag == 0x00  && ad7171_get_cur_temp_state() != USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP)
	{
		m_is_get_curr_temp_flag = 0x01;
			
		if(bt_temp_glue_get_curr_temp_pos() != 0)
		{			
			uint8_t tmp_buf[6];

			uint32_t sec = 0x00;

			sec = bt_temp_glue_calu_curr_pos_time();

			memcpy(&tmp_buf[0], (uint8_t*)&sec, sizeof(sec));

			memcpy(&tmp_buf[sizeof(sec) - 1], &st_temperature_pkt.temperature_pkt_buf[curr_temperature_pos], sizeof(st_temperature_pkt.temperature_pkt_buf[curr_temperature_pos]));
			
			APP_ERROR_CHECK(realtime_temp_send(&m_hts, ad7171_get_cur_temp_state(), &tmp_buf[0], sizeof(tmp_buf)));
			
		}
	}
}

