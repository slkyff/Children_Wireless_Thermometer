
#include "user_drv_clock.h"
#include "app_timer.h"
#include "app_trace.h"
#include "sc_ht_common_def.h"
#include <string.h>

#include "user_drv_console.h"

//#define CLK_DISABLE_LOGS

#ifndef CLK_DISABLE_LOGS
#define CLK_LOG  app_trace_log  /**< Used for logging details. */
#define CLK_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define CLK_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define CLK_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //CLK_DISABLE_LOGS
#define CLK_DUMP(...)           /**< Disables dumping of octet streams. */
#define CLK_LOG(...)            /**< Disables detailed logs. */
#define CLK_ERR(...)            /**< Disables error logs. */
#define CLK_TRC(...)            /**< Disables traces. */
#endif 


#define CLOCK_SHUT_TIMER_TICK		APP_TIMER_TICKS(14400000, APP_TIMER_PRESCALER)	//关机后定时器超时为4H
#define CLOCK_SEC_TIMER_TICK		APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)		//正常开机运行超时为1S
#define ONE_SECOND_CNT				993

app_timer_id_t m_timer_clock;

uint32_t m_last_rtc1_cnt = 0x00;
uint32_t m_curr_rtc1_cnt = 0x00;
uint32_t m_curr_second = 0x00;
uint32_t m_delt_rtc1_cnt = 0x00;

state_clock_type m_curr_clock_type = STOP_STATE_CLOCK;	//当前运行的时钟类型

void clock_get(uint32_t* p_cur_sec)
{
	*p_cur_sec = m_curr_second;
	
}

uint8_t clock_set(uint32_t target_sec)
{
	m_curr_second = target_sec;							//设置时间

	return 0;
}

static void get_curr_sec(void)
{
	
	APP_ERROR_CHECK(app_timer_cnt_get(&m_curr_rtc1_cnt));

	uint32_t delt = 0x00;
	
	if(m_curr_rtc1_cnt > m_last_rtc1_cnt)
	{
		delt = 0x00;
	}
	else
	{//counter 溢出,则补偿回来
		delt = 0xffffff;
	}

	uint32_t tmp_ms = 0x00;

	tmp_ms = (m_curr_rtc1_cnt + delt - m_last_rtc1_cnt + m_delt_rtc1_cnt);
		
	m_curr_second += (tmp_ms / ONE_SECOND_CNT);		  //换算秒数
	m_delt_rtc1_cnt = tmp_ms % ONE_SECOND_CNT;		  //保存ms值
	m_last_rtc1_cnt = m_curr_rtc1_cnt;											//更新last_rtc1_cnt计数,防止长时间再开机会算出个很大的数值
}

void clock_timeout_handler(void* p_context)
{//时钟超时处理回调
	APP_ERROR_CHECK_BOOL((p_context != NULL));

	state_clock_type en_clock_type = *(state_clock_type*)p_context;
	
	en_clock_type = en_clock_type;

	get_curr_sec();
	
//	if(user_console_get_flag() == 2)
//	{
//		char szTrcStr[14] = {0};
//		itoa(m_curr_second, &szTrcStr[0]);
//		CLK_TRC("%s\n", &szTrcStr[0]);
//	}
}


void clock_stop(void)
{
	APP_ERROR_CHECK(app_timer_stop(m_timer_clock));
}

void clock_start(state_clock_type en_clock_type)
{
	uint32_t timer_tick;

	if(m_curr_clock_type != en_clock_type)
	{
		m_curr_clock_type = en_clock_type;

		get_curr_sec();

		clock_stop();																	//先关闭时钟定时器
	
		timer_tick = (en_clock_type == START_STATE_CLOCK) ? CLOCK_SEC_TIMER_TICK : CLOCK_SHUT_TIMER_TICK;	//选择定时间隔

		APP_ERROR_CHECK(app_timer_start(m_timer_clock, timer_tick, (void*)&m_curr_clock_type));	//重新启动时钟定时器
	}
}

void clock_res_init(void)
{
	APP_ERROR_CHECK(app_timer_create(&m_timer_clock, APP_TIMER_MODE_REPEATED, clock_timeout_handler));
	
}

