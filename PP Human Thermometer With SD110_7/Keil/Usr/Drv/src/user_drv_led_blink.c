/***
***	-Led_blink.c主要实现开机时的INDICATE_LED慢闪,关机时INDICATE_LED快闪。
*** -按键单次按下指示灯单闪一次
*** -Indication Led为共享资源，同一个阶段只能有一个功能可见。
****/

#include "app_timer.h"
#include "sc_ht_common_def.h"
#include "user_drv_led_blink.h"
#include "app_trace.h"

#define LB_DISABLE_LOGS        /**< Enable this macro to disable any logs from this module. */

#ifndef LB_DISABLE_LOGS
#define LB_LOG  app_trace_log  /**< Used for logging details. */
#define LB_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define LB_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define LB_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //LB_DISABLE_LOGS
#define LB_DUMP(...)           /**< Disables dumping of octet streams. */
#define LB_LOG(...)            /**< Disables detailed logs. */
#define LB_ERR(...)            /**< Disables error logs. */
#define LB_TRC(...)            /**< Disables traces. */
#endif 

#define FAST_BLINK_ON_TIME		15
#define FAST_BLINK_OFF_TIME		385
#define FAST_BLINK_ON_TICK		APP_TIMER_TICKS(FAST_BLINK_ON_TIME, APP_TIMER_PRESCALER)
#define FAST_BLINK_OFF_TICK		APP_TIMER_TICKS(FAST_BLINK_OFF_TIME, APP_TIMER_PRESCALER)
#define FAST_BLINK_PERIOD		2500
#define FAST_BLINK_CNT			(FAST_BLINK_PERIOD)/(FAST_BLINK_ON_TIME + FAST_BLINK_OFF_TIME)
#define SLOW_BLINK_ON_TIME		50
#define SLOW_BLINK_OFF_TIME		1950
#define SLOW_BLINK_ON_TICK		APP_TIMER_TICKS(SLOW_BLINK_ON_TIME, APP_TIMER_PRESCALER)
#define SLOW_BLINK_OFF_TICK		APP_TIMER_TICKS(SLOW_BLINK_OFF_TIME, APP_TIMER_PRESCALER)
#define SLOW_BLINK_PERIOD		30000	
#define SLOW_BLINK_CNT			(SLOW_BLINK_PERIOD)/(SLOW_BLINK_ON_TIME + SLOW_BLINK_OFF_TIME)
//#define SINGLE_BLINK_ON_TICK	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
//#define SINGLE_BLINK_OFF_TICK	APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define SINGLE_BLINK_ON_TICK	APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)
#define SINGLE_BLINK_OFF_TICK	APP_TIMER_TICKS(950, APP_TIMER_PRESCALER)
		
app_timer_id_t m_timer_led_blink;

bool                            m_is_indicate_led_blinking;                            /**< Variable to indicate if the indication LED is blinking. */
user_drv_led_blink_mode_t m_led_blink_state = USER_DRV_INDICATE_LED_NO_BLINK;

uint16_t m_fast_blink_cnt = 0x00;
uint16_t m_slow_blink_cnt = 0x00;

void led_blink_start(user_drv_led_blink_mode_t led_blink_mode)
{
	if(!m_is_indicate_led_blinking)
	{
		uint32_t             err_code;
        static volatile uint8_t context_buf[2];	//第一个字节表示led亮灭的状态，第二个字节表示led所处的闪烁状态

		INDICATION_LED_TRUN_ON;					//点亮led

		m_is_indicate_led_blinking = true;
		
		context_buf[0] = INDICATION_LED_ON;
		context_buf[1] = led_blink_mode;		//记录indication_led亮灭状态和所处状态
		
		m_led_blink_state = led_blink_mode;		//记录全局led_blink_state
		
		if(led_blink_mode == USER_DRV_INDICATE_LED_FAST_BLINK)
		{
			m_fast_blink_cnt = 0;
			
			err_code = app_timer_start(m_timer_led_blink, FAST_BLINK_ON_TICK, (void *)&context_buf[0]);
			APP_ERROR_CHECK(err_code);
		}
		else if(led_blink_mode == USER_DRV_INDICATE_LED_SLOW_BLINK)
		{
			m_slow_blink_cnt = 0;
			
			err_code = app_timer_start(m_timer_led_blink, SLOW_BLINK_ON_TICK, (void *)&context_buf[0]);
			APP_ERROR_CHECK(err_code);
		}
		else if(led_blink_mode == USER_DRV_INDICATE_LED_SINGLE_BLINK)
		{
			err_code = app_timer_start(m_timer_led_blink, SINGLE_BLINK_ON_TICK, (void *)&context_buf[0]);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			LB_ERR("Led Blink Module Start Timer Exception\n");
			APP_ERROR_CHECK(8);	
		}
	}
}

void led_blink_stop(user_drv_led_blink_mode_t led_blink_mode)
{
	if(m_is_indicate_led_blinking)
	{
		uint32_t err_code;

		m_is_indicate_led_blinking = false;

		INDICATION_LED_TRUN_OFF;				//关闭LED

		if(led_blink_mode == USER_DRV_INDICATE_LED_FAST_BLINK)
		{
			m_fast_blink_cnt = 0;
		
			err_code = app_timer_stop(m_timer_led_blink);
			APP_ERROR_CHECK(err_code);
		}
		else if(led_blink_mode == USER_DRV_INDICATE_LED_SLOW_BLINK)
		{
			m_slow_blink_cnt = 0;
		
			err_code = app_timer_stop(m_timer_led_blink);
			APP_ERROR_CHECK(err_code);
		}
		else if(led_blink_mode == USER_DRV_INDICATE_LED_SINGLE_BLINK)
		{			
			err_code = app_timer_stop(m_timer_led_blink);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			LB_ERR("Led Blink Module Stop Timer Exception\n");
			APP_ERROR_CHECK(8);	
		}

		m_led_blink_state  = USER_DRV_INDICATE_LED_NO_BLINK;
		
	}
}

void led_blink_timeout_handler(void* p_context)
{
	uint8_t* 		pcontext; 
	uint8_t 		led_state;
	uint8_t 		led_blink_state;
	uint32_t        next_timer_interval = 0;
	uint32_t        err_code;

	APP_ERROR_CHECK_BOOL(p_context != NULL);

	INDICATION_LED_TOGGLE;

	pcontext = (uint8_t *)p_context;

	*pcontext = (*pcontext == INDICATION_LED_ON) ? INDICATION_LED_OFF : INDICATION_LED_ON;
	led_state = *pcontext;								//取出led亮灭状态
	led_blink_state = *(pcontext + 1);					//取出led所处状态字节
	
	if(led_blink_state == USER_DRV_INDICATE_LED_FAST_BLINK)
	{
		m_fast_blink_cnt = (led_state == INDICATION_LED_ON) ? (m_fast_blink_cnt + 1) : (m_fast_blink_cnt);
	
		next_timer_interval = (led_state == INDICATION_LED_OFF) ? (FAST_BLINK_OFF_TICK) : (FAST_BLINK_ON_TICK); 	
	}
	else if(led_blink_state == USER_DRV_INDICATE_LED_SLOW_BLINK)
	{
		m_slow_blink_cnt = (led_state == INDICATION_LED_ON) ? (m_slow_blink_cnt + 1) : (m_slow_blink_cnt);
		
		next_timer_interval = (led_state == INDICATION_LED_OFF) ? (SLOW_BLINK_OFF_TICK) : (SLOW_BLINK_ON_TICK); 	
	}
	else if(led_blink_state == USER_DRV_INDICATE_LED_SINGLE_BLINK)
	{
		led_blink_stop(USER_DRV_INDICATE_LED_SINGLE_BLINK);
	}
	else 
	{
		LB_ERR("Led Blink Module Timer Exception\n");
		APP_ERROR_CHECK(8);
	}

	if((led_blink_state != USER_DRV_INDICATE_LED_SINGLE_BLINK) && (m_fast_blink_cnt != FAST_BLINK_CNT) &&(m_slow_blink_cnt != SLOW_BLINK_CNT))
	{//非单次闪烁和不满足超过闪烁次数的，再次启动定时器
		err_code = app_timer_start(m_timer_led_blink, next_timer_interval, (void*)pcontext);

		APP_ERROR_CHECK(err_code);
	}
	else if(m_fast_blink_cnt == FAST_BLINK_CNT)
	{
		led_blink_stop(USER_DRV_INDICATE_LED_FAST_BLINK);
	}
	else if(m_slow_blink_cnt == SLOW_BLINK_CNT)
	{
		led_blink_stop(USER_DRV_INDICATE_LED_SLOW_BLINK);
	}
	else
	{//

	}
}

user_drv_led_blink_mode_t led_blink_get_state(void)
{
	return m_led_blink_state;
}


void led_blink_res_init(void)
{//初始化led 闪烁使用到的资源
    nrf_gpio_cfg_output(INDICATION_LED_NO);

	uint32_t errcode = app_timer_create(&m_timer_led_blink, APP_TIMER_MODE_SINGLE_SHOT, \
					   led_blink_timeout_handler);

	APP_ERROR_CHECK(errcode);
}


