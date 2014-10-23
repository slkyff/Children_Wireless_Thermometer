
#include "user_drv_pwr_ctrl.h"
#include "user_drv_clock.h"
#include "app_trace.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"
#include "sc_ht_perpin_def.h"
#include "sc_ht_common_def.h"
#include "user_drv_led_blink.h"
#include "nordic_common.h"
#include "user_drv_ad7171op.h"
#include "user_drv_offline_storage.h"

#include "user_app_evt.h"
#include "ble.h"


//#define PCT_DISABLE_LOGS        /**< Enable this macro to disable any logs from this module. */

#ifndef PCT_DISABLE_LOGS
#define PCT_LOG  app_trace_log  /**< Used for logging details. */
#define PCT_ERR  app_trace_log  /**< Used for logging errors in the module. */
#define PCT_TRC  app_trace_log  /**< Used for getting trace of execution in the module. */
#define PCT_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //PCT_DISABLE_LOGS
#define PCT_DUMP(...)           /**< Disables dumping of octet streams. */
#define PCT_LOG(...)            /**< Disables detailed logs. */
#define PCT_ERR(...)            /**< Disables error logs. */
#define PCT_TRC(...)            /**< Disables traces. */
#endif 

#define BUTTON_DETECTION_DELAY               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define APP_GPIOTE_MAX_USERS                 1                                          /**< Maximum number of users of the GPIOTE handler. */
#define POWER_KEY_PIN_NO					 KEY0										/**< Turn on or turn off key dependente on the long time push and state*/						

#define KEY_LONG_PUSH_SHUT_TIMEOUT			 APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) //长按关机键检测
#define KEY_PUSH_BOOT_TIMEOUT			     APP_TIMER_TICKS(350, APP_TIMER_PRESCALER)	//短按开机键检测

enum
{
	KEY_PUSH_TYPE_NONE,
	KEY_PUSH_TYPE_SHORT,
	KEY_PUSH_TYPE_LONG
};														//按键按下类型

user_drv_pwr_ctrl_sys_state_t m_sys_state = USER_DRV_PWR_CTRL_SYS_OFFSTATE;	//初始为关闭状态

uint8_t m_key_push_type = KEY_PUSH_TYPE_NONE;
bool m_is_key_long_push_timeout = false;				//长按检测是否超时
bool m_is_key_long_push_timer_start = false;
app_timer_id_t m_timer_key_long_push;					//按键长按检测超时定时器


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */


static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
	uint32_t delay = 0x00;

    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case POWER_KEY_PIN_NO:
				{
					if(!m_is_key_long_push_timer_start)
					{
						uint32_t delay_tick = 0x00;

//						delay_tick = (m_sys_state == USER_DRV_PWR_CTRL_SYS_ONSTATE) ? KEY_LONG_PUSH_SHUT_TIMEOUT : KEY_PUSH_BOOT_TIMEOUT;

						if(m_sys_state == USER_DRV_PWR_CTRL_SYS_ONSTATE)
						{
							uint16_t size;
							APP_ERROR_CHECK(factory_info_get(FACTORY_INFO_OP_CODE_GET_SHUT_DELAY, (uint8_t*)&delay, (uint8_t*)&size));
							delay_tick = APP_TIMER_TICKS(delay, APP_TIMER_PRESCALER);
						}
						else if(m_sys_state == USER_DRV_PWR_CTRL_SYS_OFFSTATE)
						{
							uint16_t size;
							APP_ERROR_CHECK(factory_info_get(FACTORY_INFO_OP_CODE_GET_BOOT_DELAY, (uint8_t*)&delay, (uint8_t*)&size));
							delay_tick = APP_TIMER_TICKS(delay, APP_TIMER_PRESCALER);
						}

						uint32_t err_code = app_timer_start(m_timer_key_long_push, delay_tick, NULL);
						APP_ERROR_CHECK(err_code);
						m_is_key_long_push_timer_start = true;
					}
				}
                break;

            default:
            	APP_ERROR_HANDLER(pin_no);
        }
    }
	else if(button_action == APP_BUTTON_RELEASE)
	{
		switch(pin_no)
		{
			case POWER_KEY_PIN_NO:
				{
					if(m_is_key_long_push_timeout)
					{
						m_is_key_long_push_timeout = false;

						m_sys_state = (m_sys_state == USER_DRV_PWR_CTRL_SYS_ONSTATE) ? USER_DRV_PWR_CTRL_SYS_OFFSTATE : USER_DRV_PWR_CTRL_SYS_ONSTATE;
					}
					else
					{
						if(m_sys_state == USER_DRV_PWR_CTRL_SYS_ONSTATE)
						{
//							led_blink_start(USER_DRV_INDICATE_LED_SINGLE_BLINK);
						}
					}
		
					if(m_is_key_long_push_timer_start)
					{					
						uint32_t err_code = app_timer_stop(m_timer_key_long_push);
						APP_ERROR_CHECK(err_code);
						m_is_key_long_push_timer_start = false;
					}
				}
				break;
			default:
				APP_ERROR_HANDLER(pin_no);
		}
	}
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    static app_button_cfg_t buttons[] =
    {
        {POWER_KEY_PIN_NO,   false, NRF_GPIO_PIN_PULLUP, button_event_handler},
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}

static volatile uint8_t ad7171_mode = EN_AD7171_NORMAL_CLT_MODE;
void key_long_push_timeout_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);

	m_is_key_long_push_timeout  = true;

	if(m_sys_state == USER_DRV_PWR_CTRL_SYS_ONSTATE)
	{	
		user_drv_led_blink_mode_t tmp;

		tmp = led_blink_get_state();
		
		if(USER_DRV_INDICATE_LED_NO_BLINK != tmp)
		{
			led_blink_stop(tmp);								//关机则停止正在闪烁模式的led
		}
		
		led_blink_start(USER_DRV_INDICATE_LED_FAST_BLINK);	

		advertising_stop();										//关机停止广播,从新初始化广播参数

		ad7171_stop_acquisite();
		
		extern void reset_global_flag_when_disconneted_or_shut(void);
		reset_global_flag_when_disconneted_or_shut();

		extern uint32_t stop_current_connection(void);
		APP_ERROR_CHECK(stop_current_connection());
		
		clock_start(STOP_STATE_CLOCK);							//关机启动4h定时器
	}
	else if(m_sys_state == USER_DRV_PWR_CTRL_SYS_OFFSTATE)
	{
//		led_blink_start(USER_DRV_INDICATE_LED_SINGLE_BLINK);
		led_blink_start(USER_DRV_INDICATE_LED_SLOW_BLINK);

		advertising_start();									//开机开始广播
				
		ad7171_start_acquisite(APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER), (uint8_t*)&ad7171_mode);

		clock_start(START_STATE_CLOCK);							//开机启动1s定时器
	}
}

void pwr_ctrl_res_init(void)
{
	uint8_t err_code = app_timer_create(&m_timer_key_long_push, APP_TIMER_MODE_SINGLE_SHOT, \
						key_long_push_timeout_handler);
	APP_ERROR_CHECK(err_code);
	
	gpiote_init();
	buttons_init();
	
	app_button_enable();
	
}



