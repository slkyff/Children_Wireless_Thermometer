#ifndef USER_DRV_LED_BLINK_H
#define USER_DRV_LED_BLINK_H

#include "sc_ht_perpin_def.h"

#define INDICATION_LED_NO					 LED0										/**< Indicate the program state and give the user some mean*/

#define INDICATION_LED_ON					 1
#define INDICATION_LED_OFF					 0

#define INDICATION_LED_TRUN_ON					nrf_gpio_pin_set(INDICATION_LED_NO)
#define INDICATION_LED_TRUN_OFF					nrf_gpio_pin_clear(INDICATION_LED_NO)
#define INDICATION_LED_TOGGLE					nrf_gpio_pin_toggle(INDICATION_LED_NO) 			//·­×ªled io×´Ì¬

typedef enum
{
	USER_DRV_INDICATE_LED_NO_BLINK,
	USER_DRV_INDICATE_LED_FAST_BLINK = 1,
	USER_DRV_INDICATE_LED_SLOW_BLINK ,
	USER_DRV_INDICATE_LED_SINGLE_BLINK
}user_drv_led_blink_mode_t;

extern user_drv_led_blink_mode_t led_blink_get_state(void);
extern void led_blink_res_init(void);
extern void led_blink_start(user_drv_led_blink_mode_t led_blink_mode);
extern void led_blink_stop(user_drv_led_blink_mode_t led_blink_mode);

#endif

