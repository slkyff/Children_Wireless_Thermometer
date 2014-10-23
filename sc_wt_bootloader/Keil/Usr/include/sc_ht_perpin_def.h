
#ifndef SC_HT_PERPIN_DEF_H
#define SC_HT_PERPIN_DEF_H

#include <stdbool.h>
#include "nrf_gpio.h"

#define PP_SC_PHY
#ifdef PP_SC_PHY
//user interface pin def
#define KEY0					13
#define LED0					17
#define LED1					19

//trace uart pin def
#define RTS_PIN_NO				18	
#define CTS_PIN_NO				16

#define TX_PIN_NO				9
#define RX_PIN_NO				11
#define HWFC					false

//ad7171 ctrl pin def
#define SENSE_PWR_CTRL_PIN_NUM	14
#define AD7171_PD_PIN_NUM		12
#define AD7171_CLK_PIN_NUM		8
#define AD7171_DOUT_PIN_NUM		10

#define NTC_SWITCH_PIN_NUM		15

#endif

#ifdef AKII
#define KEY0					16
#define LED0					18
#define LED1					19

#define RTS_PIN_NO				10	
#define CTS_PIN_NO				8

#define TX_PIN_NO				9
#define RX_PIN_NO				11
#define HWFC					false

#define SENSE_PWR_CTRL_PIN_NUM	24
#define AD7171_PD_PIN_NUM		25
#define AD7171_CLK_PIN_NUM		28
#define AD7171_DOUT_PIN_NUM		29

#define NTC_SWITCH_PIN_NUM		30

#endif

#ifdef NRF_BOARD
#define KEY0					0							
#define LED0					8
#define LED1					9

#define RTS_PIN_NO				18	
#define CTS_PIN_NO				19

#define TX_PIN_NO				17
#define RX_PIN_NO				16
#define HWFC					false

#define SENSE_PWR_CTRL_PIN_NUM	24
#define AD7171_PD_PIN_NUM		25
#define AD7171_CLK_PIN_NUM		28
#define AD7171_DOUT_PIN_NUM		29

#define NTC_SWITCH_PIN_NUM		30

#endif

#ifdef SC_PHY
//user interface pin def
#define KEY0					13
#define LED0					18
#define LED1					19

//trace uart pin def
#define RTS_PIN_NO				15	
#define CTS_PIN_NO				16

#define TX_PIN_NO				9
#define RX_PIN_NO				11
#define HWFC					false


//ad7171 ctrl pin def
#define SENSE_PWR_CTRL_PIN_NUM	14
#define AD7171_PD_PIN_NUM		12
#define AD7171_CLK_PIN_NUM		8
#define AD7171_DOUT_PIN_NUM		10

#define NTC_SWITCH_PIN_NUM		17

#endif

#endif
