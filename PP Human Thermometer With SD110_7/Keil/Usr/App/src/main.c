/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hts_main main.c
 * @{
 * @ingroup ble_sdk_app_hts
 * @brief Health Thermometer Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Health Thermometer service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 */
#include "user_app_evt.h" 
 
#include "app_scheduler.h"
#include "nrf_soc.h"


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
	uint32_t err_code = 0x00;
		
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for application main entry.
 */

int main(void)
{
	
 	user_app_para_init();
	user_app_func_init();


    // Enter main loop.
    for (;;)
    {
//    	app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
