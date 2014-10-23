
#include "user_app_evt.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "nrf_delay.h"

#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_dfu.h"

#include "ble_conn_params.h"
#include "ble_sensorsim.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "app_trace.h"

#include "sc_ht_perpin_def.h"
#include "sc_ht_common_def.h"

#include "user_drv_ad7171op.h"
#include "user_drv_led_blink.h"
#include "user_drv_pwr_ctrl.h"
#include "user_drv_clock.h"
#include "user_drv_console.h"
#include "user_drv_bt_comm.h"
#include "user_drv_offline_storage.h"
#include "user_drv_factory_mode.h"
#include "user_drv_self_def_service.h"

//lint -e553
#ifdef SVCALL_AS_NORMAL_FUNCTION
#include "ser_phy_debug_app.h"
#endif


#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define APP_FAST_ADV_INTERVAL                400                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_FAST_ADV_TIMEOUT_IN_SECONDS      30	                                        /**< The advertising timeout in units of seconds. */

#define APP_SLOW_ADV_INTERVAL				 2048										
#define APP_SLOW_ADV_TIMEOUT_IN_SECONDS		 1800										/**< Half hour to advertising timeout. */

//快速设置  广播超时参数
//#define APP_FAST_ADV_INTERVAL                400                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
//#define APP_FAST_ADV_TIMEOUT_IN_SECONDS      5                                        /**< The advertising timeout in units of seconds. */

//#define APP_SLOW_ADV_INTERVAL				 2048										
//#define APP_SLOW_ADV_TIMEOUT_IN_SECONDS		 5										/**< Half hour to advertising timeout. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */

#define MIN_CELCIUS_DEGREES                  3688                                       /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS                  3972                                       /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT            36                                         /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */

#define TEMP_COLLECT_INTERVAL				 APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

//#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds) */
//#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
//#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(150, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(200, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        4                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(3600, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */


#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APPL_LOG							 app_trace_log								/**< Debug logger macro that will be used in this file to logging of debug information over UART. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @retval     Result converted to millivolts.
 */
 
#define SCHED_MAX_EVENT_DATA_SIZE        MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                             BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                 10                                             /**< Maximum number of events in the scheduler queue. */

#define ADV_TX_POWER						(-8)											/**< advertising tx power numerical.*/
#define CONN_TX_POWER						 (0)											/**< connecting tx power numerical.*/

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

static app_timer_id_t                        m_battery_timer_id;                        /**< Battery timer. */
static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by device manager */

static bool                                  m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */

static int8_t 								 m_tx_power;								/**< tx power indication. */

extern uint32_t m_ad7171_acquisite_interval;											/**< ad7171 acquisite interval. */
extern app_timer_id_t m_timer_ad7171_acquise_interval;

extern ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
extern ble_hts_t                             m_hts;                                     /**< Structure used to identify the health thermometer service. */
extern ble_dfu_t							 m_dfus;
extern ble_sds_t							 m_sds;

extern bool                                  m_hts_meas_realtime_ind_conf_pending; /**< Flag to keep track of when an indication confirmation is pending. */
extern bool 								 m_hts_meas_history_ind_conf_pending;
extern bool 								 m_hts_meas_update_time_rsp_ind_conf_pending;
extern bool 								 m_hts_meas_set_config_rsp_ind_conf_pengding;

extern bool 							     m_is_pwrup_first_clt_battery;								 

typedef enum
{
    BLE_NO_ADV,                                                                         /**< No advertising running. */
    BLE_DIRECTED_ADV,                                                                   /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,                                                             /**< Advertising with whitelist. */
    BLE_FAST_ADV,                                                                       /**< Fast advertising running. */
    BLE_SLOW_ADV,                                                                       /**< Slow advertising running. */
    BLE_SLEEP,                                                                          /**< Go to system-off. */
    BLE_IDLE,																			/**< Go to idle. */
} ble_advertising_mode_t;
uint8_t                               m_advertising_mode;                        /**< Variable to keep track of when we are advertising. */

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{

	APPL_LOG("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);		

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Uncomment the line below to use.
//	ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
#if SVCALL_AS_NORMAL_FUNCTION
    for(;;);
#else
	if(error_code == DEAD_BEEF)
	{
		NVIC_SystemReset();
	}

#endif
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    adc_bat_measure_start();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	static char wh_device_name[sizeof(DEVICE_NAME)] = DEVICE_NAME;
	uint8_t tmp_buf[8] = {0};
	uint8_t tmp_cnt = 0;

	factory_info_get(FACTORY_INFO_OP_CODE_GET_DEVICE_NAME, &tmp_buf[0], &tmp_cnt);

	memcpy((uint8_t*)&wh_device_name[6], (uint8_t*)&tmp_buf[0], 6);					//获取设备名称

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)wh_device_name,
                                          strlen(wh_device_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

	m_tx_power = ADV_TX_POWER;

	err_code = sd_ble_gap_tx_power_set(m_tx_power);							/**< Set Tx Power. */
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(uint8_t adv_flags)
{
    uint32_t      err_code;
    ble_advdata_t advdata;						//advertising data
	ble_advdata_t scan_rsp_data;				//scan response data
  		
	int8_t		  tx_power = ADV_TX_POWER;
	
    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
    };

	m_advertising_mode = BLE_NO_ADV;

    // Build and set advertising data	
    memset(&advdata, 0, sizeof(advdata));

//    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(adv_flags);
    advdata.flags.p_data            = &adv_flags;
	advdata.p_tx_power_level		= &tx_power;	
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

	memset(&scan_rsp_data, 0, sizeof(scan_rsp_data));

	scan_rsp_data.name_type			=	BLE_ADVDATA_FULL_NAME;
	scan_rsp_data.include_appearance	= true;

    err_code = ble_advdata_set(&advdata, &scan_rsp_data);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in]   p_bas  Battery Service structure.
 * @param[in]   p_evt  Event received from the Battery Service.
 */
void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t *p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
			if(m_is_pwrup_first_clt_battery == false)
			{
				m_is_pwrup_first_clt_battery = true;
				adc_bat_measure_start();
			}
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t err_code;
    uint32_t count;

	ble_gap_adv_params_t adv_params;

	ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // Verify if there is any flash access pending, if yes delay starting advertising until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    
    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

	static char wh_device_name[sizeof(DEVICE_NAME)] = DEVICE_NAME;
	uint8_t tmp_buf[8] = {0};
	uint8_t tmp_cnt = 0;

	factory_info_get(FACTORY_INFO_OP_CODE_GET_DEVICE_NAME, &tmp_buf[0], &tmp_cnt);

	memcpy((uint8_t*)&wh_device_name[6], (uint8_t*)&tmp_buf[0], 6);					//获取设备名称

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)wh_device_name,
                                          strlen(wh_device_name));
    APP_ERROR_CHECK(err_code);


	if(m_tx_power != ADV_TX_POWER)
	{
		m_tx_power = ADV_TX_POWER;

		err_code = sd_ble_gap_tx_power_set(m_tx_power);			//设置广播Radio发射功率
		APP_ERROR_CHECK(err_code);					
	}
	
	// Initialize advertising parameters (used when starting advertising).
	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type 	   = BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr = NULL;							 // Undirected advertisement.
	adv_params.fp		   = BLE_GAP_ADV_FP_ANY;

	switch(m_advertising_mode)
	{
		case BLE_NO_ADV:
			{
				m_advertising_mode = BLE_FAST_ADV;
			}
		case BLE_FAST_ADV:
			{
				advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

				adv_params.interval = APP_FAST_ADV_INTERVAL;
				adv_params.timeout  = APP_FAST_ADV_TIMEOUT_IN_SECONDS;

				m_advertising_mode = BLE_SLOW_ADV;		

				APPL_LOG("[APPL]: ASSERT: %s\r\n", "BLE_FAST_ADV...");
				
				break;
			}
		case BLE_SLOW_ADV:
			{
				advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

				adv_params.interval = APP_SLOW_ADV_INTERVAL;
				adv_params.timeout  = APP_SLOW_ADV_TIMEOUT_IN_SECONDS;

				m_advertising_mode = BLE_IDLE;

				APPL_LOG("[APPL]: ASSERT: %s\r\n", "BLE_SLOW_ADV...");
				
				break;
			}
		default:
			{
				break;
			}
	}

    err_code = sd_ble_gap_adv_start(&adv_params);
	
    APP_ERROR_CHECK(err_code);
}

void advertising_stop(void)
{
	if(m_advertising_mode != BLE_NO_ADV)
	{
		m_advertising_mode = BLE_NO_ADV;

		sd_ble_gap_adv_stop();											//停止广播

		APPL_LOG("[APPL]: ASSERT: %s\r\n", "BLE_ADV_STOP!");	
	}
}


static bool m_is_connecting = false;

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:

			m_advertising_mode = BLE_NO_ADV;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

			if(CONN_TX_POWER != m_tx_power)
			{
				m_tx_power = CONN_TX_POWER;
				err_code = sd_ble_gap_tx_power_set(m_tx_power); 			//设置连接时Radio发射功率
				APP_ERROR_CHECK(err_code);
			}

			user_bt_comm_connect_timeout_start();
			m_is_connecting = true;

			APPL_LOG("[APPL]: ASSERT: %s\r\n", "CONNECTED_EVT_HAPPEN!");
						
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            m_conn_handle               = BLE_CONN_HANDLE_INVALID;

			m_is_connecting = false;

			reset_global_flag_when_disconneted_or_shut();

			APPL_LOG("[APPL]: ASSERT: %s\r\n", "DISCONNECTED_EVT_HAPPEN!");

            advertising_start();
			
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
				if(m_advertising_mode == BLE_IDLE)
				{										//run in idle state
					APPL_LOG("[APPL]: ASSERT: %s\r\n", "SLOW_ADV_TIMEOUT!");	
					
					m_advertising_mode = BLE_NO_ADV;
					
				}
				else 
				{
					APPL_LOG("[APPL]: ASSERT: %s\r\n", "FAST_ADV_TIMEOUT!");
									
					advertising_start();				//start advertise
				}
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }	
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint16_t user_get_current_con_handle(void)
{
	return m_conn_handle;
}

uint32_t stop_current_connection(void)
{
	if(m_is_connecting == true)
	{
		return sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	}
	return NRF_SUCCESS;
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_hts_on_ble_evt(&m_hts, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	ble_sds_on_ble_evt(&m_sds, p_ble_evt);
	ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
//    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);
	
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
//    uint32_t err_code;
//    bool     is_indication_enabled;

    switch(p_event->event_id)
    {
        case DM_EVT_LINK_SECURED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            //       ble_bondmngr_on_ble_evt() in ble_evt_dispatch().
//            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
//            APP_ERROR_CHECK(err_code);

//            if (is_indication_enabled)
//            {
////                temperature_measurement_send();
//            }
            break;
        default:
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;
    
    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = (nrf_gpio_pin_read(13) == 0);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

	if(err_code == NRF_SUCCESS)
	{
		pstorage_handle_t tmp_storage_handler; 
		pstorage_handle_t* p_storage_handler = &tmp_storage_handler;
		
		dm_ble_get_storage_handle(&p_storage_handler);

		APPL_LOG("[APPL]: NVM_STAT: DM_STORAGE_START_ADDR 0X%08X\r\n", (p_storage_handler)->block_id);
	}

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

void user_app_para_init(void)
{
	ad7171_para_init();
}

static void nvm_statistics_info(void)
{
	APPL_LOG("\r\n");
	
	APPL_LOG("[APPL]: NVM_STAT : APP_START_ADDR       0X%08X\r\n", NRF_UICR->CLENR0);
	APPL_LOG("[APPL]: NVM_STAT : PSTORAGE_START_ADDR  0X%08X\r\n", PSTORAGE_DATA_START_ADDR);
	APPL_LOG("[APPL]: NVM_STAT : PSTORAGE_END_ADDR    0X%08X\r\n", PSTORAGE_DATA_END_ADDR);
	APPL_LOG("[APPL]: NVM_STAT : PSTORAGE_SWAP_ADDR   0X%08X\r\n", PSTORAGE_SWAP_ADDR);
	APPL_LOG("[APPL]: NVM_STAT : BOOTLOAD_START_ADDR  0X%08X\r\n", NRF_UICR->BOOTLOADERADDR);
	
	APPL_LOG("\r\n");
}

uint32_t reset_reason_mask = 0x00;
uint32_t reset_reason_mask_other = 0x00;

void user_app_func_init(void)
{
	// Initialize.		
    app_trace_init();
	
	timers_init();
	
    led_blink_res_init();
	
	ad7171_res_init();
	
	pwr_ctrl_res_init();
	
	clock_res_init();

	nvm_statistics_info();
		
	APPL_LOG("[APPL]: ASSERT: %s\r\n", "BLE_STACK_INIT_START...");
	INDICATION_LED_TRUN_ON;
    ble_stack_init();
	APPL_LOG("[APPL]: ASSERT: %s\r\n", "BLE_STACK_INIT_END...");
	
	sd_power_reset_reason_get(&reset_reason_mask);
	sd_power_reset_reason_clr(reset_reason_mask);
	sd_power_reset_reason_get(&reset_reason_mask_other);

	APPL_LOG("[APPL]: ASSERT: RESET %08X\r\n", reset_reason_mask);
	
	INDICATION_LED_TRUN_OFF;
		
	// Initialize persistent storage module.
	uint32_t err_code = 0x00;
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

	APP_ERROR_CHECK(user_drv_offline_storage_init());			//离线数据存储初始化
		
	APP_ERROR_CHECK(user_drv_factory_info_init());				//厂商信息初始化
	
    device_manager_init();										//设备管理初始化	

	factory_mode_exec();										//工厂模式执行

    gap_params_init();
	
    advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
	
	user_bt_comm_res_init();
	
    conn_params_init();

	APPL_LOG("[APPL]: ASSERT: %s\r\n", "SYS_FUNC_INIT_FINISH!");
	
}


