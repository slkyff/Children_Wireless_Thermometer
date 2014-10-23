/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_srv_hts Health Thermometer Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Health Thermometer Service module.
 *
 * @details This module implements the Health Thermometer Service.
 *
 *          If an event handler is supplied by the application, the Health Thermometer 
 *          Service will generate Health Thermometer Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Health Thermometer Service
 *       module by calling ble_hts_on_ble_evt() from the from the @ref ble_stack_handler function.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_HTS_H__
#define BLE_HTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

// Temperature Type measurement locations
#define BLE_HTS_TEMP_TYPE_ARMPIT      1
#define BLE_HTS_TEMP_TYPE_BODY        2
#define BLE_HTS_TEMP_TYPE_EAR         3
#define BLE_HTS_TEMP_TYPE_FINGER      4
#define BLE_HTS_TEMP_TYPE_GI_TRACT    5
#define BLE_HTS_TEMP_TYPE_MOUTH       6
#define BLE_HTS_TEMP_TYPE_RECTUM      7
#define BLE_HTS_TEMP_TYPE_TOE         8
#define BLE_HTS_TEMP_TYPE_EAR_DRUM    9

/**@brief Health Thermometer Service event type. */
typedef enum
{
    HTS_REALTIME_EVT_INDICATION_ENABLED = 1,                                         /**< Health Thermometer realtime value indication enabled event. */
    HTS_REALTIME_EVT_INDICATION_DISABLE,                                        /**< Health Thermometer realtime value indication disabled event. */
    HTS_REALTIME_EVT_INDICATION_CONFIRMED,                                       /**< Confirmation of a temperature measurement realtime indication has been received. */

	HTS_HISTORY_EVT_INDICATION_ENABLED,
	HTS_HISTORY_EVT_INDICATION_DISABLE,
	HTS_HISTORY_EVT_INDICATION_CONFIRMED,

	HTS_UPDATE_TIME_RSP_EVT_INDICATION_ENABLED,
	HTS_UPDATE_TIME_RSP_EVT_INDICATION_DISABLE,
	HTS_UPDATE_TIME_RSP_EVT_INDICATION_CONFIRMED,

	HTS_SET_CONFIG_RSP_EVT_INDICATION_ENABLED,
	HTS_SET_CONFIG_RSP_EVT_INDICATION_DISABLE,
	HTS_SET_CONFIG_RSP_EVT_INDICATION_CONFIRMED,
	
} ble_hts_evt_type_t;

/**@brief Health Thermometer Service event. */
typedef struct
{
    ble_hts_evt_type_t evt_type;                                            /**< Type of event. */
} ble_hts_evt_t;

// Forward declaration of the ble_hts_t type. 
typedef struct ble_hts_s ble_hts_t;

//Recive data event handler
typedef void (*ble_hts_rcv_data_handler_t)(ble_hts_t* p_hts, uint8_t* data, uint16_t length, uint16_t datasrc_handle);

/**@brief Health Thermometer Service event handler type. */
typedef void (*ble_hts_evt_handler_t) (ble_hts_t * p_hts, ble_hts_evt_t * p_evt);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;

typedef struct
{
	uint8_t len;
	uint8_t* p_buf;
}hts_idc_char_dat_t;																/**< Wireless thermometer measurement service indication characteristic. */

typedef hts_idc_char_dat_t	hts_para_char_dat_t;

typedef hts_idc_char_dat_t	hts_write_char_dat_t;

/**@brief Health Thermometer Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_hts_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */

	hts_idc_char_dat_t*			 p_realtime_char_buf;						/**< Point to realtime characteristic buf. */
	hts_idc_char_dat_t* 		 p_history_char_buf;						/**< Point to history  characteristisc buf. */
	hts_idc_char_dat_t* 		 p_update_time_rsp_char_buf;				/**< Point to update time response characteristic buf. */
	hts_idc_char_dat_t*			 p_set_config_rsp_char_buf;					/**< Point to set config response characteristic buf.*/
	hts_para_char_dat_t*		 p_parameter_char_buf;						/**< Point to parameter characteristic buf. */

	hts_write_char_dat_t* 		 p_update_time_req_char_buf;			    /**< Point to update time request characteristic buf. */
	hts_write_char_dat_t*		 p_set_config_req_char_buf;					/**< Point to set config request characteristic buf. */
	hts_write_char_dat_t*		 p_get_current_temp_char_buf;				/**< Point to get current temperature characteristic buf. */
		
    ble_srv_cccd_security_mode_t hts_meas_realtime_attr_md;                 /**< Initial security level for health thermometer measurement realtime data attribute */
	ble_srv_cccd_security_mode_t hts_meas_history_attr_md;					/**< Initial security level for health thermometer measurement history data attribute */
	ble_srv_cccd_security_mode_t update_time_response_attr_md;				/**< Initial security level for update time response attribute */
	ble_srv_cccd_security_mode_t set_config_response_attr_md;				/**< Initial security level for set config response attribute */
	
	ble_srv_security_mode_t		 hts_meas_parameter_attr_md;				/**< Initial security level for Measure Parameter attribute */ 

	ble_srv_security_mode_t		 update_time_request_attr_md;				/**< Initial security level for update time request attribute.*/
	ble_srv_security_mode_t		 set_config_request_attr_md;				/**< Initial security level for set config request attribute. */
	ble_srv_security_mode_t		 get_current_temp_attr_md;					/**< Initial security level for get current temperature attribute. */
	
	ble_srv_security_mode_t      hts_temp_type_attr_md;                     /**< Initial security level for health thermometer tempearture type attribute */
    uint8_t                      temp_type_as_characteristic;               /**< Set non-zero if temp type given as characteristic */
    uint8_t                      temp_type;                                 /**< Temperature type if temperature characteristic is used */
	
	ble_hts_rcv_data_handler_t 	 data_handler;								/**< Event handler to be called for handling received data. */

} ble_hts_init_t;

/**@brief Health Thermometer Service structure. This contains various status information for
 *        the service. */
typedef struct ble_hts_s
{
    ble_hts_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
    uint16_t                     service_handle;                            /**< Handle of Health Thermometer Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t 	 meas_indication_handles;					/**< Handle related to the Wireless Thermometer Measurement Indication Type Characteristic. */

    ble_gatts_char_handles_t     meas_realtime_handles;                     /**< Handles related to the Wireless Thermometer Measurement Realtime characteristic. */
	ble_gatts_char_handles_t     meas_history_handles;						/**< Handles related to the Wireless Thermometer Measurement History characteristic */	
	ble_gatts_char_handles_t	 update_time_response_handles;				/**< Handles related to the Wireless Thermometer Update time response */
	ble_gatts_char_handles_t	 set_config_response_handles;				/**< Handles related to the Wireless Thermometer Set config response */

	ble_gatts_char_handles_t 	 meas_parameter_handles;					/**< Handles related to the Wireless Thermometer Measurement Parameter */	

	ble_gatts_char_handles_t 	 update_time_request_handles;				/**< Handles related to the Wireless Thermometer Update time request */
	ble_gatts_char_handles_t 	 set_config_request_handles;				/**< Handles related to the Wireless Thermometer Set config request */
	ble_gatts_char_handles_t 	 get_curr_temp_handles;						/**< Handles related to the Wireless Thermometer Get Current temperature data */
	
    ble_gatts_char_handles_t     temp_type_handles;                         /**< Handles related to the Health Thermometer Temperature Type characteristic. */
    uint8_t                      temp_type;                                 /**< Temperature type indicates where the measurement was taken. */
	
	uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */

	ble_hts_rcv_data_handler_t   data_handler;								/**< Event handler to be called for handling received data. */
	
} ble_hts_t;

/**@brief Health Thermometer Service measurement structure. This contains a Health Thermometer
 *        measurement. */
typedef struct ble_hts_meas_s
{
    bool                         temp_in_fahr_units;                        /**< True if Temperature is in Fahrenheit units, Celcius otherwise. */
    bool                         time_stamp_present;                        /**< True if Time Stamp is present. */
    bool                         temp_type_present;                         /**< True if Temperature Type is present. */
    ieee_float32_t               temp_in_celcius;                           /**< Temperature Measurement Value (Celcius). */
    ieee_float32_t               temp_in_fahr;                              /**< Temperature Measurement Value (Fahrenheit). */
    ble_date_time_t              time_stamp;                                /**< Time Stamp. */
    uint8_t                      temp_type;                                 /**< Temperature Type. */
} ble_hts_meas_t;

/**@brief Function for initializing the Health Thermometer Service.
 *
 * @param[out]  p_hts       Health Thermometer Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_hts_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_hts_init(ble_hts_t * p_hts, const ble_hts_init_t * p_hts_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Health Thermometer Service.
 *
 * @param[in]   p_hts      Health Thermometer Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_hts_on_ble_evt(ble_hts_t * p_hts, ble_evt_t * p_ble_evt);


/**@brief Function for checking if indication of Temperature Measurement is currently enabled.
 *
 * @param[in]   p_hts                  Health Thermometer Service structure.
 * @param[out]  p_indication_enabled   TRUE if indication is enabled, FALSE otherwise.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_hts_is_indication_enabled(uint16_t handle, bool * p_indication_enabled);


#endif // BLE_HTS_H__

/** @} */
