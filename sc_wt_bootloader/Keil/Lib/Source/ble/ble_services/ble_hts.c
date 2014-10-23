/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

//modify a lot by slk (2014.08.20)

#include "ble_hts.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"


#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Health Thermometer Measurement. */

#define PARAMETER_SIZE	4

//add by slk
#define BLE_RCV_CMD_MAX_RX_CHAR_LEN	(GATT_MTU_SIZE_DEFAULT - 3)

#define UUID_PRV_DEF_REALTIME_TEMPDATA	0x2A1C
#define UUID_PRV_DEF_HISTORY_TEMPDATA	0x2A1F
#define UUID_PRV_DEF_MEAS_PARAMETER		0x2A21
#define UUID_PRV_DEF_UPDATE_TIME_REQ	0x2AB1
#define UUID_PRV_DEF_UPDATE_TIME_RSP	0x2AB2
#define UUID_PRV_DEF_SET_CONFIG_REQ		0x2AB3
#define UUID_PRV_DEF_SET_CONFIG_RSP		0x2AB4
#define UUID_PRV_DEF_GET_CURR_TEMP		0x2AB5

// Health Thermometer Measurement flag bits
#define HTS_MEAS_FLAG_TEMP_UNITS_BIT (0x01 << 0)  /**< Temperature Units flag. */
#define HTS_MEAS_FLAG_TIME_STAMP_BIT (0x01 << 1)  /**< Time Stamp flag. */
#define HTS_MEAS_FLAG_TEMP_TYPE_BIT  (0x01 << 2)  /**< Temperature Type flag. */

static uint16_t                 service_handle;


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    p_hts->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_hts->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the Blood Pressure Measurement characteristic.
 *
 * @param[in]   p_hts         Health Thermometer Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_cccd_write(ble_hts_t * p_hts, ble_gatts_evt_write_t * p_evt_write)
{
    if ((p_evt_write->len == 2) && (p_hts->evt_handler != NULL))
    {
		ble_hts_evt_t evt;
	
        // CCCD written, update indication state
        if (p_hts->meas_realtime_handles.cccd_handle == p_evt_write->handle)
        {
            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = HTS_REALTIME_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = HTS_REALTIME_EVT_INDICATION_DISABLE;
            }

        }
		else if(p_hts->meas_history_handles.cccd_handle == p_evt_write->handle)
		{
			if(ble_srv_is_indication_enabled(p_evt_write->data))
			{
				evt.evt_type = HTS_HISTORY_EVT_INDICATION_ENABLED;
			}
			else
			{
				evt.evt_type = HTS_HISTORY_EVT_INDICATION_DISABLE;
			}
		}
		else if(p_hts->update_time_response_handles.cccd_handle == p_evt_write->handle)
		{
			if(ble_srv_is_indication_enabled(p_evt_write->data))
			{
				evt.evt_type = HTS_UPDATE_TIME_RSP_EVT_INDICATION_ENABLED;
			}
			else
			{
				evt.evt_type = HTS_UPDATE_TIME_RSP_EVT_INDICATION_DISABLE;
			}
		}
		else if(p_hts->set_config_response_handles.cccd_handle == p_evt_write->handle)
		{
			if(ble_srv_is_indication_enabled(p_evt_write->data))
			{
				evt.evt_type = HTS_SET_CONFIG_RSP_EVT_INDICATION_ENABLED;
			}
			else 
			{
				evt.evt_type = HTS_SET_CONFIG_RSP_EVT_INDICATION_DISABLE;
			}
		}
		else
		{
			//TODO:
		}
		
		p_hts->evt_handler(p_hts, &evt);
    }
	else
	{
		//TODO:
	}
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_hts->meas_realtime_handles.cccd_handle) ||(p_evt_write->handle == p_hts->meas_history_handles.cccd_handle) ||\
		(p_evt_write->handle == p_hts->set_config_response_handles.cccd_handle) || (p_evt_write->handle == p_hts->update_time_response_handles.cccd_handle))
    {//each indication type characteristic indication cccd write process
		on_cccd_write(p_hts, p_evt_write);
    }
	else 
	{
		if(p_hts->data_handler != NULL)
		{
			p_hts->data_handler(p_hts, p_evt_write->data, p_evt_write->len, p_evt_write->handle);
		}
	}
}


/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_hvc(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

	 ble_hts_evt_t evt;

    if (p_hvc->handle == p_hts->meas_realtime_handles.value_handle)
    {
        evt.evt_type = HTS_REALTIME_EVT_INDICATION_CONFIRMED;
    }
	else if(p_hvc->handle == p_hts->meas_history_handles.value_handle)
	{
		evt.evt_type = HTS_HISTORY_EVT_INDICATION_CONFIRMED;
	}
	else if(p_hvc->handle == p_hts->set_config_response_handles.value_handle)
	{
		evt.evt_type = HTS_SET_CONFIG_RSP_EVT_INDICATION_CONFIRMED;
	}
	else if(p_hvc->handle == p_hts->update_time_response_handles.value_handle)
	{
		evt.evt_type = HTS_UPDATE_TIME_RSP_EVT_INDICATION_CONFIRMED;
	}
	else 
	{
		//TODO:
	}
	
	p_hts->evt_handler(p_hts, &evt);
}


void ble_hts_on_ble_evt(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_hts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_hts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_hts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_hts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Health Thermometer Measurement.
 *
 * @param[in]   p_hts              Health Thermometer Service structure.
 * @param[in]   p_hts_meas         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t hts_measurement_encode(ble_hts_t      * p_hts,
                                      ble_hts_meas_t * p_hts_meas,
                                      uint8_t        * p_encoded_buffer)
{
    uint8_t  flags = 0;
    uint8_t  len   = 1;
    uint32_t encoded_temp;

    // Flags field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;
    }
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
    }

    // Temperature Measurement Value field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;

        encoded_temp = ((p_hts_meas->temp_in_fahr.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_fahr.mantissa <<  0) & 0x00FFFFFF);
    }
    else
    {
        encoded_temp = ((p_hts_meas->temp_in_celcius.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_celcius.mantissa <<  0) & 0x00FFFFFF);
    }
    len += uint32_encode(encoded_temp, &p_encoded_buffer[len]);

    // Time Stamp field
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
        len   += ble_date_time_encode(&p_hts_meas->time_stamp, &p_encoded_buffer[len]);
    }

    // Temperature Type field
    if (p_hts_meas->temp_type_present)
    {
        flags                  |= HTS_MEAS_FLAG_TEMP_TYPE_BIT;
        p_encoded_buffer[len++] = p_hts_meas->temp_type;
    }

    // Flags field
    p_encoded_buffer[0] = flags;

    return len;
}

/**@brief Function for adding Indicate type characteristics.
 *
 * @param[in]   uuid			characteristic short uuid.
 * @param[in]   p_char_value	point to characteristic buf value
 * @param[in] 	char_len		characteristic buf length
 * @param[in]	idc_char_attr_md indication characteristics attribute metadata
 * @param[in]	p_handles		point to characteristic handles.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t	indication_char_add(uint16_t 							uuid, 
									uint8_t 							*p_char_value, 
									uint16_t 							char_len, 
									const ble_srv_cccd_security_mode_t* idc_char_attr_md, 
									ble_gatts_char_handles_t* 			p_handles)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    APP_ERROR_CHECK_BOOL(char_len > 0);

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	cccd_md.write_perm = idc_char_attr_md->cccd_write_perm;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.indicate	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md 			= &cccd_md;
	char_md.p_sccd_md			= NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc		=	BLE_GATTS_VLOC_STACK;
	attr_md.read_perm	=	idc_char_attr_md->read_perm;
	attr_md.write_perm	=	idc_char_attr_md->write_perm;
	attr_md.rd_auth		=	0;
	attr_md.wr_auth		=	0;
	attr_md.vlen		=	1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= char_len;
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= char_len;
	attr_char_value.p_value		= p_char_value;

	return sd_ble_gatts_characteristic_add(service_handle, \
										   &char_md, \
										   &attr_char_value, \
										   p_handles);		
}

/**@brief Function for adding write type characteristics.
 *
 * @param[in]   uuid			characteristic short uuid.
 * @param[in]   p_char_value	point to characteristic buf value
 * @param[in] 	char_len		characteristic buf length
 * @param[in]	idc_char_attr_md indication characteristics attribute metadata
 * @param[in]	p_handles		point to characteristic handles.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t write_char_add(uint16_t 		uuid,
							   uint8_t* 		p_char_value,
							   uint16_t			char_len,
							   const ble_srv_security_mode_t* write_char_attr_md,
							   ble_gatts_char_handles_t* p_handles)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    APP_ERROR_CHECK_BOOL(char_len > 0);

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md 			= NULL;
	char_md.p_sccd_md			= NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc		=	BLE_GATTS_VLOC_STACK;
	attr_md.read_perm	=	write_char_attr_md->read_perm;
	attr_md.write_perm	=	write_char_attr_md->write_perm;
	attr_md.rd_auth		=	0;
	attr_md.wr_auth		=	0;
	attr_md.vlen		=	1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= char_len;
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= char_len;
	attr_char_value.p_value		= p_char_value;

	return sd_ble_gatts_characteristic_add(service_handle, \
										   &char_md, \
										   &attr_char_value, \
										   p_handles);	
}

/**@brief Function for adding Health Thermometer Measurement parameters characteristics.
 *
 * @param[in]   p_hts        Health Thermometer Service structure.
 * @param[in]   p_hts_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 static uint32_t hts_measurement_parameter_char_add(ble_hts_t* p_hts, const ble_hts_init_t* p_hts_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
	
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read		= 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = NULL;
    char_md.p_sccd_md           = NULL;
    
	BLE_UUID_BLE_ASSIGN(ble_uuid, UUID_PRV_DEF_MEAS_PARAMETER);			//Measurement Parameter UUID 
    
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_hts_init->hts_meas_parameter_attr_md.read_perm;
    attr_md.write_perm = p_hts_init->hts_meas_parameter_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = p_hts_init->p_parameter_char_buf->len;		
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = p_hts_init->p_parameter_char_buf->len;
    attr_char_value.p_value      = (uint8_t*)p_hts_init->p_parameter_char_buf->p_buf;
    
    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_hts->meas_parameter_handles);	
}

/**@brief Function for adding Temperature Type characteristics.
 *
 * @param[in]   p_hts        Health Thermometer Service structure.
 * @param[in]   p_hts_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t hts_temp_type_char_add(ble_hts_t * p_hts, const ble_hts_init_t * p_hts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             init_value_temp_type;
    uint8_t             init_value_encoded[1];

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_TYPE_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_hts_init->hts_temp_type_attr_md.read_perm;
    attr_md.write_perm = p_hts_init->hts_temp_type_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_value_temp_type  = p_hts_init->temp_type;
    init_value_encoded[0] = init_value_temp_type;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = init_value_encoded;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_hts->temp_type_handles);
}

uint32_t ble_hts_init(ble_hts_t * p_hts, const ble_hts_init_t * p_hts_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_hts->evt_handler = p_hts_init->evt_handler;
    p_hts->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_hts->temp_type   = p_hts_init->temp_type;

	//slk add
	p_hts->data_handler	= p_hts_init->data_handler;
	
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEALTH_THERMOMETER_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	p_hts->service_handle	=	service_handle;
		
    // Add measurement realtime characteristic
	err_code = indication_char_add(UUID_PRV_DEF_REALTIME_TEMPDATA, p_hts_init->p_realtime_char_buf->p_buf, p_hts_init->p_realtime_char_buf->len, 
								   &p_hts_init->hts_meas_realtime_attr_md, &p_hts->meas_realtime_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// Add measurement history characteristic
	err_code = indication_char_add(UUID_PRV_DEF_HISTORY_TEMPDATA, p_hts_init->p_history_char_buf->p_buf, p_hts_init->p_history_char_buf->len,
								   &p_hts_init->hts_meas_history_attr_md, &p_hts->meas_history_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add measurement parameter characteristic
	err_code = hts_measurement_parameter_char_add(p_hts, p_hts_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

    // Add temperature type characteristic
    if (p_hts_init->temp_type_as_characteristic)
    {
        err_code = hts_temp_type_char_add(p_hts, p_hts_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

	// Add update time request characteristic
	err_code = write_char_add(UUID_PRV_DEF_UPDATE_TIME_REQ, p_hts_init->p_update_time_req_char_buf->p_buf, p_hts_init->p_update_time_req_char_buf->len,
							  &p_hts_init->update_time_request_attr_md, &p_hts->update_time_request_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Add update time response characteristic
	err_code = indication_char_add(UUID_PRV_DEF_UPDATE_TIME_RSP, p_hts_init->p_update_time_rsp_char_buf->p_buf, p_hts_init->p_update_time_rsp_char_buf->len,
								   &p_hts_init->update_time_response_attr_md, &p_hts->update_time_response_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	//Add set config request characteristic
	err_code = write_char_add(UUID_PRV_DEF_SET_CONFIG_REQ, p_hts_init->p_set_config_req_char_buf->p_buf, p_hts_init->p_set_config_req_char_buf->len,
							 &p_hts_init->set_config_request_attr_md, &p_hts->set_config_request_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Add set config response characteristic
	err_code = indication_char_add(UUID_PRV_DEF_SET_CONFIG_RSP, p_hts_init->p_set_config_rsp_char_buf->p_buf, p_hts_init->p_set_config_rsp_char_buf->len,
								   &p_hts_init->set_config_response_attr_md, &p_hts->set_config_response_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add get current temperature characteristic
	err_code = write_char_add(UUID_PRV_DEF_GET_CURR_TEMP, p_hts_init->p_get_current_temp_char_buf->p_buf, p_hts_init->p_get_current_temp_char_buf->len,
							  &p_hts_init->get_current_temp_attr_md, &p_hts->get_curr_temp_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

    return NRF_SUCCESS;
}


uint32_t ble_hts_is_indication_enabled(uint16_t handle, bool * p_indication_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    }
    return err_code;
}

