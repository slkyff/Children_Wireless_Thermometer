
#include "user_drv_self_def_service.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"


#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Health Thermometer Measurement. */


//add by slk
#define BLE_RCV_CMD_MAX_RX_CHAR_LEN	(GATT_MTU_SIZE_DEFAULT - 3)

#define UUID_PRV_DEF_SELF_DEF_SEVICE	0x1C1C
#define UUID_PRV_DEF_SELF_DEF_SEND		0x2B2B
#define UUID_PRV_DEF_SELF_DEF_RCV		0x2C2C

static uint16_t                 service_handle;


static void on_connect(ble_sds_t * p_sds, ble_evt_t * p_ble_evt)
{
    p_sds->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

static void on_disconnect(ble_sds_t * p_sds, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sds->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_cccd_write(ble_sds_t * p_sds, ble_gatts_evt_write_t * p_evt_write)
{
    if ((p_evt_write->len == 2) && (p_sds->evt_handler != NULL))
    {
		ble_sds_evt_t evt;
	
        // CCCD written, update indication state
        if (p_sds->self_def_send_handles.cccd_handle == p_evt_write->handle)
        {
            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = SELF_DEF_SEND_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = SELF_DEF_SEND_EVT_INDICATION_DISABLED;
            }
        }
		else
		{
			//TODO:
		}
		
		p_sds->evt_handler(p_sds, &evt);
    }
	else
	{
		//TODO:
	}
}

static void on_write(ble_sds_t * p_sds, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_sds->self_def_send_handles.cccd_handle))
    {
		on_cccd_write(p_sds, p_evt_write);
    }
	else 
	{
		if(p_sds->data_handler != NULL)
		{
			p_sds->data_handler(p_sds, p_evt_write->data, p_evt_write->len, p_evt_write->handle);
		}
	}
}


static void on_hvc(ble_sds_t * p_sds, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

	 ble_sds_evt_t evt;

    if(p_hvc->handle == p_sds->self_def_send_handles.value_handle)
	{
		evt.evt_type = SELF_DEF_SEND_EVT_INDICATION_CONFIRMED;
	}
	else 
	{
		//TODO:
	}
	
	p_sds->evt_handler(p_sds, &evt);
}


void ble_sds_on_ble_evt(ble_sds_t * p_sds, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sds, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_sds, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

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

uint32_t ble_sds_init(ble_sds_t * p_sds, const ble_sds_init_t * p_sds_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_sds->evt_handler = p_sds_init->evt_handler;
    p_sds->conn_handle = BLE_CONN_HANDLE_INVALID;

	//slk add
	p_sds->data_handler	= p_sds_init->data_handler;
	
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, UUID_PRV_DEF_SELF_DEF_SEVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	p_sds->service_handle	=	service_handle;
		
    // Add self def send characteristic
	err_code = indication_char_add(UUID_PRV_DEF_SELF_DEF_SEND, p_sds_init->p_self_def_send_char_buf->p_buf, p_sds_init->p_self_def_send_char_buf->len, 
								   &p_sds_init->sds_self_def_send_attr_md, &p_sds->self_def_send_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// Add self def rcv characteristic
	err_code = write_char_add(UUID_PRV_DEF_SELF_DEF_RCV, p_sds_init->p_self_def_rcv_char_buf->p_buf, p_sds_init->p_self_def_rcv_char_buf->len,
							  &p_sds_init->sds_self_def_rcv_attr_md, &p_sds->self_def_rcv_handles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
    return NRF_SUCCESS;
}

uint32_t ble_sds_send_is_indication_enabled(ble_sds_t * p_sds, bool * p_indication_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_sds->self_def_send_handles.cccd_handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    }
    return err_code;
}


