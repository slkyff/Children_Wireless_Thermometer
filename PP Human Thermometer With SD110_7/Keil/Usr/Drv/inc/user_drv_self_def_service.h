
#ifndef USER_DRV_SELF_DEF_SERVICE_H
#define USER_DRV_SELF_DEF_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

typedef enum
{
    SELF_DEF_SEND_EVT_INDICATION_ENABLED = 1,                                       
    SELF_DEF_SEND_EVT_INDICATION_DISABLED,                                       
    SELF_DEF_SEND_EVT_INDICATION_CONFIRMED,                                    
} ble_sds_evt_type_t;

typedef struct
{
    ble_sds_evt_type_t evt_type;                                            /**< Type of event. */
} ble_sds_evt_t;

// Forward declaration of the ble_hts_t type. 
typedef struct ble_sds_s ble_sds_t;

typedef void (*ble_sds_rcv_data_handler_t)(ble_sds_t* p_sds, uint8_t* data, uint16_t length, uint16_t datasrc_handle);

typedef void (*ble_sds_evt_handler_t) (ble_sds_t * p_sds, ble_sds_evt_t * p_evt);


typedef struct
{
	uint8_t len;
	uint8_t* p_buf;
}sds_idc_char_dat_t;														

typedef sds_idc_char_dat_t sds_write_char_dat_t;

typedef struct
{
    ble_sds_evt_handler_t        evt_handler;                               

	sds_idc_char_dat_t*			 p_self_def_send_char_buf;

	sds_write_char_dat_t*		 p_self_def_rcv_char_buf;

	ble_srv_cccd_security_mode_t sds_self_def_send_attr_md;

	ble_srv_security_mode_t		 sds_self_def_rcv_attr_md;
			
	ble_sds_rcv_data_handler_t 	 data_handler;								

} ble_sds_init_t;

typedef struct ble_sds_s
{
    ble_sds_evt_handler_t        evt_handler;                               
    uint16_t                     service_handle;                           
	ble_gatts_char_handles_t     self_def_send_handles;
	
	ble_gatts_char_handles_t	 self_def_rcv_handles;
	
	uint16_t                     conn_handle;                               

	ble_sds_rcv_data_handler_t   data_handler;								
	
} ble_sds_t;


uint32_t ble_sds_init(ble_sds_t * p_sds, const ble_sds_init_t * p_sds_init);


void ble_sds_on_ble_evt(ble_sds_t * p_sds, ble_evt_t * p_ble_evt);


uint32_t ble_sds_send_is_indication_enabled(ble_sds_t * p_sds, bool * p_indication_enabled);

#endif 

