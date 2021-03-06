
#ifndef SC_HT_BT_COMM_DEF_H
#define SC_HT_BT_COMM_DEF_H

#define USER_COMM_PKT_RSV_SIZE 0

#define USER_COMM_PKT_TYPE_POS	0
#define USER_COMM_PKT_TYPE_FIELD_POS	2

#define USER_COMM_PKT_TYPE_CMD		1
#define USER_COMM_PKT_TYPE_DATA		2

#define USER_COMM_PKT_HEADER_FLAG	0x7C
#define USER_COMM_PKT_TAIL_FLAG 	0x7D

#define USER_COMM_PKT_CMDID_UPDATE_TIME_REQ		0x01
#define USER_COMM_PKT_CMDID_UPDATE_TIME_RSP		0x10
#define USER_COMM_PKT_CMDID_UPDATE_TIME_REQ_SIZE	0x08
#define USER_COMM_PKT_CMDID_UPDATE_TIME_RSP_SIZE 	0x05

#define USER_COMM_PKT_CMDID_SET_CONFIG_REQ		0x02
#define USER_COMM_PKT_CMDID_SET_CONFIG_RSP		0x20
#define USER_COMM_PKT_CMDID_SET_CONFIG_REQ_SIZE	0x0A
#define USER_COMM_PKT_CMDID_SET_CONFIG_RSP_SIZE	0x05

#define USER_COMM_PKT_CMDID_MEAS_PARAM			0x30
#define USER_COMM_PKT_CMDID_MEAS_PARAM_SIZE		0x08			

#define USER_COMM_PKT_CMDID_GET_CURR_TEMP		0x04
#define USER_COMM_PKT_CMDID_GET_CURR_TEMP_SIZE	0x04

#define USER_COMM_PKT_DATAID_HISTORY_TEMP		0x01
#define USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP 0x02
#define USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP 0x03
#define USER_COMM_PKT_DATAID_UNAVA_TEMP			0x04

#define USER_COMM_PKT_DATAID_HISTORY_TEMP		0x01
#define USER_COMM_PKT_DATAID_REALTIME_STABLE_TEMP 0x02
#define USER_COMM_PKT_DATAID_REALTIME_UNSTABLE_TEMP 0x03
#define USER_COMM_PKT_DATAID_UNAVAILABLE_TEMP	0x04


#endif

