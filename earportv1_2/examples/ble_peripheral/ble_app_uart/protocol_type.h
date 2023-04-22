#ifndef __PROTOCOL_TYPE_H__
#define __PROTOCOL_TYPE_H__


#define  SENSOR_DATA_LEN    144                           //传感器数据所占字节数

/* ---------------------------枚举和结构体定义---------------------------------*/
typedef enum
{
	CMD_REQ_GET_DEVICE_ID = 0x00,
	CMD_REQ_GET_VERSION = 0x01,
	CMD_REQ_START_SAMPLE = 0x02,
	CMD_REQ_STOP_SAMPLE = 0x03,
	CMD_REQ_GET_SAMPLE = 0x04,
	CMD_REQ_GET_POWER_STATUS = 0x05,
	CMD_REQ_CALIBRATE = 0x06,
	CMD_REQ_SOFT_RESET = 0x08, // csy_1227
	CMD_REQ_START_SAMEPLE_AND_SAVE = 0x0a, /*持续采集传感器数据并保存*/
	CMD_REQ_GET_SAVED_SENSOR_DATE = 0x0b,  /*获取存储的传感器数据.*/
	CMD_REQ_STOP_SAMEPLE_AND_SAVE = 0x0c, /*持续采集传感器数据并保存*/
} eu_cmd_req;

typedef enum
{
	CMD_RESP_GET_DEVICE_ID = 0x80,
	CMD_RESP_GET_VERSION = 0x81,
	CMD_RESP_START_SAMPLE = 0x82,
	CMD_RESP_STOP_SAMPLE = 0x83,
	CMD_RESP_GET_SAMPLE = 0x84,
	CMD_RESP_GET_POWER_STATUS = 0x85,
	CMD_RESP_CALIBRATION = 0x86,

} eu_cmd_resp;

typedef struct
{
	uint32_t flag : 8;
	uint32_t device_id : 4;
	uint32_t command : 4;
	uint32_t length : 8;
	uint32_t sequence_id : 4;
	uint32_t misc : 4;
} st_ble_frame_header;




#endif //__PROTOCOL_TYPE_H__