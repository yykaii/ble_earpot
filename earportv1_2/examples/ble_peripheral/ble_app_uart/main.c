/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
// csy_1227 #include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
// csy_1227 #include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

#include "sdk_config.h"
#include "app_config.h"

#include "bmi160.h"
#include "bmm150.h"

#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_flash.h"
#include "sys_queue.h"

/* ---------------------------局部变量定义---------------------------------*/
static uint8_t g_tx_sequence_id = 0;	 // BLE发送数据包序号
static uint8_t g_acquire_enable = 0;	 // 数据采集使能
static uint8_t g_active_upload_flag = 0; // 主动上报数据标志
static uint8_t g_active_upload_idx = 0;	 // 主动上报数据索引
// static uint8_t  g_packet_cnt = 0;            //数据包计数
static uint8_t g_ble_connect_flag = 0;	  // BLE连接标志
static uint8_t g_ble_connect_changed = 0; // BLE连接状态变化标志
static uint8_t g_ble_adv_flag = 0;		  // BLE广播状态标志
static uint16_t g_bat_volt = 0;			  // 电池电压
static uint16_t g_bus_volt = 0;			  // Vbus volt
// csy_1227 static uint16_t g_sensor_data_buf[5][9];     //传感器数据缓冲区
static uint16_t g_sensor_data_buf[8][9]; // 传感器数据缓冲区//csy_1227
static uint8_t g_sleep_needed = 0;		 // 是否需要深度休眠（在电压低或者充电状态下，需要主动断开连接，停止广播）

nrf_drv_wdt_channel_id m_channel_id;   // 看门狗channel定义
static uint8_t get_volt_cnt = 0;	   // get bat volt cnt
static uint8_t saadc_init_flag = 0;	   // adc init flag
static uint8_t peri_poweroff_flag = 0; // spi and bmx160 power off flag: 0 is power on; 1 is power off
static uint8_t led_poweroff_flag = 0;  // led poweroff flag
static uint8_t blue_led_cnt = 0;	   // led toggle cnt,  1:2
static uint8_t packet_sequence_id = 0; // fifo sample packet sequence id
static uint8_t spi_init_flag = 0;	   // spi init complete flag


/*------------------------------------------------------------------------------------------------------------------------------*/
#define  SENSOR_DATA_LEN    144                           //传感器数据所占字节数

static queue_list_t g_s_save_sensor_data_queue;           //存储传感器数据的队列.
static uint8_t g_s_get_sensor_data_and_save;              //获取传感器数据并存储标志.
static uint8_t g_s_erase_flag = 1;                        //flash需擦除标志.
static uint16_t g_s_sensor_data_buf[SENSOR_DATA_LEN/2];   //用来存储获取的传感器数据
static uint32_t g_s_flash_write_offset;                   //写flash的偏移地址
static uint32_t g_s_flash_read_offset;                    //读取flash的偏移地址
static uint8_t  g_s_flash_write_sensor_len = 160;         //传感器数据实际有144字节,但写入flash时地址需要4字节对齐,因此按照160的倍数开始写.
static uint8_t  g_s_send_sensor_data_flag;                //获取传感器数据标志
static uint8_t g_s_send_sensor_buf[SENSOR_DATA_LEN];      //蓝牙发送缓存

// SAADC sample buffer size :2
#define SAMPLES_IN_BUFFER 2

// SAADC sample buffer
static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];

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

/* 蓝牙接收帧和发送帧头变量定义 */
st_ble_frame_header rx_frame_header;
st_ble_frame_header tx_frame_header;

APP_TIMER_DEF(bmi160_timer);
APP_TIMER_DEF(app_timer);

/* ---------------------------函数定义---------------------------------*/
/* 计算Checksum，用于蓝牙传输协议 */
static uint8_t calc_checksum(const uint8_t *data, uint8_t len);

/* 获取电池电压 */
static void usr_sample_volt(void);		// csy_0308  sample bat and bus vot at the same time
static uint16_t usr_get_bat_volt(void); // csy_0308 get bat volt
static uint16_t usr_get_bus_volt(void); // csy_0308 get bus volt

/* 获取充电状态，0:charging 1: idle*/
uint8_t usr_get_charge_status();

/* ---------------------------蓝牙协议栈相关定义------------------------*/
// 注意：广播间隔调整需要修改APP_ADV_INTERVAL
// 100ms version parameter: MIN_CONN_INTERVAL~MAX_CONN_INTERVAL, 20~35.  SLAVE_LATENCY:4
// 50ms version parameter: MIN_CONN_INTERVAL~MAX_CONN_INTERVAL, 16~32.  SLAVE_LATENCY:2

#define APP_BLE_CONN_CFG_TAG 1								 /**< A tag identifying the SoftDevice BLE configuration. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN	 /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_BLE_OBSERVER_PRIO 3								 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL 1600								 /**< The advertising interval (in units of 0.625 ms.).ble广播间隔2秒*/
#define APP_ADV_DURATION 0									 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(40, UNIT_1_25_MS)	 // MSEC_TO_UNITS(40, UNIT_1_25_MS)	 //csy_0127   /**< Minimum acceptable connection interval (30 ms) */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(46, UNIT_1_25_MS)	 // MSEC_TO_UNITS(46, UNIT_1_25_MS)  //csy_0127    /**< Maximum acceptable connection interval (50 ms) */
#define SLAVE_LATENCY 2										 // csy_1207								             /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)	 /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3						 /**< Number of attempts before giving up the connection parameter negotiation. */
#define DEAD_BEEF 0xDEADBEEF								 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE 256								 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256								 /**< UART RX buffer size. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);						  /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);							  /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);				  /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;			   /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] =									   /**< Universally unique service identifier. */
	{
		{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)DEVICE_NAME,
										  strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
	uint32_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	uint32_t err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail = false;
	cp_init.evt_handler = on_conn_params_evt;
	cp_init.error_handler = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
	APP_ERROR_CHECK(err_code);

	// Prepare wakeup buttons.
	err_code = bsp_btn_ble_sleep_mode_prepare();
	APP_ERROR_CHECK(err_code);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	uint32_t err_code;

	switch (ble_adv_evt)
	{
	case BLE_ADV_EVT_FAST:
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
		break;
	case BLE_ADV_EVT_IDLE:
		sleep_mode_enter();
		break;
	default:
		break;
	}
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
	uint32_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		NRF_LOG_INFO("BLE Connected");
		g_ble_connect_flag = 1;
		g_ble_connect_changed = 1;
		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		APP_ERROR_CHECK(err_code);

		break;

	case BLE_GAP_EVT_DISCONNECTED:
		NRF_LOG_INFO("BLE Disconnected");
		g_ble_connect_flag = 0;
		g_ble_connect_changed = 1;
		g_acquire_enable = 0;
		g_active_upload_flag = 0;
		g_active_upload_idx = 0;
		g_s_get_sensor_data_and_save = 0;
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
	{
		NRF_LOG_DEBUG("PHY update request.");
		ble_gap_phys_t const phys =
			{
				.rx_phys = BLE_GAP_PHY_2MBPS, // BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_2MBPS  // BLE_GAP_PHY_AUTO,
			};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
	}
	break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// Pairing not supported
		err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
										 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
										 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		// No implementation needed.
		break;
	}
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
	{
		m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
		// NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
	}
	NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
				  p_gatt->att_mtu_desired_central,
				  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
	ret_code_t err_code;

	err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
	uint32_t err_code;
	switch (event)
	{
	case BSP_EVENT_SLEEP:
		sleep_mode_enter();
		break;

	case BSP_EVENT_DISCONNECT:
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BSP_EVENT_WHITELIST_OFF:
		if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
		{
			err_code = ble_advertising_restart_without_whitelist(&m_advertising);
			if (err_code != NRF_ERROR_INVALID_STATE)
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		break;

	default:
		break;
	}
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t err_code;
	int8_t tx_power_level = 4; // csy_1227 set ble tx power level to 4dB
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	init.advdata.p_tx_power_level = &tx_power_level; // csy_1227

	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids = m_adv_uuids;

	init.config.ble_adv_fast_enabled = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
	// init.config.ble_adv_on_disconnect_disabled = true;

	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
	err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_power_level); // csy_1227
	APP_ERROR_CHECK(err_code);																				 // csy_1227
}

/* ---------------------------UART相关接口函数---------------------------------*/

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */

/**@snippet [Handling the data received over UART] */
#if defined(UART_PRESENT) // csy_1227
void uart_event_handle(app_uart_evt_t *p_event)
{
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
	static uint8_t index = 0;
	uint32_t err_code;

	switch (p_event->evt_type)
	{
	case APP_UART_DATA_READY:
		UNUSED_VARIABLE(app_uart_get(&data_array[index]));
		index++;

		if ((data_array[index - 1] == '\n') ||
			(data_array[index - 1] == '\r') ||
			(index >= m_ble_nus_max_data_len))
		{
			if (index > 1)
			{
				NRF_LOG_DEBUG("Ready to send data over BLE NUS");
				NRF_LOG_HEXDUMP_DEBUG(data_array, index);

				do
				{
					uint16_t length = (uint16_t)index;
					err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
					if ((err_code != NRF_ERROR_INVALID_STATE) &&
						(err_code != NRF_ERROR_RESOURCES) &&
						(err_code != NRF_ERROR_NOT_FOUND))
					{
						APP_ERROR_CHECK(err_code);
					}
				} while (err_code == NRF_ERROR_RESOURCES);
			}

			index = 0;
		}
		break;

	case APP_UART_COMMUNICATION_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_communication);
		break;

	case APP_UART_FIFO_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_code);
		break;

	default:
		break;
	}
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
	uint32_t err_code;
	app_uart_comm_params_t const comm_params =
	{
		.rx_pin_no = RX_PIN_NUMBER,
		.tx_pin_no = TX_PIN_NUMBER,
		.rts_pin_no = RTS_PIN_NUMBER,
		.cts_pin_no = CTS_PIN_NUMBER,
		.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
		.use_parity = false,
#if defined(UART_PRESENT)
		.baud_rate = NRF_UART_BAUDRATE_115200
#else
		.baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
	};

	APP_UART_FIFO_INIT(&comm_params,
					   UART_RX_BUF_SIZE,
					   UART_TX_BUF_SIZE,
					   uart_event_handle,
					   APP_IRQ_PRIORITY_LOWEST,
					   err_code);
	APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */
#endif

/* ---------------------------按键和指示灯相关接口函数，暂未使用---------------------------------*/
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool *p_erase_bonds)
{
	bsp_event_t startup_event;

	uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = bsp_btn_ble_init(NULL, &startup_event);
	APP_ERROR_CHECK(err_code);

	*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

#if BMI160_INTERFACE_SPI == 1

#define SPI_INSTANCE 0												   /**< SPI instance index. */
static volatile bool spi_xfer_done;									   // SPI数据传输完成标志
static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
// csy_0313  static nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;//csy_0207
static uint8_t spi_tx_buf[256]; /**< TX buffer. */
static uint8_t spi_rx_buf[256]; /**< RX buffer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
					   void *p_context)
{
	spi_xfer_done = true;
}

#define SPI_SS_PIN 0
#define SPI_MISO_PIN 12
// #define SPI_MOSI_PIN 4
#define SPI_MOSI_PIN 14 // csy_0308  seconde version PCB
#define SPI_SCK_PIN 1

void spi_master_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin = SPI_SCK_PIN;
	// csy_0316 spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	spi_config.frequency = NRF_DRV_SPI_FREQ_500K; // csy_0316
	APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}

void spi_enable(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin = SPI_SCK_PIN;
	// csy_0316 spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	spi_config.frequency = NRF_DRV_SPI_FREQ_500K; // csy_0316
	APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}

void spi_disable(void)
{
	nrf_drv_spi_uninit(&m_spi);
}

int8_t nrf52_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;
	spi_xfer_done = false;
	// printf("%s\r\n",__FUNCTION__);

	spi_tx_buf[0] = reg_addr;

	ret = nrf_drv_spi_transfer(&m_spi, spi_tx_buf, len + 1, spi_rx_buf, len + 1);
	// APP_ERROR_CHECK();
	while (!spi_xfer_done)
		;
	// nrf_delay_ms(1);
	if (reg_addr == 0x80)
	{
		// printf("%x,%x,%x,%x\r\n",spi_rx_buf[0],spi_rx_buf[1],spi_rx_buf[2],spi_rx_buf[3]);
	}
	memcpy(data, &spi_rx_buf[1], len);

	return ret;
}

int8_t nrf52_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;
	// printf("%s\r\n",__FUNCTION__);

	spi_xfer_done = false;

	spi_tx_buf[0] = reg_addr;
	memcpy(&spi_tx_buf[1], data, len);
	ret = nrf_drv_spi_transfer(&m_spi, spi_tx_buf, len + 1, spi_rx_buf, len + 1);
	while (!spi_xfer_done)
		;
	// nrf_delay_ms(1);
	return ret;
}
#endif

/* ---------------------------Nordic日志模块初始化函数---------------------------------*/

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/* -------------------------------电源低功耗接口函数------------------------------------*/

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
	ret_code_t err_code;
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
	uint8_t err_code = 0;
	// if ((NRF_LOG_PROCESS() == false) && (g_sleep_needed ==1))   //满足进入超低功耗条件
	if (g_sleep_needed == 1) // csy_1227 满足进入超低功耗条件	//under charging status, or normal status but bat volt below 3.4v. then mcu step into lowpower
	{
		if (g_ble_connect_flag == 1) // disconnect BLE
		{
			// 如果在蓝牙连接状态，断开连接
			// err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			// if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
			// {
			// 	APP_ERROR_CHECK(err_code);
			// 	// NRF_LOG_INFO("disconnect err code:%d", err_code);
			// }
			// else
			// {
			// 	// 修改连接连接状态及变化标志
			// 	g_ble_connect_flag = 0;
			// 	g_ble_connect_changed = 1;
			// }
		}
		if (peri_poweroff_flag == 0) // if bmx160 power on
		{
			// 非采集状态下(idle lowpower + ble disconnect + stop sample), turn off SPI , then power off BMX160
			// uint8_t state = nrf_gpio_pin_out_read(GPIO_MEMS_PWR_EN);
			// if (state == 0x0)
			//{
			// NRF_LOG_INFO("MEMS power off:%d", nrf_gpio_pin_out_read(GPIO_MEMS_PWR_EN));
			nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
												//}
			peri_poweroff_flag = 1;				// power off
		}
		if (spi_init_flag == 1)
		{
			spi_disable(); // csy_0316: turn off the spi
			spi_init_flag = 0;
			// NRF_LOG_INFO("spi_uninit: idle_state_handle");
		}
		// MCU进入低功耗状态
		nrf_pwr_mgmt_run();
		// NRF_LOG_INFO("idle_state_handle: mcu wake up");//csy_0205
	}
	// else if((NRF_LOG_PROCESS() == false) &&(g_ble_connect_flag == 0))//normal mode , ble disconnect
	else if (g_ble_connect_flag == 0) ////csy_1227 normal mode , ble disconnect
	{
		// 连接断开状态，MCU进入低功耗状态
		// g_acquire_enable = 0;  //主动关闭数据采集
		if (peri_poweroff_flag == 0)
		{
			// 关闭bmx160的电源  //csy_1227
			// uint8_t state = nrf_gpio_pin_out_read(GPIO_MEMS_PWR_EN);
			// if (state == 0x0)
			//{
			nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
			//}
			peri_poweroff_flag = 1; // csy_1227
		}
		if (spi_init_flag == 1)
		{
			spi_disable(); // csy_0316: turn off the spi
			spi_init_flag = 0;
		}
		nrf_pwr_mgmt_run();
		// NRF_LOG_INFO("idle_state_handle2: mcu wake up 2");//csy_0205
	}
}

/* -------------------------------BMX160用户接口函数------------------------------------*/
// 驱动中调用了博世SDK中的函数，实现了几乎所有BMX160提供的功能的支持
// 通过app_config.h中的宏来控制

/*! bmi160 Device address */
#define BMI160_DEV_ADDR BMI160_I2C_ADDR

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

/*! @brief variable to hold the bmi160 mag data */
struct bmi160_sensor_data bmi160_mag;

extern volatile uint8_t int1_flag;
extern volatile uint8_t int2_flag;

#if defined(STEP_COUNTER)
uint16_t bmi160_step_count = 0; // stores the step counter value
#endif

#if defined(FIFO_POLL) || defined(FIFO_WM_INT) || defined(STEP_COUNTER)

uint16_t bmi160_fifo_ready = 0;
uint8_t bmi160_step_print = 0;

/* Declare memory to store the raw FIFO buffer information */
uint8_t fifo_buff[FIFO_SIZE];

/* Modify the FIFO buffer instance and link to the device instance */
struct bmi160_fifo_frame fifo_frame;
struct bmi160_sensor_data fifo_acc_data[10];
struct bmi160_sensor_data fifo_gyro_data[10];
struct bmi160_aux_data fifo_mag_data[10];

uint8_t acc_frames_req = 1;
uint8_t gyro_frames_req = 1;
uint8_t mag_frames_req = 1;

uint16_t bmi160_fifo_idx;
#endif

void print_bmi160_rslt(int8_t rslt)
{
	switch (rslt)
	{
	case BMI160_OK:
		/* Do nothing */
		break;
	case BMI160_E_NULL_PTR:
		NRF_LOG_INFO("Error [%d] : Null pointer", rslt);
		break;
	case BMI160_E_COM_FAIL:
		NRF_LOG_INFO("Error [%d] : Communication failure", rslt);
		break;
	case BMI160_E_DEV_NOT_FOUND:
		NRF_LOG_INFO("Error [%d] : Device not found", rslt);
		break;
	case BMI160_E_OUT_OF_RANGE:
		NRF_LOG_INFO("Error [%d] : Out of range", rslt);
		break;
	case BMI160_E_INVALID_INPUT:
		NRF_LOG_INFO("Error [%d] : Invalid input", rslt);
		break;
	case BMI160_E_ACCEL_ODR_BW_INVALID:
		NRF_LOG_INFO("Error [%d] : BMI160_E_ACCEL_ODR_BW_INVALID", rslt);
		break;
	case BMI160_E_GYRO_ODR_BW_INVALID:
		NRF_LOG_INFO("Error [%d] : BMI160_E_GYRO_ODR_BW_INVALID", rslt);
		break;
	case BMI160_E_LWP_PRE_FLTR_INT_INVALID:
		NRF_LOG_INFO("Error [%d] : BMI160_E_LWP_PRE_FLTR_INT_INVALID", rslt);
		break;
	case BMI160_E_LWP_PRE_FLTR_INVALID:
		NRF_LOG_INFO("Error [%d] : BMI160_E_LWP_PRE_FLTR_INVALID", rslt);
		break;
	case BMI160_E_AUX_NOT_FOUND:
		NRF_LOG_INFO("Error [%d] : BMI160_E_AUX_NOT_FOUND", rslt);
		break;
	case BMI160_FOC_FAILURE:
		NRF_LOG_INFO("Error [%d] : BMI160_FOC_FAILURE", rslt);
		break;
	case BMI160_READ_WRITE_LENGHT_INVALID:
		NRF_LOG_INFO("Error [%d] : BMI160_READ_WRITE_LENGHT_INVALID", rslt);
		break;
	default:
		NRF_LOG_INFO("Error [%d] : Unknown error code", rslt);
		break;
	}
}

#if defined(USE_EXT_BMM150)

#define BMM150_AUX_I2C_ADDRESS 0x10

/*! Auxiliary interface trim register */
#define BMI_AUX_IF_TRIM UINT8_C(0x68)
#define BMI_ASDA_PUPSEL_OFF UINT8_C(0x00)
#define BMI_ASDA_PUPSEL_40K UINT8_C(0x01)
#define BMI_ASDA_PUPSEL_10K UINT8_C(0x02)
#define BMI_ASDA_PUPSEL_2K UINT8_C(0x03) /* User defined auxiliary read function */

int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	(void)id;
	return bmi160_aux_read(reg_addr, reg_data, len, &bmi160dev);
}

int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	(void)id;
	return bmi160_aux_write(reg_addr, reg_data, len, &bmi160dev);
}

struct bmm150_dev bmm150 =
	{
		.intf = BMM150_I2C_INTF,
		.read = (bmm150_com_fptr_t)bmm150_aux_read,
		.write = (bmm150_com_fptr_t)bmm150_aux_write,
		.delay_ms = (bmm150_delay_fptr_t)nrf_delay_ms,
};

int8_t app_open_bmi160_aux(struct bmi160_dev *dev) // mag init set
{
	int8_t rslt = BMI160_OK;
	uint8_t regdata;
	uint8_t bmm150_data_start = BMM150_DATA_X_LSB;

	// NRF_LOG_INFO("app_open_bmi160_aux,%d", dev->aux_cfg.aux_sensor_enable);

#if 1
	regdata = BMI_ASDA_PUPSEL_2K;
	rslt = bmi160_set_regs(BMI_AUX_IF_TRIM, &regdata, 1, dev);

	dev->aux_cfg.aux_sensor_enable = BMI160_ENABLE;			// auxiliary sensor enable
	dev->aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR; // auxiliary sensor address
	dev->aux_cfg.manual_enable = BMI160_ENABLE;				// setup mode enable
	// csy_1219 dev->aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_0;	// burst read of 2 byte  //csy  burst_len is 1 byte.
	dev->aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; // csy_1219 burst read of 8 byte
	dev->aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;		   // ODR setting need keep Synchronize bmm150_set_presetmode(), for debug use
	// dev->aux_cfg.aux_odr = BMI160_AUX_ODR_50HZ;//csy_1206
	/* Initialize the auxiliary sensor interface */
	rslt = bmi160_aux_init(dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160 aux init error:%d\n", rslt);
		return rslt;
	}
	// rslt = bmi160_config_aux_mode(dev);
	// if (rslt != BMI160_OK)
	//{
	//	NRF_LOG_INFO("bmi160 config aux mode error:%d\n", rslt);
	//	return rslt;
	// }

	/* Initialize bmm150 */
	rslt = bmm150_init(&bmm150);
	if (rslt != BMM150_OK)
	{
		NRF_LOG_INFO("bmm150 init error:%d\n", rslt);
		return rslt;
	}
#endif

	bmm150.settings.preset_mode = BMM150_PRESETMODE_REGULAR; // BMM150_PRESETMODE_ENHANCED;
	rslt = bmm150_set_presetmode(&bmm150);
	if (rslt != BMM150_OK)
	{
		NRF_LOG_INFO("bmm150 set op error:%d\n", rslt);
		return rslt;
	}
	/* Set the power mode to either normal or forced mode */
	bmm150.settings.pwr_mode = BMM150_FORCED_MODE;
	rslt = bmm150_set_op_mode(&bmm150);
	if (rslt != BMM150_OK)
	{
		NRF_LOG_INFO("bmm150 set op error:%d\n", rslt);
		return rslt;
	}

	/* Set the auxiliary sensor to auto mode */
	dev->aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ; // for debug use
	// dev->aux_cfg.aux_odr = BMI160_AUX_ODR_50HZ; //csy_1206// for debug use
	rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, dev); // set mag into data mode, set dev->aux_cfg.manual_enable = Disable
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160 set aux auto mode error:%d\n", rslt);
		return rslt;
	}

	return rslt;
}
#endif

#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
int8_t app_open_bmi160_fifo(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;
	// uint16_t index = 0;

#if defined(FIFO_WM_INT)
	struct bmi160_int_settg int_config;
#endif

	fifo_frame.data = fifo_buff;
	fifo_frame.length = 1024;
	dev->fifo = &fifo_frame;

	/* Clear FIFO configuration register */
	rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK, BMI160_DISABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("Clear FIFO configuration register error, error code: %d", rslt);
		return rslt;
	}

	// for 100 Hz ODR, with heading
#if defined(ACC_ONLY)
	rslt = bmi160_set_fifo_wm(43, dev); // 7*50=350 bytes, water mark value was word unit, cann't be greater than 255, 87*4=348
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_wm error, error code: %d", rslt);
	}
	rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_HEADER, BMI160_ENABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_config error, error code: %d", rslt);
	}
#elif defined(GYRO_ONLY)
	bmi160_set_fifo_wm(87, dev); // 7*50=350 bytes, water mark value was word unit, cann't be greater than 255, 87*4=348
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_wm error, error code: %d", rslt);
	}
	rslt = bmi160_set_fifo_config(BMI160_FIFO_GYRO | BMI160_FIFO_HEADER, BMI160_ENABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_config error, error code: %d", rslt);
	}
#elif defined(USE_EXT_BMM150) && !defined(GYRO_ONLY) && !defined(ACC_ONLY) && !defined(ACC_GYRO)
	bmi160_set_fifo_wm(87, dev); // 7*5=35 bytes, water mark value was word unit, cann't be greater than 255, 87*4=348
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_wm error, error code: %d", rslt);
	}
	rslt = bmi160_set_fifo_config(BMI160_FIFO_AUX | BMI160_FIFO_HEADER, BMI160_ENABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_config error, error code: %d", rslt);
	}
	//_get_mag_config_fifo(dev);
#elif defined(ACC_GYRO) && !defined(USE_EXT_BMM150)
	// 13*5=65 bytes, water mark value was word unit, cann't be greater than 255, 16.25*4=65
	bmi160_set_fifo_wm(16, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_wm error, error code: %d", rslt);
	}
	rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO | BMI160_FIFO_HEADER, BMI160_ENABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_config error, error code: %d", rslt);
	}
#elif defined(ACC_GYRO) && defined(USE_EXT_BMM150)
	// 19*2=38 bytes, water mark value was word unit, cann't be greater than 255, 9.5*4=38
	// csy_1219 bmi160_set_fifo_wm(9, dev);  //int cannot report to NRF8205, because pin is not connect
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_wm error, error code: %d", rslt);
	}
	rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO | BMI160_FIFO_AUX | BMI160_FIFO_HEADER, BMI160_ENABLE, dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("bmi160_set_fifo_config error, error code: %d", rslt);
	}
#endif

	/* Flush FIFO */
	rslt = bmi160_set_fifo_flush(dev);
	if (rslt != BMI160_OK)
	{
		NRF_LOG_INFO("Flush FIFO error, error code: %d", rslt);
		return rslt;
	}

#if defined(FIFO_WM_INT)
	/* Select the Interrupt channel/pin */
	int_config.int_channel = BMI160_INT_CHANNEL_2; // Interrupt channel/pin 2

	/* Select the Interrupt type */
	int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT; // Choosing interrupt
	/* Select the interrupt channel/pin settings */
	int_config.int_pin_settg.output_en = BMI160_ENABLE;			// Enabling interrupt pins to act as output pin
	int_config.int_pin_settg.output_mode = BMI160_DISABLE;		// Choosing push-pull mode for interrupt pin
	int_config.int_pin_settg.output_type = BMI160_ENABLE;		// Choosing active high output
	int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;			// Choosing edge triggered output
	int_config.int_pin_settg.input_en = BMI160_DISABLE;			// Disabling interrupt pin to act as input
	int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; // non-latched output
	int_config.fifo_wtm_int_en = BMI160_ENABLE;					// Enable FIFO watermark interrupt

	/* Set the FIFO watermark interrupt */
	bmi160_set_int_config(&int_config, dev);
#endif

	return rslt;
}

int8_t app_close_bmi160_fifo(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

	return rslt;
}
#endif

#if defined(STEP_COUNTER)
int8_t app_open_bmi160_step_counter(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;
	uint8_t step_enable = 1; // enable the step counter

	rslt = bmi160_set_step_counter(step_enable, dev);

#if 0
	//struct bmp38x_acc_step_detect_int_cfg int_cfg;
	//PDEBUG("step_detector_en=%X, min_threshold=%X", int_cfg.step_detector_en, int_cfg.min_threshold);
	struct bmi160_int_settg int_cfg;
	int_cfg.int_type = BMI160_STEP_DETECT_INT;
	int_cfg.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;
	int_cfg.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_ROBUST;//BMI160_STEP_DETECT_NORMAL, BMI160_STEP_DETECT_SENSITIVE
	bmi160_set_int_config(&int_cfg, dev);
#endif
	return rslt;
}

int8_t app_close_bmi160_step_counter(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

	return rslt;
}
#endif

int8_t app_open_bmi160_tap(struct bmi160_dev *dev, uint8_t feature_enable)
{
	int8_t rslt = BMI160_OK;
	struct bmi160_int_settg int_config;

	if (feature_enable > 0)
	{
		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1; /* Interrupt channel/pin 1 */

		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en = BMI160_ENABLE;			/* Enabling interrupt pins to act as output pin */
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;		/* Choosing push-pull mode for interrupt pin */
		int_config.int_pin_settg.output_type = BMI160_ENABLE;		/* Choosing active low output */
		int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE;		/* Choosing edge triggered output */
		int_config.int_pin_settg.input_en = BMI160_DISABLE;			/* Disabling interrupt pin to act as input */
		int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; /* non-latched output */

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_SINGLE_TAP_INT; /* Choosing tap interrupt */

		/* Select the Any-motion interrupt parameters */
		int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_ENABLE; /* 1- Enable tap, 0- disable tap */
		int_config.int_type_cfg.acc_tap_int.tap_thr = 2;			/* Set tap threshold */
		int_config.int_type_cfg.acc_tap_int.tap_dur = 2;			/* Set tap duration */
		int_config.int_type_cfg.acc_tap_int.tap_shock = 0;			/* Set tap shock value */
		int_config.int_type_cfg.acc_tap_int.tap_quiet = 0;			/* Set tap quiet duration */
		int_config.int_type_cfg.acc_tap_int.tap_data_src = 1;		/* data source 0 : filter or 1 : pre-filter */

		/* Set the Any-motion interrupt */
		rslt = bmi160_set_int_config(&int_config, dev); /* sensor is an instance of the structure bmi160_dev  */
		NRF_LOG_INFO("bmi160_set_int_config(tap enable) status:%d", rslt);
	}
	else
	{
		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;
		int_config.int_pin_settg.output_en = BMI160_DISABLE; /* Disabling interrupt pins to act as output pin */
		int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_SINGLE_TAP_INT;			 /* Choosing Tap interrupt */
		int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_DISABLE; /* 1- Enable tap, 0- disable tap */

		/* Set the Data ready interrupt */
		rslt = bmi160_set_int_config(&int_config, dev); /* sensor is an instance of the structure bmi160_dev */
		NRF_LOG_INFO("bmi160_set_int_config(tap disable) status:%d", rslt);
	}

	return rslt;
}

int8_t app_open_bmi160_no_motion(struct bmi160_dev *dev, uint8_t feature_enable)
{
	int8_t rslt = BMI160_OK;
	struct bmi160_int_settg int_config;

	if (feature_enable > 0)
	{
		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1; /* Interrupt channel/pin 1 */

		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en = BMI160_ENABLE;					/* Enabling interrupt pins to act as output pin */
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;				/* Choosing push-pull mode for interrupt pin: 0, push pull; 1, open drain*/
		int_config.int_pin_settg.output_type = BMI160_ENABLE;				/* Choosing active low output: 0, active low; 1, active high*/
		int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;					/* Choosing edge triggered output: 0, level; 1, edge */
		int_config.int_pin_settg.input_en = BMI160_DISABLE;					/* Disabling interrupt pin to act as input */
		int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_10_MILLI_SEC; /* non-latched output */

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_SLOW_NO_MOTION_INT; /* Choosing no motion interrupt */

		/* Select the Any-motion interrupt parameters */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_x = BMI160_ENABLE;	 /*! no motion interrupt x */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_y = BMI160_ENABLE;	 /*! no motion interrupt y */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_z = BMI160_ENABLE;	 /*! no motion interrupt z */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_dur = 1;			 /*! no motion duration */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_sel = BMI160_ENABLE; /*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_src = 0;			 /*! data source 0- filter & 1 pre-filter*/
		int_config.int_type_cfg.acc_no_motion_int.no_motion_thres = 20;			 /*! no motion threshold */

		/* Set the Any-motion interrupt */
		rslt = bmi160_set_int_config(&int_config, dev); /* sensor is an instance of the structure bmi160_dev  */
		NRF_LOG_INFO("bmi160_set_int_config(no motion enable) status:%d", rslt);
	}
	else
	{
		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;
		int_config.int_pin_settg.output_en = BMI160_DISABLE; /* Disabling interrupt pins to act as output pin */
		int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_SLOW_NO_MOTION_INT;					  /* Choosing no motion interrupt */
		int_config.int_type_cfg.acc_no_motion_int.no_motion_sel = BMI160_DISABLE; /* 1- Enable tap, 0- disable tap */

		/* Set the Data ready interrupt */
		rslt = bmi160_set_int_config(&int_config, dev); /* sensor is an instance of the structure bmi160_dev */
		NRF_LOG_INFO("bmi160_set_int_config(no motion disable) status:%d", rslt);
	}

	return rslt;
}

int8_t app_open_bmi160_acc(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

/* Select the Output data rate, range of accelerometer sensor */
#if defined(TAP)
	dev->accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
#else
	dev->accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	// dev->accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;//csy_1206
#endif
	dev->accel_cfg.range = BMI160_ACCEL_RANGE_4G;

#if defined(SUPPORT_LOWPOWER)
	/* Select the power mode of accelerometer sensor */
	dev->accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
	dev->accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
#else
	/* Select the power mode of accelerometer sensor */
	dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
#endif
	dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(dev);
	// NRF_LOG_INFO("bmi160_set_sens_conf rslt=%d", rslt);

	return rslt;
}

int8_t app_close_bmi160_acc(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

	dev->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(dev);

	return rslt;
}

int8_t app_open_bmi160_gyro(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

	/* Select the Output data rate, range of Gyroscope sensor */
	dev->gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	// dev->gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;//csy_1206
	// csy_1219dev->gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
	dev->gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS; // csy_1219
#if defined(SUPPORT_LOWPOWER)
	/* Select the power mode of Gyroscope sensor */
	dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	dev->gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;
#else
	/* Select the power mode of Gyroscope sensor */
	dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
#endif

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(dev);

	return rslt;
}

int8_t app_close_bmi160_gyro(struct bmi160_dev *dev)
{
	int8_t rslt = BMI160_OK;

	dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(dev);

	return rslt;
}

static void _get_mag_config_fifo(struct bmi160_dev *hdev)
{
	/* Enter the data register of BMM150 to "auto_mode_addr" here it is 0x42 */
	uint8_t auto_mode_addr = 0x42;
	hdev->aux_cfg.aux_odr = BMI160_AUX_ODR_25HZ;
	bmi160_set_aux_auto_mode(&auto_mode_addr, hdev);

	/* Disable other FIFO settings */
	bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK, BMI160_DISABLE, hdev);

	/* Enable the required FIFO settings */
	bmi160_set_fifo_config(BMI160_FIFO_AUX | BMI160_FIFO_HEADER, BMI160_ENABLE, hdev);
}

#if defined(FIFO_POLL) || defined(STEP_COUNTER)
void bmi160_timer_callback(void)
{
	bmi160_fifo_ready = 1;
}
#endif

void print_bmi160_int_status(struct bmi160_dev *dev)
{
	// int8_t rslt = BMI160_OK;
	volatile union bmi160_int_status int_status;

	// if(int1_flag == 1)
	{
		memset((void *)int_status.data, 0x00, sizeof(int_status.data));
		bmi160_get_int_status(BMI160_INT_STATUS_ALL, (union bmi160_int_status *)&int_status, dev);
		NRF_LOG_INFO("data: 0x%02X, 0x%02X, 0x%02X, 0x%02X", int_status.data[0], int_status.data[1], int_status.data[2], int_status.data[3]);

		if (int_status.bit.step)
		{
			NRF_LOG_INFO("Step detector interrupt occured");
		}

		if (int_status.bit.anym)
		{
			NRF_LOG_INFO("Any motion interrupt occured");
		}

		if (int_status.bit.nomo)
		{
			NRF_LOG_INFO("No motion interrupt occured");
		}

		if (int_status.bit.flat_int)
		{
			NRF_LOG_INFO("Flat detection interrupt occured");
		}

		if (int_status.bit.s_tap)
		{
			NRF_LOG_INFO("Single TAP interrupt occured");
		}

		if (int_status.bit.d_tap)
		{
			NRF_LOG_INFO("Double TAP interrupt occured");
		}

		if (int_status.bit.tap_sign)
		{
			NRF_LOG_INFO("TAP sign 1");
		}
		else
		{
			NRF_LOG_INFO("TAP sign 0");
		}

		if (int_status.bit.orient)
		{
			NRF_LOG_INFO("Orientation interrupt occured");
		}

		if (int_status.bit.high_g)
		{
			NRF_LOG_INFO("High-g interrupt occured");
		}

		if (int_status.bit.low_g)
		{
			NRF_LOG_INFO("Low-g interrupt occured");
		}

		memset((void *)int_status.data, 0x00, sizeof(int_status.data));
		// int1_flag = 0;
	}
}

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief   internal API is used to initialize the sensor interface
 */

/*!
 * @brief   This internal API is used to initialize the bmi160 sensor with default
 */
static void init_bmi160(void);

/*!
 * @brief   This internal API is used to initialize the sensor driver interface
 */
static void init_bmi160_sensor_driver_interface(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi160 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160(void)
{
	int16_t rslt;

	rslt = bmi160_soft_reset(&bmi160dev);
	nrf_delay_ms(200);

	rslt = bmi160_init(&bmi160dev);
	if (rslt == BMI160_OK)
	{
		// NRF_LOG_INFO("Init bmx160 success,chip id:0x%x\n", bmi160dev.chip_id);
	}
	else
	{
		// NRF_LOG_INFO("Init bmx160 failed!\n");
	}

#if defined(ACC_ONLY)
	app_open_bm160_acc(&bmi160dev);
	// app_close_bmi160_gyro(&bmi160dev);
#elif defined(GYRO_ONLY)
	app_close_bmi160_acc(&bmi160dev);
	app_open_bmi160_gyro(&bmi160dev);

	/* Select the power mode */
	bmi160dev.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
	bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Set the Power mode  */
	rslt = bmi160_set_power_mode(&bmi160dev);
#elif defined(ACC_GYRO)
	app_open_bmi160_acc(&bmi160dev);
	app_open_bmi160_gyro(&bmi160dev);
#endif

#if defined(USE_EXT_BMM150)
	app_open_bmi160_aux(&bmi160dev);
#endif

#if defined(TAP)
	app_open_bmi160_tap(&bmi160dev, BMI160_ENABLE);
#endif

#if defined(NO_MOTION)
	app_open_bmi160_no_motion(&bmi160dev, BMI160_ENABLE);
#endif

#if defined(TAP) || defined(NO_MOTION)
	app_enable_mcu_int1_pin();
#endif

#if defined(FIFO_POLL)
	app_open_bmi160_fifo(&bmi160dev);
#elif defined(FIFO_WM_INT)
	app_open_bmi160_fifo(&bmi160dev);
	app_enable_mcu_int2_pin();
#endif
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */

#if (BMI160_INTERFACE_I2C == 1)
// TWI
#define TWI_INSTANCE_ID 0

static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

int8_t nrf52_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret = 0;
	uint8_t data_out[2];

	data_out[0] = reg_addr;
	memcpy(&data_out[1], data, len);

	ret = nrf_drv_twi_tx(&m_twi, dev_addr, data_out, len + 1, false);

	return ret;
}

int8_t nrf52_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	ret_code_t ret;

	ret = nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, true);
	if (ret != NRF_SUCCESS)
	{
		return ret;
	}
	nrf_delay_ms(2);

	ret = nrf_drv_twi_rx(&m_twi, dev_addr, data, len);
	if (ret != NRF_SUCCESS)
	{
		return ret;
	}
	nrf_delay_ms(2);

	return ret;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event->type)
	{
	case NRF_DRV_TWI_EVT_DONE:
		m_xfer_done = true;
		break;
	default:
		break;
	}
}
void twi_master_init(void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config = {
		.scl = TWI_SCL_M,
		.sda = TWI_SDA_M,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false};

	err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

#endif

/*!
 * @brief This API reads sensor data, stores it in
 * the bmi160_sensor_data structure pointer passed by the user.
 */
int8_t bmi160_get_mag_data(struct bmi160_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Array to store auxiliary data */
	uint8_t aux_data[8] = {0};

	/* Reading data from BMI160 data registers */
	rslt = bmi160_read_aux_data_auto_mode(aux_data, dev);
	if (rslt == BMI160_OK)
	{
		/*get aux data*/
		for (uint8_t i = 0; i < 8; i++)
		{
			// NRF_LOG_INFO("aux_data [%d] = %x\n", i, aux_data[i]);
		}

		/* Compensating the raw auxiliary data available from the BMM150 API */
		rslt = bmm150_aux_mag_data(aux_data, &bmm150);
		// printf("RAW mag data: %d,%d,%d,%d,%d,%d,%d,%d", aux_data[0], aux_data[1], aux_data[2], aux_data[3], aux_data[4], aux_data[5], aux_data[6], aux_data[7]);
		if (rslt == BMI160_OK)
		{
			NRF_LOG_INFO("Compensated MAG X:%d ,Y:%d,Z:%d", bmm150.data.x, bmm150.data.y, bmm150.data.z);
		}
	}
	else
	{
		NRF_LOG_INFO("bmi160_get_mag_data failed, rstl=%d", rslt);
	}

	return rslt;
}
/**@brief Application main function.
 */

static void init_bmi160_sensor_driver_interface(void)
{
#if (BMI160_INTERFACE_I2C == 1)
	/* I2C setup */
	/* link read/write/delay function of host system to appropriate
	 * bmi160 function call prototypes */
	bmi160dev.write = nrf52_write_i2c;
	bmi160dev.read = nrf52_read_i2c;
	bmi160dev.delay_ms = nrf_delay_ms;

	/* set correct i2c address */
	bmi160dev.id = BMI160_DEV_ADDR;
	bmi160dev.intf = BMI160_I2C_INTF;
#elif (BMI160_INTERFACE_SPI == 1)
	/* SPI setup */
	/* link read/write/delay function of host system to appropriate
	 *  bmi160 function call prototypes */
	bmi160dev.write = nrf52_write_spi;
	bmi160dev.read = nrf52_read_spi;
	bmi160dev.delay_ms = nrf_delay_ms;
	bmi160dev.id = NULL;
	bmi160dev.intf = BMI160_SPI_INTF;
#endif
}

/* -------------------------------蓝牙数据用户层发送接收数据处理相关函数------------------------------------*/

uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
	uint8_t i = 0;
	uint8_t checksum = data[0];

	for (i = 1; i < len; i++)
	{
		checksum ^= data[i];
	}

	return checksum;
}

static void ble_send_data(void *buf, uint8_t length)
{
	uint32_t err_code;

	do
	{
		uint16_t len = length;
		err_code = ble_nus_data_send(&m_nus, buf, &len, m_conn_handle);
		if ((err_code != NRF_ERROR_INVALID_STATE) &&
			(err_code != NRF_ERROR_RESOURCES) &&
			(err_code != NRF_ERROR_NOT_FOUND))
		{
			APP_ERROR_CHECK(err_code);
		}
	} while (err_code == NRF_ERROR_RESOURCES);

	// NRF_LOG_INFO("send: %d,max:%d",length,BLE_NUS_MAX_DATA_LEN);
}

static void nus_data_handler(ble_nus_evt_t *p_evt)
{
	uint32_t i;

	uint8_t tx_frame_buf[32];
	uint8_t checksum = 0;

#if 1
	if (p_evt->type == BLE_NUS_EVT_RX_DATA)
	{
		memcpy(&rx_frame_header, p_evt->params.rx_data.p_data, sizeof(st_ble_frame_header));
		checksum = calc_checksum(p_evt->params.rx_data.p_data + 1, rx_frame_header.length - 2);
		// NRF_LOG_INFO("checksum:0x%02x", checksum & 0xff);
		//  if ((rx_frame_header.flag == 0x5A) && (checksum == p_evt->params.rx_data.p_data[ble_frame_rx.length-1]))
		if (rx_frame_header.flag == 0x5A)
		{
			switch (rx_frame_header.command)
			{
			case CMD_REQ_GET_DEVICE_ID:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5 + strlen(DEVICE_SN);
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				memcpy(tx_frame_buf + sizeof(st_ble_frame_header), DEVICE_SN, strlen(DEVICE_SN)); // TODO
				tx_frame_buf[sizeof(st_ble_frame_header) + strlen(DEVICE_SN)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				break;
			case CMD_REQ_GET_VERSION:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5 + 2;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header) + 0] = 0x0C; // 固件版本号定义，0x0C对应12，V1.2
				tx_frame_buf[sizeof(st_ble_frame_header) + 1] = 0x01; // 硬件版本号定义
				tx_frame_buf[sizeof(st_ble_frame_header) + 2] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				break;

			case CMD_REQ_CALIBRATE:
				if (rx_frame_header.misc == 0)
				{
				}
				else if (rx_frame_header.misc == 1)
				{
				}
				else if (rx_frame_header.misc == 2)
				{
				}
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;
				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				break;

			case CMD_REQ_START_SAMPLE:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				g_acquire_enable = 1;
				g_active_upload_flag = 0;
				g_active_upload_idx = 0;
				break;
			case CMD_REQ_STOP_SAMPLE:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				g_acquire_enable = 0;
				g_active_upload_flag = 0;
				g_active_upload_idx = 0;
				break;
			case CMD_REQ_GET_SAMPLE:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				g_active_upload_flag = 0;
				packet_sequence_id = 0; // csy_1227

				if (g_acquire_enable)
				{
					// NRF_LOG_INFO("acquire type:%d", rx_frame_header.misc);
#if !defined(FIFO_POLL)
					if (rx_frame_header.misc == 0)
					{
						tx_frame_header.length = 5 + 6;
						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header), &bmi160_accel, 6);
						tx_frame_buf[sizeof(st_ble_frame_header) + 6] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
					}
					else if (rx_frame_header.misc == 1)
					{
						tx_frame_header.length = 5 + 6;
						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header), &bmi160_gyro, 6);
						tx_frame_buf[sizeof(st_ble_frame_header) + 6] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
					}
					else if (rx_frame_header.misc == 2)
					{
						tx_frame_header.length = 5 + 6;
						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header), &bmm150.data, 6);
						tx_frame_buf[sizeof(st_ble_frame_header) + 6] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
					}
					else if (rx_frame_header.misc == 3)
					{
						// packet number
						tx_frame_header.misc = 0x2;

						// #1
						tx_frame_header.sequence_id = 0;
						tx_frame_header.length = 5 + 15;
						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header) + 0, &bmi160_accel, 6);
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header) + 6, &bmi160_gyro, 6);
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header) + 12, &bmm150.data, 3);
						tx_frame_buf[sizeof(st_ble_frame_header) + 15] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
						// #2
						tx_frame_header.sequence_id = 1;
						tx_frame_header.length = 5 + 3;
						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						uint8_t *p = (uint8_t *)&bmm150.data + 3;
						memcpy(tx_frame_buf + sizeof(st_ble_frame_header), p, 3);
						tx_frame_buf[sizeof(st_ble_frame_header) + 3] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
					}
					else
#endif
						if (rx_frame_header.misc == 7)
					{
						g_active_upload_flag = 1;
						g_active_upload_idx = 0;

						tx_frame_header.flag = 0xA5;
						tx_frame_header.device_id = rx_frame_header.device_id;
						tx_frame_header.command = rx_frame_header.command;
						tx_frame_header.length = 5;
						tx_frame_header.sequence_id = g_tx_sequence_id++;
						tx_frame_header.misc = 0x7;

						memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
						tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
						ble_send_data(tx_frame_buf, tx_frame_header.length);
					}
				}
				else
				{
					tx_frame_header.length = 5;
					memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
					tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
					ble_send_data(tx_frame_buf, tx_frame_header.length);
				}

				break;
			case CMD_REQ_GET_POWER_STATUS:
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5 + 1;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;

				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = g_bat_volt / 20.0;
				tx_frame_buf[sizeof(st_ble_frame_header) + 1] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);
				break;

			case CMD_REQ_SOFT_RESET:				// csy_1227
				g_acquire_enable = 0;				// stop sample data
				nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
				nrf_delay_ms(200);
				NVIC_SystemReset(); // soft reset MCU
				break;

			case CMD_REQ_START_SAMEPLE_AND_SAVE:
			    /*开始获取传感器数据并存储*/
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				tx_frame_header.misc = 0x0;
				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);

				g_acquire_enable = 1;
				g_s_get_sensor_data_and_save = 1;
				g_s_erase_flag = 1;
				g_s_flash_read_offset = 0;

				break;
			case CMD_REQ_GET_SAVED_SENSOR_DATE:
				/*获取存储的传感器数据*/
				tx_frame_header.flag = 0xA5;
				tx_frame_header.device_id = rx_frame_header.device_id;
				tx_frame_header.command = rx_frame_header.command;
				tx_frame_header.length = 5;
				tx_frame_header.sequence_id = g_tx_sequence_id++;
				if((!queue_empty(&g_s_save_sensor_data_queue))||(g_s_flash_read_offset+g_s_flash_write_sensor_len < FLASH_AVAILABLE_SIZE))
				{
					//还有数据.
					tx_frame_header.misc = 0x0;
				}
				else
				{
					tx_frame_header.misc = 0x1;
				}
				memcpy(tx_frame_buf, &tx_frame_header, sizeof(st_ble_frame_header));
				tx_frame_buf[sizeof(st_ble_frame_header)] = calc_checksum(&tx_frame_buf[1], tx_frame_header.length - 2);
				ble_send_data(tx_frame_buf, tx_frame_header.length);

				g_s_send_sensor_data_flag = 1;

				break;

			default:;
			}
		}
		else
		{
			// NRF_LOG_INFO("checksum err:%x,%x", checksum, p_evt->params.rx_data.p_data[rx_frame_header.length - 1]);
		}
	}
#else
	if (p_evt->type == BLE_NUS_EVT_RX_DATA)
	{
		if(p_evt->params.rx_data.p_data[0] == 0x00)
		{
			NRF_LOG_INFO("rx 0x00");
			g_s_flash_read_offset = 0;
			g_acquire_enable = 1;
			g_s_get_sensor_data_and_save = 1;
			g_s_erase_flag = 1;
		}
		else if(p_evt->params.rx_data.p_data[0] == 0x01)
		{
			NRF_LOG_INFO("rx 0x01");
			g_s_send_sensor_data_flag = 1;
		}
	}
#endif
}
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t err_code;
	ble_nus_init_t nus_init;
	nrf_ble_qwr_init_t qwr_init = {0};

	// Initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	// Initialize NUS.
	memset(&nus_init, 0, sizeof(nus_init));

	nus_init.data_handler = nus_data_handler;

	err_code = ble_nus_init(&m_nus, &nus_init);
	APP_ERROR_CHECK(err_code);
}

void ble_send_packet(uint8_t* sensor_buf)
{
	st_ble_frame_header tx_header;
	uint8_t tx_buf[149];
	uint8_t *ptr_buf = NULL;

	tx_header.flag = 0xA5;
	tx_header.device_id = rx_frame_header.device_id;

	tx_header.command = 0x4;	// csy_1209 response ID.
	tx_header.length = 5 + 144; // csy_1227, 8 samples per packet
	tx_header.misc = 0;			// total 6 packets,payload 15 Bytes per packet

	ptr_buf = (uint8_t *)sensor_buf;

	tx_header.sequence_id = packet_sequence_id; // csy_1227 packet index
	memcpy(tx_buf, &tx_header, sizeof(st_ble_frame_header));
	memcpy(tx_buf + sizeof(st_ble_frame_header), ptr_buf, 144);
	tx_buf[sizeof(st_ble_frame_header) + 144] = calc_checksum(&tx_buf[1], tx_header.length - 2);
	ble_send_data(tx_buf, tx_header.length);
	packet_sequence_id++;
	if (packet_sequence_id >= 16)
	{
		packet_sequence_id = 0;
	}
}

/* -------------------------------电压采集相关函数------------------------------------*/
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
}

void saadc_init(void) // csy_0326
{
	ret_code_t err_code;
	// nrf_drv_saadc_config_t saadc_config;
	// saadc_config.low_power_mode = true;
	// saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;

	nrf_saadc_channel_config_t channel_config =
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	// err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);
}

// csy_0327
uint16_t usr_get_volt(void)
{
	nrf_saadc_value_t saadc_val;
	nrf_drv_saadc_sample_convert(0, &saadc_val);

	// NRF_LOG_INFO(" %d mV", 6 * 3 * saadc_val * 0.6 / 1024 * 1000 * 1.047);

	return (6 * 3 * saadc_val * 0.6 / 1024 * 1000 * 1.047); // coef = 1.047, test value
}

/* -------------------------------充电状态检测相关函数------------------------------------*/
// 0:charging 1: idle
uint8_t usr_get_charge_status()
{
	return nrf_gpio_pin_read(GPIO_CHARGE);
}

/* -------------------------------看门狗及喂狗相关函数------------------------------------*/
void wdt_event_handler(void)
{
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
	// 用户定时器，通过RTC实现的，在低功耗时仍然可以工作
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}

// RTC定时中断处理
void app_timer_callback(void)
{
	uint8_t err_code, volt_cnt;
	uint16_t tmp_val;

	// 喂狗
	nrfx_wdt_channel_feed(0);
	// WH 蓝牙链接才检测电池电量ADC
	if (g_ble_connect_flag == 1) // under ble connect,then every 1min check BAT volt
	{
		saadc_init();
		g_bat_volt = usr_get_volt();
		nrfx_saadc_uninit();
		// if ((usr_get_charge_status() == 0) || ((usr_get_charge_status() == 1)&&(g_bat_volt < 3400)))//under charging status, or normal status but bat volt below 3.4v. then mcu step into lowpower
		if (g_bat_volt < 3450) // csy_0328
		{
			g_sleep_needed = 1;
			g_acquire_enable = 0; // 主动关闭数据采集
		}
		else
		{
			g_sleep_needed = 0;
		}
	}
}

void endian_transfer(uint8_t* data, int len)
{
    uint8_t tmp;
    if(len%2 == 0)
    {
        for(int i=0; i<(len>>1); i++)
        {
            tmp = data[2*i];
            data[2*i] = data[2*i+1];
            data[2*i+1] = tmp;
        }
    }
}

int check_flash_data_is_valid(uint8_t* data, uint8_t len)
{
	uint8_t valid_flag = 0;
	uint8_t check_len = len;

	for(int i=0; i<check_len; i++)
	{
		if(0xFF != data[i])
		{
			valid_flag = 1;
		}
	}
	return valid_flag==1?0:1;
}

void ble_send_ram_and_flash_sensor_data(void)
{
	static int send_cnt;

	//将存储的传感器数据通过蓝牙发送出去.
	memset(g_s_send_sensor_buf, 0, SENSOR_DATA_LEN);
	if(!queue_empty(&g_s_save_sensor_data_queue))
	{
		uint8_t len = queue_out(&g_s_save_sensor_data_queue, g_s_send_sensor_buf);
		endian_transfer(g_s_send_sensor_buf, len);
		ble_send_packet(g_s_send_sensor_buf);
		send_cnt += 1;
	}
	else
	{
		//发送flash内的数据.
		if(g_s_flash_read_offset+g_s_flash_write_sensor_len < FLASH_AVAILABLE_SIZE)
		{
			NRF_LOG_INFO("read addr:0x%x", FLASH_START_ADDR+g_s_flash_read_offset);
			if(NRF_SUCCESS == nrf_flash_read(FLASH_START_ADDR+g_s_flash_read_offset, g_s_send_sensor_buf, SENSOR_DATA_LEN))
			{
				if(0 == check_flash_data_is_valid(g_s_send_sensor_buf, 144))
				{
					//认为当前读取的为有效数据.
					endian_transfer(g_s_send_sensor_buf, SENSOR_DATA_LEN);
					ble_send_packet(g_s_send_sensor_buf);
					send_cnt += 1;
				}
				g_s_flash_read_offset+=g_s_flash_write_sensor_len;
			}
			else
			{
				NRF_LOG_INFO("read error");
			}
		}
		else
		{
			NRF_LOG_INFO("queue and flash are empty, send finish,cnt:%d", send_cnt);
			g_s_send_sensor_data_flag = 0;
			send_cnt = 0;
		}
	}
}

int erase_save_sensor_flash_sector(void)
{
	//擦除用于存储传感器数据的flash空间.
	uint8_t erase_ret = 0;
	
	for(int i=0; i<FLASH_SECTOR_NUM; i++)
	{
		if(0 != nrf_flash_erase(FLASH_START_ADDR+SECTOR_SIZE*i))
		{
			return 1;
		}
	}
	return erase_ret;
}

void save_sensor_data_to_ram_and_flash(void* sensor_data, int len)
{
	//先把数据存到ram,ram不够时存储到flash.
	if(!queue_full(&g_s_save_sensor_data_queue))
	{
		queue_in(&g_s_save_sensor_data_queue, (uint8_t*)sensor_data, len);
	}
	else
	{
		//存储到flash内.
		if(g_s_flash_write_offset+g_s_flash_write_sensor_len < FLASH_AVAILABLE_SIZE)
		{
			NRF_LOG_INFO("write 0x%x", FLASH_START_ADDR+g_s_flash_write_offset);
			nrf_flash_write(FLASH_START_ADDR+g_s_flash_write_offset, sensor_data, len);
			g_s_flash_write_offset += g_s_flash_write_sensor_len;
		}
		else
		{
			//存储满了不再存储.
			NRF_LOG_INFO("queue and flash full!!");
			g_acquire_enable = 0;
			g_s_get_sensor_data_and_save = 0;
		}
	}
}

int main(void)
{
	uint8_t rslt = 0;
	struct bmi160_dev *dev;
	uint8_t err_code;
	uint8_t i;
	uint8_t Err_Timeout = 0;

	// 日志初始化
#if NRF_LOG_ENABLED
	log_init(); // csy_0205, low power mode, can disable log_init()
#endif // NRF_LOG_ENABLED

	// RTC定时器初始化
	timers_init();

	// GPIO初始化
	nrf_gpio_cfg_input(GPIO_CHARGE, GPIO_PIN_CNF_PULL_Pullup); // Charge status
	nrf_gpio_cfg_output(GPIO_LED_BLUE);
	nrf_gpio_pin_set(GPIO_LED_BLUE); // LED ON
	nrf_gpio_cfg_output(GPIO_MEMS_PWR_EN);
	nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
//	nrf_gpio_cfg_input(GPIO_BMX160_INT, NRF_GPIO_PIN_PULLUP);  //添加INT 检测

	// 蓝牙协议栈相关初始化
	power_management_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	g_ble_adv_flag = 1;
	NRF_LOG_INFO("%s,%d,ble init finish,ver:0x02", __FUNCTION__, __LINE__);
	//fstorage初始化
	nrf_flash_init();
	//queue初始化.
	queue_init(&g_s_save_sensor_data_queue);
	// 传感器接口初始化
	// Init bmx160,After sensor init introduce 200 msec sleep
#if (BMI160_INTERFACE_I2C == 1)
	twi_master_init();
#elif (BMI160_INTERFACE_SPI == 1)
// csy_0324 spi_master_init();
// csy_0324 spi_init_flag = 1;//csy_0205
#endif

	// csy_0324	init_bmi160_sensor_driver_interface();//include spi interface

	// 用户定时器  2000ms启动一次，检测电压和充电状态//
	err_code = app_timer_create(&app_timer, APP_TIMER_MODE_REPEATED, app_timer_callback);
	err_code = app_timer_start(app_timer, APP_TIMER_TICKS(60000), NULL); // csy_0325, modify to 60s

#if defined(FIFO_POLL) || defined(STEP_COUNTER)
	// rslt = app_timer_create(&bmi160_timer,APP_TIMER_MODE_REPEATED,bmi160_timer_callback);
	// rslt = app_timer_start(bmi160_timer,APP_TIMER_TICKS(1000),NULL);
#endif

	// 看门狗csy 0127, wdt test.
	nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
	err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
	APP_ERROR_CHECK(err_code);
	nrf_drv_wdt_enable();

	// 电压采集ADC初始化
	saadc_init();
	g_bat_volt = usr_get_volt(); // csy_1227 get bat volt at first time
	nrfx_saadc_uninit();		 // csy_0325
	nrf_gpio_pin_clear(GPIO_LED_BLUE);

	// Enter main loop.
	for (;;)
	{
#if NRF_LOG_ENABLED
		NRF_LOG_FLUSH();
#endif // NRF_LOG_ENABLED
		//  低功耗处理
		idle_state_handle();

		if (g_ble_connect_flag == 1)
		{
			if(1 == g_s_send_sensor_data_flag)
			{
				// send ram and flash sensor data 
				ble_send_ram_and_flash_sensor_data();
			}
			
			if (g_acquire_enable)
			{
				//先擦除内部flash.
				if(1 == g_s_erase_flag)
				{
					g_s_erase_flag = 0;
					erase_save_sensor_flash_sector();
				}
				if (spi_init_flag == 0)
				{
					spi_enable(); // csy_0316
					init_bmi160_sensor_driver_interface();
					spi_init_flag = 1;
				}
				if (peri_poweroff_flag == 1) ////数据采集使能，如果传感器没有上电，上电之
				{
					uint8_t state = nrf_gpio_pin_out_read(GPIO_MEMS_PWR_EN);
					if (state == 0x1)
					{
						nrf_gpio_pin_clear(GPIO_MEMS_PWR_EN); // bmx160 power on
						nrf_delay_ms(100);
						init_bmi160();
						peri_poweroff_flag = 0; // power on
					}
				}
#if defined(FIFO_POLL)			  // FIFO采集模式，定时采集
				nrf_delay_ms(78); // csy_1227, 80ms sample and transport to app
				// 去掉延迟改为int检测 WH
				// 添加INT 检测
				dev = &bmi160dev;
				fifo_frame.length = 8 * 25; // csy_1227
				acc_frames_req = 8;			// csy_1227
				gyro_frames_req = 8;		// csy_1227
				mag_frames_req = 8;			// csy_1227
				rslt = bmi160_get_fifo_data(dev);
				if (rslt == BMI160_OK)
				{
#if defined(USE_EXT_BMM150)
					/* Parse the FIFO data to extract mag data from the FIFO buffer */
					rslt = bmi160_extract_aux(fifo_mag_data, &mag_frames_req, dev);
#endif
#if defined(GYRO_ONLY) || defined(ACC_GYRO)
					/* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
					rslt = bmi160_extract_gyro(fifo_gyro_data, &gyro_frames_req, dev);
#endif
#if defined(ACC_ONLY) || defined(ACC_GYRO)
					/* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
					bmi160_extract_accel(fifo_acc_data, &acc_frames_req, dev);
#endif
					if (g_active_upload_flag)
					{
						// 传感器数据整理到发送缓冲区
						for (i = 0; i < 8; i++) // csy_1227
						{
							g_sensor_data_buf[i][0] = fifo_acc_data[i].x;
							g_sensor_data_buf[i][1] = fifo_acc_data[i].y;
							g_sensor_data_buf[i][2] = fifo_acc_data[i].z;
							g_sensor_data_buf[i][3] = fifo_gyro_data[i].x;
							g_sensor_data_buf[i][4] = fifo_gyro_data[i].y;
							g_sensor_data_buf[i][5] = fifo_gyro_data[i].z;
							bmm150_aux_mag_data(fifo_mag_data[i].data, &bmm150);
							g_sensor_data_buf[i][6] = bmm150.data.x;
							g_sensor_data_buf[i][7] = bmm150.data.y;
							g_sensor_data_buf[i][8] = bmm150.data.z;
						}
						ble_send_packet((uint8_t* )g_sensor_data_buf);
					}
					if(g_s_get_sensor_data_and_save == 1)
					{
						/*采集传感器数据并存储*/
						memset(g_s_sensor_data_buf, 0, SENSOR_DATA_LEN);
						for(i=0; i<8; i++)
						{
							g_s_sensor_data_buf[9*i+0] = fifo_acc_data[i].x;
							g_s_sensor_data_buf[9*i+1] = fifo_acc_data[i].y;
							g_s_sensor_data_buf[9*i+2] = fifo_acc_data[i].z;
							g_s_sensor_data_buf[9*i+3] = fifo_gyro_data[i].x;
							g_s_sensor_data_buf[9*i+4] = fifo_gyro_data[i].y;
							g_s_sensor_data_buf[9*i+5] = fifo_gyro_data[i].z;
							bmm150_aux_mag_data(fifo_mag_data[i].data, &bmm150);
							g_s_sensor_data_buf[9*i+6] = bmm150.data.x;
							g_s_sensor_data_buf[9*i+7] = bmm150.data.y;
							g_s_sensor_data_buf[9*i+8] = bmm150.data.z;
						}
						save_sensor_data_to_ram_and_flash(g_s_sensor_data_buf, SENSOR_DATA_LEN);
					}
				}
#else // 单次采集模式
				nrf_delay_ms(10);
				/* To read only Accel data */
				// rslt = bmi160_get_sensor_data(BMI160_TIME_SEL | BMI160_ACCEL_SEL, &bmi160_accel, NULL, &bmi160dev);

				/* To read only Gyro data */
				// rslt = bmi160_get_sensor_data(BMI160_TIME_SEL | BMI160_GYRO_SEL, NULL, &bmi160_gyro, &bmi160dev);

				/* To read both Accel and Gyro data */
				rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
				NRF_LOG_INFO("ACCEL X:%d,Y:%d,Z:%d", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);
				NRF_LOG_INFO("GYRO X:%d,Y:%d,Z:%d", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);

				/* To read mag data */
				// bmi160_get_mag_data(&bmi160dev);
				rslt = bmm150_read_mag_data(&bmm150);
				NRF_LOG_INFO("MAG X:%d,Y:%d,Z:%d", bmm150.data.x, bmm150.data.y, bmm150.data.z);

				g_sensor_data_buf[g_active_upload_idx][0] = bmi160_accel.x;
				g_sensor_data_buf[g_active_upload_idx][1] = bmi160_accel.y;
				g_sensor_data_buf[g_active_upload_idx][2] = bmi160_accel.z;
				g_sensor_data_buf[g_active_upload_idx][3] = bmi160_gyro.x;
				g_sensor_data_buf[g_active_upload_idx][4] = bmi160_gyro.y;
				g_sensor_data_buf[g_active_upload_idx][5] = bmi160_gyro.z;
				g_sensor_data_buf[g_active_upload_idx][6] = bmm150.data.x;
				g_sensor_data_buf[g_active_upload_idx][7] = bmm150.data.y;
				g_sensor_data_buf[g_active_upload_idx][8] = bmm150.data.z;

				if (g_active_upload_flag)
				{
					g_active_upload_idx++;
					if (g_active_upload_idx == 5)
					{
						// 发送传感器数据
						ble_send_packet((uint8_t* )g_sensor_data_buf);
						g_active_upload_idx = 0;
					}
				}
#endif
			}
			else
			{
				if (peri_poweroff_flag == 0) // power on
				{
					nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
														//}
					peri_poweroff_flag = 1;				// power off
				}
				if (spi_init_flag == 1)
				{
					spi_disable(); // csy_0316: turn off the spi
					spi_init_flag = 0;
				}
			}
		}
		else if (g_ble_connect_flag == 0)
		{
			// WH修改，这里的MOS管开关实际没有效果，IO口有漏电
			nrfx_saadc_uninit(); // csy 0325
			spi_disable();		 // csy 0316: turn off the spi
			nrf_gpio_cfg_default(GPIO_CHARGE);
			nrf_gpio_cfg_default(GPIO_VBAT_SENSE);
			nrf_gpio_cfg_default(GPIO_VBUS_SENSE);
			nrf_gpio_cfg_default(GPIO_BMX160_INT);
			nrf_gpio_cfg_output(GPIO_LED_BLUE);
			nrf_gpio_cfg_output(GPIO_MEMS_PWR_EN);
			nrf_gpio_pin_set(GPIO_MEMS_PWR_EN); // bmx160 power off
			nrf_gpio_pin_clear(GPIO_LED_BLUE);	// bmx160 power offg
		}
	}
}

/**
 * @
 */
