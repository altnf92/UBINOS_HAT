/*
 * LAP_main.c
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */


#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <malloc.h>

#include "HAT_main.h"

#include "ble_stack.h"
#include "LAP_api.h"
#include "LAP_main.h"
#include "ble_gap.h"
#include "ble_process.h"
#include "ble_profile.h"

#include "sw_config.h"
#include "hw_config.h"

#include "LED.h"


//----------------------------------------------------------------------------
#define PAC_TYPE_INDEX			0
#define PAC_SERVICE_INDEX		1
#define PAC_SEQ_INDEX			2
#define PAC_LEN_INDEX			3
#define PAC_CMD_INDEX			4

#define PAC_APP_ID_INDEX		5
#define PAC_APP_MAX_CRNT_INDEX	6
#define PAC_ACT_PWR_INDEX		6
#define PAC_ACT_LVL_INDEX		7


#define PAC_TYPE 				0x88
#define SERVICE_ID				0x17
#define SEQ_NUM					0x11

#define LEN_OPT					0x02
#define LEN_CAL					0x01
#define LEN_ACK					0x01
#define LEN_RES					0x04

#define CMD_OPT					0x01
#define CMD_CAL					0x02
#define CMD_RES					0x03
#define CMD_REQ					0x04
#define CMD_TST					0x09


#define CMD_ACK_OPT					0x01		// ACK for CMD
#define CMD_ACK_CAL					0x02
#define CMD_ACK_RES					0x03

// ACK for CALIB SEQ
#define MSG_CALIB_NOT_ACCESSIBLE	0x20
#define MSG_CALIB_ACCEPTED			0x21
#define MSG_CALIB_DONE				0x22
#define MSG_CHECK_SEQ_FAILED		0x30
#define MSG_CHECK_SEQ_WAIT_SUCCESS	0x31
#define MSG_CHECK_SEQ_SUCCESS		0x32

#define MAX_REPLY_COUNT				5


static uint8_t res_ack_flag = 0;
//----------------------------------------------------------------------------




uint16_t test_target_conn_handle = BLE_CONN_HANDLE_INVALID;
paar_uuidhandle test_target_uuid_handle;
uint8_t test_send_count = 0;

static msgq_pt LAP_msgq;

APP_TIMER_DEF(scan_fail_timeout_timer);

uint8_t test_paar_id[4] = {TEST_TARTGET_PAAR_ID0, TEST_TARTGET_PAAR_ID1,
									TEST_TARTGET_PAAR_ID2, TEST_TARTGET_PAAR_ID3};

static void send_test_msg_central(uint16_t conn_handle, uint16_t handle)
{
	uint8_t* temp_packet;

	temp_packet = (uint8_t*)malloc(PAAR_MAXIMUM_PACKET_SIZE);

	if(temp_packet != NULL)
	{
		memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);
		temp_packet[0] = test_send_count++;
		if(test_send_count >= 4)
			test_send_count = 1;

		printf("BLE send msg : test_msg %d\r\n", temp_packet[0]);

		LAP_send_ble_msg_central(conn_handle, handle, temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
	}
}

static void send_test_msg_peripheral()
{
	uint8_t* temp_packet;

	temp_packet = (uint8_t*)malloc(PAAR_MAXIMUM_PACKET_SIZE);

	if(temp_packet != NULL)
	{
		memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);
		temp_packet[PAC_CMD_INDEX] = CMD_TST;
		printf("BLE send msg : test_msg %d\r\n",temp_packet[0]);

		LAP_send_ble_msg_peripheral(temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
	}
}


//---------------------------------------------------------------------
void BLE_send_short(uint8_t sig)
{
	uint8_t* temp_packet;

	temp_packet = (uint8_t*)malloc(PAAR_MAXIMUM_PACKET_SIZE);

	if(temp_packet != NULL)
	{
		memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);
		temp_packet[PAC_TYPE_INDEX] 	= PAC_TYPE;
		temp_packet[PAC_SERVICE_INDEX] 	= SERVICE_ID;
		temp_packet[PAC_SEQ_INDEX] 		= SEQ_NUM;
		temp_packet[PAC_LEN_INDEX] 		= LEN_ACK;

		temp_packet[PAC_CMD_INDEX] 		= sig;
		printf("Short msg : %X\r\n", temp_packet[PAC_CMD_INDEX]);

		task_sleep(TEST_SEND_MSG_DELAY);

		LAP_send_ble_msg_peripheral(temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
	}

}
void BLE_send_res(uint8_t App_ID, uint8_t Act_ON, uint8_t Act_LVL)
{
	uint8_t* temp_packet;
	res_ack_flag = MAX_REPLY_COUNT;


	temp_packet = (uint8_t*)malloc(PAAR_MAXIMUM_PACKET_SIZE);

	if(temp_packet != NULL)
	{
		memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);
		temp_packet[PAC_TYPE_INDEX] 	= PAC_TYPE;
		temp_packet[PAC_SERVICE_INDEX] 	= SERVICE_ID;
		temp_packet[PAC_SEQ_INDEX] 		= SEQ_NUM;
		temp_packet[PAC_LEN_INDEX] 		= LEN_RES;

		temp_packet[PAC_CMD_INDEX] 		= CMD_RES;
		temp_packet[PAC_APP_ID_INDEX] 	= App_ID;
		temp_packet[PAC_ACT_PWR_INDEX] 	= Act_ON;
		temp_packet[PAC_ACT_LVL_INDEX] 	= Act_LVL;

		task_sleep(TEST_SEND_MSG_DELAY);

		printf("RES msg : PWR %d, LVL %d\r\n", temp_packet[PAC_ACT_PWR_INDEX], temp_packet[PAC_ACT_LVL_INDEX]);

		LAP_send_ble_msg_peripheral(temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
	}
}
//---------------------------------------------------------------------


static void send_cccd_handle_enable(uint16_t conn_handle, uint16_t cccd_handle)
{
	uint8_t* temp_packet;

	temp_packet = (uint8_t*)malloc(2);

	memset(temp_packet, 0, 2);

	temp_packet[0] = NRF_NOTI_INDI_ENABLE;		// ble notification msg 데이터
	temp_packet[1] = 0x00;

	printf("BLE send msg : CCCD enable\r\n");

	LAP_send_ble_msg_central(conn_handle, cccd_handle, temp_packet, 2);

}

static void processing_LAP_Central_Conn_timeout(LAPEvt_msgt LAP_evt_msg)
{
	printf("Connection timeout\r\n");
	printf("BLE SCAN start\r\n");
	uint8_t* temp_test_paar_id;

	temp_test_paar_id = (uint8_t*)malloc(TEST_TARTGET_PAAR_ID_SIZE);

	temp_test_paar_id[0] = TEST_TARTGET_PAAR_ID0;
	temp_test_paar_id[1] = TEST_TARTGET_PAAR_ID1;
	temp_test_paar_id[2] = TEST_TARTGET_PAAR_ID2;
	temp_test_paar_id[3] = TEST_TARTGET_PAAR_ID3;

	task_sleep(TEST_SCAN_START_DELAY);

	printf("BLE SCAN start\r\n");
	LAP_start_ble_scan(temp_test_paar_id);
}

static void processing_LAP_Central_Scan_timeout(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE SCAN timeout\r\n");

	uint8_t* temp_test_paar_id;

	temp_test_paar_id = (uint8_t*)malloc(TEST_TARTGET_PAAR_ID_SIZE);

	temp_test_paar_id[0] = TEST_TARTGET_PAAR_ID0;
	temp_test_paar_id[1] = TEST_TARTGET_PAAR_ID1;
	temp_test_paar_id[2] = TEST_TARTGET_PAAR_ID2;
	temp_test_paar_id[3] = TEST_TARTGET_PAAR_ID3;

	task_sleep(TEST_SCAN_START_DELAY);

	printf("BLE SCAN start\r\n");
	LAP_start_ble_scan(temp_test_paar_id);
}

static void processing_LAP_Central_Scan_result(LAPEvt_msgt LAP_evt_msg)
{
	ble_gap_evt_adv_report_t* target_adv_pkt;

	target_adv_pkt = PAAR_get_ble_scan_target_result();

	//save cccd/rx/tx handle
	LAP_save_uuid_handle((ble_gap_evt_adv_report_t*) target_adv_pkt, &test_target_uuid_handle);

	task_sleep(TEST_CONNECTION_DELAY);

	printf("BLE Connection : test Target\r\n");
	//start ble connection to target PAAR ID device
	LAP_start_ble_connect((ble_gap_evt_adv_report_t*) target_adv_pkt);
}

static void processing_LAP_Central_Connected(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE Central connect\r\n");

	uuidhandle temp_uuid_handle;

	int r;

	//save test_connection_handle
	test_target_conn_handle = LAP_evt_msg.conn_handle;

	task_sleep(TEST_SEND_MSG_DELAY);

	//send cccd enable
	send_cccd_handle_enable(test_target_conn_handle, test_target_uuid_handle.cccd_handle);

	task_sleep(TEST_SEND_MSG_DELAY);

	//send test msg
	send_test_msg_central(test_target_conn_handle, test_target_uuid_handle.rx_handle);
}

static void processing_LAP_Central_Disconnected(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE central disconnect\r\n");

	uint8_t* temp_test_paar_id;

	temp_test_paar_id = (uint8_t*)malloc(TEST_TARTGET_PAAR_ID_SIZE);

	temp_test_paar_id[0] = TEST_TARTGET_PAAR_ID0;
	temp_test_paar_id[1] = TEST_TARTGET_PAAR_ID1;
	temp_test_paar_id[2] = TEST_TARTGET_PAAR_ID2;
	temp_test_paar_id[3] = TEST_TARTGET_PAAR_ID3;

	task_sleep(TEST_SCAN_START_DELAY);

	printf("BLE SCAN start\r\n");
	LAP_start_ble_scan(temp_test_paar_id);
}

static void processing_LAP_Central_Data_Received(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE msg rv : %d\r\n", LAP_evt_msg.msg[0]);

	task_sleep(TEST_SEND_MSG_DELAY);
}

static void processing_LAP_Peripheral_Connected(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE Peripheral connect\r\n");
	test_target_conn_handle = LAP_evt_msg.conn_handle;
}

static void processing_LAP_Peripheral_Disconnected(LAPEvt_msgt LAP_evt_msg)
{

	task_sleep(TEST_ADV_START_DELAY);
	printf("BLE Peripheral disconnect\r\n");

	printf("BLE ADV start\r\n");
	LAP_start_ble_adv_LIDx();
}
static void processing_LAP_Peripheral_Data_Received(LAPEvt_msgt LAP_evt_msg){
	printf("CMD : %X\r\n", LAP_evt_msg.msg[PAC_CMD_INDEX]);

	uint8_t* msg;

	switch(LAP_evt_msg.msg[PAC_CMD_INDEX]){

	case CMD_OPT :
		BLE_send_short(CMD_ACK_OPT);

		msg = (uint8_t*)malloc(sizeof(uint8_t) * 2);

		msg[0] = LAP_evt_msg.msg[PAC_APP_ID_INDEX];
		msg[1] = LAP_evt_msg.msg[PAC_APP_MAX_CRNT_INDEX];

		HAT_event_send(HAT_APPLIANCE_ID_SET, 0, msg);
		break;

	case CMD_CAL :
		BLE_send_short(CMD_ACK_CAL);
		HAT_event_send(HAT_CALIB_REQUEST, 0, NULL);
		break;

	case CMD_REQ :
		HAT_event_send(HAT_SAADC_REQUEST, 0, NULL);
		break;

	case CMD_ACK_RES :
		HAT_event_send(HAT_LVL_ACK, 0, NULL);
		break;

	default : break;
	}

	task_sleep(TEST_SEND_MSG_DELAY);
}


static void processing_LAP_Central_event(LAPEvt_msgt LAP_evt_msg)
{
	switch(LAP_evt_msg.status)
	{
	case LAP_CENTRAL_ST_SCAN_TIMEOUT :
		processing_LAP_Central_Scan_timeout(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_CONN_TIMEOUT :
		processing_LAP_Central_Conn_timeout(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_SCAN_RESULT :
		processing_LAP_Central_Scan_result(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_CONNECTED :
		processing_LAP_Central_Connected(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_DISCONNECTED :
		processing_LAP_Central_Disconnected(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_DATA_RECEIVED :
		processing_LAP_Central_Data_Received(LAP_evt_msg);
		break;
	}
}

static void processing_LAP_Peripheral_event(LAPEvt_msgt LAP_evt_msg)
{
	switch(LAP_evt_msg.status)
	{
	case LAP_PERIPHERAL_ST_CONNECTED :
		processing_LAP_Peripheral_Connected(LAP_evt_msg);
		break;
	case LAP_PERIPHERAL_ST_DISCONNECTED :
		processing_LAP_Peripheral_Disconnected(LAP_evt_msg);
		break;
	case LAP_PERIPHERAL_ST_DATA_RECEIVED :
		processing_LAP_Peripheral_Data_Received(LAP_evt_msg);
		break;
	}
}

void processing_LAP_LIDx_event(LAPEvt_msgt LAP_evt_msg)
{

}

void processing_LAP_PNIP_event(LAPEvt_msgt LAP_evt_msg)
{

}

void processing_LAP_AMD_event(LAPEvt_msgt LAP_evt_msg)
{

}

void LAP_Protocol_start_operation()
{

#if(HAT_SW_MODE_SETUP == HAT_SW_MODE_TEST_CENTRAL)
	uint8_t* temp_test_paar_id;

	temp_test_paar_id = (uint8_t*)malloc(TEST_TARTGET_PAAR_ID_SIZE);

	temp_test_paar_id[0] = TEST_TARTGET_PAAR_ID0;
	temp_test_paar_id[1] = TEST_TARTGET_PAAR_ID1;
	temp_test_paar_id[2] = TEST_TARTGET_PAAR_ID2;
	temp_test_paar_id[3] = TEST_TARTGET_PAAR_ID3;

	printf("BLE SCAN start\r\n");
	LAP_start_ble_scan(temp_test_paar_id);
#elif(HAT_SW_MODE_SETUP == HAT_SW_MODE_TEST_PERIPHERAL)
	printf("BLE ADV start\r\n");
	task_sleep(TEST_ADV_START_DELAY);
	LAP_start_ble_adv_LIDx();
#endif
}

void scan_fail_timer_handler()
{
	/*
	LAP_start_ble_scan(NULL);

	app_timer_start(scan_fail_timeout_timer, APP_TIMER_TICKS(5000), NULL);
	*/
}


void LAP_main_task(void* arg){
	int r;
	LAPEvt_msgt LAP_evt_msg;

	ble_stack_init_wait();

	LAP_Protocol_start_operation();

	for (;;) {
		r = msgq_receive(LAP_msgq, (unsigned char*) &LAP_evt_msg);
		if (0 != r) {
			logme("fail at msgq_receive\r\n");
		} else {
			switch( LAP_evt_msg.event ){
			case LAP_CENTRAL_EVT :
				processing_LAP_Central_event(LAP_evt_msg);
				break;
			case LAP_PERIPHERAL_EVT :
				processing_LAP_Peripheral_event(LAP_evt_msg);
				break;


			case LAP_LIDx_EVT :
				processing_LAP_LIDx_event(LAP_evt_msg);
				break;
			case LAP_PNIP_EVT :
				processing_LAP_PNIP_event(LAP_evt_msg);
				break;
			case LAP_AMD_EVT :
				processing_LAP_AMD_event(LAP_evt_msg);
				break;
			}

			if( LAP_evt_msg.msg != NULL ){
				free(LAP_evt_msg.msg);
			}
		}
	}
}

void LAP_main_task_init(void){
	int r;

	r = msgq_create(&LAP_msgq, sizeof(LAPEvt_msgt), 20);
	if (0 != r) {
		printf("fail at msgq create\r\n");
	}

	r = task_create(NULL, LAP_main_task, NULL, task_gethighestpriority()-2, 512, NULL);
	if (r != 0) {
		printf("== LAP_main_task failed \n\r");
	} else {
		printf("== LAP_main_task created \n\r");
	}
}


int LAP_event_send(uint8_t evt, uint8_t state, uint16_t conn_handle, uint16_t handle,
															uint32_t msg_len, uint8_t* msg)
{
	LAPEvt_msgt lap_msg;

	lap_msg.event = evt;
	lap_msg.status = state;
	lap_msg.handle = handle;
	lap_msg.conn_handle = conn_handle;
	lap_msg.msg_len = msg_len;
	lap_msg.msg = msg;

	return msgq_send(LAP_msgq, (unsigned char*) &lap_msg);
}
