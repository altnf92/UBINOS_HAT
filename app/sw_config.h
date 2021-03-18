/*
 * sw_config.h
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */

#ifndef APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_
#define APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define HAT_SW_MODE_TEST_CENTRAL		0
#define HAT_SW_MODE_TEST_PERIPHERAL		1

#define HAT_SW_MODE_SETUP				HAT_SW_MODE_TEST_PERIPHERAL

// Definition : PAAR ID(4 Byte)
#define PAAR_ID_0					(0x11)
#define PAAR_ID_1					(0x17)
#define PAAR_ID_2					(0x35)
#define PAAR_ID_3					(0xD4)

#define PAAR_ID_0_FOR_APP			0x99


#define TEST_TARTGET_PAAR_ID_SIZE	4

#define TEST_TARTGET_PAAR_ID0	0x11
#define TEST_TARTGET_PAAR_ID1	0x17
#define TEST_TARTGET_PAAR_ID2	0x35
#define TEST_TARTGET_PAAR_ID3	0xD4

#define TEST_SEND_MSG_DELAY			500
#define TEST_SCAN_START_DELAY		500
#define TEST_ADV_START_DELAY		500
#define TEST_CONNECTION_DELAY		100
//////////////////////////////////////////////////////////
#define APP_TIMER_PRESCALER         0               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE			2               /**< Size of timer operation queues. */
//////////////////////////////////////////////////////////
#define UNDEFINED										0XFF

// Nordic BLE stack communication enable
#define NRF_NOTIFICATION_ENABLE				0x01
#define NRF_INDICATION_ENABLE				0x02
#define NRF_NOTI_INDI_ENABLE				0x03

/* status byte in Advertising packet payloads */
#define LIDx_STATUS									0x00
#define PNIP_STATUS									0x01
#define REQ_CONN_SOSP_STATUS						0x02
#define REQ_CONN_SMARTDEVICE_STATUS					0x04
#define REQ_LIDx_STATUS								0x08

/**< Name of device. Will be included in the advertising data. */
#define PAAR_DEVICE_NAME                	"HAT_TAG"

#define PAAR_DEVICE_ID_0 				0xBB
#define PAAR_DEVICE_ID_1 				0x00
#define PAAR_DEVICE_ID_2				0x7A
#define PAAR_DEVICE_ID_3 				(PAAR_TAG_DEVICE_TYPE <<2)

///////////////////////////////////////////////////////////////////////////
#endif /* APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_ */
