/*
 * HAT_main.c
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */

#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "HAT_main.h"
#include "ble_stack.h"
#include "Button.h"
#include "LED.h"
#include "TMR_PPI_SAADC.h"
#include "LAP_main.h"



#define MSG_CALIB_NOT_ACCESSIBLE	0x20
#define MSG_CALIB_ACCEPTED			0x21
#define MSG_CALIB_DONE				0x22
#define MSG_CHECK_SEQ_FAILED		0x30
#define MSG_CHECK_SEQ_WAIT_SUCCESS	0x31
#define MSG_CHECK_SEQ_SUCCESS		0x32

#define DEFAULT_APP_ID				0xFF
#define MAX_REPLY_CNT				5


#define APP_TICK_EVENT_INTERVAL     APP_TIMER_TICKS(2000)
APP_TIMER_DEF(m_tick_timer);


static msgq_pt HAT_msgq;
uint8_t ovr, lvl;
uint8_t Appliance_ID = DEFAULT_APP_ID;
uint8_t App_max_crnt = 15;
uint8_t res_reply_flag = MAX_REPLY_CNT;
ret_code_t err_code;

// extern!!
uint8_t option_flag = 0;		// 가전 ID 등의 옵션이 적용되었는가



void app_tick_handler(void* p_context){
	if(res_reply_flag > 0){
		LED_toggle(PIN_LED4);

		HAT_event_send(HAT_REPLY_TMR_EVT, 0, NULL);
		res_reply_flag--;
	}

	else{
		err_code = app_timer_stop(m_tick_timer);
		APP_ERROR_CHECK(err_code);

		res_reply_flag = MAX_REPLY_CNT;
	}
}


void HAT_main_task(void* arg){
	int r;
	HATEvt_msgt HAT_evt_msg;

	nrf_drv_gpiote_init();
	ble_stack_init_wait();
	LED_init();


	// saadc ------------------------------------------------------------------------------------------------------------------------------------------------------------
	nrf_drv_saadc_uninit();
	NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
	NVIC_ClearPendingIRQ(SAADC_IRQn);
	// Clear all previous unit settings and interrupt settings.


	saadc_sampling_event_init();		// Initializing TMR2 & PPI
	saadc_init();						// Initializing SAADC

	SAADC_PACKET saadc_packet;
	saadc_packet.CRNT_LVL = 0;
	saadc_packet.CRNT_OVR = 0;

	extern uint8_t calib_flag;
	// saadc ------------------------------------------------------------------------------------------------------------------------------------------------------------


    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_tick_timer, APP_TIMER_MODE_REPEATED, app_tick_handler);
    APP_ERROR_CHECK(err_code);



    /*	err_code = app_timer_start(m_tick_timer, APP_TICK_EVENT_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
     *
     * 	err_code = app_timer_stop(m_tick_timer);
		APP_ERROR_CHECK(err_code);
     */


    LED_toggle(PIN_LED1);
    LED_toggle(PIN_LED2);

	for (;;) {
		r = msgq_receive(HAT_msgq, (unsigned char*) &HAT_evt_msg);
		if (0 != r) {
			logme("fail at msgq_receive\r\n");
		}
		else {
			switch( HAT_evt_msg.event ){

			case HAT_APPLIANCE_ID_SET:
				Appliance_ID = HAT_evt_msg.msg[0];
				App_max_crnt = HAT_evt_msg.msg[1];
				// default ID - 0xFF, default max_crnt - 15

				option_flag = 1;
				break;

			case HAT_CALIB_REQUEST:
				saadc_off_calib();
				BLE_send_short(MSG_CALIB_ACCEPTED);
				break;
			case HAT_CALIB_DONE:
				BLE_send_short(MSG_CALIB_DONE);
				break;

			case HAT_CALIB_CHECK_WAIT_SUCCESS:
				BLE_send_short(MSG_CHECK_SEQ_WAIT_SUCCESS);
				break;

			case HAT_CALIB_CHECK_SUCCES:
				BLE_send_short(MSG_CHECK_SEQ_SUCCESS);
				break;
			case HAT_CALIB_CHECK_FAIL:
				saadc_sampling_event_disable();

				BLE_send_short(MSG_CHECK_SEQ_FAILED);
				break;


			case HAT_LVL_ACK:
				err_code = app_timer_stop(m_tick_timer);
				APP_ERROR_CHECK(err_code);

				res_reply_flag = MAX_REPLY_CNT;
				break;

			/* Three Events in Power Measurements.
			 * 1. Power on 	- Events occur in callback functions in SAADC.c.
			 * 2. Power off - Same as 1.
			 * 1, 2 events are both treated as PT_SAADC_LIMIT_BOUND.
			 * 3. Request from communication module - Event occurs in on_ble_evt in PT_ble_main.
			 * The way events occur is all different, but the tasks that must be done are all the same, so they are treated the same.*/
			//-------------------------------------------------------------------------
			case HAT_SAADC_LIMIT_BOUND:
			case HAT_SAADC_REQUEST:
			case HAT_CRNT_LVL_CHANGE:
				if(calib_flag == 2){
					saadc_get_packet(&saadc_packet, App_max_crnt);					// Get the current lvl.

					ovr = saadc_packet.CRNT_OVR;
					lvl = saadc_packet.CRNT_LVL;

					err_code = app_timer_start(m_tick_timer, APP_TICK_EVENT_INTERVAL, NULL);
				    APP_ERROR_CHECK(err_code);
			case HAT_REPLY_TMR_EVT:
					/* App ID + ON/OFF + LVL */
					BLE_send_res(Appliance_ID, ovr, lvl);
					LED_toggle(PIN_LED2);
				}
				break;
			//-------------------------------------------------------------------------
			default :
				break;

			}



			if( HAT_evt_msg.msg != NULL ){
				free(HAT_evt_msg.msg);
			}

		}
	}
}


void HAT_main_task_init(void){
	int r;

	r = msgq_create(&HAT_msgq, sizeof(HATEvt_msgt), 20);
	if (0 != r) {
		printf("fail at msgq create\r\n");
	}

	r = task_create(NULL, HAT_main_task, NULL, task_gethighestpriority()-2, 512, NULL);
	if (r != 0) {
		printf("== HAT_main_task failed \n\r");
	} else {
		printf("== HAT_main_task created \n\r");
	}
}

int HAT_event_send(uint8_t evt, uint8_t state, uint8_t* msg)
{
	HATEvt_msgt hat_msg;

	hat_msg.event = evt;
	hat_msg.status = state;
	hat_msg.msg = msg;

	return msgq_send(HAT_msgq, (unsigned char*) &hat_msg);
}
