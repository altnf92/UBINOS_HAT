/*
 * TMR_PPI_SAADC.h
 *
 *  Created on: 2020. 5. 27.
 *      Author: khs
 */

#include "HAT_main.h"
#include "TMR_PPI_SAADC.h"

#include "nrf_drv_ppi.h"


#include "LAP_main.h"



static nrf_saadc_value_t       m_buffer_pool[BUFFER_CNT_IN_USE][SAMPLES_IN_BUFFER];		// Two buffers receiving ADC data - each buffer contains SAMPLES_IN_BUFFER data.
static nrf_ppi_channel_t       m_ppi_channel;											// PPI CHANNEL setting variable
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_NUMBER);			// TMR setting variable - we use TMR2

int16_t saadc_limit 			= NRF_DRV_SAADC_LIMITH_DISABLED;
uint8_t limit_flag				= 0;
// Variables for limit event

//extern!!!
uint8_t calib_flag				= 0;					// 0 - No calibration has been made yet.
														// 1 - Will receive calib value at next TMR event cycle
														// 2 - Calibrated - can go back to 1(re-calibration).

uint8_t calib_cnt				= 0;					// calib cycle count
uint16_t calib_val				= 0;					// calibrated default value.

uint8_t check_off_time_flag		= 0;					// This variable decreases by 1 per cycle for the first time after the calibration until power on.
uint8_t check_on_time_flag		= 0;					// This variable decreases by 1 per cycle after power is on. When this variable reaches 0, the check sequence is terminated.
uint8_t check_success_flag		= 0;					// Set during the initial check sequence. Clears once the sensor value falls below the calibration.


uint32_t ticks					= 0;					// Major cycle ticks(Waiting)
uint32_t micro_ticks			= 0;					// Minor cycle micro_ticks(operating)
uint32_t tmp_ticks				= 0;					// variables for temporary storage at check sequence(after calib)
uint8_t cnt_micro				= 0;					// Variables to record the number of small-cycle tours.
uint8_t limit_cnt				= 0;					// Count how many values exceed the limit bounder within a cycle.


int16_t local_min				= RESOLUTION;				// Minimum value within a cycle - proportional to electrical power.
int16_t min_for_packet			= RESOLUTION;				// The Minimum value of the previous cycle. The value for transmission.


int8_t	current_lvl				= 0;					// measured current lvl.
int8_t	current_lvl_past		= 0;					// crnt lvl one cycle before.
int8_t	abs_lvl					= 0;					// crnt lvl independent from calib
uint16_t abs_zero_val			= 10750;

/* timer_handler
 * Reference for TMR Initialization
 * Not used */
void timer_handler(nrf_timer_event_t event_type, void* p_context){};


/* saadc_sampling_event_enable
 * Activate the PPI channel. */
void saadc_sampling_event_enable(void)
{
	ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}
/* saadc_sampling_event_disable
 * Deactivate the PPI channel. */
void saadc_sampling_event_disable(void)
{
	ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}


/* saadc_sampling_event_init
 * Initializing TMR2.
 * Setting PPI. */
void saadc_sampling_event_init(void)
{
	ret_code_t err_code;
	err_code = nrf_drv_ppi_init();
	APP_ERROR_CHECK(err_code);
	//	Initializing PPI.

	nrf_drv_timer_config_t t_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
	t_config.frequency = NRF_TIMER_FREQ_31250Hz;						// Select the slowest frequency as the timer frequency
	t_config.bit_width = NRF_TIMER_BIT_WIDTH_16;						// Timer Buffer Size
	// The rest of the settings are defaulted.

	err_code = nrf_drv_timer_init(&m_timer, &t_config, timer_handler);	// Initializing TMR
	APP_ERROR_CHECK(err_code);


	// Calculating ticks
	ticks = nrf_drv_timer_ms_to_ticks(&m_timer, TIMER_COMPARE_MS - SENSOR_RIPPLE_CYCLE);						// ex) 1000 - 20 = 980
	if(SAMPLING_TIMES != 0){
		micro_ticks = ticks * SENSOR_RIPPLE_CYCLE / (TIMER_COMPARE_MS - SENSOR_RIPPLE_CYCLE) / SAMPLING_TIMES;	// ex) [{980 * (20 / 980) } / sampling times]
		if(micro_ticks == 0) micro_ticks = ticks;
	}
	// TIMER_COMPARE_MS - SENSOR_RIPPLE_CYCLE : f a ripple exists in the sensor results,
	// a sampling event is generated as soon as the sensor ripple cycle is established. - ticks
	// Within the sampling handler, the timer period is reset to micro_ticks and sampled by SAMPLING_TIMES as of the first event.
	// After sampling as much as SAMPLING_TIMES, use the largest of the data, then change the timer period back to ticks.


	nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	// An event occurs every time(msec) corresponding to ticks, Empty the timer buffer at the event.

	nrf_drv_timer_enable(&m_timer);		// enable TMR

	uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
	uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();
	// Event addresses for tasks and triggers for PPI channel

	err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
	APP_ERROR_CHECK(err_code);
	// Assign a PPI channel to the handler value, and set up trigger and task.
}


/* saadc_callback
 * saadc event handler func
 * Event occurs when the sampling buffer is full. */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	if(p_event->type == NRF_DRV_SAADC_EVT_DONE){

		ret_code_t err_code;
		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);


		if(local_min > p_event->data.done.p_buffer[0]) local_min = p_event->data.done.p_buffer[0];
		// Find the maximum value within a cycle.
		if(p_event->data.done.p_buffer[0] < calib_val - LIMIT_BOUND) limit_cnt++;
		// Count how many values are beyond the bounder within a cycle.

		if(cnt_micro++ == 0 && SAMPLING_TIMES != 1){
			nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, micro_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
		}	// After the large cycle ticks are passed, replace the TMR compare cycle with a small cycle micro_ticks.
		else if(cnt_micro == SAMPLING_TIMES - 1 || SAMPLING_TIMES == 1){
			// When giving the last micro_ticks, replace the TMR cycle as a tick.
			nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

			min_for_packet = local_min;
			current_lvl_past = current_lvl;

			current_lvl 	= (calib_val - min_for_packet) 		* REVERSE_GAIN * VREF * 0.707/*RMS RATIO*/ / SENSITIVITY / RESOLUTION;
			abs_lvl			= (abs_zero_val - min_for_packet) 	* REVERSE_GAIN * VREF * 0.707/*RMS RATIO*/ / SENSITIVITY / RESOLUTION;
			// -------------------------------------------------------------- ????????? ??????
			//current_lvl = (max_for_packet - calib_val) * SENSOR_RANGE / RESOLUTION * 0.707/*RMS RATIO*/ * VREF;
			// 2??? amp ?????? -> ?????? ??????
			//




			// ------------------------------------- Calib & Check_Seq --------------------------------------------------
			/* Calib line step
			 * Make calib line by avg
			 * else - make a total value(CALIB_CYCLE times)
			 * if - 	make a avg val / evt send to main
			 * 			set the flag for next step(check_off_time_flag) */
			if(calib_flag == 1) {
				if(calib_cnt == 0){
					calib_flag = 2;
					calib_val = calib_val / CALIB_CYCLE;

					check_off_time_flag = CHECK_WAIT_CYCLE;					// Connection to next case

					HAT_event_send(HAT_CALIB_DONE, 0, NULL);
				}
				else{
					calib_cnt -= 1;
					calib_val += local_min;
				}
			}
			/* Wait step
			 * Waiting for pwr on
			 * Decrease the flag by 1 each time a cycle returns.
			 * if - If power is not detected while all flags are consumed, fail_evt is sent.
			 * else if - 	Set the flag for next step(check_on_time_flag) when power is detected.
			 * 				The cycle is switched(ticks => ticks / CHECK_SEQ_TIME_RATIO). */
			else if(check_off_time_flag != 0){
				check_off_time_flag--;

				if(check_off_time_flag == 0){
					calib_flag = 0;
					HAT_event_send(HAT_CALIB_CHECK_FAIL, 0, NULL);
				}
				else if(limit_cnt > LIMIT_BOUND_COUNT){
					check_on_time_flag = CHECK_SEQ_CYCLE;
					check_off_time_flag = 0;
					check_success_flag = 1;

					HAT_event_send(HAT_CALIB_CHECK_WAIT_SUCCESS, 0, NULL);
					tmp_ticks = ticks;
					ticks = ticks / CHECK_SEQ_TIME_RATIO;
				}

			}
			/* Check step
			 * Check if power is off during the period.
			 * Decrease the flag by 1 each time a cycle returns.
			 * if 1 - success_flag cleared when power is off.
			 * if 2 - All time flags consumed.
			 * 		if - Send success evt when flag is set
			 * 		else - Send fail evt */
			else if(check_on_time_flag != 0){
				check_on_time_flag -= 1;

				if(limit_cnt <= LIMIT_BOUND_COUNT){
					check_success_flag = 0;
				}

				if(check_on_time_flag == 0){
					ticks = tmp_ticks;

					if(check_success_flag) {
						HAT_event_send(HAT_CALIB_CHECK_SUCCES, 0, NULL);
					}
					else {
						calib_flag = 0;
						HAT_event_send(HAT_CALIB_CHECK_FAIL, 0, NULL);
					}
				}
			}
			// ------------------------------------- Calib & Check_Seq --------------------------------------------------

			else if(limit_cnt > LIMIT_BOUND_COUNT && !limit_flag) {
				// When the appliance is turned on
				// Outward flag - power on
				//---------------------------------------------------------------------------------
				limit_flag = 1;
				HAT_event_send(HAT_SAADC_LIMIT_BOUND, 0, NULL);
				//---------------------------------------------------------------------------------
			}
			else if(limit_flag){
				// After the limit event occurs, it is monitored to see if it goes down the bound again.
				if(limit_cnt < LIMIT_BOUND_COUNT){
					// Outward flag - power off
					//---------------------------------------------------------------------------------
					limit_flag = 0;
					//---------------------------------------------------------------------------------

					// Outward val
					//---------------------------------------------------------------------------------
					HAT_event_send(HAT_SAADC_LIMIT_BOUND, 0, NULL);
					//---------------------------------------------------------------------------------
				}


				else if(current_lvl - current_lvl_past >= 2 || current_lvl - current_lvl_past <= -2){
					HAT_event_send(HAT_CRNT_LVL_CHANGE, 0, NULL);
				}
				// ?????? ?????? ??? ????????? - ???????????? 3.0?????? ?????? ????????? ?????? ??? ???.

			}

			local_min = 16383;
			cnt_micro = 0;
			limit_cnt = 0;
			// ??? ??????????????? local_peak??? cnt_micro??? ???????????????.
		}

	}
}


/* saadc_init
 * SAADC ????????? ????????? ???????????? ???????????????.
 * AIN??? ?????? ????????? ??????, ???????????? ???????????????.
 * ????????? ????????? ????????? ??????????????????. */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config_1;

    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                 // ADC resolution 10bit
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           // ???????????? ???????????? oversample - ?????? ????????????
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               // Set SAADC interrupt to low priority.

    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         // ?????? ????????? ???????????? ????????? ???????????????.
    APP_ERROR_CHECK(err_code);

    //Configure SAADC channel
    channel_config_1.reference = NRF_SAADC_REFERENCE_VDD4;	                           		// NRF_SAADC_REFERENCE_VDD4 - VDD/4 | "_INTERNAL - 0.6V
    channel_config_1.gain = NRF_SAADC_GAIN1_3;              	                            // Gain
    channel_config_1.acq_time = NRF_SAADC_ACQTIME_40US;                                     // Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS.
    channel_config_1.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config_1.pin_p = NRF_SAADC_INPUT_AIN2;      // AIN4 	- POS                   // Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config_1.pin_n = NRF_SAADC_INPUT_DISABLED;  // NO PIN 	- NEG                   // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config_1.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              // Disable pullup resistor on the input pin
    channel_config_1.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              // Disable pulldown resistor on the input pin

    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL_A, &channel_config_1);							// ?????? ???????????? ????????? ???????????????.
    APP_ERROR_CHECK(err_code);
    nrf_drv_saadc_limits_set(SAADC_CHANNEL_A, NRF_DRV_SAADC_LIMITL_DISABLED, saadc_limit);				// ?????? 1?????? ????????? ????????? ??????????????? ????????????.


	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);			// m_buffer_pool??? ??????????????? ??????!
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);			// m_buffer_pool??? ??????????????? ??????!
	APP_ERROR_CHECK(err_code);
	// ???????????? ????????? ????????? ????????? ??????????????????. - ??????????????? ????????? ???????????? ?????? ???????????? ??????.
}


/* saadc_off_calib
 * calib_flag??? set?????? callback?????? calib ?????? ????????????.
 * ?????? ?????? ????????? PPI??? ???????????????. */
void saadc_off_calib(void){
	check_off_time_flag		= 0;
	check_on_time_flag		= 0;
	check_success_flag		= 0;

	calib_cnt = CALIB_CYCLE;
	calib_val = 0;

	local_min = 16383;
	cnt_micro = 0;
	limit_cnt = 0;

	if(calib_flag == 0){
		saadc_sampling_event_enable();		// PPI ??????
	}

	calib_flag = 1;
}


/* saadc_get_packet
 * ?????? ?????? ?????? - ?????? ????????? ?????? ?????? - ??? ????????? ???????????? ?????? */
void saadc_get_packet(SAADC_PACKET* node, uint8_t crnt_max){
	if(crnt_max <= abs_lvl) 				node->CRNT_OVR = 1;
	else									node->CRNT_OVR = 0;

	node->CRNT_LVL = current_lvl + limit_flag;
	if(node->CRNT_LVL > MAX_CRNT) node->CRNT_LVL = MAX_CRNT;
																		// MAX_CRNT??? ???????????? ?????? MAX_CRNT??? ?????????.
}
