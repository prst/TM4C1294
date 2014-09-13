/* *****************************************************************************
 *
 * ************************************************************************** */

#include <stdlib.h>

#include "TM4C1294_edu.h"
#include "time_delays.h"


/* *****************************************************************************
 *
 * ************************************************************************** */
extern  uint32_t  leds_position;

extern  t_Scheduler timer_leds5x8;
extern  t_Scheduler timer_leds1on, timer_leds1off;
extern  t_Scheduler timer_leds2on, timer_leds2off;
extern  t_Scheduler timer_leds3on, timer_leds3off;
extern  t_Scheduler timer_leds4on, timer_leds4off;
extern  t_Scheduler timer_leds8x8;
extern  t_Scheduler timer_keys;
extern  t_Scheduler timer_fft;
extern  t_Scheduler timer_rnd;



/* *****************************************************************************
 *
 * ************************************************************************** */
uint32_t  u32_rtc_time=0;
uint32_t  u32_rtc_date=0;



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_usr_init_scheduler_and_all_timers (void) {
    timer_leds5x8.timer_is_set = 1;
    timer_leds5x8.set_timer_limit  = 20; // set period for 5x8 7-segment display

    timer_leds8x8.timer_is_set = 1;
    timer_leds8x8.set_timer_limit  = 20; // set period for timer counter

    timer_keys.timer_is_set = 1;
    timer_keys.ready_to_use = 1;
    timer_keys.set_timer_limit  = 3000; // set period for 8x8 leds display

    timer_fft.timer_is_set = 1;
    timer_fft.ready_to_use = 1;
    timer_fft.set_timer_limit  = 200; // set period for 8x8 leds display

    timer_rnd.timer_is_set = 1;
    timer_rnd.ready_to_use = 1;
    timer_rnd.set_timer_limit  = 40000; // set period for 8x8 leds display

	timer_leds1on.timer_is_set = 1;
    timer_leds1on.set_timer_limit  = 5000; // set period for led as time OFF
    timer_leds1on.ready_to_use = 1;
    timer_leds1on.common_rule = &leds_position;
    timer_leds1off.timer_is_set = 1;
    timer_leds1off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds1off.ready_to_use = 1;
    timer_leds1off.common_rule = &leds_position;

    timer_leds2on.timer_is_set = 1;
    timer_leds2on.set_timer_limit  = 5000; // set period for led as time OFF
    timer_leds2on.ready_to_use = 1;
    timer_leds2on.common_rule = &leds_position;
    timer_leds2off.timer_is_set = 1;
    timer_leds2off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds2off.ready_to_use = 1;
    timer_leds2off.common_rule = &leds_position;

    timer_leds3on.timer_is_set = 1;
    timer_leds3on.set_timer_limit  = 5000; // set period for led as time OFF
    timer_leds3on.ready_to_use = 1;
    timer_leds3on.common_rule = &leds_position;
    timer_leds3off.timer_is_set = 1;
    timer_leds3off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds3off.ready_to_use = 1;
    timer_leds3off.common_rule = &leds_position;

    timer_leds4on.timer_is_set = 1;
    timer_leds4on.set_timer_limit  = 5000; // set period for led as time OFF
    timer_leds4on.ready_to_use = 1;
    timer_leds4on.common_rule = &leds_position;
    timer_leds4off.timer_is_set = 1;
    timer_leds4off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds4off.ready_to_use = 1;
    timer_leds4off.common_rule = &leds_position;

}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_usr_init_rtc (void) {
	// Select external 32.786 kHz Oscillator
	HIB_CTL_R |=  HIB_CTL_CLK32EN; // Clocking Enable
	HIB_CTL_R &= ~HIB_CTL_OSCSEL;  // Oscillator Select

	// Hibernation RTC Trim
	HIB_RTCT_R = 0x00007FFF;

	// Hibernation RTC Counter
	HIB_RTCC_R = 0;

	// Hibernation RTC Load
	HIB_RTCLD_R = 1;

	// Hibernation RTC Match 0
	HIB_RTCM0_R = 0xFFFFFFFF;

	// Hibernation RTC Sub Seconds
	HIB_RTCSS_R = 0;

	//--------------------------------------------------------------
	// {RTC Calendar/Counter Mode Select} or {Calendar Mode}
	HIB_CALCTL_R |= HIB_CALCTL_CALEN;
	HIB_CALCTL_R |= HIB_CALCTL_CAL24;

	// RTC ENABLE
	HIB_CTL_R    |=  HIB_CTL_RTCEN;   // RTC Timer Enable

	//--------------------------------------------------------------

	//Read RTC Time
	u32_rtc_time = HIB_CAL0_R;

	//Read RTC Date
	u32_rtc_date = HIB_CAL1_R;
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_timer_stat delay_timer_leds8x8 (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;
	if (cfg==0) {
		timer_leds8x8.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_leds8x8.timer_is_set ) {
			timer_leds8x8.cur_timer_val++;
			if ( timer_leds8x8.cur_timer_val >= timer_leds8x8.set_timer_limit ) {
				 timer_leds8x8.cur_timer_val=0;
				err_code = _TIMER_READY;
			} else {
				err_code = _TIMER_NOT_READY;
			}
		} else {
			err_code = _TIMER_NOT_READY;
		}
	}

	return err_code;
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_timer_stat delay_timer_keys (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;
	if (cfg==0) {
		timer_keys.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_keys.timer_is_set ) {
			timer_keys.cur_timer_val++;
			if ( timer_keys.cur_timer_val >= timer_keys.set_timer_limit ) {
				 timer_keys.cur_timer_val=0;
				err_code = _TIMER_READY;
			} else {
				err_code = _TIMER_NOT_READY;
			}
		} else {
			err_code = _TIMER_NOT_READY;
		}
	}

	return err_code;
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_timer_stat delay_timer_leds5x8 (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;

	if (cfg==0) {
		timer_leds5x8.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_leds5x8.timer_is_set ) {
			timer_leds5x8.cur_timer_val++;
			if ( timer_leds5x8.cur_timer_val >= timer_leds5x8.set_timer_limit ) {
				 timer_leds5x8.cur_timer_val=0;
				err_code = _TIMER_READY;
			} else {
				err_code = _TIMER_NOT_READY;
			}
		} else {
			err_code = _TIMER_NOT_READY;
		}
	}

	return err_code;
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_timer_stat delay_timer_fft (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;

	if (cfg==0) {
		timer_fft.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_fft.timer_is_set ) {
			timer_fft.cur_timer_val++;
			if ( timer_fft.cur_timer_val >= timer_fft.set_timer_limit ) {
				timer_fft.cur_timer_val=0;
				err_code = _TIMER_READY;
			} else {
				err_code = _TIMER_NOT_READY;
			}
		} else {
			err_code = _TIMER_NOT_READY;
		}
	}

	return err_code;
}
//******************************************************************************


/* *****************************************************************************
 *
 * ************************************************************************** */
t_timer_stat delay_timer_random_generator (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;

	if (cfg==0) {
		timer_rnd.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_rnd.timer_is_set ) {
			timer_rnd.cur_timer_val++;
			if ( timer_rnd.cur_timer_val >= timer_rnd.set_timer_limit ) {
				timer_rnd.cur_timer_val=0;
				err_code = _TIMER_READY;
			} else {
				err_code = _TIMER_NOT_READY;
			}
		} else {
			err_code = _TIMER_NOT_READY;
		}
	}

	return err_code;
}
//******************************************************************************


