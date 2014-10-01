/* *****************************************************************************
 *
 * ************************************************************************** */

#include <stdlib.h>

#include "TM4C1294_leds.h"
#include "time_delays.h"


/* *****************************************************************************
 *
 * ************************************************************************** */
extern  uint32_t  leds_position;

/*extern  t_Scheduler timer[leds5x8];
extern  t_Scheduler timer[leds1on], timer[leds1off];
extern  t_Scheduler timer[leds2on], timer[leds2off];
extern  t_Scheduler timer[leds3on], timer[leds3off];
extern  t_Scheduler timer[leds4on], timer[leds4off];
extern  t_Scheduler timer[leds8x8];
extern  t_Scheduler timer[keys];
extern  t_Scheduler timer[fft];
extern  t_Scheduler timer[rnd];
extern  t_Scheduler timer[mouse];
*/



/* *****************************************************************************
 *
 * ************************************************************************** */
uint32_t  u32_rtc_time=0;
uint32_t  u32_rtc_date=0;



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_usr_init_scheduler_end_timers (void) {
    timer[leds5x8].timer_is_set = 1;
    timer[leds5x8].set_timer_limit  = 20; // set period for 5x8 7-segment display

    timer[leds8x8].timer_is_set = 1;
    timer[leds8x8].set_timer_limit  = 20; // set period for timer counter

    timer[keys].timer_is_set = 1;
    timer[keys].ready_to_use = 1;
    timer[keys].set_timer_limit  = 3000; // set period for 8x8 leds display

    timer[fft].timer_is_set = 1;
    timer[fft].ready_to_use = 1;
    timer[fft].set_timer_limit  = 200; // set period for 8x8 leds display

    timer[rnd].timer_is_set = 1;
    timer[rnd].ready_to_use = 1;
    timer[rnd].set_timer_limit  = 10000; // set period for random

    timer[mouse].timer_is_set = 1;
    timer[mouse].ready_to_use = 1;
    timer[mouse].set_timer_limit  = 10000; // set period for mouse

    timer[leds1on].timer_is_set = 1;
    timer[leds1on].set_timer_limit  = 5000; // set period for led as time OFF
    timer[leds1on].ready_to_use = 1;
    timer[leds1on].common_rule = &leds_position;
    timer[leds1off].timer_is_set = 1;
    timer[leds1off].set_timer_limit  = 1000;   // set period for led as time ON
    timer[leds1off].ready_to_use = 1;
    timer[leds1off].common_rule = &leds_position;

    timer[leds2on].timer_is_set = 1;
    timer[leds2on].set_timer_limit  = 5000; // set period for led as time OFF
    timer[leds2on].ready_to_use = 1;
    timer[leds2on].common_rule = &leds_position;
    timer[leds2off].timer_is_set = 1;
    timer[leds2off].set_timer_limit  = 1000;   // set period for led as time ON
    timer[leds2off].ready_to_use = 1;
    timer[leds2off].common_rule = &leds_position;

    timer[leds3on].timer_is_set = 1;
    timer[leds3on].set_timer_limit  = 5000; // set period for led as time OFF
    timer[leds3on].ready_to_use = 1;
    timer[leds3on].common_rule = &leds_position;
    timer[leds3off].timer_is_set = 1;
    timer[leds3off].set_timer_limit  = 1000;   // set period for led as time ON
    timer[leds3off].ready_to_use = 1;
    timer[leds3off].common_rule = &leds_position;

    timer[leds4on].timer_is_set = 1;
    timer[leds4on].set_timer_limit  = 5000; // set period for led as time OFF
    timer[leds4on].ready_to_use = 1;
    timer[leds4on].common_rule = &leds_position;
    timer[leds4off].timer_is_set = 1;
    timer[leds4off].set_timer_limit  = 1000;   // set period for led as time ON
    timer[leds4off].ready_to_use = 1;
    timer[leds4off].common_rule = &leds_position;

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
t_timer_stat delay_timer ( timer_name name, uint8_t cfg ) {
t_timer_stat err_code = _TIMER_NOT_READY;

	if (cfg==0) {
		timer[name].cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer[name].timer_is_set ) {
			timer[name].cur_timer_val++;
			if ( timer[name].cur_timer_val >= timer[name].set_timer_limit ) {
				timer[name].cur_timer_val=0;
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


