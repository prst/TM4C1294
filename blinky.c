//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//******************************************************************************
#define _LEDS_POS_I_SET   ( GPIO_PORTA_AHB_DATA_R |=  (1<<2) ) // Anod (Common) 4
#define _LEDS_POS_I_CLR   ( GPIO_PORTA_AHB_DATA_R &= ~(1<<2) ) // Anod (Common) 4
#define _LEDS_POS_II_SET  ( GPIO_PORTP_DATA_R |=  (1<<3) )     // Anod (Common) 1
#define _LEDS_POS_II_CLR  ( GPIO_PORTP_DATA_R &= ~(1<<3) )     // Anod (Common) 1
#define _LEDS_POS_III_SET ( GPIO_PORTQ_DATA_R |=  (1<<1) )     // Anod (Common) 2
#define _LEDS_POS_III_CLR ( GPIO_PORTQ_DATA_R &= ~(1<<1) )     // Anod (Common) 2
#define _LEDS_POS_IV_SET  ( GPIO_PORTA_AHB_DATA_R |=  (1<<3) ) // Anod (Common) 3
#define _LEDS_POS_IV_CLR  ( GPIO_PORTA_AHB_DATA_R &= ~(1<<3) ) // Anod (Common) 3
#define _LEDS_POS_V_SET   ( GPIO_PORTM_DATA_R |=  (1<<6) )     // Anod (Common) 0
#define _LEDS_POS_V_CLR   ( GPIO_PORTM_DATA_R &= ~(1<<6) )     // Anod (Common) 0
//******************************************************************************


//******************************************************************************
typedef enum {
	_TIMER_OK     = 0,
	_TIMER_NOT_OK = 1
} t_timer_stat;
//******************************************************************************


//******************************************************************************
typedef struct {
	uint8_t  ready_to_use :1;
	uint8_t  timer_is_set :1;
	uint32_t delay_limit;
	uint32_t current;
} t_Scheduler;
//******************************************************************************

t_Scheduler timer_leds5x8;
t_Scheduler timer_leds1;
t_Scheduler timer_leds2;

uint32_t  dly1 = 0;
uint32_t  dly2 = 0;
uint32_t  dly3 = 0;
uint32_t  dly4 = 0;



//******************************************************************************
void drv_init_gpio ( void /*unsigned char val*/ ) {
    volatile uint32_t ui32Loop;

    // Enable the GPIO port that is used for the on-board LED.
    SYSCTL_RCGCGPIO_R =
		SYSCTL_RCGCGPIO_R14 | //SYSCTL_RCGCGPIO_R14=GPIO Port Q Run Mode Clock
		SYSCTL_RCGCGPIO_R13 | //SYSCTL_RCGCGPIO_R13=GPIO Port P Run Mode Clock
    	SYSCTL_RCGCGPIO_R12 | //SYSCTL_RCGCGPIO_R12=GPIO Port N Run Mode Clock
		SYSCTL_RCGCGPIO_R11 | //SYSCTL_RCGCGPIO_R11=GPIO Port M Run Mode Clock
    	SYSCTL_RCGCGPIO_R9  | //SYSCTL_RCGCGPIO_R9= GPIO Port K Run Mode Clock
    	SYSCTL_RCGCGPIO_R1  | //SYSCTL_RCGCGPIO_R1= GPIO Port B Run Mode Clock
    	SYSCTL_RCGCGPIO_R0  ; //SYSCTL_RCGCGPIO_R0= GPIO Port A Run Mode Clock

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop = SYSCTL_RCGCGPIO_R;

    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTN_DIR_R = 0x01;
    GPIO_PORTN_DEN_R = 0x01;
}
//******************************************************************************


//******************************************************************************
void drv_led_7segments_init ( void /*unsigned char val*/ ) {
    // Initial 7-segments
    GPIO_PORTA_AHB_DIR_R = 0xFF;  GPIO_PORTA_AHB_DEN_R = 0xFF;
    GPIO_PORTA_AHB_PP_R    |= 1;
    GPIO_PORTA_AHB_PC_R    |= 3;
    GPIO_PORTA_AHB_DR12R_R |= 1;
    GPIO_PORTA_AHB_DR8R_R  |= 1;
    GPIO_PORTA_AHB_DR4R_R  |= 1;

    GPIO_PORTB_AHB_DIR_R = 0xFF;  GPIO_PORTB_AHB_DEN_R = 0xFF;

    GPIO_PORTK_DIR_R     = 0xFF;  GPIO_PORTK_DEN_R     = 0xFF;

    GPIO_PORTP_DIR_R     = 0xFF;  GPIO_PORTP_DEN_R     = 0xFF;

    GPIO_PORTM_DIR_R     = 0xFF;  GPIO_PORTM_DEN_R     = 0xFF;

    GPIO_PORTQ_DIR_R     = 0xFF;  GPIO_PORTQ_DEN_R     = 0xFF;
}
//******************************************************************************


//******************************************************************************
// Possition in leds
//******************************************************************************
void drv_led_7segments_position (unsigned char pos) {
	switch ( pos ) {
		case 0: _LEDS_POS_I_SET; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 1: _LEDS_POS_I_CLR; _LEDS_POS_II_SET; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 2: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_SET; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 3: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_SET; _LEDS_POS_V_CLR; break;
		case 4: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_SET; break;
	}
	/*GPIO_PORTA_AHB_DATA_R |=  (1<<2); // Anod (Common) 0
	GPIO_PORTA_AHB_DATA_R |=  (1<<3); // Anod (Common) 1
	GPIO_PORTK_DATA_R     |=  (1<<3); // Anod (Common) 2
	GPIO_PORTP_DATA_R     |=  (1<<3); // Anod (Common) 3
	GPIO_PORTM_DATA_R     |=  (1<<6); // Anod (Common) 4*/
}
//******************************************************************************


//******************************************************************************
void drv_led_7segments_symbol ( char symbol ) {
    // Data to 7-segments
	GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
	GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
	GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
	GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
	GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
	GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
	GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
	GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H

	switch ( symbol ) {
		case 0:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			GPIO_PORTB_AHB_DATA_R &= ~(1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			//GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 1:
			//GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			//GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F - symbol 2 common?
			//GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 2:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			//GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			GPIO_PORTB_AHB_DATA_R &= ~(1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 3:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 4:
			//GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			//GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 5:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			//GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 6:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			//GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			GPIO_PORTB_AHB_DATA_R &= ~(1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 7:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			//GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
			//GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 8:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			GPIO_PORTB_AHB_DATA_R &= ~(1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 9:
			GPIO_PORTK_DATA_R     &= ~(1<<2); // Cathod - A
			GPIO_PORTA_AHB_DATA_R &= ~(1<<4); // Cathod - B
			GPIO_PORTK_DATA_R     &= ~(1<<1); // Cathod - C
			GPIO_PORTB_AHB_DATA_R &= ~(1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			GPIO_PORTK_DATA_R     &= ~(1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;
	}
}
//******************************************************************************


//******************************************************************************
int drv_led_blink (void) {
    volatile uint32_t ui32Loop;

    if (timer_leds1.ready_to_use) {
    	timer_leds2.ready_to_use=0;
        if ((timer_leds1.timer_is_set) && (++timer_leds1.current==timer_leds1.delay_limit)) {
    		timer_leds1.current=0;
        	timer_leds2.ready_to_use=1;

    		GPIO_PORTN_DATA_R |= 0x01;    // Turn on the LED.
    		//for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {}    // Delay for a bit.
    	}
    }

    if (timer_leds2.ready_to_use) {
    	timer_leds1.ready_to_use=0;
    	if ((timer_leds2.timer_is_set) && (++timer_leds2.current==timer_leds2.delay_limit)) {
		timer_leds2.current=0;
    	timer_leds1.ready_to_use=1;

		GPIO_PORTN_DATA_R &= ~(0x01);    // Turn off the LED.
		//for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {}    // Delay for a bit.
    	}
	}
}

//******************************************************************************


//******************************************************************************
t_timer_stat delay_timer_leds5x8 (uint8_t cfg) {

	if (cfg==0) {
		timer_leds5x8.current=0;
	}

	if (cfg=='?') {
		if ( timer_leds5x8.timer_is_set ) {
			if ( ++timer_leds5x8.current >= timer_leds5x8.delay_limit ) {
				return _TIMER_OK;
			} else {
				return _TIMER_NOT_OK;
			}
		} else {
			return _TIMER_NOT_OK;
		}
	}
}
//******************************************************************************


//******************************************************************************
// Blink the on-board LED.
//******************************************************************************
int main (void) {
    volatile uint32_t nn;
    t_timer_stat  t_stat;

    drv_init_gpio ();
    drv_led_7segments_init ();

    timer_leds1.timer_is_set = 1;
    timer_leds1.delay_limit  = 200000; // set period for led as time OFF
    timer_leds1.ready_to_use = 1;
    timer_leds2.timer_is_set = 1;
    timer_leds2.delay_limit  = 1000;   // set period for led as time ON
    timer_leds2.ready_to_use = 1;

    timer_leds5x8.timer_is_set = 1;
    timer_leds5x8.delay_limit  = 100; // set period for 5x8 7-segment display

    // Loop forever.
    while(1) {
		if (  _TIMER_OK == delay_timer_leds5x8('?') ) {
			if ( nn < 5 ) {
				drv_led_7segments_position ( nn );
				drv_led_7segments_symbol   ( nn );
				nn++;
    		} else {
				nn = 0;
    		}
			delay_timer_leds5x8(0);
    	}

    	drv_led_blink ();
    }
}
//******************************************************************************
