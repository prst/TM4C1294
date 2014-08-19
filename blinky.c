//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// This is start code of the EK-TM4C1294XL.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"



//*****************************************************************************
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//! A very simple example that blinks the on-board LED using direct register
//! access.
//*****************************************************************************



//******************************************************************************
//#define _LEDS_POS_I_SET   ( GPIO_PORTA_AHB_DATA_R |=  (1<<2) ) // Anod (Common) 4
//#define _LEDS_POS_I_CLR   ( GPIO_PORTA_AHB_DATA_R &= ~(1<<2) ) // Anod (Common) 4
#define _LEDS_POS_I_SET   ( GPIO_PORTQ_DATA_R |=  (1<<3) ) // Anod (Common) 4
#define _LEDS_POS_I_CLR   ( GPIO_PORTQ_DATA_R &= ~(1<<3) ) // Anod (Common) 4
#define _LEDS_POS_II_SET  ( GPIO_PORTP_DATA_R |=  (1<<3) )     // Anod (Common) 1
#define _LEDS_POS_II_CLR  ( GPIO_PORTP_DATA_R &= ~(1<<3) )     // Anod (Common) 1
#define _LEDS_POS_III_SET ( GPIO_PORTQ_DATA_R |=  (1<<1) )     // Anod (Common) 2
#define _LEDS_POS_III_CLR ( GPIO_PORTQ_DATA_R &= ~(1<<1) )     // Anod (Common) 2
//#define _LEDS_POS_IV_SET  ( GPIO_PORTA_AHB_DATA_R |=  (1<<3) ) // Anod (Common) 3
//#define _LEDS_POS_IV_CLR  ( GPIO_PORTA_AHB_DATA_R &= ~(1<<3) ) // Anod (Common) 3
#define _LEDS_POS_IV_SET  ( GPIO_PORTQ_DATA_R |=  (1<<2) ) // Anod (Common) 3
#define _LEDS_POS_IV_CLR  ( GPIO_PORTQ_DATA_R &= ~(1<<2) ) // Anod (Common) 3
#define _LEDS_POS_V_SET   ( GPIO_PORTM_DATA_R |=  (1<<6) )     // Anod (Common) 0
#define _LEDS_POS_V_CLR   ( GPIO_PORTM_DATA_R &= ~(1<<6) )     // Anod (Common) 0
//******************************************************************************


// LEDS[0,1,2,3]=[PN1,PN0,PF4,PF0]
// USER_SWITCH[0,1]=[JP0,JP1]

//******************************************************************************
// TYPES
//******************************************************************************
typedef enum {
	_TIMER_OK     = 0,
	_TIMER_NOT_OK = 1
} t_timer_stat;

typedef struct {
	uint8_t    ready_to_use   :1;
	uint8_t    timer_is_set   :1;
	uint32_t   delay_limit;
	uint32_t   current;
	uint32_t  *common_rule;
} t_Scheduler;
//******************************************************************************



//******************************************************************************
// Global and system variables
//******************************************************************************
t_Scheduler timer_leds5x8;
t_Scheduler timer_leds1on, timer_leds1off;
t_Scheduler timer_leds2on, timer_leds2off;
t_Scheduler timer_leds3on, timer_leds3off;
t_Scheduler timer_leds4on, timer_leds4off;

uint32_t  dly1 = 0;
uint32_t  dly2 = 0;
uint32_t  dly3 = 0;
uint32_t  dly4 = 0;

uint32_t    leds_position = 1;
//******************************************************************************



//******************************************************************************
// void drv_init_gpio ( void )
// Initialisation for GPIO, basic configuration and default startup settings.
//******************************************************************************
void drv_init_gpio ( void ) {
    volatile uint32_t ui32Loop;

    // Enable the GPIO port that is used for the on-board LED.
    SYSCTL_RCGCGPIO_R =
		SYSCTL_RCGCGPIO_R14 | //SYSCTL_RCGCGPIO_R14=GPIO Port Q Run Mode Clock
		SYSCTL_RCGCGPIO_R13 | //SYSCTL_RCGCGPIO_R13=GPIO Port P Run Mode Clock
    	SYSCTL_RCGCGPIO_R12 | //SYSCTL_RCGCGPIO_R12=GPIO Port N Run Mode Clock
		SYSCTL_RCGCGPIO_R11 | //SYSCTL_RCGCGPIO_R11=GPIO Port M Run Mode Clock
    	SYSCTL_RCGCGPIO_R9  | //SYSCTL_RCGCGPIO_R9= GPIO Port K Run Mode Clock
    	SYSCTL_RCGCGPIO_R5  | //SYSCTL_RCGCGPIO_R5= GPIO Port F Run Mode Clock
    	SYSCTL_RCGCGPIO_R1  | //SYSCTL_RCGCGPIO_R1= GPIO Port B Run Mode Clock
    	SYSCTL_RCGCGPIO_R0  ; //SYSCTL_RCGCGPIO_R0= GPIO Port A Run Mode Clock

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop = SYSCTL_RCGCGPIO_R;

    // ..................................
    GPIO_PORTN_DIR_R     = 0;
    GPIO_PORTN_DEN_R     = 0;
    GPIO_PORTF_AHB_DIR_R = 0;
    GPIO_PORTF_AHB_DEN_R = 0;

    // Enable the GPIO pin for the LED (PN1).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTN_DIR_R |= 0x02;
    GPIO_PORTN_DEN_R |= 0x02;

    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTN_DIR_R |= 0x01;
    GPIO_PORTN_DEN_R |= 0x01;

    // Enable the GPIO pin for the LED (PF4).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTF_AHB_DIR_R |= 0x10;
    GPIO_PORTF_AHB_DEN_R |= 0x10;

    // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTF_AHB_DIR_R |= 0x01;
    GPIO_PORTF_AHB_DEN_R |= 0x01;
}
//******************************************************************************



//******************************************************************************
// void drv_led_7segments_init ( void )
// Initialisation for leds on board, basic configuration and default startup
// settings.
//******************************************************************************
void drv_led_7segments_init ( void ) {
    // Initial 7-segments
    GPIO_PORTA_AHB_DIR_R = 0xFF;  GPIO_PORTA_AHB_DEN_R = 0xFF;
    GPIO_PORTA_AHB_PP_R    = 1;
    GPIO_PORTA_AHB_PC_R    = 3;
    GPIO_PORTA_AHB_DR12R_R |= 1;
    GPIO_PORTA_AHB_DR8R_R  |= 1;
    GPIO_PORTA_AHB_DR4R_R  |= 1;

    GPIO_PORTB_AHB_DIR_R = 0xFF;  GPIO_PORTB_AHB_DEN_R = 0xFF;
    GPIO_PORTB_AHB_PP_R    = 1;
    GPIO_PORTB_AHB_PC_R    = 3;
    GPIO_PORTB_AHB_DR12R_R |= 1;
    GPIO_PORTB_AHB_DR8R_R  |= 1;
    GPIO_PORTB_AHB_DR4R_R  |= 1;

    GPIO_PORTK_DIR_R = 0xFF;  GPIO_PORTK_DEN_R     = 0xFF;
    GPIO_PORTK_PP_R    = 1;
    GPIO_PORTK_PC_R    = 3;
    GPIO_PORTK_DR12R_R |= 1;
    GPIO_PORTK_DR8R_R  |= 1;
    GPIO_PORTK_DR4R_R  |= 1;

    GPIO_PORTP_DIR_R = 0xFF;  GPIO_PORTP_DEN_R     = 0xFF;
    GPIO_PORTP_PP_R    = 1;
    GPIO_PORTP_PC_R    = 3;
    GPIO_PORTP_DR12R_R |= 1;
    GPIO_PORTP_DR8R_R  |= 1;
    GPIO_PORTP_DR4R_R  |= 1;

    GPIO_PORTM_DIR_R = 0xFF;  GPIO_PORTM_DEN_R     = 0xFF;
    GPIO_PORTM_PP_R    = 1;
    GPIO_PORTM_PC_R    = 3;
    GPIO_PORTM_DR12R_R |= 1;
    GPIO_PORTM_DR8R_R  |= 1;
    GPIO_PORTM_DR4R_R  |= 1;

    GPIO_PORTQ_DIR_R = 0xFF;  GPIO_PORTQ_DEN_R     = 0xFF;
    GPIO_PORTQ_PP_R    = 1;
    GPIO_PORTQ_PC_R    = 3;
    GPIO_PORTQ_DR12R_R |= 1;
    GPIO_PORTQ_DR8R_R  |= 1;
    GPIO_PORTQ_DR4R_R  |= 1;

    // PQ2, PA3, also PQ3, PA2 are use one pins
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));

}
//******************************************************************************



//******************************************************************************
// void drv_led_7segments_position (unsigned char pos)
// Initialisation for Possition in leds on 7 segments indicator, basic
// configuration and default startup settings.
//******************************************************************************
void drv_led_7segments_position (unsigned char pos) {
	/*
	GPIO_PORTA_AHB_DATA_R |=  (1<<2); // Anod (Common) 0
	GPIO_PORTA_AHB_DATA_R |=  (1<<3); // Anod (Common) 1
	GPIO_PORTK_DATA_R     |=  (1<<3); // Anod (Common) 2
	GPIO_PORTP_DATA_R     |=  (1<<3); // Anod (Common) 3
	GPIO_PORTM_DATA_R     |=  (1<<6); // Anod (Common) 4
	*/
	switch ( pos ) {
		case 0: _LEDS_POS_I_SET; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 1: _LEDS_POS_I_CLR; _LEDS_POS_II_SET; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 2: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_SET; _LEDS_POS_IV_CLR; _LEDS_POS_V_CLR; break;
		case 3: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_SET; _LEDS_POS_V_CLR; break;
		case 4: _LEDS_POS_I_CLR; _LEDS_POS_II_CLR; _LEDS_POS_III_CLR; _LEDS_POS_IV_CLR; _LEDS_POS_V_SET; break;
	}
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
// LEDS[0,1,2,3]=[PN1,PN0,PF4,PF0]
//******************************************************************************
int drv_led_blink (void) {
    volatile uint32_t ui32Loop;
    //.........................................................................
    if (*(uint32_t *)timer_leds4on.common_rule==(uint32_t *)1){
        if (timer_leds1on.ready_to_use) {
        	timer_leds1off.ready_to_use=0;
            if ((timer_leds1on.timer_is_set) && (++timer_leds1on.current==timer_leds1on.delay_limit)) {
        		timer_leds1on.current=0;
            	timer_leds1off.ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x02;    // Turn on the LED.
        	}
        }
        if (timer_leds1off.ready_to_use) {
        	timer_leds1on.ready_to_use=0;
        	if ((timer_leds1off.timer_is_set) && (++timer_leds1off.current==timer_leds1off.delay_limit)) {
    			timer_leds1off.current=0;
    			timer_leds1on.ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x02);    // Turn off the LED.
    			leds_position=2;
        	}
    	}
    }
    //.........................................................................
    if (*(uint32_t *)timer_leds1on.common_rule==(uint32_t *)2){
        if (timer_leds2on.ready_to_use) {
        	timer_leds2off.ready_to_use=0;
            if ((timer_leds2on.timer_is_set) && (++timer_leds2on.current==timer_leds2on.delay_limit)) {
        		timer_leds2on.current=0;
            	timer_leds2off.ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer_leds2off.ready_to_use) {
        	timer_leds2on.ready_to_use=0;
        	if ((timer_leds2off.timer_is_set) && (++timer_leds2off.current==timer_leds2off.delay_limit)) {
    			timer_leds2off.current=0;
    			timer_leds2on.ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x01);    // Turn off the LED.
    			leds_position=3;
        	}
    	}
    }
    //.........................................................................
    if (*(uint32_t *)timer_leds2on.common_rule==(uint32_t *)3){
        if (timer_leds3on.ready_to_use) {
        	timer_leds3off.ready_to_use=0;
            if ((timer_leds3on.timer_is_set) && (++timer_leds3on.current==timer_leds3on.delay_limit)) {
        		timer_leds3on.current=0;
            	timer_leds3off.ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x10;    // Turn on the LED.
        	}
        }
        if (timer_leds3off.ready_to_use) {
        	timer_leds3on.ready_to_use=0;
        	if ((timer_leds3off.timer_is_set) && (++timer_leds3off.current==timer_leds3off.delay_limit)) {
    			timer_leds3off.current=0;
    			timer_leds3on.ready_to_use=1;
    			GPIO_PORTF_AHB_DATA_R &= ~(0x10);    // Turn off the LED.
    			leds_position=4;
        	}
    	}
    }
    //.........................................................................
    if (*(uint32_t *)timer_leds3on.common_rule==(uint32_t *)4){
        if (timer_leds4on.ready_to_use) {
        	timer_leds4off.ready_to_use=0;
            if ((timer_leds4on.timer_is_set) && (++timer_leds4on.current==timer_leds4on.delay_limit)) {
        		timer_leds4on.current=0;
            	timer_leds4off.ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer_leds4off.ready_to_use) {
        	timer_leds4on.ready_to_use=0;
        	if ((timer_leds4off.timer_is_set) && (++timer_leds4off.current==timer_leds4off.delay_limit)) {
    			timer_leds4off.current=0;
    			timer_leds4on.ready_to_use=1;
    			GPIO_PORTF_AHB_DATA_R &= ~(0x01);    // Turn off the LED.
    			leds_position=1;
        	}
    	}
    }
    //.........................................................................
    return 0;
}

//******************************************************************************



//******************************************************************************
void drv_init_timer_and_scheduler (void) {
    timer_leds1on.timer_is_set = 1;
    timer_leds1on.delay_limit  = 50000; // set period for led as time OFF
    timer_leds1on.ready_to_use = 1;
    timer_leds1on.common_rule = &leds_position;
    timer_leds1off.timer_is_set = 1;
    timer_leds1off.delay_limit  = 10000;   // set period for led as time ON
    timer_leds1off.ready_to_use = 1;
    timer_leds1off.common_rule = &leds_position;

    timer_leds2on.timer_is_set = 1;
    timer_leds2on.delay_limit  = 50000; // set period for led as time OFF
    timer_leds2on.ready_to_use = 1;
    timer_leds2on.common_rule = &leds_position;
    timer_leds2off.timer_is_set = 1;
    timer_leds2off.delay_limit  = 10000;   // set period for led as time ON
    timer_leds2off.ready_to_use = 1;
    timer_leds2off.common_rule = &leds_position;

    timer_leds3on.timer_is_set = 1;
    timer_leds3on.delay_limit  = 50000; // set period for led as time OFF
    timer_leds3on.ready_to_use = 1;
    timer_leds3on.common_rule = &leds_position;
    timer_leds3off.timer_is_set = 1;
    timer_leds3off.delay_limit  = 10000;   // set period for led as time ON
    timer_leds3off.ready_to_use = 1;
    timer_leds3off.common_rule = &leds_position;

    timer_leds4on.timer_is_set = 1;
    timer_leds4on.delay_limit  = 50000; // set period for led as time OFF
    timer_leds4on.ready_to_use = 1;
    timer_leds4on.common_rule = &leds_position;
    timer_leds4off.timer_is_set = 1;
    timer_leds4off.delay_limit  = 10000;   // set period for led as time ON
    timer_leds4off.ready_to_use = 1;
    timer_leds4off.common_rule = &leds_position;

    timer_leds5x8.timer_is_set = 1;
    timer_leds5x8.delay_limit  = 200; // set period for 5x8 7-segment display
}
//******************************************************************************



//******************************************************************************
t_timer_stat delay_timer_leds5x8 (uint8_t cfg) {
uint8_t  rc;

	if (cfg==0) {
		timer_leds5x8.current=0;
	}

	if (cfg=='?') {
		if ( timer_leds5x8.timer_is_set ) {
			if ( ++timer_leds5x8.current >= timer_leds5x8.delay_limit ) {
				rc = _TIMER_OK;
			} else {
				rc = _TIMER_NOT_OK;
			}
		} else {
			rc = _TIMER_NOT_OK;
		}
	}
	return rc;
}
//******************************************************************************



//******************************************************************************
// Blinks on the leds and indicator.
//******************************************************************************
int main (void) {
    volatile uint32_t nn;
    //t_timer_stat  t_stat;
    uint8_t data[5]={'1','3','5','7','9'};

    drv_init_timer_and_scheduler ();
    drv_init_gpio ();
    drv_led_7segments_init ();

    // Loop forever.
    while(1) {
		if (  _TIMER_OK == delay_timer_leds5x8('?') ) {
			if ( nn < 5 ) {
				drv_led_7segments_position ( nn ); // dinamic switching
				drv_led_7segments_symbol   ( data[nn]&0x0F ); // display info
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
