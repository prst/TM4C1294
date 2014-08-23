//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// This is start code of the EK-TM4C1294XL.
//
//*****************************************************************************

#include <stdlib.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//typedef  enum { FALSE, TRUE } bool;



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
	_TIMER_READY     = 0,
	_TIMER_NOT_READY = 1
} t_timer_stat;

typedef struct {
	uint8_t    ready_to_use   :1;
	uint8_t    timer_is_set   :1;
	uint32_t   set_timer_limit;
	uint32_t   cur_timer_val;
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
t_Scheduler timer_count;

uint32_t  dly1 = 0;
uint32_t  dly2 = 0;
uint32_t  dly3 = 0;
uint32_t  dly4 = 0;

uint32_t    leds_position = 1;
//******************************************************************************




//******************************************************************************
void drv_sys_init_gpio ( void );
void drv_usr_init_led_7segments ( void );
void drv_led_7segments_position (unsigned char pos);
void drv_led_7segments_symbol ( uint8_t position, uint8_t symbol, uint8_t comma );
int drv_led_blink (void);
void drv_usr_init_scheduler_and_all_timers (void);
t_timer_stat delay_timer_count (uint8_t cfg);
t_timer_stat delay_timer_leds5x8 (uint8_t cfg);
char* IntToStr(int i, char b[]);
char* FloatToLeds5x8(float ff, char b[]);
char *ultostr(unsigned long value, char *ptr, int base);
char *ftostr(float value, char *ptr, int base, uint8_t *comma);
//******************************************************************************



//******************************************************************************
// void drv_init_gpio ( void )
// Initialisation for GPIO, basic configuration and default startup settings.
//******************************************************************************
void drv_sys_init_gpio ( void ) {
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
    //GPIO_PORTF_AHB_PCTL_R;
}
//******************************************************************************



//******************************************************************************
// void drv_led_7segments_init ( void )
// Initialisation for leds on board, basic configuration and default startup
// settings.
//******************************************************************************
void drv_usr_init_led_7segments ( void ) {
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
void drv_led_7segments_symbol ( uint8_t position, uint8_t symbol, uint8_t comma ) {
    // Data to 7-segments
	GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
	GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
	GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
	GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
	GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
	GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
	GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
	GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H

	if ( symbol==0x0D ) symbol = '-';

	switch ( symbol ) {
		case '-':
			//GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
			//GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
			//GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
			//GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
			GPIO_PORTA_AHB_DATA_R &= ~(1<<5); // Cathod - G
			//GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H
		break;

		case 0:
		case '0':
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
		case '1':
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
		case '2':
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
		case '3':
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
		case '4':
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
		case '5':
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
		case '6':
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
		case '7':
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
		case '8':
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
		case '9':
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

	if ( symbol==0x00 ) {
		//symbol = '.';
		//case '.':
			//GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
			//GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
			//GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
			//GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
			//GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
			//GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
			//GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
			GPIO_PORTK_DATA_R     &= ~(1<<0); // Cathod - H
		//break;
	}

	if (position==comma) { // Cathod - H
		GPIO_PORTK_DATA_R&=~(1<<0);
	}

	if (position==(0x30+comma)) { // Cathod - H
		GPIO_PORTK_DATA_R&=~(1<<0);
	}
}
//******************************************************************************



//******************************************************************************
// LEDS[0,1,2,3]=[PN1,PN0,PF4,PF0]
//******************************************************************************
int drv_led_blink (void) {
    volatile uint32_t ui32Loop;
    //.........................................................................
    if (*timer_leds4on.common_rule==1){
        if (timer_leds1on.ready_to_use) {
        	timer_leds1off.ready_to_use=0;
            if ((timer_leds1on.timer_is_set) && (++timer_leds1on.cur_timer_val==timer_leds1on.set_timer_limit)) {
        		timer_leds1on.cur_timer_val=0;
            	timer_leds1off.ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x02;    // Turn on the LED.
        	}
        }
        if (timer_leds1off.ready_to_use) {
        	timer_leds1on.ready_to_use=0;
        	if ((timer_leds1off.timer_is_set) && (++timer_leds1off.cur_timer_val==timer_leds1off.set_timer_limit)) {
    			timer_leds1off.cur_timer_val=0;
    			timer_leds1on.ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x02);    // Turn off the LED.
    			leds_position=2;
        	}
    	}
    }
    //.........................................................................
    if (*timer_leds1on.common_rule==2){
        if (timer_leds2on.ready_to_use) {
        	timer_leds2off.ready_to_use=0;
            if ((timer_leds2on.timer_is_set) && (++timer_leds2on.cur_timer_val==timer_leds2on.set_timer_limit)) {
        		timer_leds2on.cur_timer_val=0;
            	timer_leds2off.ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer_leds2off.ready_to_use) {
        	timer_leds2on.ready_to_use=0;
        	if ((timer_leds2off.timer_is_set) && (++timer_leds2off.cur_timer_val==timer_leds2off.set_timer_limit)) {
    			timer_leds2off.cur_timer_val=0;
    			timer_leds2on.ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x01);    // Turn off the LED.
    			leds_position=3;
        	}
    	}
    }
    //.........................................................................
    if (*timer_leds2on.common_rule==3){
        if (timer_leds3on.ready_to_use) {
        	timer_leds3off.ready_to_use=0;
            if ((timer_leds3on.timer_is_set) && (++timer_leds3on.cur_timer_val==timer_leds3on.set_timer_limit)) {
        		timer_leds3on.cur_timer_val=0;
            	timer_leds3off.ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x10;    // Turn on the LED.
        	}
        }
        if (timer_leds3off.ready_to_use) {
        	timer_leds3on.ready_to_use=0;
        	if ((timer_leds3off.timer_is_set) && (++timer_leds3off.cur_timer_val==timer_leds3off.set_timer_limit)) {
    			timer_leds3off.cur_timer_val=0;
    			timer_leds3on.ready_to_use=1;
    			GPIO_PORTF_AHB_DATA_R &= ~(0x10);    // Turn off the LED.
    			leds_position=4;
        	}
    	}
    }
    //.........................................................................
    if (*timer_leds3on.common_rule==4){
        if (timer_leds4on.ready_to_use) {
        	timer_leds4off.ready_to_use=0;
            if ((timer_leds4on.timer_is_set) && (++timer_leds4on.cur_timer_val==timer_leds4on.set_timer_limit)) {
        		timer_leds4on.cur_timer_val=0;
            	timer_leds4off.ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer_leds4off.ready_to_use) {
        	timer_leds4on.ready_to_use=0;
        	if ((timer_leds4off.timer_is_set) && (++timer_leds4off.cur_timer_val==timer_leds4off.set_timer_limit)) {
    			timer_leds4off.cur_timer_val=0;
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
void drv_usr_init_scheduler_and_all_timers (void) {
    timer_leds1on.timer_is_set = 1;
    timer_leds1on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds1on.ready_to_use = 1;
    timer_leds1on.common_rule = &leds_position;
    timer_leds1off.timer_is_set = 1;
    timer_leds1off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds1off.ready_to_use = 1;
    timer_leds1off.common_rule = &leds_position;

    timer_leds2on.timer_is_set = 1;
    timer_leds2on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds2on.ready_to_use = 1;
    timer_leds2on.common_rule = &leds_position;
    timer_leds2off.timer_is_set = 1;
    timer_leds2off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds2off.ready_to_use = 1;
    timer_leds2off.common_rule = &leds_position;

    timer_leds3on.timer_is_set = 1;
    timer_leds3on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds3on.ready_to_use = 1;
    timer_leds3on.common_rule = &leds_position;
    timer_leds3off.timer_is_set = 1;
    timer_leds3off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds3off.ready_to_use = 1;
    timer_leds3off.common_rule = &leds_position;

    timer_leds4on.timer_is_set = 1;
    timer_leds4on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds4on.ready_to_use = 1;
    timer_leds4on.common_rule = &leds_position;
    timer_leds4off.timer_is_set = 1;
    timer_leds4off.set_timer_limit  = 1000;   // set period for led as time ON
    timer_leds4off.ready_to_use = 1;
    timer_leds4off.common_rule = &leds_position;

    timer_leds5x8.timer_is_set = 1;
    timer_leds5x8.set_timer_limit  = 100; // set period for 5x8 7-segment display

    timer_count.timer_is_set = 1;
    timer_count.set_timer_limit  = 100; // set period for 5x8 7-segment display
}
//******************************************************************************



//******************************************************************************
t_timer_stat delay_timer_count (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;
	if (cfg==0) {
		timer_count.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_count.timer_is_set ) {
			timer_count.cur_timer_val++;
			if ( timer_count.cur_timer_val >= timer_count.set_timer_limit ) {
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



//******************************************************************************
t_timer_stat delay_timer_leds5x8 (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;

	if (cfg==0) {
		timer_leds5x8.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_leds5x8.timer_is_set ) {
			if ( ++timer_leds5x8.cur_timer_val >= timer_leds5x8.set_timer_limit ) {
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



//******************************************************************************
// Number in to string converter
//******************************************************************************
// itoa
char* IntToStr(int i, char b[]) {
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}
//******************************************************************************



//******************************************************************************
// Number in to string converter
//******************************************************************************
// itoa
char* FloatToLeds5x8(float ff, char b[]) {
    char const digit[] = "0123456789";
    char *p = b;
    int shifter=0, i=0, pos=0, stepen=0, mux=0, in1=0;
    float ostatok;

    ostatok = ff-(int)ff;
    if (ostatok<0) ostatok*=-1;
    if (ostatok<0.0001) ostatok*=10000000;
    if (ostatok<0.001) ostatok*=1000000;
    if (ostatok<0.01) ostatok*=100000;
    if (ostatok<0.1) ostatok*=10000;
    if (ostatok<1) ostatok*=1000;

    i = ff;
    in1 = (int)ff;

	for (pos=0; pos<5; pos++) {
		switch (pos){
			case 0: mux=0; break;
			case 1: mux=10; break;
			case 2: mux=100; break;
			case 3: mux=1000; break;
			case 4: mux=10000; break;
			case 5: mux=100000; break;
		}
		for ( stepen=0; (uint16_t)(in1-(stepen*mux))<mux; stepen++ ) {
			if (stepen<pos) {
				b[pos]=stepen;
			}
		}
    }

    if (i<0) {
        *p++ = '-';
        i *= -1;
    }

    shifter = i;

    do { //Move to where representation ends
        ++p;
        shifter = shifter/10;
        *p = shifter;
    } while (shifter) ;

    //*p = '\0';
    *p = '.';

    do { //Move to where representation ends
        ++p;
        ostatok = ostatok/10;
        *p = ostatok;
    } while ((int)ostatok) ;

    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);

    return b;
}
//******************************************************************************



//******************************************************************************
char *ultostr(unsigned long value, char *ptr, int base)
{
  unsigned long t = 0, res = 0;
  unsigned long tmp = value;
  int count = 0;

  if (NULL == ptr) {
    return NULL;
  }

  if (tmp == 0) {
    count++;
  }

  while(tmp > 0) {
    tmp = tmp/base;
    count++;
  }

  ptr += count;
  *ptr = '\0';

  do {
    res = value - base * (t = value / base);
    if (res < 10) {
      * -- ptr = '0' + res;
    } else
    if ((res >= 10) && (res < 16)) {
        * --ptr = 'A' - 10 + res;
    }
  } while ((value = t) != 0);

  return(ptr);
}//******************************************************************************



//******************************************************************************
char *ftostr(float value, char *ptr, int base, uint8_t *comma)
{
  unsigned long t=0, res=0;
  unsigned long tmp=value;
  unsigned long val=0;
  int           count=0;
  float         ostatok_float=0;
  uint32_t      celoe=0, ostatok_int=0;
  uint8_t       less_then_1=0;
  uint8_t       dlina_ostatka=0;

  if (NULL == ptr) {
    return NULL;
  }

  if (NULL == comma) {
    return NULL;
  }

  /*if (tmp == 0) {
    count++;
  }

  while(tmp > 0) {
    tmp = tmp/base;
    count++;
  }*/

  celoe = (int)value;
  ostatok_float = value - celoe;

  if (celoe<1) {
	  *comma=0;
  	  less_then_1 = 1;
		ostatok_int = (uint32_t)((ostatok_float * 10000));
		dlina_ostatka = 5 - (*comma);
		ptr += 0;
  }
  else
  if (celoe<10) {
	  *comma=0;
		ostatok_int = (uint32_t)((ostatok_float * 10000));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 1;
  }
  else
  if (celoe<100) {
	  *comma=1;
		ostatok_int = (uint32_t)((ostatok_float * 1000));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 2;
  }
  else
  if (celoe<1000) {
	  *comma=2;
		ostatok_int = (uint32_t)((ostatok_float * 100));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 3;
  }
  else
  if (celoe<10000) {
	  *comma=3;
		ostatok_int = (uint32_t)((ostatok_float * 10));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 4;
  }
  else
  if (celoe<100000) {
	  *comma=4;
		ostatok_int = (uint32_t)((ostatok_float * 1));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 5;
  }

//  ptr += count;
//  *ptr = '\0';  // set first 0

  val = value;

  do {  // display - Celoe
	t = val / base;
    res = val - base * t; // res = drobnaya ot value
    if (res < 10) {
      *--ptr = '0' + res;
    }
  } while ((val = t) != 0);

  if (celoe<1) { ptr += 5;  }
  else
  if (celoe<10) { ptr += 4; }
  else
  if (celoe<100) { ptr += 4; }
  else
  if (celoe<1000) { ptr += 4; }
  else
  if (celoe<10000) { ptr += 4; }
  else
  if (celoe<100000) { ptr += 4; }

  if (*comma<5) { // Display - Ostatok
      //*--ptr = '0' + res;
	  do {
	    if (dlina_ostatka>0) {
			t = ostatok_int / base;
			res = ostatok_int - base * t; // res = drobnaya ot value
			if (res < 10) {
			  *ptr-- = '0' + res;
			}
	      dlina_ostatka--;
	    }
	  } while ( (ostatok_int=t)!=0 || dlina_ostatka!=0 );
  }

  return(ptr);
}//******************************************************************************



char data[5] ={ 1,  3,  5,  7,  9 };
char datas[5]={'1','3','5','7','9'};


//******************************************************************************
// Blinks on the leds and indicator.
//******************************************************************************
int main (void) {
    volatile uint32_t pos;
    volatile uint32_t cnt=0;
    char    *tmp;
    float   in=0;
    uint8_t comma;
	char    *p_str;

	//double num=34267;
	//char   output[10];
	//snprintf(output,10,"%f",num);

	//char buffer [50];
	//unsigned long a = 5;
	//int n;
	//n=sprintf (buffer, "%lu", a);

    //SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_OSC|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    //SysCtlPWMClockSet(SYSCTL_PWMDIV_1); //Set the PWM clock to the system clock.

    drv_sys_init_gpio ();
    drv_usr_init_led_7segments ();
    drv_usr_init_scheduler_and_all_timers ();

    in = 0.0001;

	while(1) {  // Loop forever.
		if (  _TIMER_READY == delay_timer_leds5x8('?') ) {
			if ( _TIMER_READY == delay_timer_count('?') ) {
				in+=0.0001;
				delay_timer_count(0);
			}
			if ( pos < 5 ) {
				//tmp = ltoa(in, (char *)data); // ltoa - max string 34 bytes.
				//tmp = IntToStr(in, (char *)data); // ltoa - max string 34 bytes.
				//tmp = FloatToLeds5x8(in, data); // ltoa - max string 34 bytes.
				//ultostr(56789,data,10); //  value, *ptr, base
				p_str = ftostr ( in, data, 10, &comma ); // value, *ptr, base, *comma_possition
				drv_led_7segments_symbol (pos, data[pos], comma); // display info
				drv_led_7segments_position (pos); // dinamic switching
				pos++;
				cnt++;
    		} else {
				pos = 0;
    		}
			delay_timer_leds5x8(0);
    	}
		drv_led_blink ();
    }
}
//******************************************************************************
