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

//#include "inc/hw_types.h"
//#include "drivers/pinout.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"

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
#define  BTN_PCS_0  (1)
#define  BTN_PCS_1  (2)
//******************************************************************************


// LEDS[0,1,2,3]=[PN1,PN0,PF4,PF0]
// USER_SWITCH[0,1]=[PJ0,PJ1]

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
t_Scheduler timer_keys_usr1_2;

uint32_t  dly1 = 0;
uint32_t  dly2 = 0;
uint32_t  dly3 = 0;
uint32_t  dly4 = 0;

uint32_t    leds_position = 1;
//******************************************************************************




//******************************************************************************
void drv_sys_init_gpio ( void );
void drv_usr_init_led_7segments ( void );
void drv_usr_init_led_8x8 ( void );
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

/*
Datasheet @ page 757 @
Texas Instruments-Production Data @ Tiva TM4C1294NCPDT Microcontroller
Table 10-7. GPIO Register Map

Offset 	Name Type 	Reset 					Description page
0x000 	GPIODATA 	RW 		0x0000.0000 	GPIO Data 759
0x400 	GPIODIR 	RW 		0x0000.0000 	GPIO Direction 760
0x404 	GPIOIS 		RW 		0x0000.0000 	GPIO Interrupt Sense 761
0x408 	GPIOIBE 	RW 		0x0000.0000 	GPIO Interrupt Both Edges 762
0x40C 	GPIOIEV 	RW 		0x0000.0000 	GPIO Interrupt Event 763
0x410 	GPIOIM 		RW 		0x0000.0000 	GPIO Interrupt Mask 764
0x414 	GPIORIS 	RO 		0x0000.0000 	GPIO Raw Interrupt Status 765
0x418 	GPIOMIS 	RO 		0x0000.0000 	GPIO Masked Interrupt Status 767
0x41C 	GPIOICR 	W1C 	0x0000.0000 	GPIO Interrupt Clear 769
0x420 	GPIOAFSEL 	RW 		- 				GPIO Alternate Function Select 770
0x500 	GPIODR2R 	RW 		0x0000.00FF 	GPIO 2-mA Drive Select 772
0x504 	GPIODR4R 	RW 		0x0000.0000 	GPIO 4-mA Drive Select 773
0x508 	GPIODR8R 	RW 		0x0000.0000 	GPIO 8-mA Drive Select 774
0x50C 	GPIOODR 	RW 		0x0000.0000 	GPIO Open Drain Select 775
0x510 	GPIOPUR 	RW 		- 				GPIO Pull-Up Select 776
0x514 	GPIOPDR 	RW 		0x0000.0000 	GPIO Pull-Down Select 778
0x518 	GPIOSLR 	RW 		0x0000.0000 	GPIO Slew Rate Control Select 780
0x51C 	GPIODEN 	RW 		- 				GPIO Digital Enable 781
0x520 	GPIOLOCK 	RW 		0x0000.0001 	GPIO Lock 783
0x524 	GPIOCR 		- 		- 				GPIO Commit 784
0x528 	GPIOAMSEL 	RW 		0x0000.0000 	GPIO Analog Mode Select 786
0x52C 	GPIOPCTL 	RW 		- 				GPIO Port Control 787
0x530 	GPIOADCCTL 	RW 		0x0000.0000 	GPIO ADC Control 789
0x534 	GPIODMACTL 	RW 		0x0000.0000 	GPIO DMA Control 790
0x538 	GPIOSI 		RW 		0x0000.0000 	GPIO Select Interrupt 791
0x53C 	GPIODR12R 	RW 		0x0000.0000 	GPIO 12-mA Drive Select 792
0x540 	GPIOWAKEPEN RW 		0x0000.0000 	GPIO Wake Pin Enable 793
0x544 	GPIOWAKELVL RW 		0x0000.0000 	GPIO Wake Level 795
0x548 	GPIOWAKESTAT RO 	0x0000.0000 	GPIO Wake Status 797
0xFC0 	GPIOPP 		RO 		0x0000.0001 	GPIO Peripheral Property 799
0xFC4 	GPIOPC 		RW 		0x0000.0000 	GPIO Peripheral Configuration 800
0xFD0 	GPIOPeriphID4 RO 	0x0000.0000 	GPIO Peripheral Identification 4 803
0xFD4 	GPIOPeriphID5 RO 	0x0000.0000 	GPIO Peripheral Identification 5 804
0xFD8 	GPIOPeriphID6 RO 	0x0000.0000 	GPIO Peripheral Identification 6 805
0xFDC 	GPIOPeriphID7 RO 	0x0000.0000 	GPIO Peripheral Identification 7 806
0xFE0 	GPIOPeriphID0 RO 	0x0000.0061 	GPIO Peripheral Identification 0 807
0xFE4 	GPIOPeriphID1 RO 	0x0000.0000 	GPIO Peripheral Identification 1 808
0xFE8 	GPIOPeriphID2 RO 	0x0000.0018 	GPIO Peripheral Identification 2 809
0xFEC 	GPIOPeriphID3 RO 	0x0000.0001 	GPIO Peripheral Identification 3 810
0xFF0 	GPIOPCellID0 RO 	0x0000.000D 	GPIO PrimeCell Identification 0 811
0xFF4 	GPIOPCellID1 RO 	0x0000.00F0 	GPIO PrimeCell Identification 1 812
0xFF8 	GPIOPCellID2 RO 	0x0000.0005 	GPIO PrimeCell Identification 2 813
0xFFC 	GPIOPCellID3 RO 	0x0000.00B1 	GPIO PrimeCell Identification 3 814 */

    // Enable the GPIO port that is used for the on-board LED.
    SYSCTL_RCGCGPIO_R =
		SYSCTL_RCGCGPIO_R14 | //SYSCTL_RCGCGPIO_R14=GPIO Port Q Run Mode Clock
		SYSCTL_RCGCGPIO_R13 | //SYSCTL_RCGCGPIO_R13=GPIO Port P Run Mode Clock
    	SYSCTL_RCGCGPIO_R12 | //SYSCTL_RCGCGPIO_R12=GPIO Port N Run Mode Clock
		SYSCTL_RCGCGPIO_R11 | //SYSCTL_RCGCGPIO_R11=GPIO Port M Run Mode Clock
    	SYSCTL_RCGCGPIO_R9  | //SYSCTL_RCGCGPIO_R9= GPIO Port K Run Mode Clock
    	SYSCTL_RCGCGPIO_R8  | //SYSCTL_RCGCGPIO_R8= GPIO Port J Run Mode Clock
    	SYSCTL_RCGCGPIO_R7  | //SYSCTL_RCGCGPIO_R7= GPIO Port H Run Mode Clock
    	SYSCTL_RCGCGPIO_R5  | //SYSCTL_RCGCGPIO_R5= GPIO Port F Run Mode Clock
    	SYSCTL_RCGCGPIO_R4  | //SYSCTL_RCGCGPIO_R4= GPIO Port E Run Mode Clock
    	SYSCTL_RCGCGPIO_R3  | //SYSCTL_RCGCGPIO_R3= GPIO Port D Run Mode Clock
    	SYSCTL_RCGCGPIO_R2  | //SYSCTL_RCGCGPIO_R2= GPIO Port C Run Mode Clock
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


    // Enable the GPIO pin for the SW0 (PJ0).  Set the direction as input, and
    // enable the GPIO pin for digital function.
    GPIO_PORTJ_AHB_DIR_R |= 0x00;
    GPIO_PORTJ_AHB_DEN_R |= 0x03; // SW0 and SW1: 0x01 and 0x02
    GPIO_PORTJ_AHB_PUR_R |= 0x03; // Pull-Up Resistor
    GPIO_PORTJ_AHB_PP_R = 1;
    GPIO_PORTJ_AHB_PC_R = 3;

}
//******************************************************************************



//******************************************************************************
// void drv_led_7segments_init ( void )
// Initialisation for leds on board, basic configuration and default startup
// settings.
//******************************************************************************
void drv_usr_init_led_7segments ( void ) {
    // Initial 7-segments
    GPIO_PORTA_AHB_DIR_R  = 0xFF;  GPIO_PORTA_AHB_DEN_R = 0xFF;
    GPIO_PORTA_AHB_PP_R   = 1;
    GPIO_PORTA_AHB_PC_R   = 3;
    GPIO_PORTA_AHB_DR12R_R|=1; GPIO_PORTA_AHB_DR8R_R|=1; GPIO_PORTA_AHB_DR4R_R|=1;

    GPIO_PORTB_AHB_DIR_R  = 0xFF;  GPIO_PORTB_AHB_DEN_R = 0xFF;
    GPIO_PORTB_AHB_PP_R   = 1;
    GPIO_PORTB_AHB_PC_R   = 3;
    GPIO_PORTB_AHB_DR12R_R|=1; GPIO_PORTB_AHB_DR8R_R|=1; GPIO_PORTB_AHB_DR4R_R|=1;

    GPIO_PORTK_DIR_R  = 0xFF;  GPIO_PORTK_DEN_R = 0xFF;
    GPIO_PORTK_PP_R   = 1;
    GPIO_PORTK_PC_R   = 3;
    GPIO_PORTK_DR12R_R|=1; GPIO_PORTK_DR8R_R|=1; GPIO_PORTK_DR4R_R|=1;

    GPIO_PORTP_DIR_R  = 0xFF;  GPIO_PORTP_DEN_R = 0xFF;
    GPIO_PORTP_PP_R   = 1;
    GPIO_PORTP_PC_R   = 3;
    GPIO_PORTP_DR12R_R|=1; GPIO_PORTP_DR8R_R|=1; GPIO_PORTP_DR4R_R|=1;

    GPIO_PORTM_DIR_R = 0xFF;  GPIO_PORTM_DEN_R = 0xFF;
    GPIO_PORTM_PP_R    = 1;
    GPIO_PORTM_PC_R    = 3;
    GPIO_PORTM_DR12R_R|=1; GPIO_PORTM_DR8R_R|=1; GPIO_PORTM_DR4R_R|=1;

    GPIO_PORTQ_DIR_R  = 0xFF;  GPIO_PORTQ_DEN_R
    = 0xFF;
    GPIO_PORTQ_PP_R   = 1;
    GPIO_PORTQ_PC_R   = 3;
    GPIO_PORTQ_DR12R_R|=1; GPIO_PORTQ_DR8R_R|=1; GPIO_PORTQ_DR4R_R|=1;

    // PQ2, PA3, also PQ3, PA2 are use one pins
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));

}
//******************************************************************************



//******************************************************************************
// void drv_led_7segments_init ( void )
// Initialisation for leds on board, basic configuration and default startup
// settings.
//******************************************************************************
void drv_usr_init_led_8x8 ( void ) {
    // Initial 7-segments
    GPIO_PORTC_AHB_DIR_R   = 0xFF;  GPIO_PORTC_AHB_DEN_R = 0xFF;
    GPIO_PORTC_AHB_PP_R    = 1;
    GPIO_PORTC_AHB_PC_R    = 3;
    GPIO_PORTC_AHB_DR12R_R|= 1; GPIO_PORTC_AHB_DR8R_R |= 1; GPIO_PORTC_AHB_DR4R_R |= 1;

    GPIO_PORTD_AHB_DIR_R   = 0xFF;  GPIO_PORTD_AHB_DEN_R = 0xFF;
    GPIO_PORTD_AHB_PP_R    = 1;
    GPIO_PORTD_AHB_PC_R    = 3;
    GPIO_PORTD_AHB_DR12R_R|= 1; GPIO_PORTD_AHB_DR8R_R |= 1; GPIO_PORTD_AHB_DR4R_R |= 1;

    GPIO_PORTE_AHB_DIR_R   = 0xFF;  GPIO_PORTE_AHB_DEN_R = 0xFF;
    GPIO_PORTE_AHB_PP_R    = 1;
    GPIO_PORTE_AHB_PC_R    = 3;
    GPIO_PORTE_AHB_DR12R_R|= 1; GPIO_PORTE_AHB_DR8R_R |= 1; GPIO_PORTE_AHB_DR4R_R |= 1;

    GPIO_PORTH_AHB_DIR_R   = 0xFF;  GPIO_PORTH_AHB_DEN_R = 0xFF;
    GPIO_PORTH_AHB_PP_R    = 1;
    GPIO_PORTH_AHB_PC_R    = 3;
    GPIO_PORTH_AHB_DR12R_R|= 1; GPIO_PORTH_AHB_DR8R_R |= 1; GPIO_PORTH_AHB_DR4R_R |= 1;

    GPIO_PORTN_DIR_R       = 0xFF;  GPIO_PORTN_DEN_R = 0xFF;
    GPIO_PORTN_PP_R        = 1;
    GPIO_PORTN_PC_R        = 3;
    GPIO_PORTN_DR12R_R    |= 1; GPIO_PORTN_DR8R_R |= 1; GPIO_PORTN_DR4R_R |= 1;
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
void drv_led_8x8 ( uint8_t x, uint8_t y, uint8_t t ) {
	GPIO_PORTC_AHB_DATA_R |=  (1<<4); // Cathod - X0
	GPIO_PORTC_AHB_DATA_R |=  (1<<5); // Cathod - X1
	GPIO_PORTC_AHB_DATA_R |=  (1<<6); // Cathod - X2
	GPIO_PORTE_AHB_DATA_R |=  (1<<5); // Cathod - X3
	GPIO_PORTE_AHB_DATA_R |=  (1<<0); // Cathod - X4
	GPIO_PORTE_AHB_DATA_R |=  (1<<1); // Cathod - X5
	GPIO_PORTE_AHB_DATA_R |=  (1<<2); // Cathod - X6
	GPIO_PORTE_AHB_DATA_R |=  (1<<3); // Cathod - x7
	switch ( x ) {
		case 0: GPIO_PORTC_AHB_DATA_R &= ~(1<<4); break; // Cathod - X0
		case 1: GPIO_PORTC_AHB_DATA_R &= ~(1<<5); break; // Cathod - X1
		case 2: GPIO_PORTC_AHB_DATA_R &= ~(1<<6); break; // Cathod - X2
		case 3: GPIO_PORTE_AHB_DATA_R &= ~(1<<5); break; // Cathod - X3
		case 4: GPIO_PORTE_AHB_DATA_R &= ~(1<<0); break; // Cathod - X3
		case 5: GPIO_PORTE_AHB_DATA_R &= ~(1<<1); break; // Cathod - X3
		case 6: GPIO_PORTE_AHB_DATA_R &= ~(1<<2); break; // Cathod - X5
		case 7:	GPIO_PORTE_AHB_DATA_R &= ~(1<<3); break; // Cathod - X6
		/*default:
			GPIO_PORTC_AHB_DATA_R |=  (1<<4); // Cathod - X0
			GPIO_PORTC_AHB_DATA_R |=  (1<<6); // Cathod - X1
			GPIO_PORTC_AHB_DATA_R |=  (1<<6); // Cathod - X2
			GPIO_PORTE_AHB_DATA_R |=  (1<<5); // Cathod - X3
			GPIO_PORTE_AHB_DATA_R |=  (1<<0); // Cathod - X4
			GPIO_PORTE_AHB_DATA_R |=  (1<<1); // Cathod - X5
			GPIO_PORTE_AHB_DATA_R |=  (1<<2); // Cathod - X6
			GPIO_PORTE_AHB_DATA_R |=  (1<<3); // Cathod - x7
		break;*/
	}


	/*case 0: */ GPIO_PORTM_DATA_R     &= ~(1<<4); //break; // Cathod - y0
	/*case 1: */ GPIO_PORTA_AHB_DATA_R &= ~(1<<6); //break; // Cathod - y1
	/*case 2: */ GPIO_PORTD_AHB_DATA_R &= ~(1<<3); //break; // Cathod - y2
	/*case 3: */ GPIO_PORTB_AHB_DATA_R &= ~(1<<2); //break; // Cathod - y3
	/*case 4: */ GPIO_PORTB_AHB_DATA_R &= ~(1<<3); //break; // Cathod - y4
	/*case 5: */ GPIO_PORTC_AHB_DATA_R &= ~(1<<7); //break; // Cathod - y5
	/*case 6: */ GPIO_PORTE_AHB_DATA_R &= ~(1<<4); //break; // Cathod - y6
	/*case 7: */ GPIO_PORTD_AHB_DATA_R &= ~(1<<7); //break; // Cathod - X7
	/* */
	/*case 8: */ GPIO_PORTD_AHB_DATA_R &= ~(1<<1); //break; // Cathod - y8
	/*case 9: */ GPIO_PORTP_DATA_R     &= ~(1<<2); //break; // Cathod - y9
	/*case 10:*/ GPIO_PORTD_AHB_DATA_R &= ~(1<<0); //break; // Cathod - y10
	/*case 11:*/ GPIO_PORTM_DATA_R     &= ~(1<<3); //break; // Cathod - y11
	/*case 12:*/ GPIO_PORTH_AHB_DATA_R &= ~(1<<2); //break; // Cathod - y12
	/*case 13:*/ GPIO_PORTH_AHB_DATA_R &= ~(1<<3); //break; // Cathod - y13
	/*case 14:*/ GPIO_PORTN_DATA_R     &= ~(1<<2); //break; // Cathod - y14
	/*case 15:*/ GPIO_PORTN_DATA_R     &= ~(1<<3); //break; // Cathod - X15

	switch ( y ) {
		case 0x00: GPIO_PORTM_DATA_R     |=  (1<<4); break; // Cathod - y0
		case 0x01: GPIO_PORTA_AHB_DATA_R |=  (1<<6); break; // Cathod - y1
		case 0x02: GPIO_PORTD_AHB_DATA_R |=  (1<<3); break; // Cathod - y2
		case 0x03: GPIO_PORTB_AHB_DATA_R |=  (1<<2); break; // Cathod - y3
		case 0x04: GPIO_PORTB_AHB_DATA_R |=  (1<<3); break; // Cathod - y4
		case 0x05: GPIO_PORTC_AHB_DATA_R |=  (1<<7); break; // Cathod - y5
		case 0x06: GPIO_PORTE_AHB_DATA_R |=  (1<<4); break; // Cathod - y6
		case 0x07: GPIO_PORTD_AHB_DATA_R |=  (1<<7); break; // Cathod - X7

		case 0x08: GPIO_PORTD_AHB_DATA_R |=  (1<<1); break; // Cathod - y8
		case 0x09: GPIO_PORTP_DATA_R     |=  (1<<2); break; // Cathod - y9
		case 0x0a: GPIO_PORTD_AHB_DATA_R |=  (1<<0); break; // Cathod - y10
		case 0x0b: GPIO_PORTM_DATA_R     |=  (1<<3); break; // Cathod - y11
		case 0x0c: GPIO_PORTH_AHB_DATA_R |=  (1<<2); break; // Cathod - y12
		case 0x0d: GPIO_PORTH_AHB_DATA_R |=  (1<<3); break; // Cathod - y13
		case 0x0e: GPIO_PORTN_DATA_R     |=  (1<<2); break; // Cathod - y14
		case 0x0f: GPIO_PORTN_DATA_R     |=  (1<<3); break; // Cathod - X15
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
    timer_leds1off.set_timer_limit  = 10000;   // set period for led as time ON
    timer_leds1off.ready_to_use = 1;
    timer_leds1off.common_rule = &leds_position;

    timer_leds2on.timer_is_set = 1;
    timer_leds2on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds2on.ready_to_use = 1;
    timer_leds2on.common_rule = &leds_position;
    timer_leds2off.timer_is_set = 1;
    timer_leds2off.set_timer_limit  = 10000;   // set period for led as time ON
    timer_leds2off.ready_to_use = 1;
    timer_leds2off.common_rule = &leds_position;

    timer_leds3on.timer_is_set = 1;
    timer_leds3on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds3on.ready_to_use = 1;
    timer_leds3on.common_rule = &leds_position;
    timer_leds3off.timer_is_set = 1;
    timer_leds3off.set_timer_limit  = 10000;   // set period for led as time ON
    timer_leds3off.ready_to_use = 1;
    timer_leds3off.common_rule = &leds_position;

    timer_leds4on.timer_is_set = 1;
    timer_leds4on.set_timer_limit  = 50000; // set period for led as time OFF
    timer_leds4on.ready_to_use = 1;
    timer_leds4on.common_rule = &leds_position;
    timer_leds4off.timer_is_set = 1;
    timer_leds4off.set_timer_limit  = 10000;   // set period for led as time ON
    timer_leds4off.ready_to_use = 1;
    timer_leds4off.common_rule = &leds_position;

    timer_leds5x8.timer_is_set = 1;
    timer_leds5x8.set_timer_limit  = 200; // set period for 5x8 7-segment display

    timer_count.timer_is_set = 1;
    timer_count.set_timer_limit  = 100; // set period for timer counter

    timer_keys_usr1_2.timer_is_set = 1;
    timer_keys_usr1_2.ready_to_use = 1;
    timer_keys_usr1_2.set_timer_limit  = 5000; // set period for 8x8 leds display
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
t_timer_stat delay_timer_keys_usr1_2 (uint8_t cfg) {
t_timer_stat err_code = _TIMER_NOT_READY;
	if (cfg==0) {
		timer_keys_usr1_2.cur_timer_val=0;
	}

	if (cfg=='?') {
		if ( timer_keys_usr1_2.timer_is_set ) {
			timer_keys_usr1_2.cur_timer_val++;
			if ( timer_keys_usr1_2.cur_timer_val >= timer_keys_usr1_2.set_timer_limit ) {
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
  else
  if (celoe<1000000) {
	  *comma=4;
		ostatok_int = (uint32_t)((ostatok_float * 0));
		dlina_ostatka = 5 - (*comma+1);
		ptr += 6;
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

  if (celoe<1) { ptr += 5;  }     else
  if (celoe<10) { ptr += 4; }     else
  if (celoe<100) { ptr += 4; }    else
  if (celoe<1000) { ptr += 4; }   else
  if (celoe<10000) { ptr += 4; }  else
  if (celoe<100000) { ptr += 4; } else
  if (celoe<1000000) { ptr += 4; }

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
    uint8_t x_pos=0, y_pos=0;
	char    *p_str;

    drv_sys_init_gpio ();
    drv_usr_init_led_7segments ();
    drv_usr_init_led_8x8 ();
    drv_usr_init_scheduler_and_all_timers ();

    in = 0.0001;

	while(1) {  // Loop forever.
		if (  _TIMER_READY == delay_timer_leds5x8('?') ) {
			if ( _TIMER_READY == delay_timer_count('?') ) {
				//in+=0.0001;
				delay_timer_count(0);
			}
			if ( pos < 5 ) {
				p_str = ftostr ( in, data, 10, &comma ); // value, *ptr, base, *comma_possition
				drv_led_7segments_symbol (pos, data[pos], comma); // display info
				drv_led_7segments_position (pos); // dinamic switching
				drv_led_8x8( x_pos/*x*/, y_pos/*y*/, 1/*t*/ ); // display info
				pos++;
				cnt++;
    		} else {
				pos = 0;
    		}
			delay_timer_leds5x8(0);
    	}
		drv_led_blink ();

		if (  _TIMER_READY == delay_timer_keys_usr1_2('?') ) {

			//if ( y_pos<16) { y_pos++; } else y_pos=0;

			if ( (GPIO_PORTJ_AHB_DATA_R & 0x03) == BTN_PCS_0 ) {
				if ( in<1 ) in += 0.00001; else
				if ( in<10 ) in += 0.0001; else
				if ( in<100 ) in += 0.001; else
				if ( in<1000 ) in += 0.01; else
				if ( in<10000 ) in += 0.1;

				if ( ++x_pos>7 ) x_pos=0;
			}
			if ( (GPIO_PORTJ_AHB_DATA_R & 0x03) == BTN_PCS_1 ) {
				if ( in>0 )
				if ( in<1 ) in -= 0.00001; else
				if ( in<10 ) in -= 0.0001; else
				if ( in<100 ) in -= 0.001; else
				if ( in<1000 ) in -= 0.01; else
				if ( in<10000 ) in -= 0.1;

				if ( ++y_pos>15 ) y_pos=0;
			}

			delay_timer_keys_usr1_2(0);
		}
    }
}
//******************************************************************************
