//*****************************************************************************
//*****************************************************************************
// TM4C1294.c - Simple example to work with LEDs.
// This is start code of the EK-TM4C1294XL.
//*****************************************************************************
//*****************************************************************************

#include "TM4C1294_leds.h"
#include "time_delays.h"
#include "ps2_mouse.h"

//#include "inc/hw_types.h"
//#include "drivers/pinout.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"

//#define  __TI_COMPILER_VERSION__

/* *****************************************************************************
 * Global and system variables
 * ************************************************************************** */
/*t_Scheduler timer_leds5x8;
t_Scheduler timer_leds1on, timer_leds1off;
t_Scheduler timer_leds2on, timer_leds2off;
t_Scheduler timer_leds3on, timer_leds3off;
t_Scheduler timer_leds4on, timer_leds4off;
t_Scheduler timer_leds8x8;
t_Scheduler timer_keys;
t_Scheduler timer_fft;
t_Scheduler timer_rnd;
t_Scheduler timer_mouse;*/

t_Scheduler  timer [ /*sizeof(timer_name)*/ 15 ];

uint32_t  dly1 = 0;
uint32_t  dly2 = 0;
uint32_t  dly3 = 0;
uint32_t  dly4 = 0;
uint32_t  leds_position;

uint8_t   a_field [8][16];
uint8_t   a_field_eat [8][16];
uint8_t   a_field_eat_16b [16];

uint8_t   data [5] = { 1,   3,   5,   7,   9 };
uint8_t   datas[5] = {'1', '3', '5', '7', '9'};


/* *****************************************************************************
 *
 * ************************************************************************** */
void  drv_sys_init_gpio_output_input ( void );
void  drv_usr_init_led_7segments ( void );
void  drv_usr_init_led_8x8 ( void );
void  drv_led_7segments_clean ( void );
void  drv_led_7segments_position (unsigned char pos);
void  drv_led_7segments_symbol ( uint8_t position, uint8_t symbol, uint8_t comma );
t_ret_code keys_scan ( t_io_values *p_io );
t_ret_code keys_analyze ( t_io_values *p_io, t_sys_values *p_sys );
char *IntToStr(int i, char b[]);
char *FloatToLeds5x8(float ff, char b[]);
char *ultostr(unsigned long value, char *ptr, int base);
uint8_t *ftostr(float value, uint8_t *p_string, int base, uint8_t *p_comma);
t_ret_code algo_Snake ( t_io_values *p_io,  t_io_Snake *p_snake );


//******************************************************************************



/* *****************************************************************************
 * void drv_sys_init_gpio_output_input ( void )
 * Initialisation for GPIO, basic configuration and default startup settings.
 * ************************************************************************** */
void drv_sys_init_gpio_output_input ( void ) {
    volatile uint32_t ui32Loop;

/*  Datasheet @ page 757 @
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
    	SYSCTL_RCGCGPIO_R10 | //SYSCTL_RCGCGPIO_R10=GPIO Port L Run Mode Clock
    	SYSCTL_RCGCGPIO_R9  | //SYSCTL_RCGCGPIO_R9= GPIO Port K Run Mode Clock
    	SYSCTL_RCGCGPIO_R8  | //SYSCTL_RCGCGPIO_R8= GPIO Port J Run Mode Clock
    	SYSCTL_RCGCGPIO_R7  | //SYSCTL_RCGCGPIO_R7= GPIO Port H Run Mode Clock
    	SYSCTL_RCGCGPIO_R6  | //SYSCTL_RCGCGPIO_R6= GPIO Port G Run Mode Clock
    	SYSCTL_RCGCGPIO_R5  | //SYSCTL_RCGCGPIO_R5= GPIO Port F Run Mode Clock
    	SYSCTL_RCGCGPIO_R4  | //SYSCTL_RCGCGPIO_R4= GPIO Port E Run Mode Clock
    	SYSCTL_RCGCGPIO_R3  | //SYSCTL_RCGCGPIO_R3= GPIO Port D Run Mode Clock
    	SYSCTL_RCGCGPIO_R2  | //SYSCTL_RCGCGPIO_R2= GPIO Port C Run Mode Clock
    	SYSCTL_RCGCGPIO_R1  | //SYSCTL_RCGCGPIO_R1= GPIO Port B Run Mode Clock
    	SYSCTL_RCGCGPIO_R0  ; //SYSCTL_RCGCGPIO_R0= GPIO Port A Run Mode Clock

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop = SYSCTL_RCGCGPIO_R;

    //--------------------------------------------------------------------------
    // OUTPUT
    //--------------------------------------------------------------------------
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

    // Enable the GPIO pin for the LED (PP0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIO_PORTP_DIR_R |= (1<<0|1<<1);
    GPIO_PORTP_DEN_R |= (1<<0|1<<1);


    //--------------------------------------------------------------------------
    // INPUT
    //--------------------------------------------------------------------------
    // Enable the GPIO pin for the SW0 (PJ0).
    // Set the direction as input, and enable the GPIO pin for digital function.
    GPIO_PORTJ_AHB_DIR_R &=~0x03;
    GPIO_PORTJ_AHB_DEN_R |= 0x03; // SW0 and SW1: 0x01 and 0x02
    GPIO_PORTJ_AHB_PUR_R |= 0x03; // Pull-Up Resistor
    GPIO_PORTJ_AHB_PP_R   = 1;
    GPIO_PORTJ_AHB_PC_R   = 3;

    // Set the direction as input, and enable the GPIO pin for digital function.
    // Inputs on Port F
    GPIO_PORTF_AHB_DIR_R &=~(1<<3|1<<2|1<<1);
    GPIO_PORTF_AHB_DEN_R |= (1<<3|1<<2|1<<1); //
    GPIO_PORTF_AHB_PUR_R |= (1<<3|1<<2|1<<1); // Pull-Up Resistor
    GPIO_PORTF_AHB_PP_R   = 1;
    GPIO_PORTF_AHB_PC_R   = 3;

    // Inputs on Port G
    GPIO_PORTG_AHB_DIR_R &=~(1<<0);
    GPIO_PORTG_AHB_DEN_R |= (1<<0); //
    GPIO_PORTG_AHB_PUR_R |= (1<<0); // Pull-Up Resistor
    GPIO_PORTG_AHB_PP_R   = 1;
    GPIO_PORTG_AHB_PC_R   = 3;

    // Inputs on Port L
    GPIO_PORTL_DIR_R &=~(1<<5|1<<4);
    GPIO_PORTL_DEN_R |= (1<<5|1<<4); //
    GPIO_PORTL_PUR_R |= (1<<5|1<<4); // Pull-Up Resistor
    GPIO_PORTL_PP_R   = 1;
    GPIO_PORTL_PC_R   = 3;


    // ========================================================================
    // PC[3:0], PD7, PE7 - are loacked by default, so read datasheet, page 743;
    // ========================================================================
    // PC[3:0] = {JTAG/SWD}
    // PD[7]   = NMI
    // PE[7]   = ????
    // ========================================================================
    // Unlock PortD PD7,
    GPIO_PORTD_AHB_LOCK_R = 0x4C4F434B; // from datasheet, page 783;
    GPIO_PORTD_AHB_CR_R  = 0xff;
}
//******************************************************************************



/* *****************************************************************************
 * void drv_init_gpio_input ( void )
 * Initialisation for GPIO, basic configuration and default startup settings.
 * ************************************************************************** */
void drv_sys_init_gpio_input ( void ) {
    volatile uint32_t ui32Loop;

/*  Datasheet @ page 757 @
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

    // Enable the GPIO port that is used for the on-board Inputs.
    SYSCTL_RCGCGPIO_R =
    	SYSCTL_RCGCGPIO_R10 | //SYSCTL_RCGCGPIO_R10=GPIO Port L Run Mode Clock
    	SYSCTL_RCGCGPIO_R8  | //SYSCTL_RCGCGPIO_R8= GPIO Port J Run Mode Clock
    	SYSCTL_RCGCGPIO_R6  | //SYSCTL_RCGCGPIO_R6= GPIO Port G Run Mode Clock
		SYSCTL_RCGCGPIO_R5  ; //SYSCTL_RCGCGPIO_R5= GPIO Port F Run Mode Clock

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop = SYSCTL_RCGCGPIO_R;

    // ..................................
    // Enable the GPIO pin for the SW0 (PJ0).
    // Set the direction as input, and enable the GPIO pin for digital function.
    GPIO_PORTJ_AHB_DIR_R &=~0x03;
    GPIO_PORTJ_AHB_DEN_R |= 0x03; // SW0 and SW1: 0x01 and 0x02
    GPIO_PORTJ_AHB_PUR_R |= 0x03; // Pull-Up Resistor
    GPIO_PORTJ_AHB_PP_R   = 1;
    GPIO_PORTJ_AHB_PC_R   = 3;

    /*// Set the direction as input, and enable the GPIO pin for digital function.
    // Inputs on Port F
    GPIO_PORTF_AHB_DIR_R &=~(1<<3|1<<2|1<<1);
    GPIO_PORTF_AHB_DEN_R |= (1<<3|1<<2|1<<1); //
    GPIO_PORTF_AHB_PUR_R |= (1<<3|1<<2|1<<1); // Pull-Up Resistor
    GPIO_PORTF_AHB_PP_R   = 1;
    GPIO_PORTF_AHB_PC_R   = 3;

    // Inputs on Port F
    GPIO_PORTG_AHB_DIR_R &=~(1<<0);
    GPIO_PORTG_AHB_DEN_R |= (1<<0); //
    GPIO_PORTG_AHB_PUR_R |= (1<<0); // Pull-Up Resistor
    GPIO_PORTG_AHB_PP_R   = 1;
    GPIO_PORTG_AHB_PC_R   = 3;

    // Inputs on Port F
    GPIO_PORTL_DIR_R &=~(1<<5|1<<4);
    GPIO_PORTL_DEN_R |= (1<<5|1<<4); //
    GPIO_PORTL_PUR_R |= (1<<5|1<<4); // Pull-Up Resistor
    GPIO_PORTL_PP_R   = 1;
    GPIO_PORTL_PC_R   = 3;*/
}



/* *****************************************************************************
 * void drv_led_7segments_init ( void )
 * Initialisation for leds on board, basic configuration and default startup
 * settings.
 * ************************************************************************** */
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

    GPIO_PORTQ_DIR_R  = 0xFF;  GPIO_PORTQ_DEN_R = 0xFF;
    GPIO_PORTQ_PP_R   = 1;
    GPIO_PORTQ_PC_R   = 3;
    GPIO_PORTQ_DR12R_R|=1; GPIO_PORTQ_DR8R_R|=1; GPIO_PORTQ_DR4R_R|=1;

    // PQ2, PA3, also PQ3, PA2 are use one pins
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));
    GPIO_PORTA_AHB_DIR_R &= ~((1<<2)|(1<<3));
}
//******************************************************************************



/* *****************************************************************************
 * void drv_usr_init_led_8x8 ( void )
 * Initialisation for leds on board, basic configuration and default startup
 * settings.
 * ************************************************************************** */
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



/* *****************************************************************************
 * void drv_led_7segments_position (unsigned char pos)
 * Initialisation for Possition in leds on 7 segments indicator, basic
 * configuration and default startup settings.
 * ************************************************************************** */
void  drv_led_7segments_clean ( void ) {

	GPIO_PORTA_AHB_DATA_R |=  (1<<2); // Anod (Common) 0
	GPIO_PORTA_AHB_DATA_R |=  (1<<3); // Anod (Common) 1
	GPIO_PORTK_DATA_R     |=  (1<<3); // Anod (Common) 2
	GPIO_PORTP_DATA_R     |=  (1<<3); // Anod (Common) 3
	GPIO_PORTM_DATA_R     |=  (1<<6); // Anod (Common) 4

    // Data to 7-segments
	GPIO_PORTK_DATA_R     |=  (1<<2); // Cathod - A
	GPIO_PORTA_AHB_DATA_R |=  (1<<4); // Cathod - B
	GPIO_PORTK_DATA_R     |=  (1<<1); // Cathod - C
	GPIO_PORTB_AHB_DATA_R |=  (1<<5); // Cathod - D
	GPIO_PORTB_AHB_DATA_R |=  (1<<4); // Cathod - E
	GPIO_PORTK_DATA_R     |=  (1<<3); // Cathod - F
	GPIO_PORTA_AHB_DATA_R |=  (1<<5); // Cathod - G
	GPIO_PORTK_DATA_R     |=  (1<<0); // Cathod - H}
}
//******************************************************************************



/* *****************************************************************************
 * void drv_led_7segments_position (unsigned char pos)
 * Initialisation for Possition in leds on 7 segments indicator, basic
 * configuration and default startup settings.
 * ************************************************************************** */
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



/* *****************************************************************************
 *
 * ************************************************************************** */
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



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_led_8x8_clear ( uint8_t x, uint8_t y, uint8_t clear ) {
	if (1==clear) {
		/*GPIO_PORTC_AHB_DATA_R |=  (1<<4); // Cathod - X0
		GPIO_PORTC_AHB_DATA_R |=  (1<<5); // Cathod - X1
		GPIO_PORTC_AHB_DATA_R |=  (1<<6); // Cathod - X2
		GPIO_PORTE_AHB_DATA_R |=  (1<<5); // Cathod - X3
		GPIO_PORTE_AHB_DATA_R |=  (1<<0); // Cathod - X4
		GPIO_PORTE_AHB_DATA_R |=  (1<<1); // Cathod - X5
		GPIO_PORTE_AHB_DATA_R |=  (1<<2); // Cathod - X6
		GPIO_PORTE_AHB_DATA_R |=  (1<<3); // Cathod - x7*/

		GPIO_PORTM_DATA_R     &= ~(1<<4); // Cathod - y0
		GPIO_PORTA_AHB_DATA_R &= ~(1<<6); // Cathod - y1
		GPIO_PORTD_AHB_DATA_R &= ~(1<<3); // Cathod - y2
		GPIO_PORTB_AHB_DATA_R &= ~(1<<2); // Cathod - y3
		GPIO_PORTB_AHB_DATA_R &= ~(1<<3); // Cathod - y4
		GPIO_PORTC_AHB_DATA_R &= ~(1<<7); // Cathod - y5
		GPIO_PORTE_AHB_DATA_R &= ~(1<<4); // Cathod - y6
		GPIO_PORTD_AHB_DATA_R &= ~(1<<7); // Cathod - X7
		GPIO_PORTD_AHB_DATA_R &= ~(1<<1); // Cathod - y8
		GPIO_PORTP_DATA_R     &= ~(1<<2); // Cathod - y9
		GPIO_PORTD_AHB_DATA_R &= ~(1<<0); // Cathod - y10
		GPIO_PORTM_DATA_R     &= ~(1<<3); // Cathod - y11
		GPIO_PORTH_AHB_DATA_R &= ~(1<<2); // Cathod - y12
		GPIO_PORTH_AHB_DATA_R &= ~(1<<3); // Cathod - y13
		GPIO_PORTN_DATA_R     &= ~(1<<2); // Cathod - y14
		GPIO_PORTN_DATA_R     &= ~(1<<3); // Cathod - X15
	}
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_led_8x8_pixel_set ( uint8_t x, uint8_t y, uint8_t val ) {

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
	}


	GPIO_PORTM_DATA_R     &= ~(1<<4); // Cathod - y0
	GPIO_PORTA_AHB_DATA_R &= ~(1<<6); // Cathod - y1
	GPIO_PORTD_AHB_DATA_R &= ~(1<<3); // Cathod - y2
	GPIO_PORTB_AHB_DATA_R &= ~(1<<2); // Cathod - y3
	GPIO_PORTB_AHB_DATA_R &= ~(1<<3); // Cathod - y4
	GPIO_PORTC_AHB_DATA_R &= ~(1<<7); // Cathod - y5
	GPIO_PORTE_AHB_DATA_R &= ~(1<<4); // Cathod - y6
	GPIO_PORTD_AHB_DATA_R &= ~(1<<7); // Cathod - y7
	GPIO_PORTD_AHB_DATA_R &= ~(1<<1); // Cathod - y8
	GPIO_PORTP_DATA_R     &= ~(1<<2); // Cathod - y9
	GPIO_PORTD_AHB_DATA_R &= ~(1<<0); // Cathod - y10
	GPIO_PORTM_DATA_R     &= ~(1<<3); // Cathod - y11
	GPIO_PORTH_AHB_DATA_R &= ~(1<<2); // Cathod - y12
	GPIO_PORTH_AHB_DATA_R &= ~(1<<3); // Cathod - y13
	GPIO_PORTN_DATA_R     &= ~(1<<2); // Cathod - y14
	GPIO_PORTN_DATA_R     &= ~(1<<3); // Cathod - X15

	switch ( y ) {
		case 0x00: GPIO_PORTM_DATA_R     |=  (1<<4); break; // Cathod - y0
		case 0x01: GPIO_PORTA_AHB_DATA_R |=  (1<<6); break; // Cathod - y1
		case 0x02: GPIO_PORTD_AHB_DATA_R |=  (1<<3); break; // Cathod - y2
		case 0x03: GPIO_PORTB_AHB_DATA_R |=  (1<<2); break; // Cathod - y3
		case 0x04: GPIO_PORTB_AHB_DATA_R |=  (1<<3); break; // Cathod - y4
		case 0x05: GPIO_PORTC_AHB_DATA_R |=  (1<<7); break; // Cathod - y5
		case 0x06: GPIO_PORTE_AHB_DATA_R |=  (1<<4); break; // Cathod - y6
		case 0x07: GPIO_PORTD_AHB_DATA_R |=  (1<<7); break; // Cathod - y7

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



/* *****************************************************************************
 *
 * ************************************************************************** */
void drv_led_8x8_show_byte ( uint8_t val, uint8_t xx, uint8_t yy ) {
#if (0)
	/*Catod: X0*/ if (val&0x01) GPIO_PORTC_AHB_DATA_R &=~(1<<4); else GPIO_PORTC_AHB_DATA_R|=(1<<4);
	/*Catod: X1*/ if (val&0x02) GPIO_PORTC_AHB_DATA_R &=~(1<<5); else GPIO_PORTC_AHB_DATA_R|=(1<<5);
	/*Catod: X2*/ if (val&0x04) GPIO_PORTC_AHB_DATA_R &=~(1<<6); else GPIO_PORTC_AHB_DATA_R|=(1<<6);
	/*Catod: X3*/ if (val&0x08) GPIO_PORTE_AHB_DATA_R &=~(1<<5); else GPIO_PORTE_AHB_DATA_R|=(1<<5);
	/*Catod: X4*/ if (val&0x10) GPIO_PORTE_AHB_DATA_R &=~(1<<0); else GPIO_PORTE_AHB_DATA_R|=(1<<0);
	/*Catod: X5*/ if (val&0x20) GPIO_PORTE_AHB_DATA_R &=~(1<<1); else GPIO_PORTE_AHB_DATA_R|=(1<<1);
	/*Catod: X6*/ if (val&0x40) GPIO_PORTE_AHB_DATA_R &=~(1<<2); else GPIO_PORTE_AHB_DATA_R|=(1<<2);
	/*Catod: X7*/ if (val&0x80) GPIO_PORTE_AHB_DATA_R &=~(1<<3); else GPIO_PORTE_AHB_DATA_R|=(1<<3);
#endif
	/*Catod: X0*/ if (val&(xx==0)) GPIO_PORTC_AHB_DATA_R &=~(1<<4); else GPIO_PORTC_AHB_DATA_R|=(1<<4);
	/*Catod: X1*/ if (val&(xx==1)) GPIO_PORTC_AHB_DATA_R &=~(1<<5); else GPIO_PORTC_AHB_DATA_R|=(1<<5);
	/*Catod: X2*/ if (val&(xx==2)) GPIO_PORTC_AHB_DATA_R &=~(1<<6); else GPIO_PORTC_AHB_DATA_R|=(1<<6);
	/*Catod: X3*/ if (val&(xx==3)) GPIO_PORTE_AHB_DATA_R &=~(1<<5); else GPIO_PORTE_AHB_DATA_R|=(1<<5);
	/*Catod: X4*/ if (val&(xx==4)) GPIO_PORTE_AHB_DATA_R &=~(1<<0); else GPIO_PORTE_AHB_DATA_R|=(1<<0);
	/*Catod: X5*/ if (val&(xx==5)) GPIO_PORTE_AHB_DATA_R &=~(1<<1); else GPIO_PORTE_AHB_DATA_R|=(1<<1);
	/*Catod: X6*/ if (val&(xx==6)) GPIO_PORTE_AHB_DATA_R &=~(1<<2); else GPIO_PORTE_AHB_DATA_R|=(1<<2);
	/*Catod: X7*/ if (val&(xx==7)) GPIO_PORTE_AHB_DATA_R &=~(1<<3); else GPIO_PORTE_AHB_DATA_R|=(1<<3);

	/*Anod: y0*/  if((15-yy)==0)  GPIO_PORTM_DATA_R    |=(1<<4); else GPIO_PORTM_DATA_R    &=~(1<<4);
	/*Anod: y1*/  if((15-yy)==1)  GPIO_PORTA_AHB_DATA_R|=(1<<6); else GPIO_PORTA_AHB_DATA_R&=~(1<<6);
	/*Anod: y2*/  if((15-yy)==2)  GPIO_PORTD_AHB_DATA_R|=(1<<3); else GPIO_PORTD_AHB_DATA_R&=~(1<<3);
	/*Anod: y3*/  if((15-yy)==3)  GPIO_PORTB_AHB_DATA_R|=(1<<2); else GPIO_PORTB_AHB_DATA_R&=~(1<<2);
	/*Anod: y4*/  if((15-yy)==4)  GPIO_PORTB_AHB_DATA_R|=(1<<3); else GPIO_PORTB_AHB_DATA_R&=~(1<<3);
	/*Anod: y5*/  if((15-yy)==5)  GPIO_PORTC_AHB_DATA_R|=(1<<7); else GPIO_PORTC_AHB_DATA_R&=~(1<<7);
	/*Anod: y6*/  if((15-yy)==6)  GPIO_PORTE_AHB_DATA_R|=(1<<4); else GPIO_PORTE_AHB_DATA_R&=~(1<<4);
	/*Anod: y7*/  if((15-yy)==7)  GPIO_PORTD_AHB_DATA_R|=(1<<7); else GPIO_PORTD_AHB_DATA_R&=~(1<<7);
	/*Anod: y8*/  if((15-yy)==8)  GPIO_PORTD_AHB_DATA_R|=(1<<1); else GPIO_PORTD_AHB_DATA_R&=~(1<<1);
	/*Anod: y9*/  if((15-yy)==9)  GPIO_PORTP_DATA_R    |=(1<<2); else GPIO_PORTP_DATA_R    &=~(1<<2);
	/*Anod: y10*/ if((15-yy)==10) GPIO_PORTD_AHB_DATA_R|=(1<<0); else GPIO_PORTD_AHB_DATA_R&=~(1<<0);
	/*Anod: y11*/ if((15-yy)==11) GPIO_PORTM_DATA_R    |=(1<<3); else GPIO_PORTM_DATA_R    &=~(1<<3);
	/*Anod: y12*/ if((15-yy)==12) GPIO_PORTH_AHB_DATA_R|=(1<<2); else GPIO_PORTH_AHB_DATA_R&=~(1<<2);
	/*Anod: y13*/ if((15-yy)==13) GPIO_PORTH_AHB_DATA_R|=(1<<3); else GPIO_PORTH_AHB_DATA_R&=~(1<<3);
	/*Anod: y14*/ if((15-yy)==14) GPIO_PORTN_DATA_R    |=(1<<2); else GPIO_PORTN_DATA_R    &=~(1<<2);
	/*Anod: y15*/ if((15-yy)==15) GPIO_PORTN_DATA_R    |=(1<<3); else GPIO_PORTN_DATA_R    &=~(1<<3);

	/* Force erase last row */ GPIO_PORTM_DATA_R &= ~(1<<4); // Anod - y0

	/* 	// Pre-clean
	GPIO_PORTM_DATA_R     &= ~(1<<4); // Anod - y0
	GPIO_PORTA_AHB_DATA_R &= ~(1<<6); // Anod - y1
	GPIO_PORTD_AHB_DATA_R &= ~(1<<3); // Anod - y2
	GPIO_PORTB_AHB_DATA_R &= ~(1<<2); // Anod - y3
	GPIO_PORTB_AHB_DATA_R &= ~(1<<3); // Anod - y4
	GPIO_PORTC_AHB_DATA_R &= ~(1<<7); // Anod - y5
	GPIO_PORTE_AHB_DATA_R &= ~(1<<4); // Anod - y6
	GPIO_PORTD_AHB_DATA_R &= ~(1<<7); // Anod - X7
	GPIO_PORTD_AHB_DATA_R &= ~(1<<1); // Anod - y8
	GPIO_PORTP_DATA_R     &= ~(1<<2); // Anod - y9
	GPIO_PORTD_AHB_DATA_R &= ~(1<<0); // Anod - y10
	GPIO_PORTM_DATA_R     &= ~(1<<3); // Anod - y11
	GPIO_PORTH_AHB_DATA_R &= ~(1<<2); // Anod - y12
	GPIO_PORTH_AHB_DATA_R &= ~(1<<3); // Anod - y13
	GPIO_PORTN_DATA_R     &= ~(1<<2); // Anod - y14
	GPIO_PORTN_DATA_R     &= ~(1<<3); // Anod - X15

	switch ( 15-Line ) {
		case 0:  GPIO_PORTM_DATA_R     |=  (1<<4); break; // Anod - y0
		case 1:  GPIO_PORTA_AHB_DATA_R |=  (1<<6); break; // Anod - y1
		case 2:  GPIO_PORTD_AHB_DATA_R |=  (1<<3); break; // Anod - y2
		case 3:  GPIO_PORTB_AHB_DATA_R |=  (1<<2); break; // Anod - y3
		case 4:  GPIO_PORTB_AHB_DATA_R |=  (1<<3); break; // Anod - y4
		case 5:  GPIO_PORTC_AHB_DATA_R |=  (1<<7); break; // Anod - y5
		case 6:  GPIO_PORTE_AHB_DATA_R |=  (1<<4); break; // Anod - y6
		case 7:  GPIO_PORTD_AHB_DATA_R |=  (1<<7); break; // Anod - X7
		case 8:  GPIO_PORTD_AHB_DATA_R |=  (1<<1); break; // Anod - y8
		case 9:  GPIO_PORTP_DATA_R     |=  (1<<2); break; // Anod - y9
		case 10: GPIO_PORTD_AHB_DATA_R |=  (1<<0); break; // Anod - y10
		case 11: GPIO_PORTM_DATA_R     |=  (1<<3); break; // Anod - y11
		case 12: GPIO_PORTH_AHB_DATA_R |=  (1<<2); break; // Anod - y12
		case 13: GPIO_PORTH_AHB_DATA_R |=  (1<<3); break; // Anod - y13
		case 14: GPIO_PORTN_DATA_R     |=  (1<<2); break; // Anod - y14
		case 15: GPIO_PORTN_DATA_R     |=  (1<<3); break; // Anod - X15
	}

	// Pre-clean
	GPIO_PORTC_AHB_DATA_R |=  (1<<4); // Catod - X0
	GPIO_PORTC_AHB_DATA_R |=  (1<<5); // Catod - X1
	GPIO_PORTC_AHB_DATA_R |=  (1<<6); // Catod - X2
	GPIO_PORTE_AHB_DATA_R |=  (1<<5); // Catod - X3
	GPIO_PORTE_AHB_DATA_R |=  (1<<0); // Catod - X4
	GPIO_PORTE_AHB_DATA_R |=  (1<<1); // Catod - X5
	GPIO_PORTE_AHB_DATA_R |=  (1<<2); // Catod - X6
	GPIO_PORTE_AHB_DATA_R |=  (1<<3); // Catod - x7

	for ( ll=0; ll<8; ll++ )
	switch ( Byte & (1<<ll)  ) {
		case 0x01<<0: GPIO_PORTC_AHB_DATA_R &= ~(1<<4); break; // Catod - X0
		case 0x01<<1: GPIO_PORTC_AHB_DATA_R &= ~(1<<5); break; // Catod - X1
		case 0x01<<2: GPIO_PORTC_AHB_DATA_R &= ~(1<<6); break; // Catod - X2
		case 0x01<<3: GPIO_PORTE_AHB_DATA_R &= ~(1<<5); break; // Catod - X3
		case 0x01<<4: GPIO_PORTE_AHB_DATA_R &= ~(1<<0); break; // Catod - X3
		case 0x01<<5: GPIO_PORTE_AHB_DATA_R &= ~(1<<1); break; // Catod - X3
		case 0x01<<6: GPIO_PORTE_AHB_DATA_R &= ~(1<<2); break; // Catod - X5
		case 0x01<<7: GPIO_PORTE_AHB_DATA_R &= ~(1<<3); break; // Catod - X6
	} */
}
//******************************************************************************



/* *****************************************************************************
 * LEDS[ 0,   1,   2,   3   ]
 *    =[ PN1, PN0, PF4, PF0 ]
 * ************************************************************************** */
int drv_led_blink (void) {
    volatile uint32_t ui32Loop;
    //.........................................................................
    if (*timer[leds4on].common_rule==1){
        if (timer[leds1on].ready_to_use) {
        	timer[leds1off].ready_to_use=0;
            if ((timer[leds1on].timer_is_set) && (++timer[leds1on].cur_timer_val==timer[leds1on].set_timer_limit)) {
        		timer[leds1on].cur_timer_val=0;
            	timer[leds1off].ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x02;    // Turn on the LED.
        	}
        }
        if (timer[leds1off].ready_to_use) {
        	timer[leds1on].ready_to_use=0;
        	if ((timer[leds1off].timer_is_set) && (++timer[leds1off].cur_timer_val==timer[leds1off].set_timer_limit)) {
    			timer[leds1off].cur_timer_val=0;
    			timer[leds1on].ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x02);    // Turn off the LED.
    			leds_position=2;
        	}
    	}
    }
    //.........................................................................
    if (*timer[leds1on].common_rule==2){
        if (timer[leds2on].ready_to_use) {
        	timer[leds2off].ready_to_use=0;
            if ((timer[leds2on].timer_is_set) && (++timer[leds2on].cur_timer_val==timer[leds2on].set_timer_limit)) {
        		timer[leds2on].cur_timer_val=0;
            	timer[leds2off].ready_to_use=1;
        		GPIO_PORTN_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer[leds2off].ready_to_use) {
        	timer[leds2on].ready_to_use=0;
        	if ((timer[leds2off].timer_is_set) && (++timer[leds2off].cur_timer_val==timer[leds2off].set_timer_limit)) {
    			timer[leds2off].cur_timer_val=0;
    			timer[leds2on].ready_to_use=1;
    			GPIO_PORTN_DATA_R &= ~(0x01);    // Turn off the LED.
    			leds_position=3;
        	}
    	}
    }
    //.........................................................................
    if (*timer[leds2on].common_rule==3){
        if (timer[leds3on].ready_to_use) {
        	timer[leds3off].ready_to_use=0;
            if ((timer[leds3on].timer_is_set) && (++timer[leds3on].cur_timer_val==timer[leds3on].set_timer_limit)) {
        		timer[leds3on].cur_timer_val=0;
            	timer[leds3off].ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x10;    // Turn on the LED.
        	}
        }
        if (timer[leds3off].ready_to_use) {
        	timer[leds3on].ready_to_use=0;
        	if ((timer[leds3off].timer_is_set) && (++timer[leds3off].cur_timer_val==timer[leds3off].set_timer_limit)) {
    			timer[leds3off].cur_timer_val=0;
    			timer[leds3on].ready_to_use=1;
    			GPIO_PORTF_AHB_DATA_R &= ~(0x10);    // Turn off the LED.
    			leds_position=4;
        	}
    	}
    }
    //.........................................................................
    if (*timer[leds3on].common_rule==4){
        if (timer[leds4on].ready_to_use) {
        	timer[leds4off].ready_to_use=0;
            if ((timer[leds4on].timer_is_set) && (++timer[leds4on].cur_timer_val==timer[leds4on].set_timer_limit)) {
        		timer[leds4on].cur_timer_val=0;
            	timer[leds4off].ready_to_use=1;
            	GPIO_PORTF_AHB_DATA_R |= 0x01;    // Turn on the LED.
        	}
        }
        if (timer[leds4off].ready_to_use) {
        	timer[leds4on].ready_to_use=0;
        	if ((timer[leds4off].timer_is_set) && (++timer[leds4off].cur_timer_val==timer[leds4off].set_timer_limit)) {
    			timer[leds4off].cur_timer_val=0;
    			timer[leds4on].ready_to_use=1;
    			GPIO_PORTF_AHB_DATA_R &= ~(0x01);    // Turn off the LED.
    			leds_position=1;
        	}
    	}
    }
    //.........................................................................
    return 0;
}

//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_ret_code keys_scan ( t_io_values *p_io )
{
	t_ret_code rc = RC_FAILED;

	// Check input pointers
	if ( p_io==NULL ) return RC_PTR_FAIL;

	rc = RC_OK;

	if ( GPIO_PORTF_AHB_DATA_R & (1<<1) ) { p_io->keys.k5=0; } else { p_io->keys.k5=1; }
	if ( GPIO_PORTF_AHB_DATA_R & (1<<2) ) { p_io->keys.k4=0; } else { p_io->keys.k4=1; }
	if ( GPIO_PORTF_AHB_DATA_R & (1<<3) ) { p_io->keys.k3=0; } else { p_io->keys.k3=1; }
	if ( GPIO_PORTG_AHB_DATA_R & (1<<0) ) { p_io->keys.k2=0; } else { p_io->keys.k2=1; }
	if ( GPIO_PORTL_DATA_R & (1<<4)     ) { p_io->keys.k1=0; } else { p_io->keys.k1=1; }
	if ( GPIO_PORTL_DATA_R & (1<<5)     ) { p_io->keys.k0=0; } else { p_io->keys.k0=1; }

	if ( (GPIO_PORTJ_AHB_DATA_R & 0x03) == BTN_PCS_0 ) {
		if ( p_io->in<99999 )
		if ( p_io->in<1 ) p_io->in += 0.0001; else
		if ( p_io->in<10 ) p_io->in += 0.001; else
		if ( p_io->in<100 ) p_io->in += 0.01; else
		if ( p_io->in<1000 ) p_io->in += 0.1; else
		if ( p_io->in<10000 ) p_io->in +=  1;

		if ( ++p_io->x>7 ) p_io->x=0;
		//srand ( p_io->x *100 );
	}
	if ( (GPIO_PORTJ_AHB_DATA_R & 0x03) == BTN_PCS_1 ) {
		if ( p_io->in>0 )
		if ( p_io->in<1 ) p_io->in -= 0.0001; else
		if ( p_io->in<10 ) p_io->in -= 0.001; else
		if ( p_io->in<100 ) p_io->in -= 0.01; else
		if ( p_io->in<1000 ) p_io->in -= 0.1; else
		if ( p_io->in<10000 ) p_io->in -=  1;

		if ( ++p_io->y>15 ) p_io->y=0;
		//srand ( p_io->y *100 );
	}

	return rc;
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
t_ret_code keys_analyze ( t_io_values *p_io, t_sys_values *p_sys ) {
t_ret_code rc = RC_FAILED;

	// Check input pointers
	if ( p_io==NULL | p_sys==NULL ) return RC_PTR_FAIL;

	rc = RC_OK;

	p_sys->key0 =0;
	p_sys->key1 =0;
	p_sys->key2 =0;
	p_sys->key3 =0;
	p_sys->key4 =0;
	p_sys->key5 =0;

	if ( p_io->keys.k0 ) p_sys->key0 =1;
	if ( p_io->keys.k1 ) p_sys->key1 =1;
	if ( p_io->keys.k2 ) p_sys->key2 =1;
	if ( p_io->keys.k3 ) p_sys->key3 =1;
	if ( p_io->keys.k4 ) p_sys->key4 =1;
	if ( p_io->keys.k5 ) p_sys->key5 =1;

	return rc;
}
//******************************************************************************



/* *****************************************************************************
 * Number in to string converter (itoa)
 * ************************************************************************** */
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



/* *****************************************************************************
 * Number in to string converter
 * ************************************************************************** */
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



/* *****************************************************************************
 *
 * ************************************************************************** */
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
}
//******************************************************************************



/* *****************************************************************************
 *
 * ************************************************************************** */
uint8_t *ftostr(float value, uint8_t *p_string, int base, uint8_t *p_comma)
{
  unsigned long t=0;
  unsigned long res=0;
  unsigned long val=0;
  float         ostatok_float=0;
  uint32_t      celoe=0, ostatok_int=0;
  uint8_t       dlina_ostatka=0;

  if (NULL == p_string) {
    return NULL;
  }

  if (NULL == p_comma) {
    return NULL;
  }

  celoe = (int)value;
  ostatok_float = value - celoe;

  if (celoe<1) {
	  *p_comma=0;
		ostatok_int = (uint32_t)((ostatok_float * 10000));
		dlina_ostatka = 5 - (*p_comma);
		p_string += 0;
  } else
  if (celoe<10) {
	  *p_comma=0;
		ostatok_int = (uint32_t)((ostatok_float * 10000));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 1;
  } else
  if (celoe<100) {
	  *p_comma=1;
		ostatok_int = (uint32_t)((ostatok_float * 1000));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 2;
  } else
  if (celoe<1000) {
	  *p_comma=2;
		ostatok_int = (uint32_t)((ostatok_float * 100));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 3;
  } else
  if (celoe<10000) {
	  *p_comma=3;
		ostatok_int = (uint32_t)((ostatok_float * 10));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 4;
  } else
  if (celoe<100000) {
	  *p_comma=4;
		ostatok_int = (uint32_t)((ostatok_float * 1));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 5;
  } else
  if (celoe<1000000) {
	  *p_comma=4;
		ostatok_int = (uint32_t)((ostatok_float * 0));
		dlina_ostatka = 5 - (*p_comma+1);
		p_string += 6;
  }

  val = value;

  do {  // display - Celoe
	t = val / base;
    res = val - base * t; // res = drobnaya ot value
    if (res < 10) {
      *--p_string = '0' + res;
    }
  } while ((val = t) != 0);

  if (celoe<1) { p_string += 5;  }     else
  if (celoe<10) { p_string += 4; }     else
  if (celoe<100) { p_string += 4; }    else
  if (celoe<1000) { p_string += 4; }   else
  if (celoe<10000) { p_string += 4; }  else
  if (celoe<100000) { p_string += 4; } else
  if (celoe<1000000) { p_string += 4; }

  if (*p_comma<5) { // Display - Ostatok
	  do {
	    if (dlina_ostatka>0) {
			t = ostatok_int / base;
			res = ostatok_int - base * t; // res = drobnaya ot value
			if (res < 10) {
			  *p_string-- = '0' + res;
			}
	      dlina_ostatka--;
	    }
	  } while ( (ostatok_int=t)!=0 || dlina_ostatka!=0 );
  }

  return (p_string);
}
//******************************************************************************



//volatile unsigned long delay;

//******************************************************************************
// configure the system to get its clock from the PLL
//******************************************************************************
void drv_sys_init_pll (void) {
	uint32_t timeout;

	// 1) Once POR has completed, the PIOSC is acting as the system clock.  Just in case
	//  this function has been called previously, be sure that the system is not being
	//  clocked from the PLL while the PLL is being reconfigured.
	SYSCTL_RSCLKCFG_R &= ~SYSCTL_RSCLKCFG_USEPLL;

	// 2) Power up the MOSC by clearing the NOXTAL bit in the SYSCTL_MOSCCTL_R register.

	// 3) Since crystal mode is required, clear the PWRDN bit.  The datasheet says to do
	//  these two operations in a single write access to SYSCTL_MOSCCTL_R.
	SYSCTL_MOSCCTL_R &= ~( SYSCTL_MOSCCTL_NOXTAL | SYSCTL_MOSCCTL_PWRDN );
	//  Wait for the MOSCPUPRIS bit to be set in the SYSCTL_RIS_R register, indicating
	//  that MOSC crystal mode is ready.
	while ( ( SYSCTL_RIS_R & SYSCTL_RIS_MOSCPUPRIS )==0 ) { };

	// 4) Set both the OSCSRC and PLLSRC fields to 0x3 in the
	//  SYSCTL_RSCLKCFG_R register at offset 0x0B0.
	//  Temporarily get run/sleep clock from 25 MHz main oscillator.
	SYSCTL_RSCLKCFG_R =
			( SYSCTL_RSCLKCFG_R & ~SYSCTL_RSCLKCFG_OSCSRC_M) + SYSCTL_RSCLKCFG_OSCSRC_MOSC;
	//  PLL clock from main oscillator.
	SYSCTL_RSCLKCFG_R =
			( SYSCTL_RSCLKCFG_R & ~SYSCTL_RSCLKCFG_PLLSRC_M ) + SYSCTL_RSCLKCFG_PLLSRC_MOSC;

	// 5) If the application also requires the MOSC to be the deep-sleep clock source,
	//  then program the DSOSCSRC field in the SYSCTL_DSCLKCFG_R register to 0x3.
	//  Get deep-sleep clock from main oscillator (few examples use deep-sleep; optional).
	SYSCTL_DSCLKCFG_R =
			( SYSCTL_DSCLKCFG_R & ~SYSCTL_DSCLKCFG_DSOSCSRC_M ) + SYSCTL_DSCLKCFG_DSOSCSRC_MOSC;

	// 6) Write the SYSCTL_PLLFREQ0_R and SYSCTL_PLLFREQ1_R registers with the values of
	//  Q, N, MINT, and MFRAC to configure the desired VCO frequency setting.
	// ************
	//  The datasheet implies that the VCO frequency can go as high as 25.575 GHz
	//  with MINT=1023 and a 25 MHz crystal.  This is clearly unreasonable.  For lack
	//  of a recommended VCO frequency, this program sets Q, N, and MINT for a VCO
	//  frequency of 480 MHz with MFRAC=0 to reduce jitter.  To run at a frequency
	//  that is not an integer divisor of 480 MHz, change this section.
	//  fVC0 = (fXTAL/(Q + 1)/(N + 1))*(MINT + (MFRAC/1,024))
	//  fVCO = 480,000,000 Hz (arbitrary, but presumably as small as needed)
	#define FXTAL 25000000  // fixed, this crystal is soldered to the Connected Launchpad
	#define Q            0
	#define N            4  // chosen for reference frequency within 4 to 30 MHz
	#define MINT        96  // 480,000,000 = (25,000,000/(0 + 1)/(4 + 1))*(96 + (0/1,024))
	#define MFRAC        0  // zero to reduce jitter
	//  SysClk = fVCO / (PSYSDIV + 1)
	#define SYSCLK (FXTAL/(Q+1)/(N+1))*(MINT+MFRAC/1024)/(PSYSDIV+1)
	SYSCTL_PLLFREQ0_R =
			(SYSCTL_PLLFREQ0_R&~SYSCTL_PLLFREQ0_MFRAC_M)+(MFRAC<<SYSCTL_PLLFREQ0_MFRAC_S) |
			(SYSCTL_PLLFREQ0_R&~SYSCTL_PLLFREQ0_MINT_M)+(MINT<<SYSCTL_PLLFREQ0_MINT_S);
	SYSCTL_PLLFREQ1_R =
			(SYSCTL_PLLFREQ1_R&~SYSCTL_PLLFREQ1_Q_M)+(Q<<SYSCTL_PLLFREQ1_Q_S) |
			(SYSCTL_PLLFREQ1_R&~SYSCTL_PLLFREQ1_N_M)+(N<<SYSCTL_PLLFREQ1_N_S);
	SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR;   // turn on power to PLL
	SYSCTL_RSCLKCFG_R |= SYSCTL_RSCLKCFG_NEWFREQ;  // lock in register changes

	// 7) Write the SYSCTL_MEMTIM0_R register to correspond to the new clock setting.
	//    ************
	//    Set the timing parameters to the main Flash and EEPROM memories, which
	//    depend on the system clock frequency.  See Table 5-12 in datasheet.
	if(SYSCLK < 16000000) {
		// FBCHT/EBCHT = 0, FBCE/EBCE = 0, FWS/EWS = 0
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x0<<22) + (0x0<<21) + (0x0<<16) + (0x0<<6) + (0x0<<5) + (0x0);
	} else if (SYSCLK == 16000000) {
		// FBCHT/EBCHT = 0, FBCE/EBCE = 1, FWS/EWS = 0
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x0<<22) + (0x1<<21) + (0x0<<16) + (0x0<<6) + (0x1<<5) + (0x0);
	} else if (SYSCLK <= 40000000) {
		// FBCHT/EBCHT = 2, FBCE/EBCE = 0, FWS/EWS = 1
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x2<<22) + (0x0<<21) + (0x1<<16) + (0x2<<6) + (0x0<<5) + (0x1);
	} else if (SYSCLK <= 60000000) {
		// FBCHT/EBCHT = 3, FBCE/EBCE = 0, FWS/EWS = 2
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x3<<22) + (0x0<<21) + (0x2<<16) + (0x3<<6) + (0x0<<5) + (0x2);
	} else if (SYSCLK <= 80000000) {
		// FBCHT/EBCHT = 4, FBCE/EBCE = 0, FWS/EWS = 3
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x4<<22) + (0x0<<21) + (0x3<<16) + (0x4<<6) + (0x0<<5) + (0x3);
	} else if (SYSCLK <= 100000000) {
		// FBCHT/EBCHT = 5, FBCE/EBCE = 0, FWS/EWS = 4
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x5<<22) + (0x0<<21) + (0x4<<16) + (0x5<<6) + (0x0<<5) + (0x4);
	} else if (SYSCLK <= 120000000) {
		// FBCHT/EBCHT = 6, FBCE/EBCE = 0, FWS/EWS = 5
		SYSCTL_MEMTIM0_R =
				(SYSCTL_MEMTIM0_R & ~0x03EF03EF) +
				(0x6<<22) + (0x0<<21) + (0x5<<16) + (0x6<<6) + (0x0<<5) + (0x5);
	} else {
		// A setting is invalid, and the PLL cannot clock the system faster than 120 MHz.
		// Skip the rest of the initialization, leaving the system clocked from the MOSC,
		// which is a 25 MHz crystal.
		return;
	}

	// 8) Wait for the SYSCTL_PLLSTAT_R register to indicate that the PLL has reached
	//    lock at the new operating point (or that a timeout period has passed and lock
	//    has failed, in which case an error condition exists and this sequence is
	//    abandoned and error processing is initiated).
	timeout = 0;
	while ( ((SYSCTL_PLLSTAT_R & SYSCTL_PLLSTAT_LOCK ) == 0) && (timeout < 0xFFFF)) {
		timeout = timeout + 1;
	}

	if (timeout == 0xFFFF) {
		// The PLL never locked or is not powered.
		// Skip the rest of the initialization, leaving the system clocked from the MOSC,
		// which is a 25 MHz crystal.
		return;
	}

	// 9)Write the SYSCTL_RSCLKCFG_R register's PSYSDIV value, set the USEPLL bit to
	//   enabled, and set the MEMTIMU bit.
	SYSCTL_RSCLKCFG_R =
		(SYSCTL_RSCLKCFG_R & ~SYSCTL_RSCLKCFG_PSYSDIV_M) + (PSYSDIV & SYSCTL_RSCLKCFG_PSYSDIV_M) |
		SYSCTL_RSCLKCFG_MEMTIMU |
		SYSCTL_RSCLKCFG_USEPLL;
}
//******************************************************************************



//******************************************************************************
//debug code
//******************************************************************************
// delay function for testing from sysctl.c
// which delays 3*ulCount cycles
// ... Delay(16666667); // delay ~1 sec @ 50 MHz
// ... Delay(2000000);  // delay ~0.05 sec @ 120 MHz (0.375 sec @ 16 MHz; 0.078125 sec @ 76.8 MHz)
#ifdef __TI_COMPILER_VERSION__
	void Delay (uint32_t ulCount) {        // @ Code Composer Studio Code
		__asm (
		  " subs    r0, #1 \n "
		  " bne     Delay \n "
		  " bx      lr \n "  );
		//while (ulCount-->0) {};
	}
#else
	__asm void Delay (uint32_t ulCount) {  // @ Keil uVision Code
		subs    r0, #1
		bne     Delay
		bx      lr    }
#endif // #ifdef __TI_COMPILER_VERSION__
//******************************************************************************



//******************************************************************************
//#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
//#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
//#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value
//******************************************************************************
// Initialize SysTick with busy wait running at bus clock.
//******************************************************************************
void drv_sys_init_SysTick ( void ) {
  NVIC_ST_CTRL_R    = 0;                    // disable SysTick during setup
  NVIC_ST_RELOAD_R  = NVIC_ST_RELOAD_M;     // maximum reload value
  NVIC_ST_CURRENT_R = 0;                    // any write to current clears it
										    // enable SysTick with core clock
  NVIC_ST_CTRL_R    = NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC;
}
//******************************************************************************



//******************************************************************************
// Time delay using busy wait.
// The delay parameter is in units of the 6 MHz core clock. (167 nsec)
//******************************************************************************
void SysTick_Wait ( unsigned long delay ) {
  volatile unsigned long  elapsedTime;
  unsigned long           startTime = NVIC_ST_CURRENT_R;
  do {
    elapsedTime = ( startTime-NVIC_ST_CURRENT_R ) & 0x00FFFFFF;
  }
  while ( elapsedTime <= delay );
}
//******************************************************************************



//******************************************************************************
// Time delay using busy wait.
// 10000us equals 10ms
//******************************************************************************
void SysTick_Wait_10ms ( unsigned long delay ) {
  unsigned long  i;
  for ( i=0; i<delay; i++ ) {
    SysTick_Wait ( 60000 );  // wait 10ms (assumes 6 MHz clock)
  }
}
//******************************************************************************



//******************************************************************************
extern int snake_main ( void ) ;

extern uint32_t  u32_rtc_time;
extern uint32_t  u32_rtc_date;
t_io_values      io  = {0};
t_sys_values     sys = {0};
t_io_Snake       s_snake = { .algo_step = S_INIT };


/* *****************************************************************************
 * Blinks on the leds and indicator.
 * ************************************************************************** */
int main (void) {
              //uint8_t    rc=1;
              uint8_t    comma;
              uint8_t   *p_str=0;
    volatile  uint32_t   pos;
    volatile  uint32_t   cnt=0;

    //SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    drv_sys_init_SysTick ();
    drv_sys_init_pll ();
    drv_sys_init_gpio_output_input ();
    drv_usr_init_led_7segments ();
    drv_usr_init_led_8x8 ();
    drv_usr_init_rtc ();
    drv_usr_init_scheduler_end_timers ();

    leds_position = 1;
    io.in = 0.0001; // Set to default value

	while (1) { // Loop forever.
	//	drv_led_blink (); // timer_leds4on.common_rule

		//if (sys.key_pressed==0)
		//	drv_led_7segments_clean();
		//else
		if ( _TIMER_READY == delay_timer ( leds5x8, '?' ) ) {
			if ( pos < 5 ) {
				p_str = ftostr ( io.in, data, 10, &comma ); // value, *ptr, base, *comma
				if (p_str!=NULL) {
					drv_led_7segments_symbol ( pos, data[pos], comma ); // display info
					drv_led_7segments_position ( pos ); // dinamic switching
					//drv_led_8x8_pixel_set ( io.x, io.y, 0 ); // display info
				}
				pos++;
				cnt++;
    		} else pos=0;
    	}

		if ( _TIMER_READY == delay_timer ( keys, '?' ) ) {
			keys_scan ( &io );
			keys_analyze ( &io, &sys );
			if ( sys.key0 ||sys.key1 ||sys.key2 ||sys.key3 ||sys.key4 ||sys.key5 ) {
				sys.key_pressed=1;
				if (sys.key5) { if ( io.in < 99999 ) io.in += 0.001; }
				if (sys.key4) { if ( io.in > 0 )     io.in -= 0.001; }
				if (sys.key3) { if ( io.in < 99999 ) io.in += 0.01; }
				if (sys.key2) { if ( io.in > 0 )     io.in -= 0.01; }
				if (sys.key1) { if ( io.in < 99999 ) io.in += 0.1; }
				if (sys.key0) { if ( io.in > 0 )     io.in -= 0.1; }
				//srand ( (char)io.in ); // Initiate seek random number
				//s_snake.rnd = rnd_fill_field( (io.in)*100 );
			} else {
				sys.key_pressed=0;
			}
		}

		if ( _TIMER_READY == delay_timer( leds8x8, '?' ) ) {
			//int  rc;
			//rc = snake_main();

			if ( RC_OK != algo_Snake (&io, &s_snake) ) { error_forever_loop; }

			/*  char  yy=0;
			float val=0;
			rc = fft_calc();
			if ( yy<16 ) {
				//val = abs(buf_out[yy]*10);
				if ( buf_out[yy] <0 ) {  val = -1*(buf_out[yy] * 10000);
				} else {                 val =    (buf_out[yy] * 10000); }
				drv_led_8x8_show_byte( (uint8_t)val, xx, yy );
				yy++;
			} else {
				yy=0;
			}  */
		}

		/*  if ( _TIMER_READY == delay_timer ( rnd, '?' ) ) {
			if ( ++io.rnd>7 ) io.rnd=0;

			u32_rtc_time = HIB_CAL0_R;  //Read RTC Time
			u32_rtc_date = HIB_CAL1_R;  //Read RTC Date
		}  */

		if ( _TIMER_READY == delay_timer ( mouse, '?' ) ) {
			//mouse_main();
    		GPIO_PORTN_DATA_R ^= 0x02;    // Toogle on the LED.
		}
	}

	//return rc;
}
//******************************************************************************
