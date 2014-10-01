
//******************************************************************************
//******************************************************************************

#include <stdlib.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"

#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"



//******************************************************************************
//******************************************************************************

//typedef  enum { FALSE, TRUE } bool;


/*#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define SYSCTL_RIS_R_            (*((volatile unsigned long *)0x400FE050))
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define SYSCTL_RCC_R            (*((volatile unsigned long *)0x400FE060))
#define SYSCTL_RCC_SYSDIV_M     0x07800000  // System Clock Divisor
#define SYSCTL_RCC_SYSDIV_4     0x01800000  // System clock /4
#define SYSCTL_RCC_SYSDIV_5     0x02000000  // System clock /5
#define SYSCTL_RCC_SYSDIV_6     0x02800000  // System clock /6
#define SYSCTL_RCC_SYSDIV_7     0x03000000  // System clock /7
#define SYSCTL_RCC_SYSDIV_8     0x03800000  // System clock /8
#define SYSCTL_RCC_SYSDIV_9     0x04000000  // System clock /9
#define SYSCTL_RCC_SYSDIV_10    0x04800000  // System clock /10
#define SYSCTL_RCC_SYSDIV_11    0x05000000  // System clock /11
#define SYSCTL_RCC_SYSDIV_12    0x05800000  // System clock /12
#define SYSCTL_RCC_SYSDIV_13    0x06000000  // System clock /13
#define SYSCTL_RCC_SYSDIV_14    0x06800000  // System clock /14
#define SYSCTL_RCC_SYSDIV_15    0x07000000  // System clock /15
#define SYSCTL_RCC_SYSDIV_16    0x07800000  // System clock /16
#define SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider
#define SYSCTL_RCC_PWRDN        0x00002000  // PLL Power Down
#define SYSCTL_RCC_OEN          0x00001000  // PLL Output Enable
#define SYSCTL_RCC_BYPASS       0x00000800  // PLL Bypass
#define SYSCTL_RCC_XTAL_M       0x000003C0  // Crystal Value
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // 6 MHz Crystal
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  // 8 MHz Crystal
#define SYSCTL_RCC_XTAL_25MHZ   0x00000B6D  // 25 MHz Crystal
#define SYSCTL_RCC_OSCSRC_M     0x00000030  // Oscillator Source
#define SYSCTL_RCC_OSCSRC_MAIN  0x00000000  // MOSC
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOC      0x00000004  // port C Clock Gating Control
#define GPIO_PORTC5             (*((volatile unsigned long *)0x40006080))
*/
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCGPIO_R12     0x00001000  // GPIO Port N Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R8      0x00000100  // GPIO Port J Run Mode Clock
                                            // Gating Control
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRGPIO_R12       0x00001000  // GPIO Port N Peripheral Ready
#define SYSCTL_PRGPIO_R8        0x00000100  // GPIO Port J Peripheral Ready

#define PSYSDIV 3


// The #define statement PSYSDIV in PLL.h initializes the PLL to the desired frequency.
// -----------------------------------------------------------------------------
// bus frequency is 480MHz/(PSYSDIV+1) = 480MHz/(3+1) = 120 MHz
// IMPORTANT: See Step 6) of PLL_Init().  If you change something, change 480 MHz.
// see the table at the end of this file
#define SYSCTL_RIS_R                  (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_RIS_MOSCPUPRIS         0x00000100  // MOSC Power Up Raw Interrupt Status
#define SYSCTL_MOSCCTL_R              (*((volatile uint32_t *)0x400FE07C))
#define SYSCTL_MOSCCTL_PWRDN          0x00000008  // Power Down
#define SYSCTL_MOSCCTL_NOXTAL         0x00000004  // No Crystal Connected
#define SYSCTL_RSCLKCFG_R             (*((volatile uint32_t *)0x400FE0B0))
#define SYSCTL_RSCLKCFG_MEMTIMU       0x80000000  // Memory Timing Register Update
#define SYSCTL_RSCLKCFG_NEWFREQ       0x40000000  // New PLLFREQ Accept
#define SYSCTL_RSCLKCFG_USEPLL        0x10000000  // Use PLL
#define SYSCTL_RSCLKCFG_PLLSRC_M      0x0F000000  // PLL Source
#define SYSCTL_RSCLKCFG_PLLSRC_MOSC   0x03000000  // MOSC is the PLL input clock source
#define SYSCTL_RSCLKCFG_OSCSRC_M      0x00F00000  // Oscillator Source
#define SYSCTL_RSCLKCFG_OSCSRC_MOSC   0x00300000  // MOSC is oscillator source
#define SYSCTL_RSCLKCFG_PSYSDIV_M     0x000003FF  // PLL System Clock Divisor
#define SYSCTL_MEMTIM0_R              (*((volatile uint32_t *)0x400FE0C0))
#define SYSCTL_DSCLKCFG_R             (*((volatile uint32_t *)0x400FE144))
#define SYSCTL_DSCLKCFG_DSOSCSRC_M    0x00F00000  // Deep Sleep Oscillator Source
#define SYSCTL_DSCLKCFG_DSOSCSRC_MOSC 0x00300000  // MOSC
#define SYSCTL_PLLFREQ0_R             (*((volatile uint32_t *)0x400FE160))
#define SYSCTL_PLLFREQ0_PLLPWR        0x00800000  // PLL Power
#define SYSCTL_PLLFREQ0_MFRAC_M       0x000FFC00  // PLL M Fractional Value
#define SYSCTL_PLLFREQ0_MINT_M        0x000003FF  // PLL M Integer Value
#define SYSCTL_PLLFREQ0_MFRAC_S       10
#define SYSCTL_PLLFREQ0_MINT_S        0
#define SYSCTL_PLLFREQ1_R             (*((volatile uint32_t *)0x400FE164))
#define SYSCTL_PLLFREQ1_Q_M           0x00001F00  // PLL Q Value
#define SYSCTL_PLLFREQ1_N_M           0x0000001F  // PLL N Value
#define SYSCTL_PLLFREQ1_Q_S           8
#define SYSCTL_PLLFREQ1_N_S           0
#define SYSCTL_PLLSTAT_R              (*((volatile uint32_t *)0x400FE168))
#define SYSCTL_PLLSTAT_LOCK           0x00000001  // PLL Lock

/* PSYSDIV  SysClk (Hz)
  3     120,000,000
  4      96,000,000
  5      80,000,000
  7      60,000,000
  9      48,000,000
 15      30,000,000
 19      24,000,000
 29      16,000,000
 39      12,000,000
 79       6,000,000  */


/* *****************************************************************************
 *
 * ************************************************************************** */
//#define _LEDS_POS_I_SET   ( GPIO_PORTA_AHB_DATA_R |=  (1<<2) ) // Anod (Common) 4
//#define _LEDS_POS_I_CLR   ( GPIO_PORTA_AHB_DATA_R &= ~(1<<2) ) // Anod (Common) 4
#define _LEDS_POS_I_SET   ( GPIO_PORTQ_DATA_R |=  (1<<3) ) // Anod (Common) 4
#define _LEDS_POS_I_CLR   ( GPIO_PORTQ_DATA_R &= ~(1<<3) ) // Anod (Common) 4
#define _LEDS_POS_II_SET  ( GPIO_PORTP_DATA_R |=  (1<<3) ) // Anod (Common) 1
#define _LEDS_POS_II_CLR  ( GPIO_PORTP_DATA_R &= ~(1<<3) ) // Anod (Common) 1
#define _LEDS_POS_III_SET ( GPIO_PORTQ_DATA_R |=  (1<<1) ) // Anod (Common) 2
#define _LEDS_POS_III_CLR ( GPIO_PORTQ_DATA_R &= ~(1<<1) ) // Anod (Common) 2
//#define _LEDS_POS_IV_SET  ( GPIO_PORTA_AHB_DATA_R |=  (1<<3) ) // Anod (Common) 3
//#define _LEDS_POS_IV_CLR  ( GPIO_PORTA_AHB_DATA_R &= ~(1<<3) ) // Anod (Common) 3
#define _LEDS_POS_IV_SET  ( GPIO_PORTQ_DATA_R |=  (1<<2) ) // Anod (Common) 3
#define _LEDS_POS_IV_CLR  ( GPIO_PORTQ_DATA_R &= ~(1<<2) ) // Anod (Common) 3
#define _LEDS_POS_V_SET   ( GPIO_PORTM_DATA_R |=  (1<<6) ) // Anod (Common) 0
#define _LEDS_POS_V_CLR   ( GPIO_PORTM_DATA_R &= ~(1<<6) ) // Anod (Common) 0
//******************************************************************************
#define  BTN_PCS_0  (1)
#define  BTN_PCS_1  (2)
//******************************************************************************
#define  error_forever_loop   for(;;){}
//******************************************************************************


// LEDS[0,1,2,3]=[PN1,PN0,PF4,PF0]
// USER_SWITCH[0,1]=[PJ0,PJ1]

/* *****************************************************************************
 * TYPES
 * ************************************************************************** */
typedef  enum {
	_TIMER_READY     = 0,
	_TIMER_NOT_READY = 1
} t_timer_stat;


/* Discriptor of timer settings */
typedef  struct {
	uint8_t    ready_to_use   :1;
	uint8_t    timer_is_set   :1;
	uint32_t   set_timer_limit;
	uint32_t   cur_timer_val;
	uint32_t  *common_rule;
} t_Scheduler;

typedef  enum {
	/* 0 */  leds5x8,
	/* 1 */  leds1on,
	/* 2 */  leds1off,
	/* 3 */  leds2on,
	/* 4 */  leds2off,
	/* 5 */  leds3on,
	/* 6 */  leds3off,
	/* 7 */  leds4on,
	/* 8 */  leds4off,
	/* 9 */  leds8x8,
	/* 10 */ keys,
	/* 11 */ fft,
	/* 12 */ rnd,
	/* 14 */ mouse,
	/* 15 */ name_end
} timer_name;

extern t_Scheduler  timer [ /*sizeof(timer_name)*/ 15 ];


/* Discriptor of Keys */
typedef  struct {
	uint8_t  k0 :1;
	uint8_t  k1 :1;
	uint8_t  k2 :1;
	uint8_t  k3 :1;
	uint8_t  k4 :1;
	uint8_t  k5 :1;
	uint8_t  reserv :2;
} t_io_keys;


/* Discriptor of I/O */
typedef  struct {
	float      in;
	uint8_t    x;
	uint8_t    y;
	t_io_keys  keys;
	uint8_t    rnd;
} t_io_values;


/* Discriptor of Keys */
typedef  struct {
	uint16_t  key0         :1;
	uint16_t  key1         :1;
	uint16_t  key2         :1;
	uint16_t  key3         :1;
	uint16_t  key4         :1;
	uint16_t  key5         :1;
	uint16_t  key_pressed  :1;
	uint16_t  status       :3;
	uint16_t  reserved     :6;
} t_sys_values;


/* Discriptor of zmeyka */
typedef  struct {
	uint8_t   *body;
	uint8_t   *ptr_field;
	uint8_t   *ptr_field_eat;
	uint32_t  len        :4;  // Lenght
	uint32_t  direction  :2;  // 0:Up, 1:Down, 2:Left, 3:Right
	uint32_t  blinkhead  :2;  // Blinking state for head of Snake
	uint32_t  curr_x     :3;  // Current position of head, axis X
	uint32_t  curr_y     :4;  // Current position of head, axis X
	uint32_t  next_x     :3;  // Calculated next position of head, axis X
	uint32_t  next_y     :4;  // Calculated next position of head, axis X
	uint32_t  algo_step  :4;  // Algorithm step
	uint32_t  rezerv     :6;  // ...
	uint32_t  rnd;            // Random value
} t_io_Snake;


typedef  enum {
	RC_OK = 0,
	RC_PTR_FAIL,
	RC_FAILED
} t_ret_code;


typedef  enum {
	S_INIT = 0,
	S_START_RANDOM,
	S_BEGIN,
	S_MOVE,
	S_EAT_YES,
	S_EAT_NO,
	S_RANDOM,
	S_CHANGE_DIRECTION,
	S_BORDER_END
} t_algo_step;

//******************************************************************************


//******************************************************************************
#define  FFT_NUM  (32)
extern double  buf_in[FFT_NUM];
extern double  buf_out[2*FFT_NUM];

//******************************************************************************


//******************************************************************************
void drv_led_8x8_pixel_set ( uint8_t x, uint8_t y, uint8_t val );
void drv_led_8x8_show_byte ( uint8_t Byte, uint8_t xx, uint8_t yy );
int  drv_led_blink (void);

void Delay (uint32_t ulCount);
void drv_sys_init_SysTick ( void );
void SysTick_Wait ( unsigned long delay );
void SysTick_Wait_10ms ( unsigned long delay );


//******************************************************************************


