
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
typedef enum {
	_TIMER_READY     = 0,
	_TIMER_NOT_READY = 1
} t_timer_stat;


/* Discriptor of timer settings */
typedef struct {
	uint8_t    ready_to_use   :1;
	uint8_t    timer_is_set   :1;
	uint32_t   set_timer_limit;
	uint32_t   cur_timer_val;
	uint32_t  *common_rule;
} t_Scheduler;


/* Discriptor of Keys */
typedef struct {
	uint8_t  k0 :1;
	uint8_t  k1 :1;
	uint8_t  k2 :1;
	uint8_t  k3 :1;
	uint8_t  k4 :1;
	uint8_t  k5 :1;
	uint8_t  reserv :2;
} t_io_keys;


/* Discriptor of I/O */
typedef struct {
	float      in;
	uint8_t    x;
	uint8_t    y;
	t_io_keys  keys;
	uint8_t    rnd;
} t_io_values;


/* Discriptor of Keys */
typedef struct {
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
typedef struct {
	uint8_t   *body;
	uint8_t   *ptr_field;
	uint8_t   *ptr_field_eat;
	uint32_t  len       :4;  // Lenght
	uint32_t  direction :2;  // 0:Up, 1:Down, 2:Left, 3:Right
	uint32_t  blinkhead :2;  // Blinking state for head of Snake
	uint32_t  curr_x    :3;  // Current position of head, axis X
	uint32_t  curr_y    :4;  // Current position of head, axis X
	uint32_t  next_x    :3;  // Calculated next position of head, axis X
	uint32_t  next_y    :4;  // Calculated next position of head, axis X
	uint32_t  algo_step :4;  // Algorithm step
	uint32_t  rezerv    :6;  // ...
	uint32_t  rnd;           // Random value
} t_io_Snake;


typedef enum {
	RC_OK = 0,
	RC_PTR_FAIL,
	RC_FAILED
} t_ret_code;


typedef enum {
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
int   drv_led_blink (void);

//******************************************************************************


