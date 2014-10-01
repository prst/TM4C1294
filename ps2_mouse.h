/*
 * ps2_mouse.h
 *
 *  Created on: 27 сент. 2014 г.
 *      Author: Stan
 */

#ifndef PS2_MOUSE_H_
#define PS2_MOUSE_H_



//-----------------------------------------------------------------------------
/*interrupt [USART_RXC]*/
void  usart_rx_isr(void);
char  getchar(void);
void  gohi(int pin);
void  golo(int pin);
void  mouse_write(char data);
signed char mouse_read(void);
void  mouse_init();
void  loop1(void);
void  mouse_main(void);
//-----------------------------------------------------------------------------



#endif /* PS2_MOUSE_H_ */
