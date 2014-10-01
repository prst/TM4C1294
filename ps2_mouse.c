/*
 * File:       ps2_mouse.c
 * Created on: 27 september 2014 ã.
 * Author:     Stanislav
 */
//-----------------------------------------------------------------------------
#include "TM4C1294_leds.h"
#include "driverlib/uart.h"
#include "ps2_mouse.h"
//-----------------------------------------------------------------------------

#define USE_PS2_MOUSE (1)

//-----------------------------------------------------------------------------
#if (USE_PS2_MOUSE==1)


#define KBD_CLK        GPIO_PORTP_DATA_R  // 2
#define KBD_DATA       GPIO_PORTP_DATA_R  // 3
#define KBD_CLK_DIR    GPIO_PORTP_DIR_R   // 2
#define KBD_DATA_DIR   GPIO_PORTP_DIR_R   // 3

#define MCLK           2
#define MDATA          3
#define CLK_READ       GPIO_PORTP_DATA_R & (1<<MCLK)
#define DATA_READ      GPIO_PORTP_DATA_R & (1<<MDATA)

#define FRAMING_ERROR        (1<<FE)
#define PARITY_ERROR         (1<<UPE)
#define DATA_OVERRUN         (1<<DOR)
#define DATA_REGISTER_EMPTY  (1<<UDRE)
#define RX_COMPLETE          (1<<RXC)

// USART Receiver buffer
#define RX_BUFFER_SIZE  (8)


//-----------------------------------------------------------------------------
// External Interrupt 0 service routine
char         mstat;
signed char  mx;
signed char  my;
signed int   acu_x, acu_y;
//char         buf[5];
char         maska,i;

char         rx_buffer[RX_BUFFER_SIZE];

// This flag is set on USART Receiver buffer overflow
char          rx_buffer_overflow; //type `bit`

#if ( RX_BUFFER_SIZE < 256 )
unsigned char rx_wr_index,rx_rd_index,rx_counter;
#else
unsigned int  rx_wr_index,rx_rd_index,rx_counter;
#endif



//-----------------------------------------------------------------------------
// USART Receiver interrupt service routine
/*interrupt [USART_RXC]*/ void usart_rx_isr(void) {
	char status, data;

	status = UARTRxErrorGet ( UART0_BASE );  //UCSRA;
	data   = UARTCharGet ( UART0_BASE );  //UDR;
	//if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
	if ( UARTBusy(UART0_BASE) == 0 )
	if ( status == 0 ) {
		rx_buffer[rx_wr_index]=data;
		if ( ++rx_wr_index == RX_BUFFER_SIZE ) rx_wr_index=0;
		if ( ++rx_counter  == RX_BUFFER_SIZE ) {
			rx_counter=0;
			rx_buffer_overflow=1;
		};
	};
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
//#pragma used+
char getchar(void) {
	char data;

	while ( rx_counter==0 );
	data=rx_buffer[rx_rd_index];
	if (++rx_rd_index == RX_BUFFER_SIZE)
		rx_rd_index=0;

	//#asm("cli")
	--rx_counter;
	//#asm("sei")

	return data;
}
//#pragma used-
#endif
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void gohi(int pin) {
	switch (pin) {
		case MDATA:
			 KBD_DATA     |=  0x02;
			 KBD_DATA_DIR &= ~0x02;
		break;
		case MCLK:
			 KBD_CLK      |=  0x02;
			 KBD_CLK_DIR  &= ~0x02;
		break;
	};
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void golo(int pin) {
	switch (pin) {
		case MDATA:
			KBD_DATA     &= ~0x02;
			KBD_DATA_DIR |=  0x02;
		break;
		case MCLK:
			KBD_CLK      &= ~0x02;
			KBD_CLK_DIR  |=  0x02;
		break;
	};
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void mouse_write(char data) {
	char  i;
	char  parity = 1;

	/* put pins in output mode */
	gohi(MDATA);
	gohi(MCLK);
	//delay_us(300);
	Delay(300);

	golo(MCLK);
	//delay_us(300);
	Delay(300);

	golo(MDATA);
	//delay_us(10);
	Delay(100);

	/* start bit */
	gohi(MCLK);

	/* wait for mouse to take control of clock); */
	while (CLK_READ == 1);

	/* clock is low, and we are clear to send data */
	for (i=0; i < 8; i++) {
		if (data & 0x01) {
			gohi(MDATA);
		}
		else {
			golo(MDATA);
		}

		/* wait for clock cycle */
		while (CLK_READ == 0);
		while (CLK_READ == 1);
		parity = parity ^ (data & 0x01);
		data = data >> 1;
	}

	/* parity */
	if (parity) {
		gohi(MDATA);
	} else {
		golo(MDATA);
	}

	while (CLK_READ == 0);
	while (CLK_READ == 1);

	/* stop bit */
	gohi(MDATA);
	//delay_us(50);
	Delay(500);

	while (CLK_READ == 1);

	/* wait for mouse to switch modes */
	while ((CLK_READ ==0) || (DATA_READ == 0));

	/* put a hold on the incoming data. */
	golo(MCLK);
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
/* Get a byte of data from the mouse */
signed char mouse_read(void) {
	signed char data = 0x00;
	int         i;
	char        mask = 0x01;

	/* start the clock */
	gohi(MCLK);
	gohi(MDATA);
	//delay_us(50);
	Delay(500);

	while (CLK_READ ==1);
	//delay_us(5);  /* not sure why */
	Delay(50);

	while(CLK_READ ==0);
	for (i=0; i < 8; i++) {
		while(CLK_READ ==1);
		if (DATA_READ == 1) {
		  data = data | mask;
		}
		while (CLK_READ ==0);
		mask = mask << 1;
	}

	/* eat parity bit, which we ignore */
	while ( CLK_READ == 1 );
	while ( CLK_READ == 0 );

	/* eat stop bit */
	while ( CLK_READ == 1 );
	while ( CLK_READ == 0 );

	/* put a hold on the incoming data. */
	golo(MCLK);

	return data;
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void mouse_init() {
	gohi(MCLK);
	gohi(MDATA);
	mouse_write(0xff);
	mouse_read();  /* ack byte */
	mouse_read();  /* blank */
	mouse_read();  /* blank */
	mouse_write(0xf0);  /* remote mode */
	mouse_read();  /* ack */
	//delay_us(100);
	Delay(1000);
}
//-----------------------------------------------------------------------------


//---------------------------------------------------------------------------
void loop1(void) {
	/* get a reading from the mouse */
	mouse_write(0xeb);  /* give me data! */
	mouse_read();      /* ignore ack */
	mstat = mouse_read();
	mx = mouse_read();
	my = mouse_read();
	acu_x+=mx;
	acu_y+=my;
	//delay_ms(20);  /* twiddle */
	Delay(2000);
}
//-----------------------------------------------------------------------------


//---------------------------------------------------------------------------
void mouse_main(void) {
/*
	// Declare your local variables here
	// Input/Output Ports initialization
	// Port A initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTA=0x00;
	DDRA=0x00;

	// Port B initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTB=0x00;
	DDRB=0x00;

	// Port C initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTC=0x00;
	DDRC=0x00;

	// Port D initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTD=0x00;
	DDRD=0x00;

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=FFh
	// OC0 output: Disconnected
	TCCR0=0x00;
	TCNT0=0x00;
	OCR0=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=FFFFh
	// OC1A output: Discon.
	// OC1B output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=0x00;
	TCCR1B=0x00;
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=FFh
	// OC2 output: Disconnected
	ASSR=0x00;
	TCCR2=0x00;
	TCNT2=0x00;
	OCR2=0x00;

	// External Interrupt(s) initialization
	// INT0: Off
	// INT0 Mode: Off
	// INT1: Off
	// INT2: Off
	MCUCR=0x00;
	MCUCSR=0x00;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK=0x00;

	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART Mode: Asynchronous
	// USART Baud Rate: 1200
	UCSRA=0x00;
	UCSRB=0x98;
	UCSRC=0x86;
	UBRRH=0x01;
	UBRRL=0xA0;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// Analog Comparator Input Capture by Timer/Counter 1: Off
	ACSR=0x80;
	SFIOR=0x00;

	// Global enable interrupts
	#asm("sei")
*/

	static char mouse_stage = 0;

	if (mouse_stage==0) {
		mouse_stage++;
		mouse_init();
	}

	if (mouse_stage==1)
	//while (1)
	{
		loop1();

		/*
		puts(" mstat reg: ");
		maska=0b10000000;
		for (i=0;i<8;i++) { //we will write one by one bit of this register
			if ((maska&mstat))
				putchar('1');
			else
				putchar('0');
			maska>>=1;//shift mask for one position  to right to get one by one bit for uart write of individual bits in mstat register
		}

		puts(" x reg: ");
		itoa(mx, buf);
		puts(buf);
		puts(" x position: ");
		itoa(acu_x, buf);
		puts(buf);
		puts(" y reg: ");
		itoa(my, buf);
		puts(buf);
		puts(" y position: ");
		itoa(acu_y, buf);
		puts(buf);

		delay_ms(200);
		putchar(13);
		*/
	};
}
//-----------------------------------------------------------------------------

#endif //#ifdef (USE_PS2_MOUSE==1)
