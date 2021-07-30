
#ifndef FTDI_USART_H_					/* Define library H file if not defined */
#define FTDI_USART_H_

#ifndef F_CPU
#define F_CPU 8000000UL							/* Define CPU clock Frequency e.g. here its 8MHz */
#endif

#define BAUDRATE 2400

#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)	/* Define prescale value */

#include <stdio.h>

void FTDI_init()
{
	//UCSRA = 0b00000010;
	UCSRA = 0b00000000; // single speed mode
	UCSRB = 0b00011000;
	UCSRC = 0b10000110;
	
	UBRRH = BAUD_PRESCALE >> 8;
	UBRRL = BAUD_PRESCALE; // 2400 bps
}


// https://www.avrfreaks.net/forum/passing-argument1-fdevopen-incompatible-pointer
int FTDI_send(char data, FILE *unused){
	while ((UCSRA & (1<<UDRE)) == 0x00);
	UDR = data;
	return 0;
}

int FTDI_receive(FILE *unused){
	while ((UCSRA & (1<<RXC)) == 0x00);
	return UDR;
}

void FTDI_redirect_io() {
	stdout = fdevopen(FTDI_send, NULL);
	stdin = fdevopen(NULL, FTDI_receive);
}


#endif /* FTDI_USART_H_ */