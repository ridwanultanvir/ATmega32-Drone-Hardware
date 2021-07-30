/*
1705016: changed for project 
			
Max Voltage (X Volt)		5	
Analogue Channel			ADC0
ADLAR Adjustment			Right
ADC Clock Division Factor	16

*/ 

#undef  __AVR_ATmega32__
#define __AVR_ATmega32__

// #ifndef F_CPU
//#define F_CPU 1000000UL // 1 MHz clock speed
#define F_CPU 4000000UL // 1 MHz clock speed
// #endif
#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTC6
#define EN eS_PORTC7

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "lcd.h"
#include "FTDI_USART.h"

// not in-place
#define CHK_BIT(x, n) (x & (1 << n))
#define SET_BIT(x, n) (x | (1 << n))
#define CLR_BIT(x, n) (x & ~(1 << n))
#define TOG_BIT(x, n) (x ^ (1 << n))
// in place
#define SET_BIT_(x, n) (x |= (1 << n))
#define CLR_BIT_(x, n) (x &= ~(1 << n))
#define TOG_BIT_(x, n) (x ^= (1 << n))

//**** FOR ADC button : kon channel select: @Tanvir */

void ADC_Init()
{
	DDRA = 0x00;		/* Make ADC port as input */
	ADCSRA = 0x87;		/* Enable ADC, fr/128  */
	ADMUX = 0x40;		/* Vref: Avcc, ADC channel: 0 */
}

int ADC_Read(char channel)
{
	int ADC_value;
	
	ADMUX = (0x40) | (channel & 0x07);/* set input channel to read */
	ADCSRA |= (1<<ADSC);	/* start conversion */
	while((ADCSRA &(1<<ADIF))== 0);	/* monitor end of conversion interrupt flag */
	
	ADCSRA |= (1<<ADIF);	/* clear interrupt flag */
	ADC_value = (int)ADCL;	/* read lower byte */
	ADC_value = ADC_value + (int)ADCH*256;/* read higher 2 bits, Multiply with weightage */

	return ADC_value;		/* return digital value */
}

//void UART_init() {
	//UCSRA = 0;
	//UCSRB = 0b00001000; //TXEN = 1 korsi 
	//UCSRC = 0b10000110;
	//
	//UBRRH = 0x00;
	//UBRRL = 0xcf;
//}
void UART_send(uint16_t data ) {
	////&& performs "short-circuit" evaluation
	while((UCSRA & (1<<UDRE)) == 0);
	UDR = data;
}
//
//volatile bool uart_send_ok = false;
volatile uint8_t value_8bit0;
volatile uint8_t value_8bit1 ;
//
//ISR(INT2_vect) {
	//int modeOperation = 0x72; //'r'
	//UART_send(modeOperation);
	//UART_send(value_8bit0);
	//_delay_ms(50);
	//modeOperation = 0x70; //'p'
	//UART_send(modeOperation);
	//UART_send(value_8bit1);
	//
//}

void config_lcd_io_pins() {
	// Use PORTC pins for I/O directly
	MCUCSR = (1 << JTD);
	MCUCSR = (1 << JTD);

	SET_BIT_(DDRC, 6);
	SET_BIT_(DDRC, 7);

	SET_BIT_(DDRD, 4);
	SET_BIT_(DDRD, 5);
	SET_BIT_(DDRD, 6);
	SET_BIT_(DDRD, 7);
}

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

uint16_t valudeadc[5];
int joystick_offset[4] = {0};

void calibrate_joystick() {
	int times = 100;
	for (int j = 0; j < times; ++j) {
		for (int i = 0; i < 4; ++i) {
			joystick_offset[i] += ADC_Read(i) >> 2;
			//printf("%s%d", labels[i], valudeadc[i]);
		}
	}
	
	for (int i = 0; i < 4; ++i) {
		joystick_offset[i] /= times;	
		printf("%d\r\n", joystick_offset[i]);
	}
	
	for (int i = 0; i < 3; ++i) {
		joystick_offset[i] -= 255 / 2;
	}
	// joystick_offset[3] -= 127/2; // Throttle
	//joystick_offset[0] -= 2;
	// joystick_offset[2] -= 2;
	
}

volatile int hard_stop = 0;

ISR (INT2_vect) {
	// hard_stop = 1;
	// valudeadc[4] = '1';
	TOG_BIT_(valudeadc[4], 0);
	_delay_ms(1000);
}

void stop_switch_init() {
	// SET_BIT_(PORTB, 2);
	CLR_BIT_(DDRB, 2);
	GICR = (1 << INT2);
	// SET_BIT_(MCUCSR, ISC2); // Rising edge, so 1
	CLR_BIT_(MCUCSR, ISC2); // Falling edge, so 0
	sei();		
}

void setup() {
	config_lcd_io_pins();
	Lcd4_Init();
	
	
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,1);
	Lcd4_Write_String("Welcome!");
	_delay_ms(1000);
	
	ADC_Init(); 
	// DDRB = 0x00; //B = input pin
	//PB2 = INT2
	
	//GICR = 1<<INT2;
	//MCUCSR = 1<<ISC2; ///INT2 er jonno
	//sei();
	//for INT2 
	
	FTDI_init();
	FTDI_redirect_io();
	
	Lcd4_Clear();	
	Lcd4_Set_Cursor(1,1);
	Lcd4_Write_String("Calibrating JS");
	//calibrate_joystick();
	Lcd4_Write_String("Calibration done");
	// while(1);
	
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,1);
	Lcd4_Write_String("R 000, P 000");
	Lcd4_Set_Cursor(2,1);
	Lcd4_Write_String("Y 000, T 000");
	
	
	stop_switch_init();	
	valudeadc[4] = '0';	
}



int main(void)
{	
	setup();
	
	
	while(1)
	{
		//printf("\r\n----------------\r\n");
		
		char labels[5][4] = {"ROL", "PIT", "YAW", "THR", "STP"};
		//char labels_debug[5][10] = {"roll", "pitch", "yaw", "throttle"};
		// printf("Dummy");
		for (int i = 0; i < 4; ++i) {
			valudeadc[i] = ADC_Read(i) >> 2;
			//printf("%s%d", labels[i], valudeadc[i]);
			valudeadc[i] -= joystick_offset[i];
		}
		
		for (int j = 0; j < 3; ++j) {
			for (int i = 0; i < 5; ++i) {
				printf("%s", labels[i]);
				UART_send(valudeadc[i]);
				
				// printf("%s %d ", labels_debug[i], valudeadc[i]);
				// _delay_ms(3);
			}			
		}
		
		if (valudeadc[4] == '1') {
			valudeadc[4] = '0';
		}
		
		char buffer[5];		
		int percent_value;
		Lcd4_Set_Cursor(1,3);
		percent_value = MIN((int)((valudeadc[0])/255.0*100), 100);
		percent_value -= 50;
		itoa(percent_value, buffer, 10);
		for (int i = 0; i < 3 - strlen(buffer); ++i) {
			Lcd4_Write_String(" ");
		}
		Lcd4_Write_String(buffer);	
		
		Lcd4_Set_Cursor(1,10);
		percent_value = MIN((int)((valudeadc[1])/255.0*100), 100);
		percent_value -= 50;
		itoa(percent_value, buffer, 10);
		for (int i = 0; i < 3 - strlen(buffer); ++i) {
			Lcd4_Write_String(" ");
		}
		Lcd4_Write_String(buffer);
		
		Lcd4_Set_Cursor(2,3);
		percent_value = MIN((int)((valudeadc[2])/255.0*100), 100);
		percent_value -= 50;
		itoa(percent_value, buffer, 10);
		for (int i = 0; i < 3 - strlen(buffer); ++i) {
			Lcd4_Write_String(" ");
		}
		Lcd4_Write_String(buffer);
		
		Lcd4_Set_Cursor(2,10);
		percent_value = MIN((int)((valudeadc[3])/255.0*100), 100);
		itoa(percent_value, buffer, 10);
		for (int i = 0; i < 3 - strlen(buffer); ++i) {
			Lcd4_Write_String(" ");
		}
		Lcd4_Write_String(buffer);
			
		
		
		
		_delay_ms(10);
		//printf("\r\n");
		
		
		
		/***	VOLT print 
		char resultStr[16] = {0};
		Lcd4_Set_Cursor(1,1);
		
		

		float valueadc_float = (valudeadc0*5.0)/1024.0;
		
		sprintf(resultStr, "ADC0: %.3f", valueadc_float);
		Lcd4_Write_String(resultStr);
		
		//_delay_ms(2000);
		Lcd4_Set_Cursor(2,1);
		
		valueadc_float = (valudeadc1*5.0)/1024.0;
		
		sprintf(resultStr, "ADC1: %.3f", valueadc_float);
		Lcd4_Write_String(resultStr);
		*/ 
		
		
		
		
		//*** send to RX */ 
		
		// valudeadc[0] = valudeadc[0] >>2; 
		// valudeadc1 = valudeadc1 >>2; 
		//10 bit data er 8 bit (MSB side theke) Nibo 
		// value_8bit0 = valudeadc0; 
		// value_8bit1 = valudeadc1; 
		
		//char resultStr[16] = {0};
		// Lcd4_Set_Cursor(1,1);
		
		
		//sprintf(resultStr, "ADC0: %d", value_8bit0);
		// Lcd4_Write_String(resultStr);
		
		//_delay_ms(2000);
		//Lcd4_Set_Cursor(2,1);

		
		// sprintf(resultStr, "ADC1: %d", value_8bit1);
		// Lcd4_Write_String(resultStr);
		
		//_delay_ms(1000); 
		//INT2 = PINB2 number pin; otate logic probe 
		//if(PINB == 0b00000100)		
		 
	}
}