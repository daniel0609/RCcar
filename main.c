/*
 * main.c
 *
 *  Created on: 7 lut 2017
 *      Author: Daniel
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t timer = 0;
volatile uint8_t interrupt =0;

void IO_init();
void pwm_init();
void interrupt_init();


ISR(INT0_vect)
{
	interrupt = 1;
}

int main()
{
	IO_init();
	pwm_init();
	interrupt_init();
//	usart_init(9600);
	while(1)
	{
		if(interrupt == 1)
		{
			if(PIND & 0x10)
			{
				TCNT0 = 0x00;
				TCCR0 |= (1<<CS01);
			}
			else
			{
				TCCR0 &= ~(1<<CS01);
				OCR1A = TCNT0 * 8;
			}
			interrupt = 0;
		}
	}
}
void IO_init()
{
	DDRD |= (1<<PIN7) | (1<<PIN6);		// LEDs
	DDRD &= ~(1<<PIN2) & ~(1<<PIN3);	// Interupts
	DDRD &= ~(1<<PIN4);					// Checking state of interrupt
	DDRB |= (1<<PIN1) | (1<<PIN2);		// PWM
}
void pwm_init()
{

	TCCR1A |= (1<<COM1A1) //Zmiana stanu wyj�cia OC1A na niski przy por�wnaniu A
											|  (1<<COM1B1) //Zmiana stanu wyj�cia OC1B na niski przy por�wnaniu B
											|  (1<<WGM11); //Tryb 14 (FAST PWM, TOP=ICR1)

	TCCR1B |= (1<<WGM13) | (1<<WGM12)  //Tryb 14 (FAST PWM, TOP=ICR1)
		    										   |  (1<<CS10);

	ICR1 = 2508;	//a wi�c cz�stotliwo�� = CLK/ICR1 = 50Hz

	OCR1A=1003;		//Warto�� pocz�tkowa por�wnania A (Wyj�cie OC1A - PB1)

	OCR1B=1775;		//Warto�� pocz�tkowa por�wnania B (Wyj�cie OC1B - PB2)

}
void interrupt_init()
{
	GICR |= (1<<INT0); 		// w��czenie obs�ugi przerwa� INT0
	MCUCR |= (1<<ISC00);	// any logical change generates an interrupt request

	GICR |= (1<<INT0);		// External Interrupt Request 1 Enable

	sei();
}
