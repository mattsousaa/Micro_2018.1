/*
 * GccApplication1.c
 *
 * Created: 10/03/2018 21:14:15
 * Author : Mateus Sousa
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#define cpl_bit(y,bit) (y^=(1<<bit))
#define LED1 PB5
#define LED2 PB1

char DS7[] = {  //-gfedcba
		0b00111111, //0
		0b00000110, //1
		0b01011011, //2
		0b01001111, //3
		0b01100110, //4
		0b01101101, //5
		0b01111101, //6
		0b00000111, //7
		0b01111111, //8
		0b01101111, //9
};

/* When a capture occurs, the counter value is copied into the ICR1 register. */

ISR(TIMER1_CAPT_vect){
	
	unsigned int store;			/* Variable to save the timer value when the button is pressed. */
	unsigned int rand_num;		/* Variable that stores a random number from store mod 10. */

	store = ICR1;				
	
	rand_num = store % 10;
	
	cpl_bit(PORTB, LED2);		/* Switches LED2 (PB1) each time the edge capture button is pressed. */
	
	switch (rand_num){
	case 0:							/*Zero in Display*/
		PORTC = DS7[0];
		break;
	case 1:							/*One in Display*/
		PORTC = DS7[1];
		PORTB &= ~(1 << PORTB3);
		break;			
	case 2:							/*Two in Display*/
		PORTC = DS7[2];
		PORTB |= (1 << PORTB3);
		break;
	case 3:							/*Three in Display*/
		PORTC = DS7[3];
		PORTB |= (1 << PORTB3);
		break;
	case 4:							/*Four in Display*/
		PORTC = DS7[4];
		PORTB |= (1 << PORTB3);
		break;
	case 5:							/*Five in Display*/
		PORTC = DS7[5];
		PORTB |= (1 << PORTB3);
		break;
	case 6:							/*Six in Display*/
		PORTC = DS7[6];
		PORTB |= (1 << PORTB3);
		break;
	case 7:							/*Seven in Display*/
		PORTC = DS7[7];
		PORTB &= ~(1 << PORTB3);
		break;
	case 8:							/*Eight in Display*/
		PORTC = DS7[8];
		PORTB |= (1 << PORTB3);
		break;
	case 9:							/*Nine in Display*/
		PORTC = DS7[9];
		PORTB |= (1 << PORTB3);
		break;
	}
}

/* At each count register overflow, LED1 (PB5) is alternated by value. */

ISR(TIMER1_OVF_vect){
	cpl_bit(PORTB, LED1); 
}

int main(void)
{
	DDRB =  0b00100010;					/* Only the LED pin PB5 and PB1 as output and the button on the PB0 as input. */
	PORTB = 0b11011101;					/* Turn off LEDS and enables pull-ups on the other pins. */
	
	/*TCCR1B => Bit 6 – ICES1: Input Capture Edge Select.
	
	This bit selects which edge on the Input Capture pin (ICP1) that is used to trigger a capture event. When
	the ICES1 bit is written to zero, a falling (negative) edge is used as trigger, and when the ICES1 bit is
	written to '1', a rising (positive) edge will trigger the capture.
	
	When a capture is triggered according to the ICES1 setting, the counter value is copied into the Input
	Capture Register (ICR1).
	
	CS12 = 0 , CS11 = 1 and CS10 = 1 <=> (clock/64) - Prescaler = 64.
	
	TCCR1B = 0b0000 0011 = 0x03;
	
	*/
	
	/* For a crystal of 16MHz we have the overflow time:
	
	T_OVF = (65535 - TCNT1) * Prescaler * 1/(16*10^6) */
	
	TCNT1 = 0x00;;						/* Start timer 1 with zero. */
	
	TCCR1B = 0x03;						/* TC1 with Prescaler = 64 and capture at the falling edge. */
	
	
	//TIMSK1 = 1 << ICIE1;				/* Enables interrupt by capture. */
	//TIMSK1 = 1 << TOIE1;				/* Enable interruption of TC1. */
	
	/*
	
	TIMSK1 => Bit 5 – ICIE: Input Capture Interrupt Enable.
	
	When this bit is written to '1', and the I-flag in the Status Register is set (interrupts globally enabled), the
	Timer/Counter1 Input Capture interrupt is enabled.
	
	Bit 0 – TOIE: Overflow Interrupt Enable.
	
	When this bit is written to '1', and the I-flag in the Status Register is set (interrupts globally enabled), the
	Timer/Counter 1 Overflow interrupt is enabled.
	
	TIMSK1 = 0b0010 0001 = 0x21;
	
	*/
	TIMSK1 = 0x21;						/* Enables capture interruption on PB1 pin and enables TC1 interrupt  */
	
	/* Basic display initialization. */
	
	DDRC  = 0xFF;			/*SET all PORTC as output */
	DDRB |= (1 << DDB3);	/* Set PB3 as output */
	PORTC = 0b01111111;		/* Initialize display with '0' */
	PORTB &= ~(1 << PORTB3); /* Initialize display with '0' */
	sei();

	while(1){
		
		/*Here goes the code*/
		
		/* Each time the button is pressed, an event on pin ICP1 (PB0) occurs and the value of the timer is stored in register ICR1. */
		
		/* Each time overflow occurs in the count register (TCNT1), the counter starts counting again from 0 to 65535. */
	}

}

