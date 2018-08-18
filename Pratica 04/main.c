#include "def_principais.h"
#include "USART.h"

uint16_t read_adc(uint8_t channel);
void ADC_init();

unsigned int flip_sensor = 0; 
unsigned char ad_value = 0;
unsigned char ad_value2 = 0;
unsigned char duty_1 = 0;
unsigned char duty_2 = 0;
unsigned char duty_3 = 0;
unsigned char duty_4 = 0;

unsigned char DEC_T = 0x0;       //two digit DS7 State for Temperature sensor
unsigned char UNI_T = 0x0;
unsigned char DEC_L = 0x0;       //two digit DS7 State for Lummen sensor
unsigned char UNI_L = 0x0;
char String[] = "Hello world!!";
unsigned char teste = 0;

ISR(TIMER2_COMPA_vect){
	if(duty_1 == 1){
		OCR0A = 70;
		duty_1 = 0;
	} else if(duty_2 == 2){
		OCR0A = 190;
		duty_2 = 0;
	} else if(duty_3 == 3){
		OCR0A = 250;
		duty_3 = 0;
	} else if(duty_4 == 4){
		OCR0A = 0;
		duty_4 = 0;
	}
}

ISR(TIMER1_OVF_vect){
	
	if(flip_sensor == 1){
		//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
		ADMUX &= 0xF0;			//Clear the older channel that was read
		ADMUX = 0b01000000;		//Defines the new ADC channel to be read
		//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
		ADCSRA |= (1<<ADSC);
		
		while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
		
		ad_value = ADC * 50/1023;
		
		UNI_T = ad_value/10;
		DEC_T = ad_value%10;
		
		USART_send(UNI_T+'0');
		USART_send(DEC_T+'0');
		
		USART_putstring("ºC\n\r");
		
		flip_sensor = 0;
		
		} else if(flip_sensor == 2){
		//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
		ADMUX &= 0xF0;			//Clear the older channel that was read
		ADMUX = 0b01000001;		//Defines the new ADC channel to be read
		//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
		ADCSRA |= (1<<ADSC);
		
		while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
		
		ad_value2 = ADC * 60/1023;
		
		UNI_L = ad_value2/10;
		DEC_L = ad_value2%10;
		
		USART_send(UNI_L+'0');
		USART_send(DEC_L+'0');
		
		USART_putstring("Lu\n\r");
		
		flip_sensor = 0;
	}
}

ISR(USART_RX_vect){
	
	char ReceivedByte;
	ReceivedByte = USART_Receive();
	//UDR0 = ReceivedByte; // Echo back the received byte back to the computer
	//USART_send('\r');
	
	switch(ReceivedByte){
		case 'A':
			cpl_bit(PORTB, 1);
		break;
		case 'B':
			cpl_bit(PORTB, 2);
		break;
		case 'C':
			cpl_bit(PORTB, 3);
		break;
		case 'T':
			flip_sensor = 1;
		break;
		case 'L':
			flip_sensor = 2;
		break;
		case '1':
			duty_1 = 1;
		break;
		case '2':
			duty_2 = 2;
		break;
		case '3':
			duty_3 = 3;
		break;
		case '0':
			duty_4 = 4;
		break;
		
		
	}
	
	/*
	if(ReceivedByte == 'A'){
		cpl_bit(PORTC, 5);
	}*/
	
	
	//USART_send(ReceivedByte+1);
}

void setup_uc(void){
	
	/************************************************************************/
	/*IO PORT CONFIG                                                        */
	/************************************************************************/
	//0 = input | 1 = output
	//if input = pull-up else port value
	
	//SET all PORTB as output
	DDRB  = 0xFF;
	PORTB = 0x00;
	
	//SET all PORTC as output, except C0 which is ANALOG 0
	DDRC  = 0b11111100;
	PORTC = 0x00;
	
	//SET all PORTD as output, except D2 used as INT0, and D0 used as UART RX;
	DDRD = 0b11111010;				//Set our PWM pin as an output (PD6)
	//Enable D2 pull up
	PORTD = 0x04;

	//Config INT0 and INT1 active on falling edge
	EICRA = 0b00001010;
	//INT0 and INT1 Interrupt MASK
	EIMSK = 0x01;
	
	//Enable Pin Change Interrupt MASK
	PCICR = 0b00000000;
	
	/*Pin Change Interrupt individual pin MASK*/
	/*bits 23 to 16*/
	PCMSK2 = 0x00;
	/*bits 14 to 08*//*Doesnt exist PCINT15*/
	PCMSK1 = 0x00;
	/*bits 07 to 00*/
	PCMSK0 = 0x00;
	
	/************TIMER 0 CONFIG************/
	//Timer/Counter Control Register A | COM0A1 COM0A0 COM0B1 COM0B0 -- WGM01 WGM00
	//Fast PWM TOP = 0xFF
	
	//Timer/Counter Control Register B | FOC0A FOC0B -- WGM02 CS02 CS01 CS00
	//Timer configuration
	TCCR0A = ((1<<COM0A1)|(1<<WGM01)|(1<<WGM00));    //Enable pwm mode in pin PD6 and set the WGM bits to Fast pwm mode
	TCCR0B = ((1<<CS01)|(1<<CS00));                  //Set prescaler to 32
	
	//Timer/Counter Register
	TCNT0  = 0x00;
	//Output Compare Register A
	OCR0A  = 1;
	//Output Compare Register B
	OCR0B  = 0;
	//Timer/Counter Interrupt Mask Register | ----- OCIE0B OCIE0A TOIE0
	TIMSK0 = 0x00;
	
	/************TIMER 1 CONFIG - ADC Trigger ************/
	/* 1 second base time */
	//Timer/Counter1 Control Register A | COM1A1 COM1A0 COM1B1 COM1B0 -- WGM11 WGM10
	TCCR1A = 0x00;						//Normal port operation, OC1A/OC1B disconnected | CTC OCR1A is TOP of counting
	
	//Timer/Counter1 Control Register B | ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	TCCR1B = 0b00000001;				//Clock is divided by 1 | Normal Mode (overflow)
	
	//Timer/Counter1 Control Register C | FOC1A FOC1B ------
	TCCR1C = 0x00;
	
	//Timer/Counter1
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	//Output Compare Register 1 A | interrupt frequency is 1Hz @ 16MHz
	OCR1AH = 0x01;
	OCR1AL = 0x00;
	//Output Compare Register 1 B
	OCR1BH = 0x00;
	OCR1BL = 0x00;
	//Input Capture Register 1
	ICR1H = 0x00;
	ICR1L = 0x00;
	//Timer/Counter1 Interrupt Mask Register | -- ICIE1 -- OCIE1B OCIE1A TOIE1
	TIMSK1 = 0x01;
	
	OCR2A = 0xFF;
	TCCR2B = 0b00000001;
	TIMSK2 = 0x02;
	
	
}

int main(void){
	
	setup_uc();
	
	ADC_init();
	
	USART_init();
	
	//Turn on all interrupts
	sei();
	
	//USART_putstring("Hello World!");
	
	while(1){

	}

}


//------------------------------------------------------------------------------------
void USART_init(void){
	
	/************************************************************************/
	/*UART			                                                        */
	/************************************************************************/
	
	/*Set baud rate */
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
//------------------------------------------------------------------------------------
void USART_send(unsigned char data){
	/* Wait for empty transmit buffer */
	while(!(UCSR0A & (1<<UDRE0)));
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

//------------------------------------------------------------------------------------
unsigned char USART_Receive(void){
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}
//------------------------------------------------------------------------------------
void USART_putstring(char* StringPtr){
	// sends the characters from the string one at a time to the USART
	while(*StringPtr != 0x00)
	{
		USART_send(*StringPtr);
		StringPtr++;
	}
}
//------------------------------------------------------------------------------------
void ADC_init(){
	//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA = 0b11000110;	//Turn on ADC | ADC Start Conversion | PSCloc = 64 (slower clock --> best precision)
	//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
	ADMUX = 0b00000000; //AREF
	//ADC Control and Status Register B  | - ACME --- ADTS2,1,0
	ADCSRB = 0x06; //comparator is not multiplexed | AD Start Conversion is started by TMR1 Overflow
}
//------------------------------------------------------------------------------------
uint16_t read_adc(uint8_t channel){
	//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
	ADMUX &= 0xF0;			//Clear the older channel that was read
	ADMUX = channel;		//Defines the new ADC channel to be read
	//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA = 0b01000000;	//Starts a new conversion
	
	while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
	
	return ADC;
	
}

