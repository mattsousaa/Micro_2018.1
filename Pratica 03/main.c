/*
 * Pratica 3.c
 *
 * Created: 14/04/2018 21:10:04
 * Author : Mateus Sousa
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define cpl_bit(y,bit) (y^=(1<<bit))

void adc_init();
uint16_t read_adc(uint8_t channel);
void restart_anywhere();

unsigned char ad_value = 0;
unsigned char ad_value2 = 0;
unsigned char DEC_T = 0x0;       //two digit DS7 State for Temperature sensor
unsigned char UNI_T = 0x0;
unsigned char DEC_L = 0x0;       //two digit DS7 State for Lummen sensor
unsigned char UNI_L = 0x0;
unsigned int timer = 0;
unsigned int timer2 = 0;
unsigned char DISP[2];
unsigned int flip_sensor = 0; 
unsigned char x = 0;
unsigned char w = 0;
unsigned char y = 0;
unsigned char z = 0;
unsigned int counter = 0;
unsigned int counter2 = 0;
unsigned int delay = 0;
unsigned int delay2 = 0;

char DS7[] = {          //pgfedcba
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
						0b01100011, //A 'º'
						0b00111001, //B 'C'
						0b00111000, //C 'L'
						0b00011100  //D 'u'
};


ISR(TIMER1_OVF_vect){
	if(flip_sensor == 0){
		//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
		ADMUX &= 0xF0;			//Clear the older channel that was read
		ADMUX = 0x00;		//Defines the new ADC channel to be read
		//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
		ADCSRA |= (1<<ADSC);
	
		while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
	} else{
		//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
		ADMUX &= 0xF0;			//Clear the older channel that was read
		ADMUX = 0x01;		//Defines the new ADC channel to be read
		//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
		ADCSRA |= (1<<ADSC);
		
		while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
	}
}

ISR(TIMER0_COMPA_vect){
	
	if(flip_sensor == 0){
		
		counter++;

		ad_value = ADC * 50/1023;
		
		DEC_T = ad_value/10;
		
		UNI_T = ad_value%10;
		
		if(counter < 0xBB8){
			if(timer == 0){
				PORTC &= ~(1<<PORTC3);
				PORTC |= 0x4;
				PORTD = DISP[x];
				DISP[x] = DS7[DEC_T];
				x++;
				timer++;
			} else{
				PORTC &= ~(1<<PORTC2);
				PORTC |= 0x8;
				PORTD = DISP[x];
				DISP[x] = DS7[UNI_T];
				x++;
				if(x==2) x = 0;
				timer = 0;
			}
		} else{
		
		delay++;
			if(delay < 0xBB8){
				counter2++;
			
				PORTC &= ~(1 << PORTC3);
				PORTC &= ~(1 << PORTC2);
			
				if(counter2 < 0xBB8){
					if(timer2 == 0){
						PORTC &= ~(1<<PORTC3);
						PORTC |= 0x4;
						PORTD = DISP[y];
						DISP[y] = DS7[10];
						y++;
						timer2++;
					} else{
						PORTC &= ~(1<<PORTC2);
						PORTC |= 0x8;
						PORTD = DISP[y];
						DISP[y] = DS7[11];
						y++;
						if(y==2) y = 0;
						timer2 = 0;
					}
				} else{
					delay2++;
					if(delay2 < 0xBB8){
						PORTC &= ~(1 << PORTC3);
						PORTC &= ~(1 << PORTC2);
					}
				}
			} else{
				PORTC &= ~(1 << PORTC3);
				PORTC &= ~(1 << PORTC2);
				restart_anywhere();
				flip_sensor ^= 1;
			}
		}
	} else{
		
		counter++;

		ad_value2 = ADC * 80/1023;
		
		DEC_T = ad_value2/10;
		
		UNI_T = ad_value2%10;
		
		if(counter < 0xBB8){
			if(timer == 0){
				PORTC &= ~(1<<PORTC3);
				PORTC |= 0x4;
				PORTD = DISP[x];
				DISP[x] = DS7[DEC_T];
				x++;
				timer++;
				} else{
				PORTC &= ~(1<<PORTC2);
				PORTC |= 0x8;
				PORTD = DISP[x];
				DISP[x] = DS7[UNI_T];
				x++;
				if(x==2) x = 0;
				timer = 0;
			}
		} else{
		
		delay++;
		if(delay < 0xBB8){
			counter2++;
			
			PORTC &= ~(1 << PORTC3);
			PORTC &= ~(1 << PORTC2);
			
			if(counter2 < 0xBB8){
				if(timer2 == 0){
					PORTC &= ~(1<<PORTC3);
					PORTC |= 0x4;
					PORTD = DISP[y];
					DISP[y] = DS7[12];
					y++;
					timer2++;
					} else{
					PORTC &= ~(1<<PORTC2);
					PORTC |= 0x8;
					PORTD = DISP[y];
					DISP[y] = DS7[13];
					y++;
					if(y==2) y = 0;
					timer2 = 0;
				}
				} else{
				delay2++;
				if(delay2 < 0xBB8){
					PORTC &= ~(1 << PORTC3);
					PORTC &= ~(1 << PORTC2);
				}
			}
			} else{
			PORTC &= ~(1 << PORTC3);
			PORTC &= ~(1 << PORTC2);
			restart_anywhere();
			flip_sensor ^= 1;
		}
	}
	}
}


void setup_uc(void){

	//SET all PORTC as output, except C0 which is ANALOG 0
	DDRC  = 0b11111110;
	PORTC = 0x00;
	
	//SET all PORTD as output, except C7
	DDRD =  0b01111111;		//0x7F
	PORTD = 0b00000000;		//0x00
	
	/************TIMER 0 CONFIG************/
	//Timer/Counter Control Register A | COM0A1 COM0A0 COM0B1 COM0B0 -- WGM01 WGM00
	TCCR0A = 0x02;	//OC0A and OC0B disconnected; CTC MODE with OCR0A as TOP
	
	//Timer/Counter Control Register B | FOC0A FOC0B -- WGM02 CS02 CS01 CS00
	TCCR0B = 0x01;	//no force OC0A,B pins; prescale = 1
	
	//Timer/Counter Register
	TCNT0  = 0x00;
	//Output Compare Register A - in CTC mode with Prescale = 1 Fint = 2kHz
	OCR0A  = 0x1;
	//Output Compare Register B
	OCR0B  = 0x00;
	//Timer/Counter Interrupt Mask Register | ----- OCIE0B OCIE0A TOIE0
	TIMSK0 = 0x02;	//Enable OCIE0A interrupt
	
	//Digital Input Disable Register 0 | -- ADC5D ADC4D ADC3D ADC2D ADC1D ADC0D
	DIDR0 = 0x07; //Disable ADC0,1,2
	
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

}

int main(void){
    /* Replace with your application code */
	
	adc_init();
	setup_uc();
	sei();
	
    while (1){
		
    }
}

void adc_init(){
	//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA = 0b11000110;	//Turn on ADC | ADC Start Conversion | PSCloc = 64 (slower clock --> best precision)
	//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
	ADMUX = 0b01000000;	
	//ADC Control and Status Register B  | - ACME --- ADTS2,1,0
	ADCSRB = 0x06; //comparator is not multiplexed | AD Start Conversion is started by TMR1 Overflow	
}

uint16_t read_adc(uint8_t channel){
	//ADC Multiplexer Selection Register | REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
	ADMUX &= 0xF0;			//Clear the older channel that was read
	ADMUX = channel;		//Defines the new ADC channel to be read
	//ADC Control and Status Register A  | ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA = 0b01000000;	//Starts a new conversion
	
	while(ADCSRA & (1<<ADSC));	//Wait until the conversion is done
	
	return ADC;

}

void restart_anywhere(){
	DEC_T = 0x0;       //two digit DS7 State for Temperature sensor
	UNI_T = 0x0;
	DEC_L = 0x0;       //two digit DS7 State for Lummen sensor
	UNI_L = 0x0;
	timer = 0;
	timer2 = 0;
	x = 0;
	w = 0;
	y = 0;
	z = 0;
	counter = 0;
	counter2 = 0;
	delay = 0;
	delay2 = 0;
}