/*
 * pratica2.c
 *
 * Created: 17/03/2018 12:51:38
 * Author : Mateus Sousa
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#define cpl_bit(y,bit) (y^=(1<<bit))

#define LED1 PB5
#define LED2 PB4
#define LED3 PB3
#define LED4 PB2
#define LED5 PB1
#define LED6 PC2

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
				
		0b01110011, //P 0xA
		0b01110111, //A 0xB
		0b01010000, //r 0xC
		0b01111001  //E 0xD
};

unsigned int blink = 0;
unsigned int timer = 0;
unsigned char DISP[2];
static unsigned char x = 0;
static unsigned char w = 0;
unsigned int y = 0;
unsigned int max_count = 0;
unsigned int count = 0;
unsigned int volta = 0;
unsigned int lock = 0;
unsigned int count2 = 0;
unsigned int delay = 0;
unsigned int digito = 10;
unsigned int vezes = 0;
unsigned int estado = 0;
unsigned int sem1 = 0;
unsigned int sem2 = 0;
unsigned int seq1 = 0;
unsigned int seq2 = 0;
short int counter = 0;	

void reset_sequence(){
	sem1 = 0;
	sem2 = 0;
	seq1 = 0;
	seq2 = 0;
	lock = 0;
	TCNT1 = 0x0;
	counter = 0;
	w = 0;
	timer = 0;
	y = 0;
	volta = 0;
	x = 0;
	blink = 0;
	PORTC &= ~(1 << PORTC4);
	PORTC |= (1 << PORTC5);
}


ISR(TIMER0_COMPA_vect){
	
	count++;
	
	if((PINB & (1 << PINB2)) && (PINB & (1 << PINB5))){	
		
		PORTC &= ~(1 << PORTC5);								
		PORTC |= (1 << PORTC4);
		
		lock = 1;
		
		count2++;			
															
		if(sem1 == 1){												
			seq1 = 1;
			} else if(sem2 == 1){										
			seq2 = 1;
		}
													
		sem1 = 5;													
		sem2 = 5;													
		
			if(timer == 0){
					PORTC &= ~(1<<PORTC1);
					PORTC |= 0x1;
					PORTD = DISP[w];
					DISP[0] = DS7[volta];
					w++;
					timer++;
				} else{
					PORTC &= ~(1<<PORTC0);
					PORTC |= 0x2;
					PORTD = DISP[w];
					DISP[w] = DS7[y%10];
					w++;
					if(w==2) w = 0;
					timer = 0;
				}
				
	} else{
		if(count < 0xBB8){
			if(blink == 0){
				PORTC &= ~(1<<PORTC1);
				PORTC |= 0x1;
				PORTD = DISP[x];
				DISP[x] = DS7[digito];
				x++;
				blink++;
			} else{
				PORTC &= ~(1<<PORTC0);
				PORTC |= 0x2;
				PORTD = DISP[x];
				DISP[x] = DS7[digito+1];
				x++;
				if(x==2) x = 0;
				blink = 0;
			}
		} else{
			
			delay++;
			if(delay < 0xBB8){
				PORTC &= ~(1 << PORTC0);
				PORTC &= ~(1 << PORTC1);
			} else{
				vezes++;
				delay = 0;
				count = 0;
				if(vezes == 1) {
					digito = 12;
				}
				if(vezes == 2) {
					digito = 10;
					vezes = 0;
				} 				
			}
		}
	}
}

ISR(TIMER1_CAPT_vect){
	
	if(estado == 0 || estado == 1){			
		sem1 = 1;
		} else if(estado == 2 || estado == 3){	
		sem2 = 1;
	}
}

ISR(TIMER1_OVF_vect){	
	
	estado++;
	
	if(sem1 == 0 && sem2 == 0){				
		if(estado == 1){						
			cpl_bit(PORTC, LED6);
			cpl_bit(PORTB, LED5);
			
			} else if(estado == 2){				
				cpl_bit(PORTB, LED5);
				cpl_bit(PORTB, LED1);
				cpl_bit(PORTB, LED3);
				cpl_bit(PORTB, LED4);
				
			} else if(estado == 3){				
				cpl_bit(PORTB, LED3);
				cpl_bit(PORTB, LED2);
				
			}  else if(estado == 4){				
				cpl_bit(PORTB, LED2);
				cpl_bit(PORTB, LED1);
				cpl_bit(PORTC, LED6);
				cpl_bit(PORTB, LED4);
				estado = 0;					
			}
		} else if(sem1 == 1){					
			if((PINC & (1 << PINC2))){
				cpl_bit(PORTC, LED6);
				cpl_bit(PORTB, LED5);
				
			} else if((PINB & (1 << PINB1))){
				cpl_bit(PORTB, LED5);
				cpl_bit(PORTB, LED4);
				
			} else if((PINB & (1 << PINB2))){
				cpl_bit(PORTB, LED1);
				cpl_bit(PORTB, LED3);
				sem1 = 0;						
				estado = 2;						
			}
		} else if(sem2 == 1){					
			if((PINB & (1 << PINB3))){
				cpl_bit(PORTB, LED3);
				cpl_bit(PORTB, LED2);
				
			} else if((PINB & (1 << PINB4))){
				cpl_bit(PORTB, LED2);
				cpl_bit(PORTB, LED1);
				
			} else if((PINB & (1 << PINB5))){
				cpl_bit(PORTB, LED4);
				cpl_bit(PORTC, LED6);
				sem2 = 0;						
				estado = 0;				
		}
	}	
}

ISR(TIMER2_OVF_vect){
	
	if(lock == 1){
		counter++;
		if(counter == 0x05){	
			if(y == 0x9){
				y = 0;
				volta++;
				counter = 0x00;
				
			} else{
				y++;
				counter = 0x00;
				if(volta == 1){
					if(seq1 == 1){				
						reset_sequence();				
						estado = 2;				
						cpl_bit(PORTB, LED1);	
						cpl_bit(PORTB, LED3);  	
					} else if(seq2 == 1){
						reset_sequence();
						estado = 0;			
						cpl_bit(PORTB, LED4);	
						cpl_bit(PORTC, LED6);	
					}
				}
			}
		}
	}
}

void setup_uc(void){
	
	DDRB =  0b00111110;		//0x3E
	PORTB = 0b00100001;		//0x21		
	
	DDRC =  0b00110111;		//0x37			
	PORTC = 0b00100100;		//0x24
	
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
	
	/************TIMER 1 CONFIG************/
	//Timer/Counter Control Register A | COM0A1 COM0A0 COM0B1 COM0B0 -- WGM01 WGM00
	TCCR1A = 0x00;	//OC0A and OC0B disconnected; CTC MODE with OCR0A as TOP
	
	//Timer/Counter Control Register B | FOC0A FOC0B -- WGM02 CS02 CS01 CS00
	TCCR1B = 0x03;	//no force OC0A,B pins; prescale = 1
	
	//Timer/Counter Register
	TCNT1  = 0x00;
	//Output Compare Register A - in CTC mode with Prescale = 1 Fint = 2kHz
	OCR1A  = 0xFF;
	//Output Compare Register B
	OCR1B  = 0x00;
	//Timer/Counter Interrupt Mask Register | ----- OCIE0B OCIE0A TOIE0
	TIMSK1 = 0x21;	//Enable OCIE0A interrupt
	
	/************TIMER 2 CONFIG************/
	//Timer/Counter Control Register B | FOC0A FOC0B -- WGM02 CS02 CS01 CS00
	TCCR2B = 0x7;	//Prescale = 1024
	//Timer/Counter Register	
	TCNT2 = 0x00;
	//Timer/Counter Interrupt Mask Register | ----- OCIE0B OCIE0A TOIE0
	TIMSK2 = 0x01;	//Enable overflow interrupt
	
	
}

int main(void){				
	
	setup_uc();
	
	sei();								

	while(1){
		
	}

}

