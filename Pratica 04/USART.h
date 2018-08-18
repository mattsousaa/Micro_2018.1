#ifndef _USART_H
#define _USART_H
#include "def_principais.h"

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void USART_init(void);
void USART_send(unsigned char data);
unsigned char USART_Receive(void);
void USART_putstring(char* StringPtr);

#endif