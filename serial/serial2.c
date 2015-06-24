#include <avr/io.h>
#include <avr/interrupt.h>


#define USART_BAUDRATE 1200
#define F_CPU 3686400

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define YELLOW 1

void delay_ms(unsigned long ms);
void blink(unsigned long ms, int led);


int main (void) {
   
   UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
   UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

   UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register

   DDRB = 0xFF;        // set port B for output. (leds)

   DDRC |=(1<<2)|(1<<3);        // set pin 2 & 3 of port  C for output, 0 and 1 for input.

   UCSRB |= (1 << RXCIE); // Enable the USART Recieve Complete interrupt (USART_RXC)  (( RXCIE ? RCXIE  ))
   sei(); //enable Global Interrupt Enable flag;

   for (;;) { // Loop forever
     PORTC=(PINC<<2)^0x0C; // shift 2 first bits and negate them.
   }   
}


// this is just a program that 'kills time' in a calibrated method
void delay_ms(unsigned long ms) {
  unsigned long delay_count = 211; // F_CPU/17500
  volatile unsigned long i;
  
  while (ms != 0) {
    for (i=0; i != delay_count; i++);
    ms--;
  }
   
}
 
void blink(unsigned long ms, int led) {
  PORTB |= (1<<led);
  delay_ms(ms);
  PORTB &= ~(1<<led);
}


ISR(USART_RXC_vect) {

   char ReceivedByte;

   ReceivedByte = UDR; // Fetch the recieved byte value into the variable "ByteReceived"
   UDR = ReceivedByte; // Echo back the received byte back to the computer 
   blink(15,YELLOW);
} 
