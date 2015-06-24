#include <avr/io.h>
#define USART_BAUDRATE 1200
#define F_CPU 3686400

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

int main (void)
{
   char ReceivedByte;

   UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
   UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

   UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register

   for (;;) // Loop forever
   {
      while ((UCSRA & (1 << RXC)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
      ReceivedByte = UDR; // Fetch the recieved byte value into the variable "ByteReceived"

      while ((UCSRA & (1 << UDRE)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
      UDR = ReceivedByte; // Echo back the received byte back to the computer
   }   
}
