#include <avr/io.h>    // header file
#include <util/delay.h>

#define true 1



void Delay(unsigned long count);

int main(void)         // program starts here
{
    DDRB = 255;        // set port B for output


     while(true) {
       PORTB = 0xFF;
       _delay_ms(250);
       PORTB = 0x00;
       _delay_ms(250);
     }



    return (0);        // return something
}

void Delay(unsigned long count) {
  while(count--);
}
