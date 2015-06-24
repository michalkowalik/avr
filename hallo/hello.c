#include <avr/io.h>    // header file

int main(void)         // program starts here
{
    DDRB = 255;        // set port B for output
    PORTB = 0;         // set port B pins to 0

    return (0);        // return something
}
