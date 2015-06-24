#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
//Local includes:
#include "lcd.h"
#include "onewire.h"
#include "ds18x20.h"

//reenable when VERBOSE enabled in ds18x20.h
//#include "uart.h"
//#define USART_BAUDRATE 9600


#define F_CPU 3686400

//define debug to run additional debug code:
//#define DEBUG
//#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//Global variables:
static char lcd_string[32];
// I'm planning for 2 sensors only:
uint8_t gSensorIDs[2][OW_ROMCODE_SIZE];
// increment global var. to get temperature measured every 30 sec.
uint8_t tenSecPeriod;


// define structure to keep max and min temperatures:
struct temperature {
  uint16_t t;
  uint8_t sign;
  };

//remeber max and min temperature measured:
//Initialize with impossible values, so they get set to current by the first readout
struct temperature max = { 999,1};
struct temperature min = { 999,0};

//Function definitions:
uint8_t search_sensors(void);
void    display_temp(const uint8_t subzero, uint8_t cel,
		     uint8_t cel_frac_bits,char where[4]);
void measure_and_display(void);


int main (void) {

  uint8_t nSensors;
  //uint8_t subzero, cel, cel_frac_bits;

  // reenable if VERBOSE in ds18x20.h enabled
  //   uart_init((UART_BAUD_SELECT((USART_BAUDRATE),F_CPU)));   

   
   TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode

   TIMSK |= (1 << OCIE1A); // Enable CTC interrupt

   OCR1A   = 36000; // Set CTC compare value to 0.1Hz (T=10s) at 3.6MHz AVR clock, with a prescaler of 1024

   TCCR1B |= ((1 << CS10) | (1 << CS12)); // Start timer at Fcpu/1024



   //Initialize LCD disp:
  lcd_init(LCD_DISP_ON);
  _delay_ms(200);

  nSensors = search_sensors();
  sprintf(lcd_string,"%u sensor(s) \navailable",nSensors); 
  lcd_puts(lcd_string);
  _delay_ms(500);

#ifdef DEBUG
  lcd_clrscr();
  sprintf(lcd_string,"%u\n%u",gSensorIDs[0][0],gSensorIDs[1][0]);
  lcd_puts(lcd_string);
#endif
 
  tenSecPeriod = 0;

  // Measure and display  temp before getting to main loop:
  measure_and_display();


  
   sei(); //enable global interrupt flag.

   for (;;) { // Loop forever
   }
}

/* END main */


ISR(TIMER1_COMPA_vect) 
{ 
  if ( tenSecPeriod++ >= 2 ) {
    measure_and_display();
    tenSecPeriod = 0;
  }
}


/**
 * Measure and display temperature:
 */
void measure_and_display(void) {
  uint8_t   subzero, cel, cel_frac_bits;
  uint16_t  decicel;
  int16_t   local_temp, min_temp, max_temp;
  char      buffer[32]="";
  char      sign_max,sign_min;

   lcd_clrscr(); /* clear the screen*/ 
   if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, &gSensorIDs[0][0] ) == DS18X20_OK ){
	 _delay_ms(DS18B20_TCONV_12BIT);
	 DS18X20_read_meas(&gSensorIDs[0][0], &subzero, &cel, &cel_frac_bits);
	 display_temp(subzero, cel, cel_frac_bits,"I");
	 strncat(buffer,lcd_string,7);

   }
   strcat(buffer,"  ");

   if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, &gSensorIDs[1][0] ) == DS18X20_OK ){
	 _delay_ms(DS18B20_TCONV_12BIT);
	 DS18X20_read_meas(&gSensorIDs[1][0], &subzero, &cel, &cel_frac_bits);
	 display_temp(subzero, cel, cel_frac_bits,"O");
	 strncat(buffer,lcd_string,7);
	 //compare current OUT readout with minimax:
	 //TODO
	 decicel = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
	 local_temp = (subzero) ? ( decicel*(-1) ) : (decicel);

	 min_temp = (min.sign) ? ( min.t*(-1) ) : (min.t);
	 max_temp = (max.sign) ? ( max.t*(-1) ) : (max.t);
	 if (min_temp > local_temp) {
	   min.sign = subzero;
	   min.t = decicel;
	 }
	 if (max_temp < local_temp) {
	   max.sign = subzero;
	   max.t = decicel;
	 }
	 
	 sign_max = max.sign ? '-' : '+';
	 sign_min = min.sign ? '-' : '+';

	 sprintf(lcd_string,"\nM:%c%d.%d  m:%c%d.%d",sign_max,(int)(max.t/10),(int)(max.t%10),sign_min,(int)(min.t/10),(int)(min.t%10) );
         strncat(buffer,lcd_string,17);
	 
   }
   lcd_puts(buffer);
}



/**
 * Discover the available sensors.
 */
uint8_t search_sensors(void) {
  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff, nSensors;
  nSensors = 0;
  for( diff = OW_SEARCH_FIRST;
          diff != OW_LAST_DEVICE && nSensors < 2 ; ) {
          // FindDS1820( &diff, &id[0] );
        DS18X20_find_sensor(&diff,&id[0]);
        if( diff == OW_PRESENCE_ERR ) {
          sprintf(lcd_string, "No Sensor found" );
          break;
        }
        if( diff == OW_DATA_ERR ) {
          sprintf(lcd_string, "Bus Error" );
          break;
        }
        for (i=0;i<OW_ROMCODE_SIZE;i++) {
          gSensorIDs[nSensors][i]=id[i];
        }
        nSensors++;
  }
  return nSensors;
}


/**
 * Display temperature:
 **/
void    display_temp(const uint8_t subzero, uint8_t cel,
		     uint8_t cel_frac_bits, char where[4]) {

    char temp[3],frac[3];
    char sign;
    uint16_t decicelsius;
   
    sign = (subzero)?'-':'+';
    decicelsius = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
    itoa((int)(decicelsius/10),temp,10);
    itoa((int)(decicelsius%10),frac,10);
    sprintf(lcd_string,"%s:%c%s.%s",where,sign,temp,frac);
}


