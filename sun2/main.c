/* Name: main.c
 * Project: SunKeyboard
 * Author: Michał Kowalik
 * Creation Date: 2009-03-06
 * Tabsize: 4
 * Copyright: (c) 2009 MK
 * License: GNU GPL v2 
 * Version 0.1
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>
#include <string.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "keycodes.h"


/* usb descriptor and sun keycodes moved to keycodes.h */


/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

/* The following variable stores the last byte received from USART */
static uchar   currentUsartByte;
static int     newUsartByte=0;


/* USB variables */
static uint8_t reportBuffer[8]={0,0,0,0,0,0,0,0}; ///< buffer for HID reports
static uint8_t idleRate;        ///< in 4ms units
static uint8_t protocolVer = 1; ///< 0 = boot protocol, 1 = report protocol
uint8_t expectReport = 0;       ///< flag to indicate if we expect an USB-report


/* prototypes: */

static void    uart_putchar(uchar c);
       uchar   usbFunctionRead(uchar *data, uchar len);
       uchar   usbFunctionWrite(uchar *data, uchar len);
       uint8_t usbFunctionSetup(uint8_t data[8]);
       void    blink(unsigned long ms, int led);
static int     usartInit();
       uchar   keyCodeToUSB(uchar key);

/* ------------------------------------------------------------------------- */

/*
 * send a byte to the keyboard.
 */
static void uart_putchar(uchar c)
{
	loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
}




/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */

uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}

uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar cLED = 0;
        if (expectReport && (len == 1)) {
		if (data[0] & USB_LED_NLOCK) {
			cLED |= 0x01;
		}
		if (data[0] & USB_LED_CLOCK) {
			cLED |= 0x08;
		}
		if (data[0] & USB_LED_SCRLCK) {
			cLED |= 0x04;
		}
		if (data[0] & USB_LED_CMPOSE) {
			cLED |= 0x02;
		}
		uart_putchar(SKBDCMD_SETLED);
		uart_putchar(cLED);
	}
    expectReport = 0;
    return 1;
}

/* ------------------------------------------------------------------------- */



/**
 * This function is called whenever we receive a setup request via USB.
 * \param data[8] eight bytes of data we received
 * \return number of bytes to use, or 0xff if usbFunctionWrite() should be
 * called
 */
uint8_t usbFunctionSetup(uint8_t data[8]) {
    usbRequest_t *rq = (void *)data;
    usbMsgPtr = reportBuffer;
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
        // class request type
        if (rq->bRequest == USBRQ_HID_GET_REPORT) {
            // wValue: ReportType (highbyte), ReportID (lowbyte)
            // we only have one report type, so don't look at wValue
            return sizeof(reportBuffer);
        } else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
            if (rq->wLength.word == 1) {
                // We expect one byte reports
                expectReport = 1;
                return 0xff; // Call usbFunctionWrite with data
            }
        } else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
            usbMsgPtr = &idleRate;
            return 1;
        } else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
            idleRate = rq->wValue.bytes[1];
        } else if (rq->bRequest == USBRQ_HID_GET_PROTOCOL) {
            if (rq->wValue.bytes[1] < 1) {
                protocolVer = rq->wValue.bytes[1];
            }
        } else if(rq->bRequest == USBRQ_HID_SET_PROTOCOL) {
            usbMsgPtr = &protocolVer;
            return 1;
        }
    } else {
        // no vendor specific requests implemented
    }
    return 0;
}


/* ------------------------------------------------------------------------- */

static int usartInit()
{
   // Turn on the transmission and reception circuitry
   UCSRB |= (1 << RXEN) | (1 << TXEN);
   // Use 8-bit character sizes
   UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
   // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   UBRRL = BAUD_PRESCALE; 
   // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8);
   // Enable the USART Recieve Complete interrupt (USART_RXC)  (( RXCIE ? RCXIE  ))
   UCSRB |= (1 << RXCIE); 
 
  return 0;
}


void blink(unsigned long ms, int led) {
  PORTB |= (1<<led);
  _delay_ms(ms);
  PORTB &= ~(1<<led);
}


uchar keyCodeToUSB(uchar key) {
 
  uchar i; 

  uchar keyUp  = key & 0x80;
  uchar usbKey = pgm_read_byte(&(sunkeycodes[key & 0x7f]));
 

  /* no key changed..  */
  if(usbKey == 0) {
    return 0;
  }

  /* Check mod keys. */
  else if ((0xe0 <= usbKey) && (usbKey <= 0xe7)) {
       if (keyUp) { 
         reportBuffer[0] &=  ~(1 << (usbKey - 0xe0));
       } else {
	 reportBuffer[0] |=  (1 << (usbKey - 0xe0));
       }
  }
  /* Normal keys. */
  else {
    if (keyUp) {
      for (i = 2; i < 8; i++) {
	  if (reportBuffer[i] == usbKey) {
	      reportBuffer[i] = 0;
	      break;
          }
      }
      //key pressed: 
    } else {
      for (i = 2; i < 8; i++) {
	  if (reportBuffer[i] == 0) {
	      reportBuffer[i] = usbKey;
	      break;
          }
      }
   }
  }
  /*key changed -- report 1 */
  return 1;
}



ISR(USART_RXC_vect)
{

   char ReceivedByte;
   char sendByte[2];
  
   // Fetch the recieved byte value into the variable "ByteReceived"
   ReceivedByte = UDR;
   //debug:
   UDR = ReceivedByte;

  if (keyCodeToUSB(ReceivedByte)) {
     newUsartByte = 1;
  }
}


int main(void)
{
uchar   i;
uint8_t updateNeeded = 0;
uint8_t idleCounter = 0;

   DDRB = 0xFF;        // set port B for output. (leds)
   TCCR0 = 5;          // timer 0 prescaler: 1024

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    usartInit();
    _delay_ms(100);
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */


   
    /* MAIN LOOP*/
    for(;;){                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
        usbPoll();
        updateNeeded = newUsartByte; //any changes?

      // check timer if we need periodic reports
        if (TIFR & (1 << TOV0)) {
            TIFR = (1 << TOV0); // reset flag
            if (idleRate != 0) { // do we need periodic reports?
                if(idleCounter > 4){ // yes, but not yet
                    idleCounter -= 5; // 22ms in units of 4ms
                } else { // yes, it is time now
                    updateNeeded = 1;
                    idleCounter = idleRate;
                }
            }
        }
   
        // if an update is needed, send the report
        if (updateNeeded && usbInterruptIsReady()) {
            updateNeeded = 0;
            newUsartByte = 0;
	    usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
