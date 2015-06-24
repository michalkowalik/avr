#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>

#include "./usbdrv/usbdrv.h"
#define DEBUG_LEVEL 0
#include "oddebug.h"


#define USART_BAUDRATE 1200

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define YELLOW 1
#define RED 0


/* USB report descriptor (length is defined in usbconfig.h)
   This has been changed to conform to the USB keyboard boot
   protocol */
char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] 
  PROGMEM = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION  
};


/* The ReportBuffer contains the USB report sent to the PC */
static uchar reportBuffer[8];    /* buffer for HID reports */
static uchar idleRate;           /* in 4 ms units */
static uchar protocolVer=1;      /* 0 is the boot protocol, 1 is report protocol */
uchar expectReport=0;
uchar LEDstate=0;

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;



//function prototypes:
static void hwInit(void);
void delay_ms(unsigned long ms);
void blink(unsigned long ms, int led);
uchar usbFunctionSetup(uchar data[8]);
uchar   usbFunctionRead(uchar *data, uchar len);
uchar usbFunctionWrite(uchar *data, uchar len);

int main (void)
{

  wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
  hwInit();
  //odDebugInit();
  usbInit(); /* Initialize USB stack processing */
  sei(); //enable Global Interrupt Enable flag;

   for (;;) // Loop forever
   {
     wdt_reset();
     usbPoll();
     //    blink(10,RED);
     //delay_ms(490);

   }   
}

static void hwInit(void) {
  //  PORTD=0xeb;   //1110 1011: activate pull-ups except on usb (usb reset)
  // DDRD=0x12; // 0001 0010
  //_delay_us(11);
  //DDRD=0x00; //0000 000: remove USB reset condition 
  //configure timer 0 for a rate of 16M/(1024 * 256) = 61.04Hz (~16ms)
  TCCR0=5;
  UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
   UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

   UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register

   DDRB = 0xFF;        // set port B for output.
   DDRC |=(1<<2)|(1<<3);        // set port C for input.

   UCSRB |= (1 << RXCIE); // Enable the USART Recieve Complete interrupt (USART_RXC)  (( RXCIE ? RCXIE  ))
}

// this is just a program that 'kills time' in a calibrated method
void delay_ms(unsigned long ms) {
  //unsigned long delay_count = 211; // F_CPU/17500
  unsigned long delay_count = 914; // F_CPU/17500
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


ISR(USART_RXC_vect)
{

   char ReceivedByte;

   ReceivedByte = UDR; // Fetch the recieved byte value into the variable "ByteReceived"
   UDR = ReceivedByte; // Echo back the received byte back to the computer 
   blink(15,YELLOW);
} 


/* ------------------------------------------------------------------------- */

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


/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}

/* ------------------------------------------------------------------------- */




//
//uchar usbFunctionSetup(uchar data[8]) {
//    usbRequest_t *rq = (void *)data;
//    usbMsgPtr = reportBuffer;
//    
//    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
//	if(rq->bRequest == USBRQ_HID_GET_REPORT){
//             /* wValue: ReportType (highbyte), ReportID (lowbyte) */
//	     /* we only have one report type, so don't look at wValue */
//	     return sizeof(reportBuffer);
//	 }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
//	           if (rq->wLength.word == 1) { /* We expect one byte reports */
//	           expectReport=1;
//		   return 0xFF; /* Call usbFunctionWrite with data */
//	        }   
//	 }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
//		     usbMsgPtr = &idleRate;
//		     return 1;
//		}else if(rq->bRequest == USBRQ_HID_SET_IDLE){
//		    idleRate = rq->wValue.bytes[1];
//		}else if(rq->bRequest == USBRQ_HID_GET_PROTOCOL) {
//		  if (rq->wValue.bytes[1] < 1) {
//		     protocolVer = rq->wValue.bytes[1];
//		  }   
//		}else if(rq->bRequest == USBRQ_HID_SET_PROTOCOL) {
//		   usbMsgPtr = &protocolVer;
//		   return 1;
//	  }   
//    }
// return 0;
//}


//uchar usbFunctionWrite(uchar *data, uchar len) {
// return 0;
//}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}


