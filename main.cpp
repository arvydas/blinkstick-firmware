/* Name: main.c
 * Project: BlinkStick
 * Author: Arvydas Juskevicius
 * Creation Date: 2013-04-02
 * Tabsize: 4
 * Copyright: (c) 2013 by Agile Innovative Ltd
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

#define LED_PORT_DDR        DDRB
#define LED_PORT_OUTPUT     PORTB
#define R_BIT            1
#define G_BIT            0
#define B_BIT            4


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
extern "C"
{
	#include "usbdrv.h"
}
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "requests.h"       /* The custom request numbers we use */
extern "C"
{
	#include "light_ws2812.h"
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

//if descriptor changes, USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH also has to be updated in usbconfig.h

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {    /* USB report descriptor */

    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0x85, 0x02,                    //   REPORT_ID (2)
    0x95, 0x20,                    //   REPORT_COUNT (32)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0x85, 0x03,                    //   REPORT_ID (3)
    0x95, 0x20,                    //   REPORT_COUNT (32)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

static volatile uint8_t r = 0;
static volatile uint8_t g = 0;
static volatile uint8_t b = 0;

static uchar currentAddress;
static uchar addressOffset;
static uchar bytesRemaining;
static uchar reportId = 0; 

static uchar replyBuffer[33]; //32 for data + 1 for report id

/* usbFunctionRead() is called when the host requests a chunk of data from
* the device. 
*/
uchar usbFunctionRead(uchar *data, uchar len)
{
	if (reportId == 1)
	{
		//Not used
		return 0;
	}
	else if (reportId == 2 || reportId == 3)
	{
		if(len > bytesRemaining)
			len = bytesRemaining;

		//Ignore the first byte of data as it's report id
		if (currentAddress == 0)
		{
			data[0] = reportId;
			eeprom_read_block(&data[1], (uchar *)0 + currentAddress + addressOffset, len - 1);
			currentAddress += len - 1;
			bytesRemaining -= (len - 1);
		}
		else
		{
			eeprom_read_block(data, (uchar *)0 + currentAddress + addressOffset, len);
			currentAddress += len;
			bytesRemaining -= len;
		}

		return len;
	}
	else
	{
		return 0;
	}

}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
* device. 
*/
uchar usbFunctionWrite(uchar *data, uchar len)
{
	if (reportId == 1)
	{
		//Set color
		OCR1B = 255 - data[1];
		OCR0B = 255 - data[2];
		OCR0A = 255 - data[3];

		return 1;
	}
	else if (reportId == 2 || reportId == 3)
	{
		if(bytesRemaining == 0)
			return 1; // end of transfer 

		if(len > bytesRemaining)
			len = bytesRemaining;

		//Ignore the first byte of data as it's report id
		if (currentAddress == 0)
		{
			eeprom_write_block(&data[1], (uchar *)0 + currentAddress + addressOffset, len);
			currentAddress += len - 1;
			bytesRemaining -= (len - 1);
		}
		else
		{
			eeprom_write_block(data, (uchar *)0 + currentAddress + addressOffset, len);
			currentAddress += len;
			bytesRemaining -= len;
		}

		return bytesRemaining == 0; // return 1 if this was the last chunk 
	}
	else
	{
		return 1;
	}
}

#define SERIAL_NUMBER_LENGTH 12 // BSXXXXXX-1.0
static int  serialNumberDescriptor[SERIAL_NUMBER_LENGTH + 1];

/* Sends the serial number when requested */
uchar usbFunctionDescriptor(usbRequest_t *rq)
{
   uchar len = 0;
   usbMsgPtr = 0;
   if (rq->wValue.bytes[1] == USBDESCR_STRING && rq->wValue.bytes[0] == 3) // 3 is the type of string descriptor, in this case the device serial number
   {
      usbMsgPtr = (uchar*)serialNumberDescriptor;
      len = sizeof(serialNumberDescriptor);
   }
   return len;
}


/* Retrieves the serial number from EEPROM */
static void SetSerial(void)
{
   serialNumberDescriptor[0] = USB_STRING_DESCRIPTOR_HEADER(SERIAL_NUMBER_LENGTH);

   uchar serialNumber[SERIAL_NUMBER_LENGTH];
   eeprom_read_block(serialNumber, (uchar *)0 + 1, SERIAL_NUMBER_LENGTH);

   for (int i =0; i < SERIAL_NUMBER_LENGTH; i++)
   {
	serialNumberDescriptor[i +1] = serialNumber[i];
   }
}



/* ------------------------------------------------------------------------- */

extern "C" usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (usbRequest_t *)data;
	reportId = rq->wValue.bytes[0];

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
	{
		switch(rq->bRequest)
		{
		case CUSTOM_RQ_SET_RED:
			r = 255 - rq->wValue.bytes[0];
			OCR0B = r;
			break;	
		case CUSTOM_RQ_SET_GREEN:
			g = 255 - rq->wValue.bytes[0];
			OCR0A = g;
			break;	
		case CUSTOM_RQ_SET_BLUE:
			b = 255 - rq->wValue.bytes[0];
			OCR1B = b;
			break;	
		}
    }
	else if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){ /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){ /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			 usbMsgPtr = replyBuffer;

			 if(reportId == 1){ //Device colors
				 
				replyBuffer[0] = 1; //report id
				replyBuffer[1] = 255 - OCR1B;
				replyBuffer[2] = 255 - OCR0B;
				replyBuffer[3] = 255 - OCR0A;

				return 4;
				 
			 }
			 else if(reportId == 2){ // Name of the device
				 replyBuffer[0] = 2; //report id

				 bytesRemaining = 33;
				 currentAddress = 0;

				 addressOffset = 32;

				 return USB_NO_MSG; /* use usbFunctionRead() to obtain data */
			 }
			 else if(reportId == 3){ // Name of the device
				 replyBuffer[0] = 3; //report id

				 bytesRemaining = 33;
				 currentAddress = 0;

				 addressOffset = 64;

				 return USB_NO_MSG; /* use usbFunctionRead() to obtain data */
			 }

			 return 0;

        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			 if(reportId == 1){ //Device colors
				bytesRemaining = 3;
				currentAddress = 0;
				return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
			 }
			 else if(reportId == 2){ // Name of the device
				currentAddress = 0;
				bytesRemaining = 32;

				addressOffset = 32;
				return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
			 }
			 else if(reportId == 3){ // Name of the device
				currentAddress = 0;
				bytesRemaining = 32;

				addressOffset = 64;
				return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
			 }
			 return 0;
        }

	}

    return 0;   /* default for not implemented requests: return no data back to host */
}

static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
 
    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if(x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
 

extern "C" void usbEventResetReady(void)
{
    cli();  // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
    calibrateOscillator();
    sei();
    eeprom_write_byte(0, OSCCAL);   // store the calibrated value in EEPROM
}

/* ------------------------------------------------------------------------- */
void pwmInit (void)
{
    /* PWM enable,  */
    GTCCR |= _BV(PWM1B) | _BV(COM1B1);

    TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1) | _BV(COM0B1);


    /* Start timer 0 and 1 */
    TCCR1 |= _BV (CS10);

    TCCR0B |=  _BV(CS00);

    /* Set PWM value to 0. */
    OCR0A = 255;   // PB0
    OCR0B = 255;   // PB1
    OCR1B = 255;   // PB4
} 

//struct CRGB { uint8_t g; uint8_t r; uint8_t b; };
//struct CRGB led[1];

int main(void)
{
	uint8_t mask;
	
	CLKPR=_BV(CLKPCE);
	CLKPR=0;			// set clock prescaler to 1 (attiny 25/45/85/24/44/84/13/13A)		
	mask=_BV(PB4);
	DDRB|=mask;
	
	while(1)
    {
		uint8_t led[3];

		led[0]=32;led[1]=32;led[2]=32;
		ws2812_sendarray_mask(&led[0], 3, _BV(PB4));
		_delay_ms(500);
		
		led[0]=32;led[1]=0;led[2]=0;
		ws2812_sendarray_mask(&led[0], 3, _BV(PB4));
		_delay_ms(500);

		/*
		uint8_t i;
		
		led[0].r=32;led[0].g=32;led[0].b=32;

		ws2812_sendarray(&led[0],3);
		_delay_ms(500);
		
		led[0].r=32;led[0].g=0;led[0].b=0;
	
		ws2812_sendarray(&led[0],3);
		_delay_ms(500);
		*/
	
    } 

	uchar   i;

    wdt_enable(WDTO_1S);

	SetSerial();
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
	



    LED_PORT_DDR |= _BV(R_BIT);   /* make the LED bit an output */
    LED_PORT_DDR |= _BV(G_BIT);   /* make the LED bit an output */
    LED_PORT_DDR |= _BV(B_BIT);   /* make the LED bit an output */
	pwmInit();

    sei();

    for(;;){                /* main event loop */
        wdt_reset();
        usbPoll();
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
