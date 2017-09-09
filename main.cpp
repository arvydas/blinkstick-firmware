/* Name: main.c
 * Project: BlinkStick
 * Author: Arvydas Juskevicius
 * Creation Date: 2013-04-02
 * Tabsize: 4
 * Copyright: (c) 2013 by Agile Innovative Ltd
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

#define LED_PORT_DDR        DDRB

#define R_BIT               PB4
#define G_BIT               PB1
#define B_BIT               PB0

#define R_PWM				OCR1B
#define G_PWM				OCR0B
#define B_PWM				OCR0A

#define MODE_RGB			0
#define MODE_RGB_INVERSE   	1
#define MODE_WS2812		   	2
#define MODE_WS2812_12BIT	   	3

#define TASK_NONE			0
#define TASK_SEND_DATA 		1
#define TASK_SET_MODE  		2

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
extern "C"
{
	#include "usbdrv.h"
	#include "light_ws2812.h"
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- LED interface ----------------------------- */
/* ------------------------------------------------------------------------- */

#define MAX_LEDS		128
#define MIN_LED_FRAME	8 * 3
#define DELAY_CYCLES	64

static uint8_t led[MAX_LEDS * 3];

const PROGMEM uint16_t ledDataCount[] = {MIN_LED_FRAME, MIN_LED_FRAME * 2, MIN_LED_FRAME * 4, MIN_LED_FRAME * 8, MAX_LEDS * 3};

/* 
	Reports:
		1: LED Data [R, G, B]
		2: Name [Binary Data 0..32]
		3: Data [Binary Data 0..32]
		4: Mode set [MODE]: 0 - RGB LED Strip, 1 - Inverse RGB LED Strip, 2 - WS2812
		5: LED Data [CHANNEL, INDEX, R, G, B]
		6: LED Frame [Channel, [G, R, B][0..7]]
		7: LED Frame [Channel, [G, R, B][0..15]]
		8: LED Frame [Channel, [G, R, B][0..31]]
		9: LED Frame [Channel, [G, R, B][0..63]]
		10: LED Frame [Channel, [G, R, B][0..127]]
		20: Frame repeat [repetitions]
	
	Memory Map:
		00      : Oscillator calibration value
		01 - 0C : Serial
		0D      : Mode
		0F - 1F : <unused>
		20 - 3F : Name
		40 - 5F : Data
		60 -    : <unused>
*/

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

//if descriptor changes, USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH also has to be updated in usbconfig.h

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {    /* USB report descriptor */

    '\x06', '\x00', '\xff',          // USAGE_PAGE (Generic Desktop)
    '\x09', '\x01',                  // USAGE (Vendor Usage 1)
    '\xa1', '\x01',                  // COLLECTION (Application)
    '\x15', '\x00',                  //   LOGICAL_MINIMUM (0)
    '\x26', '\xff', '\x00',          //   LOGICAL_MAXIMUM (255)
    '\x75', '\x08',                  //   REPORT_SIZE (8)
    '\x85', '\x01',                  //   REPORT_ID (1)
    '\x95', '\x03',                  //   REPORT_COUNT (3)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x02',                  //   REPORT_ID (2)
    '\x95', '\x20',                  //   REPORT_COUNT (32)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x03',                  //   REPORT_ID (3)
    '\x95', '\x20',                  //   REPORT_COUNT (32)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x04',                  //   REPORT_ID (4)
    '\x95', '\x01',                  //   REPORT_COUNT (1)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x05',                  //   REPORT_ID (5)
    '\x95', '\x05',                  //   REPORT_COUNT (5)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x06',                  //   REPORT_ID (6)
    '\x95', '\x19',                  //   REPORT_COUNT (25)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x07',                  //   REPORT_ID (7)
    '\x95', '\x31',                  //   REPORT_COUNT (49)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x08',                  //   REPORT_ID (8)
    '\x95', '\x61',                  //   REPORT_COUNT (97)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x09',                  //   REPORT_ID (9)
    '\x95', '\xc1',                  //   REPORT_COUNT (193)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x0a',                  //   REPORT_ID (10)
    '\x96', '\x81', '\x01',          //   REPORT_COUNT (385)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\x85', '\x14',                  //   REPORT_ID (20)
    '\x95', '\x01',                  //   REPORT_COUNT (1)
    '\x09', '\x00',                  //   USAGE (Undefined)
    '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
    '\xc0'                           // END_COLLECTION
};

static uint16_t currentAddress;
static uint16_t bytesRemaining;
static uchar reportId = 0; 
static uchar channel;

static uint8_t mode;
static uint8_t repeat = 1;
static uint8_t task = 0;
static uint16_t ledCount = 0;
static uint8_t delayCycles = 0;

/* ------------------------------------------------------------------------- */
/* ----------------------------- Helper Functions--------------------------- */
/* ------------------------------------------------------------------------- */

uchar channelToPin(uchar ch) {
	if (ch == 1) //G
	{
		return _BV(G_BIT);
	}
	else if (ch == 2) //B
	{
		return _BV(B_BIT);
	}
	else 
	{
		return _BV(R_BIT); //R
	}
}

void setRGBPWM(uint8_t r, uint8_t g, uint8_t b)
{
	R_PWM = r;   
	G_PWM = g;   
	B_PWM = b;   
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB Communication ------------------------- */
/* ------------------------------------------------------------------------- */

/* usbFunctionRead() is called when the host requests a chunk of data from
* the device. 
*/
uchar usbFunctionRead(uchar *data, uchar len)
{
	if (reportId == 1)
	{
		data[0] = 1;
		data[1] = led[1];
		data[2] = led[0];
		data[3] = led[2];
		return 4;
	}
	else if (reportId == 2 || reportId == 3)
	{
		if(len > bytesRemaining)
			len = bytesRemaining;

                uint8_t addressOffset = reportId==2 ? 32 : 64;
		//Ignore the first byte of data as it's report id
		if (currentAddress == 0)
		{
			data[0] = reportId;
			eeprom_read_block(&data[1], (uchar *)0 + addressOffset, len - 1);
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
	else if (reportId == 4)
	{
		data[0] = 4;
		data[1] = mode;
		return 2;
	}
    else if (reportId == 5)
    {
       data[0] = 5;
       data[1] = 0;
       data[2] = 0;
       data[3] = led[1];
       data[4] = led[0];
       data[5] = led[2];

       return 6;
    }
    else if (reportId >= 6 && reportId <= 10) // Serial data for LEDs
    {
       if (bytesRemaining == 0)
       {
		   return 0; // end of transfer 
       }

       if(len > bytesRemaining)
           len = bytesRemaining;

       //Ignore the first byte of data as it's report id
       if (currentAddress == 0)
       {
          data[0] = reportId;
          data[1] = 0;

          for (int i = 2; i < len; i++)
          {
              data[i] = led[currentAddress + i - 2];
          }

          currentAddress += len - 2;
          bytesRemaining -= (len - 1);
       }
       else
       {
          for (int i = 0; i < len; i++)
          {
              data[i] = led[currentAddress + i];
          }

          currentAddress += len;
          bytesRemaining -= len;
       }

       return len;
    }
    else if (reportId == 20)
    {
        data[0] = 20;
        data[1] = repeat;
        return 2;
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
		if (mode == MODE_RGB)
		{
			led[0] = data[2];
			led[1] = data[1];
			led[2] = data[3];

			//Set PWM values
			setRGBPWM(255 - led[1], 255 - led[0], 255 - led[2]);
		}
		else if (mode == MODE_RGB_INVERSE)
		{
			led[0] = data[2];
			led[1] = data[1];
			led[2] = data[3];

			//Set PWM values
			setRGBPWM(led[1], led[0], led[2]);
		}
		else if (mode == MODE_WS2812 || mode == MODE_WS2812_12BIT)
		{
			led[0] = data[2];
			led[1] = data[1];
			led[2] = data[3];

			//Set only the first LED on the first channel
			cli(); //Disable interrupts
                        if(mode == MODE_WS2812_12BIT)
                        {
			    ws2812_sendarraylow_mask(&led[0], 3, channelToPin(0));
                        }
                        else
                        {
			    ws2812_sendarray_mask(&led[0], 3, channelToPin(0));
                        }
			sei(); //Enable interrupts
		}

		return 1;
	}
	else if (reportId == 2 || reportId == 3)
	{
		if(bytesRemaining == 0)
			return 1; // end of transfer 

		if(len > bytesRemaining)
			len = bytesRemaining;

                uint8_t addressOffset = reportId==2 ? 32 : 64;
		//Ignore the first byte of data as it's report id
		if (currentAddress == 0)
		{
			eeprom_write_block(&data[1], (uchar *)0 + addressOffset, len);
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
	else if (reportId == 4)
	{
		mode = data[1];
		eeprom_write_byte((uchar *)0 + 1 + 12, mode);
		//Prepare to send the data simultaneously together with USB polling
		task = TASK_SET_MODE;
		delayCycles = 0;
		
		//Disable any USB requests while sending data to LED Strip
		usbDisableAllRequests();
		return 1;
	}
	else if (reportId == 5)
	{
		if (mode != MODE_WS2812 && mode != MODE_WS2812_12BIT)
		{
			return 1;
		}

		channel = data[1];
		uint8_t index = data[2];

		led[index * 3 + 0] = data[4];
		led[index * 3 + 1] = data[3];
		led[index * 3 + 2] = data[5];

		//Prepare to send the data simultaneously together with USB polling
		task = TASK_SEND_DATA;
		ledCount = (index + 1) * 3;
		delayCycles = 0;
		
		//Disable any USB requests while sending data to LED Strip
		usbDisableAllRequests();

		return 1;
	}
	else if (reportId >= 6 && reportId <= 10) // Serial data for LEDs
	{
		if (mode != MODE_WS2812 && mode != MODE_WS2812_12BIT)
		{
			return 1;
		}

		if (bytesRemaining == 0)
		{
			//Prepare to send the data simultaneously together with USB polling
			task = TASK_SEND_DATA;
			ledCount = pgm_read_word_near(&ledDataCount[reportId - 6]);
			delayCycles = 0;
			
			//Disable any USB requests while sending data to LED Strip
			usbDisableAllRequests();

			return 1; // end of transfer 
		}

		if(len > bytesRemaining)
			len = bytesRemaining;

		//Ignore the first byte of data as it's report id
		if (currentAddress == 0)
		{
			channel = data[1];

			for (int i = 2; i < len; i++)
			{
				led[currentAddress + i - 2] = data[i];
			}

			currentAddress += len - 2;
			bytesRemaining -= (len - 1);
		}
		else
		{
			for (int i = 0; i < len; i++)
			{
				led[currentAddress + i] = data[i];
			}

			currentAddress += len;
			bytesRemaining -= len;
		}

		if (bytesRemaining <= 0)
		{
			//Prepare to send the data simultaneously together with USB polling
			task = TASK_SEND_DATA;
			ledCount = pgm_read_word_near(&ledDataCount[reportId - 6]);
			delayCycles = 0;
			
			//Disable any USB requests while sending data to LED Strip
			usbDisableAllRequests();
		}

		return bytesRemaining == 0; // return 1 if this was the last chunk 
	}
	else if (reportId == 20)
	{
		repeat = data[1];
		return 1;
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

/* Retrieves the current mode from EEPROM */
static void SetMode(void)
{
   mode = eeprom_read_byte((uchar *)0 + 1 + 12);

   if (mode > 3)
	   mode = 0;
}


extern "C" usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (usbRequest_t *)data;
	reportId = rq->wValue.bytes[0];

	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){ /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){ /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			 if(reportId == 1){ //Device colors
				return USB_NO_MSG;
			 }
			 else if(reportId == 2){ // Name of the device
				 bytesRemaining = 33;
				 currentAddress = 0;
				 return USB_NO_MSG; 
			 }
			 else if(reportId == 3){ // Data of the device
				 bytesRemaining = 33;
				 currentAddress = 0;
				 return USB_NO_MSG; 
			 }
			 else if(reportId == 4){ // Report device mode
				 return USB_NO_MSG; 
			 }
			 else if(reportId == 5){ // Indexed LED data^M
				 currentAddress = 0;
				 bytesRemaining = 5;
				 return USB_NO_MSG; 
			 }
			 else if (reportId >= 6 && reportId <= 10) { // Serial data for LEDs
			 	currentAddress = 0;

			 	switch (reportId) {
			 	    case 6:
			 	 	   bytesRemaining = MIN_LED_FRAME * 1 + 1;
			 	 	   break;
			 	    case 7:
			 	 	   bytesRemaining = MIN_LED_FRAME * 2 + 1;
			 	 	   break;
			 	    case 8:
			 	 	   bytesRemaining = MIN_LED_FRAME * 4 + 1;
			 	 	   break;
			 	    case 9:
			 	 	   bytesRemaining = MIN_LED_FRAME * 8 + 1;
			 	 	   break;
			 	    case 10:
			 	 	   bytesRemaining = MAX_LEDS*3 + 1;
			 	 	   break;
			 	}


			 	return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
			 }
			 else if(reportId == 20){ // Set repeat
				 return USB_NO_MSG; 
			 }


			 return 0;

        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			 if(reportId == 1){ //Device colors
				bytesRemaining = 3;
				currentAddress = 0;
				return USB_NO_MSG; 
			 }
			 else if(reportId == 2){ // Name of the device
				currentAddress = 0;
				bytesRemaining = 32;
				return USB_NO_MSG; 
			 }
			 else if(reportId == 3){ // Name of the device
				currentAddress = 0;
				bytesRemaining = 32;
				return USB_NO_MSG; 
			 }
			 else if(reportId == 4){ // Mode
				currentAddress = 0;
				bytesRemaining = 1;
				return USB_NO_MSG; 
			 }
			 else if(reportId == 5){ // Indexed LED data
				currentAddress = 0;
				bytesRemaining = 5;
				return USB_NO_MSG; 
			 }
			 else if (reportId >= 6 && reportId <= 10) { // Serial data for LEDs
				 currentAddress = 0;

				 switch (reportId) {
					case 6:
					    bytesRemaining = MIN_LED_FRAME * 1 + 1;
					    break;
					case 7:
					    bytesRemaining = MIN_LED_FRAME * 2 + 1;
					    break;
					case 8:
					    bytesRemaining = MIN_LED_FRAME * 4 + 1;
					    break;
					case 9:
					    bytesRemaining = MIN_LED_FRAME * 8 + 1;
					    break;
					case 10:
					    bytesRemaining = MAX_LEDS*3 + 1;
					    break;
				 }


				 return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
			 }
			 else if(reportId == 20){
				currentAddress = 0;
				bytesRemaining = 1;
				return USB_NO_MSG; 
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

void ApplyMode(void)
{
	if (mode == MODE_RGB || mode == MODE_RGB_INVERSE)
	{
		/* PWM enable,  */
		GTCCR |= _BV(PWM1B) | _BV(COM1B1);
		TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1) | _BV(COM0B1);

		/* Start timer 0 and 1 */
		TCCR1 |= _BV (CS10);
		TCCR0B |=  _BV(CS00);

		/* Set PWM value to off after a brief 10ms blink. */
		if (mode == MODE_RGB)
		{
			setRGBPWM(32, 32, 32);
			_delay_ms(10);
			setRGBPWM(255, 255, 255);
		}
		else
		{
			setRGBPWM(223, 223, 223);
			_delay_ms(10);
			setRGBPWM(0, 0, 0);
		}
	}
	else if (mode == MODE_WS2812 || mode == MODE_WS2812_12BIT)
	{
		//Turn off PWM
		setRGBPWM(0, 0, 0);

		/* Stop timer 0 and 1 */
		TCCR1 &= ~_BV (CS10);
		TCCR0B &=  ~_BV(CS00);

		/* Disable PWM */
		GTCCR &= ~_BV(PWM1B) & ~_BV(COM1B1);
		TCCR0A &= ~_BV(WGM00) & ~_BV(WGM01) & ~_BV(COM0A1) & ~_BV(COM0B1);

		led[0]=32; led[1]=32; led[2]=32;
		ws2812_sendarray_mask(&led[0], 3, channelToPin(0));
		ws2812_sendarray_mask(&led[0], 3, channelToPin(1));
		ws2812_sendarray_mask(&led[0], 3, channelToPin(2));

		_delay_ms(10);

		led[0]=0; led[1]=0; led[2]=0;
		ws2812_sendarray_mask(&led[0], 3, channelToPin(0));
		ws2812_sendarray_mask(&led[0], 3, channelToPin(1));
		ws2812_sendarray_mask(&led[0], 3, channelToPin(2));
	}
}


void ledTransfer() {

	if (task != TASK_NONE)
	{
		if (delayCycles < DELAY_CYCLES)
		{
			delayCycles++;
			return;
		}

		if (task == TASK_SEND_DATA)
		{
			//send all data at the same time, asume there is going to be no communication over USB during this time
			uint16_t len = 3 * MAX_LEDS;

			if (len > ledCount)
			{
				len = ledCount;
			}

			cli(); //Disable interrupts
                        if(mode == MODE_WS2812_12BIT)
                        {
				for(uint8_t i=0; i<repeat; i++){
				    ws2812_sendarraylow_mask(led, len, channelToPin(channel));
				    ws2812_sendarrayhigh_mask(led, len, channelToPin(channel));
				}
                        }
                        else
                        {
				for(uint8_t i=0; i<repeat; i++){
				    ws2812_sendarray_mask(led, len, channelToPin(channel));
				}
                        }
			sei(); //Enable interrupts

			task = TASK_NONE;
			usbEnableAllRequests();
		}
		else if (task == TASK_SET_MODE)
		{
			cli(); //Disable interrupts
			ApplyMode();
			sei(); //Enable interrupts

			task = TASK_NONE;
			usbEnableAllRequests();
		}
	}

}


int main(void)
{
	wdt_enable(WDTO_1S);

	SetSerial();
	SetMode();

    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    for(uint8_t i=255; i!=0; i--){  /* fake USB disconnect for > 250 ms */
	wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
	
	//Set LED ports to output
    LED_PORT_DDR |= _BV(R_BIT);   
    LED_PORT_DDR |= _BV(G_BIT);   
    LED_PORT_DDR |= _BV(B_BIT);   

	ApplyMode();

    sei();

    for(;;){                /* main event loop */
		wdt_reset();
	  	usbPoll();

		ledTransfer();	
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
