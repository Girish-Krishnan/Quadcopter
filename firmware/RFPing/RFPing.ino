#include "radio.h"

void setup()
{
	Serial.begin(9600);  // Start up serial
	rfBegin(11);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
	
	// Send a message to other RF boards on this channel
	rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

#if(0) // This is for figuring out if AREF messes with radio function.
       #define PIN_YAW		        A0
	pinMode(analogInputToDigitalPin(PIN_YAW),  INPUT);           
	
	ADMUX_struct.refs = 0; // set reference voltage
	ADCSRA_struct.aden = 0; // restart adc
	delay(10);
	ADCSRA_struct.aden = 1; 
	delay(10);
#endif
}

char i = 'A';

void loop()
{
	uint8_t b[256];
	int len;
	
	rfWrite(i);
	Serial.print("sent :");
	Serial.println(i);
	i++;
	if (i > 'z') {
		i= 'A';
	}
	
	if (len = rfAvailable())  // If serial comes in...
	{
		rfRead(b, len);
		b[len] = 0;
		Serial.print("Received :");
		Serial.write((char*)b);
//		Serial.print("; YAW=");
//		Serial.print(analogRead(PIN_YAW));
		Serial.prinln();
	}
	
}
