#include "radio.h"

void setup()
{
	Serial.begin(9600);  // Start up serial
	rfBegin(20);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
	
	Serial.println("Echo bytes from the radio");
}

void loop()
{
	
	if (rfAvailable())  // If serial comes in...
	{
		char a = rfRead();
		rfWrite(a); // ...send it out the radio.
		Serial.write(a);
	}
}
