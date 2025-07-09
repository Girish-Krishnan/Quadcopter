#include "radio.h"

void setup()
{
	Serial.begin(9600);  // Start up serial
	rfBegin(12);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
	
	// Send a message to other RF boards on this channel
	Serial.println("Transmitting counts...");
}
char i  = 'A';

void loop()
{
	rfWrite(i);
	Serial.print(i);
	i++;
	if (i > 'Z') {
		rfWrite('\n');
		Serial.print('\nCounting: ');
		i= 'A';
	}
	delay(100);
}
