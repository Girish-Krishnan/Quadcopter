#ifndef RADIO_INCLUDED
#define RADIO_INCLUDED

/* RadioFunctions.h
  A handful of sending and receiving functions for the 
  ATmega128RFA1.
  by: Jim Lindblom
      SparkFun Electronics
  date: July 8, 2013
  license: Beerware. Use, distribut, and modify this code freely
  however you please. If you find it useful, you can buy me a beer
  some day.
  
  Functions in here:
    rfBegin(uint8_t channel) - Initializes the radio on a channel
                                between 11 and 26.
    rfWrite(uint8_t b) - Sends a byte over the radio.
    rfPrint(String toPrint) - Sends a string over the radio.
    int rfAvailable() - Returns number of characters currently
                        in receive buffer, ready to be read.
    char rfRead() - Reads a character out of the buffer.
    
  Interrupt Sub-Routines (ISRs) in here:
    TRX24_TX_END_vect - End of radio transfer.
    TRX24_RX_START_vect - Beginning of radio receive.
    TRX24_RX_END_vect - End of radio receive. Characters are 
                        collected into a buffer here.
*/
  
#include <Arduino.h> // Required for digitalWrites, etc.

void rfFlush();
uint8_t rfBegin(uint8_t channel);

void rfPrint(String toPrint);
void rfWrite(uint8_t b);
void rfWrite(uint8_t * b, uint8_t length);
unsigned int rfAvailable();
char rfRead();
char rfRead(uint8_t * buf, uint8_t len);

extern uint8_t rssiRaw;

#define GOOD_PACKET 0
#define BAD_PACKET 1
#define NO_PACKET 2

template<class T>
byte rfReceive(T & g)
{
	T t;
	//I've tried a lot of things here to try to reduce lag between remote
	//stick movement and quad response.  checking that there are enough
	//bytes available seems to help.  But it's a not clear.  Sometimes I
	//just lose connectivity for minutes or seconds.  Moving the remote
	//closer to the quad will sometimes fix this (even if I then move the
	//quad farther away).
	if (rfAvailable()>=sizeof(T)) {
		int l = rfRead((uint8_t *)&t, sizeof(T));
		if (l == sizeof(T)) {
			if (t.validate()) {
				g = t;
				return GOOD_PACKET;
			} else {
				rfFlush();
				return BAD_PACKET;
#if(0)
				Serial.print("Bad packet: ");
				Serial.print(t.get_name());
				Serial.println();
				Serial.println("Got     Expected");
				Serial.print("Magic: 0x");
				Serial.print((int)t.magic, 16);
				Serial.print("  0x");
				Serial.print(T::magic_value, 16);
				Serial.println();
				Serial.print("Hash: 0x");
				Serial.print(compute_hash(t), 16);
				Serial.print("  0x");
				Serial.print((int)t.hash, 16);
				Serial.println();
#endif
			}
		} else if (l == 0) {
			return NO_PACKET;
		} else {
			return NO_PACKET;
		}
	}  
	return false;
}

template<class T>
bool rfSend(T & g)
{
	g.bless();
	rfWrite((uint8_t *)&g, sizeof(T));
}

#endif 
