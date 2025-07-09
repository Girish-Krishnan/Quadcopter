#ifndef ROTARY_ENCODER
#define ROTARY_ENCODER
#include <Arduino.h> 

#define TRANSITIONS_PER_DETENT 4
/**
 * You can track the knobs movements and detect clicks of the knob as well.
 * For it to work, you need to call \p update() frequently.  The easiest way to
 * do this is to call it at the top of your \p loop() function and not use \p
 * delay() in your code.
 */
class RotaryEncoder {
	int pinA, pinB, currentPos, maxPos, pinButton;
	uint8_t old_AB;
	char tics;
public:

	/** \brief Constructor 
	 *
	 * The parameter identifies the pin the encoder is connected to.
	 *
	 * If you use the sketch that came with your robot, you won't need call this.
	 */
	RotaryEncoder(int pin_a, int pin_b) : pinA(pin_a), pinB(pin_b), old_AB(0), tics(0) {}

	/**
	 * \brief Setup the breakout.
	 *
	 * Call this function once in your setup() function. 
	 */
	void setup() {
		pinMode(pinA, INPUT_PULLUP);
		pinMode(pinB, INPUT_PULLUP);
		currentPos = 0;
		
		// get an initial reading on the encoder pins
		if (digitalRead(pinA) == LOW) {
			old_AB |= (1 << 0);
		}
		if (digitalRead(pinB) == LOW) {
			old_AB |= (1 << 1);
		}
	}
	
	/**
	 * \brief Update the encoder
	 *
	 * https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
	 */	
	int8_t update() {
		static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
		/**/
		old_AB <<= 2;                   //remember previous state
		old_AB |= (digitalRead(pinA) << 1)| digitalRead(pinB); //add current state
		tics += enc_states[( old_AB & 0x0f )];
		if (tics == TRANSITIONS_PER_DETENT) {
			currentPos++;
			tics = 0;
		} else if (tics == -TRANSITIONS_PER_DETENT) {
			currentPos--;
			tics =0;
		}
	}

	/**
	 * \brief Get the current value
	 *
	 * Returns the current position of the rotary encoder.  
	 */
	int getCurrentPos() {
		return currentPos;
	}

	/**
	 * \brief Set the current value
	 *
	 */
	int setCurrentPos(int v) {
		currentPos = v;
	}

};

#endif
