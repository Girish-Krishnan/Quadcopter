#include "quad_remote.h"


#include <SoftwareSerial2.h>

SoftwareSerial lcdSerial(LCD_TX_PIN, LCD_RX_PIN);

SerLCD lcd;

RotaryEncoder knob1(ENC1_A_PIN, ENC1_B_PIN);

void quad_remote_setup() {
	knob1.setup();

	EICRA_struct.isc0 = 0x01; // Any direction interrupts for encoder 3
	EICRA_struct.isc1 = 0x01;

	EICRA_struct.isc2 = 0x01;
	EICRA_struct.isc3 = 0x01;
	EICRB_struct.isc4 = 0x01;
	EICRB_struct.isc5 = 0x01;
	EICRB_struct.isc6 = 0x01;
	EICRB_struct.isc7 = 0x01;
    
	EIMSK = 0b11111111;
     
	PCICR = 1;               // enable pin-change interrupt 0
	PCMSK0 = 0b00110000;     // catch changes on all the pins for the encoder

	ADMUX_struct.refs = 0; // set reference voltage
	ADCSRA_struct.aden = 0; // restart adc
	delay(10);
	ADCSRA_struct.aden = 1; 
	delay(10);
	
	
	pinMode(ENC1_BUTTON_PIN, INPUT_PULLUP);

	pinMode(BUTTON1_PIN, INPUT_PULLUP);
	pinMode(BUTTON2_PIN, INPUT_PULLUP);

	pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
	pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
	pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
	pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
	pinMode(BUTTON_CENTER_PIN, INPUT_PULLUP);

	pinMode(analogInputToDigitalPin(PIN_YAW),  INPUT);            // Gimbal: Yaw
	pinMode(analogInputToDigitalPin(PIN_THROTTLE), INPUT);        // Gimbal: throttle
	pinMode(analogInputToDigitalPin(PIN_ROLL), INPUT);            // Gimbal: roll
	pinMode(analogInputToDigitalPin(PIN_PITCH), INPUT);           // Gimbal: pitch
     
	pinMode(analogInputToDigitalPin(BATTERY_SENSE_PIN), INPUT);
	lcdSerial.begin(9600);
	lcd.begin(lcdSerial);
}

bool is_pressed(int button)
{
	return ! digitalRead(button);
}

void nop() {
}

void nop_btn(bool) {
}

void (*knobs_update_cb)() = nop;
void (*knob1_btn_cb)(bool) = nop_btn;

void (*btn1_cb)(bool) = nop_btn;
void (*btn2_cb)(bool) = nop_btn;

void (*btn_up_cb)(bool) = nop_btn;   
void (*btn_down_cb)(bool) = nop_btn;
void (*btn_left_cb)(bool) = nop_btn;
void (*btn_right_cb)(bool) = nop_btn;
void (*btn_center_cb)(bool) = nop_btn;


ISR(PCINT0_vect)
{  
	knob1.update();
	knobs_update_cb();
}


#define BUTTON_INTERRUPT(vector, cb, pin)	\
	ISR(vector)				\
	{					\
		cb(!digitalRead(pin));		\
	}

BUTTON_INTERRUPT(INT2_vect, knob1_btn_cb, ENC1_BUTTON_PIN);

BUTTON_INTERRUPT(INT1_vect, btn2_cb, BUTTON2_PIN);
BUTTON_INTERRUPT(INT0_vect, btn1_cb, BUTTON1_PIN);

BUTTON_INTERRUPT(INT4_vect, btn_up_cb, BUTTON_UP_PIN);
BUTTON_INTERRUPT(INT3_vect, btn_down_cb, BUTTON_DOWN_PIN);
BUTTON_INTERRUPT(INT5_vect, btn_left_cb, BUTTON_LEFT_PIN);
BUTTON_INTERRUPT(INT7_vect, btn_right_cb, BUTTON_RIGHT_PIN);
BUTTON_INTERRUPT(INT6_vect, btn_center_cb, BUTTON_CENTER_PIN);


int numbers[TOTAL_CHANNELS+1];

char *labels[TOTAL_CHANNELS] = {"T ", "Y ", "P ", "R ", 
				"E1A", "E1B",
				"E1B", 
				"B1", "B2",
				"UP",
				"DOWN",
				"LEFT",
				"RIGHT",
				"CENTER"};

char pins[TOTAL_CHANNELS] = {PIN_PITCH, PIN_YAW, PIN_ROLL, PIN_THROTTLE,
			     ENC1_A_PIN, ENC1_B_PIN,
			     ENC1_BUTTON_PIN,
			     BUTTON1_PIN, BUTTON2_PIN,
			     BUTTON_UP_PIN,
			     BUTTON_DOWN_PIN,
			     BUTTON_LEFT_PIN,
			     BUTTON_RIGHT_PIN,
			     BUTTON_CENTER_PIN};
