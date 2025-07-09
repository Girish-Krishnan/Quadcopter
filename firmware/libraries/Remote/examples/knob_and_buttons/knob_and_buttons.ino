//-*- C++ -*-
#include <radio.h>

#include <quad_remote.h>      // Header file with pin definitions and setup

void knobs_update();
void knob_pressed(bool);
void btn1_pressed(bool);
void btn2_pressed(bool);

void btn_up_pressed(bool down);
void btn_down_pressed(bool down);
void btn_left_pressed(bool down);
void btn_right_pressed(bool down);
void btn_center_pressed(bool down);

void setup() {
	const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();

	// The buttons and the knob trigger these call backs.       
	knobs_update_cb = knobs_update; 
	knob1_btn_cb = knob_pressed;
	btn1_cb = btn1_pressed;
	btn2_cb = btn2_pressed;
	btn_up_cb =  btn_up_pressed;
	btn_down_cb = btn_down_pressed;
	btn_left_cb =  btn_left_pressed;
	btn_right_cb = btn_right_pressed;
	btn_center_cb =  btn_center_pressed;
     
	knobs_update(); // Initialize the knob
}

void loop() {

	if (is_pressed(BUTTON_UP_PIN)) {
		Serial.println("Up is pressed");
	}
  update_display();
}

int row = 0;
int column = 0;

void update_display() {
	lcd.clear();
	lcd.setCursor(column, row);
	lcd.print(knob1.getCurrentPos());
}

void knobs_update() {
	Serial.print("Knob: ");
	Serial.println(knob1.getCurrentPos());
}


void btn1_pressed(bool down) {
	if(down) {
		Serial.println("btn1 down");
	} else {
		Serial.println("btn1 up");    
	}
}

void btn2_pressed(bool down) {
	if(down) {
		Serial.println("btn2 down");
	}else {
		Serial.println("btn2 up");    
	}
}

void knob_pressed(bool down) {
	if(down) {
		Serial.println("knob down");
		knob1.setCurrentPos(0);
		update_display();
	}else {
		Serial.println("knob up");    
	}
}


void btn_up_pressed(bool down) {
	if(down) {
		Serial.println("up down");
		row = (row - 1) %2;
		update_display();
	} else {
		Serial.println("up up");    
	}
}

void btn_down_pressed(bool down) {
	if(down) {
		Serial.println("down down");
		row = (row + 1) %2;
		update_display();
	} else {
		Serial.println("down up");    
	}
}

void btn_left_pressed(bool down) {
	if(down) {
		Serial.println("left down");
		column = (column - 1) %16;
		update_display();
	} else {
		Serial.println("left up");
	}
}

void btn_right_pressed(bool down) {
	if(down) {
		Serial.println("right down");
		column = (column + 1) %16;
		update_display();
	} else {
		Serial.println("right up");    
	}
}

    
void btn_center_pressed(bool down) {
	if(down) {
		Serial.println("center down");
		update_display();
	} else {
		Serial.println("center up");    
	}
}
