#include<RotaryEncoder.h>
#include"MomentaryButton.h"

RotaryEncoder knob(11,10,12);

void setup() {
    Serial.begin(57600);
}

void loop() {
     /*  Serial.print("(A, B, button) = ");
  Serial.print(knob.readA(), DEC);
  Serial.print(knob.readB(), DEC);
  Serial.print(knob.read(), DEC);
  Serial.print(knob.readButton(), DEC); */
  
  switch (knob.poll()) {
  case RotaryEncoder::CCW:
      Serial.print("CCW ");
      Serial.print(knob.getValue());
      Serial.println();
      break;
  case RotaryEncoder::CW:
      Serial.print("CW ");
      Serial.print(knob.getValue());
      Serial.println();
      break;
  case RotaryEncoder::UNCHANGED:
       //Serial.print("Unchanged");
      break;
    default:
      Serial.print("Error");
  }

  if (knob.button.buttonPressed()) {
       knob.setValue(0);
       Serial.println("Button pressed");
  }
  if (knob.button.buttonReleased()) {
       Serial.println("Button released");
  }
  
  if (knob.button.buttonDown()) {
       Serial.println("Button is down");
  }
  
  delay(10);
}
