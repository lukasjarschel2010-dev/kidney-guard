/*
  Blink SOS

  Blinks the on-board LED in the Morse code SOS pattern (... --- ...),
  repeatedly, using standard Morse timing:
    dot = 1 unit, dash = 3 units, gap between symbols = 1 unit,
    gap between letters = 3 units, gap between words = 7 units.

  Based on the Arduino Blink example.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://docs.arduino.cc/hardware/

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/Blink/
*/

const unsigned long UNIT_MS = 200;  // length of one Morse time unit

// turn the LED on for the given number of units, then off for one unit
void flash(unsigned long units) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(units * UNIT_MS);
  digitalWrite(LED_BUILTIN, LOW);
  delay(UNIT_MS);
}

// blink one letter made of `count` identical symbols (dot = 1, dash = 3)
void letter(int count, unsigned long units) {
  for (int i = 0; i < count; ++i) {
    flash(units);
  }
  delay(2 * UNIT_MS);  // 1 unit already elapsed after last symbol -> 3 total
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  letter(3, 1);  // S: ...
  letter(3, 3);  // O: ---
  letter(3, 1);  // S: ...
  delay(4 * UNIT_MS);  // 3 units already elapsed after last letter -> 7 total
}
