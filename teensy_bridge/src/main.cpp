#include <Arduino.h>

// The radar uses 115200 baud, 8 data bits, even parity and one stop bit.
// USB Serial is a virtual serial port; Serial1 is RX1/TX1 on pins 0 and 1.
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8E1);
}

void loop() {
    while (Serial.available() && Serial1.availableForWrite()) {
        Serial1.write(Serial.read());
    }
    while (Serial1.available() && Serial.availableForWrite()) {
        Serial.write(Serial1.read());
    }
}
