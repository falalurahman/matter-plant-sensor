#include <Arduino.h>

#define LED_PIN 8

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    Serial.println("Blink program started");
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
