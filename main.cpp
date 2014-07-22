// Sample main.cpp file. Blinks the built-in LED, sends a message out
// USART2, and turns on PWM on pin 2.

#include <wirish/wirish.h>

void setup() {
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(14, OUTPUT);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(14, LOW);
    delay(1000);
}

void loop() {
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(14, HIGH);
    delay(500);
    digitalWrite(12, LOW);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(14, LOW);
    delay(500);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }

    return 0;
}
