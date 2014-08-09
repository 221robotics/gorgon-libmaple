// gorgon testing

#include <wirish/wirish.h>
#include "libraries/Servo/Servo.h"


/* gorgon pin defines */
const uint8_t interrupt_pins[16] __FLASH__ = { 19, 18, 20, 44, 22, 21, 24, 23, 43, 27, 40, 41, 2, 17, 34, 35 };
const uint8_t solenoid_pins[8] __FLASH__ = { 45, 42, 36, 37, 38, 7, 6, 39 };
const uint8_t pwm_pins[12] __FLASH__ = { 16, 15, 32, 47, 11, 10, 9, 8, 5, 4, 3, 33 };

// leds
#define RED_LED 12
#define BLUE_LED 13
#define GREEN_LED 14

// i2c
#define SDA 0
#define SDL 1

// servo objects for setting pwm values
Servo pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8, pwm9, pwm10, pwm11, pwm12;

// spi used to talk to arduino
HardwareSPI spi(2);


void int0_trigger() {

}

void int1_trigger() {

}

void int2_trigger() {

}

void int3_trigger() {

}

void int4_trigger() {

}

void int5_trigger() {

}

void int6_trigger() {

}

void int7_trigger() {

}

void int8_trigger() {

}

void int9_trigger() {

}

void int10_trigger() {

}

void int11_trigger() {

}

void int12_trigger() {

}

void int13_trigger() {

}

void int14_trigger() {

}

void int15_trigger() {

}

void setupPinModes() {
    // LEDs (output)
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);

    // interrupts (input)
    for (uint8_t i=0; i<16; i++) {
        pinMode(interrupt_pins[i], INPUT);
    }

    // solenoides (output)
    for (uint8_t i=0; i<8; i++) {
        pinMode(solenoid_pins[i], OUTPUT);
        digitalWrite(solenoid_pins[i], LOW);
    }

    //digitalWrite(solenoid_pins[0], HIGH);

    // pwms (pwm output)
    pwm1.attach(pwm_pins[0]);
    pwm1.write(90);
    pwm2.attach(pwm_pins[1]);
    pwm2.write(90);
    pwm3.attach(pwm_pins[2]);
    pwm3.write(90);
    pwm4.attach(pwm_pins[3]);
    pwm4.write(90);
    pwm5.attach(pwm_pins[4]);
    pwm5.write(90);
    pwm6.attach(pwm_pins[5]);
    pwm6.write(90);
    pwm7.attach(pwm_pins[6]);
    pwm7.write(90);
    pwm8.attach(pwm_pins[7]);
    pwm8.write(90);
    pwm9.attach(pwm_pins[8]);
    pwm9.write(90);
    pwm10.attach(pwm_pins[9]);
    pwm10.write(90);
    pwm11.attach(pwm_pins[10]);
    pwm11.write(90);
    pwm12.attach(pwm_pins[11]);
    pwm12.write(90);
}

void mapInterrupts() {
    attachInterrupt(interrupt_pins[0], int0_trigger, CHANGE);
    attachInterrupt(interrupt_pins[1], int1_trigger, CHANGE);
    attachInterrupt(interrupt_pins[2], int2_trigger, CHANGE);
    attachInterrupt(interrupt_pins[3], int3_trigger, CHANGE);
    attachInterrupt(interrupt_pins[4], int4_trigger, CHANGE);
    attachInterrupt(interrupt_pins[5], int5_trigger, CHANGE);
    attachInterrupt(interrupt_pins[6], int6_trigger, CHANGE);
    attachInterrupt(interrupt_pins[7], int7_trigger, CHANGE);
    attachInterrupt(interrupt_pins[8], int8_trigger, CHANGE);
    attachInterrupt(interrupt_pins[9], int9_trigger, CHANGE);
    attachInterrupt(interrupt_pins[10], int10_trigger, CHANGE);
    attachInterrupt(interrupt_pins[11], int11_trigger, CHANGE);
    attachInterrupt(interrupt_pins[12], int12_trigger, CHANGE);
    attachInterrupt(interrupt_pins[13], int13_trigger, CHANGE);
    attachInterrupt(interrupt_pins[14], int14_trigger, CHANGE);
    attachInterrupt(interrupt_pins[15], int15_trigger, CHANGE);
}

void setup() {
    setupPinModes();
    //mapInterrupts();

    spi.beginSlave();
}

void loop() {
    if (!spi.isData()) {
        // reset uC state here
        spi.beginSlave();
    }

    // echo back everything received over spi
    uint8_t cmd = spi.read();

    if (cmd == 0x00) {
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
    } else if (cmd == 0x01) {
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    }

    // 'ack'
    spi.write(0x04);
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
