// gorgon testing

#include <wirish/wirish.h>
#include "libraries/Servo/Servo.h"


/* gorgon pin defines */
const uint8_t interrupt_pins[16] __FLASH__ = { 19, 18, 20, 44, 22, 21, 24, 23, 43, 27, 40, 41, 2, 17, 34, 35 };
const uint8_t solenoid_pins[8] __FLASH__ = { 45, 42, 36, 37, 38, 7, 6, 39 };
const uint8_t pwm_pins[12] __FLASH__ = { 16, 15, 32, 47, 11, 10, 9, 8, 5, 4, 3, 33 };

volatile int32_t enc0_count = 0;
volatile int32_t enc1_count = 0;
volatile int32_t enc2_count = 0;
volatile int32_t enc3_count = 0;
volatile int32_t enc4_count = 0;
volatile int32_t enc5_count = 0;
volatile int32_t enc6_count = 0;
volatile int32_t enc7_count = 0;

unsigned char enc0_ab = 0;   // enc0 history
unsigned char enc1_ab = 0;   // enc1 history
unsigned char enc2_ab = 0;   // enc2 history
unsigned char enc3_ab = 0;   // enc3 history
unsigned char enc4_ab = 0;   // enc4 history
unsigned char enc5_ab = 0;   // enc5 history
unsigned char enc6_ab = 0;   // enc6 history
unsigned char enc7_ab = 0;   // enc7 history
const signed short enc_table[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; 

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
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[0]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[1]) == HIGH)
        state_bits | 0x02;

    enc0_ab = enc0_ab << 2;                     // move the old data left two places
    enc0_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc0_count += enc_table[(enc0_ab & 0x0F)];  // get the change from the 16 entry table
}

void int1_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[2]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[3]) == HIGH)
        state_bits | 0x02;

    enc1_ab = enc1_ab << 2;                     // move the old data left two places
    enc1_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc1_count += enc_table[(enc1_ab & 0x0F)];  // get the change from the 16 entry table
}

void int2_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[4]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[5]) == HIGH)
        state_bits | 0x02;

    enc2_ab = enc2_ab << 2;                     // move the old data left two places
    enc2_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc2_count += enc_table[(enc2_ab & 0x0F)];  // get the change from the 16 entry table
}

void int3_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[6]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[7]) == HIGH)
        state_bits | 0x02;

    enc3_ab = enc3_ab << 2;                     // move the old data left two places
    enc3_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc3_count += enc_table[(enc3_ab & 0x0F)];  // get the change from the 16 entry table
}

void int4_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[8]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[9]) == HIGH)
        state_bits | 0x02;

    enc4_ab = enc4_ab << 2;                     // move the old data left two places
    enc4_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc4_count += enc_table[(enc4_ab & 0x0F)];  // get the change from the 16 entry table
}

void int5_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[10]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[11]) == HIGH)
        state_bits | 0x02;

    enc5_ab = enc5_ab << 2;                     // move the old data left two places
    enc5_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc5_count += enc_table[(enc5_ab & 0x0F)];  // get the change from the 16 entry table
}

void int6_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[12]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[13]) == HIGH)
        state_bits | 0x02;

    enc6_ab = enc6_ab << 2;                     // move the old data left two places
    enc6_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc6_count += enc_table[(enc6_ab & 0x0F)];  // get the change from the 16 entry table
}

void int7_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[14]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[15]) == HIGH)
        state_bits | 0x02;

    enc7_ab = enc7_ab << 2;                     // move the old data left two places
    enc7_ab |= (state_bits & 0x03);             // OR in the two new bits
    enc7_count += enc_table[(enc7_ab & 0x0F)];  // get the change from the 16 entry table
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
    attachInterrupt(interrupt_pins[1], int0_trigger, CHANGE);
    attachInterrupt(interrupt_pins[2], int1_trigger, CHANGE);
    attachInterrupt(interrupt_pins[3], int1_trigger, CHANGE);
    attachInterrupt(interrupt_pins[4], int2_trigger, CHANGE);
    attachInterrupt(interrupt_pins[5], int2_trigger, CHANGE);
    attachInterrupt(interrupt_pins[6], int3_trigger, CHANGE);
    attachInterrupt(interrupt_pins[7], int3_trigger, CHANGE);
    attachInterrupt(interrupt_pins[8], int4_trigger, CHANGE);
    attachInterrupt(interrupt_pins[9], int4_trigger, CHANGE);
    attachInterrupt(interrupt_pins[10], int5_trigger, CHANGE);
    attachInterrupt(interrupt_pins[11], int5_trigger, CHANGE);
    attachInterrupt(interrupt_pins[12], int6_trigger, CHANGE);
    attachInterrupt(interrupt_pins[13], int6_trigger, CHANGE);
    attachInterrupt(interrupt_pins[14], int7_trigger, CHANGE);
    attachInterrupt(interrupt_pins[15], int7_trigger, CHANGE);
}

void setup() {
    setupPinModes();
    mapInterrupts();

    //spi.beginSlave();
}

void loop() {
    /*if (!spi.isData()) {
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
    spi.write(0x04);*/

    if ((enc0_count % 2) == 0) {
        digitalWrite(BLUE_LED, HIGH);
    } else {
        digitalWrite(BLUE_LED, LOW);
    }
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
