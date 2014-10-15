// gorgon coprocessor main

#include <wirish/wirish.h>
#include "libraries/Servo/Servo.h"


/* gorgon pin defines */
const uint8_t interrupt_pins[16] __FLASH__ = { 19, 18, 20, 44, 22, 21, 24, 23, 43, 27, 40, 41, 2, 17, 34, 35 };
const uint8_t solenoid_pins[8] __FLASH__ = { 45, 42, 36, 37, 38, 7, 6, 39 };
const uint8_t pwm_pins[12] __FLASH__ = { 16, 15, 32, 47, 11, 10, 9, 8, 5, 4, 3, 33 };

// hold encoder counts
volatile int32_t enc_count[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// hold encoder history
unsigned char enc_ab[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// lookup table for quad decoding
const signed short enc_table[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; 

// leds
#define RED_LED 12
#define BLUE_LED 13
#define GREEN_LED 14

// i2c
#define SDA 0
#define SDL 1

// servo objects for setting pwm values
Servo pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8, pwm9, pwm10, pwm11;

// spi used to talk to arduino
HardwareSPI spi(2);



// triggered on int0 A or B rising/falling edge
void int0_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[0]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[1]) == HIGH)
        state_bits | 0x02;

    enc_ab[0] = enc_ab[0] << 2;                     // move the old data left two places
    enc_ab[0] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[0] += enc_table[(enc_ab[0] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int1 A or B rising/falling edge
void int1_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[2]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[3]) == HIGH)
        state_bits | 0x02;

    enc_ab[1] = enc_ab[1] << 2;                     // move the old data left two places
    enc_ab[1] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[1] += enc_table[(enc_ab[1] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int2 A or B rising/falling edge
void int2_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[4]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[5]) == HIGH)
        state_bits | 0x02;

    enc_ab[2] = enc_ab[2] << 2;                     // move the old data left two places
    enc_ab[2] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[2] += enc_table[(enc_ab[2] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int3 A or B rising/falling edge
void int3_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[6]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[7]) == HIGH)
        state_bits | 0x02;

    enc_ab[3] = enc_ab[3] << 2;                     // move the old data left two places
    enc_ab[3] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[3] += enc_table[(enc_ab[3] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int4 A or B rising/falling edge
void int4_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[8]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[9]) == HIGH)
        state_bits | 0x02;

    enc_ab[4] = enc_ab[4] << 2;                     // move the old data left two places
    enc_ab[4] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[4] += enc_table[(enc_ab[4] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int5 A or B rising/falling edge
void int5_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[10]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[11]) == HIGH)
        state_bits | 0x02;

    enc_ab[5] = enc_ab[5] << 2;                     // move the old data left two places
    enc_ab[5] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[5] += enc_table[(enc_ab[5] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int6 A or B rising/falling edge
void int6_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[12]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[13]) == HIGH)
        state_bits | 0x02;

    enc_ab[6] = enc_ab[6] << 2;                     // move the old data left two places
    enc_ab[6] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[6] += enc_table[(enc_ab[6] & 0x0F)];  // get the change from the 16 entry table
}

// triggered on int7 A or B rising/falling edge
void int7_trigger() {
    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[14]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[15]) == HIGH)
        state_bits | 0x02;

    enc_ab[7] = enc_ab[7] << 2;                     // move the old data left two places
    enc_ab[7] |= (state_bits & 0x03);               // OR in the two new bits
    enc_count[7] += enc_table[(enc_ab[7] & 0x0F)];  // get the change from the 16 entry table
}

void setup_pin_modes() {
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
    pwm0.attach(pwm_pins[0]);
    pwm0.write(90);
    pwm1.attach(pwm_pins[1]);
    pwm1.write(90);
    pwm2.attach(pwm_pins[2]);
    pwm2.write(90);
    pwm3.attach(pwm_pins[3]);
    pwm3.write(90);
    pwm4.attach(pwm_pins[4]);
    pwm4.write(90);
    pwm5.attach(pwm_pins[5]);
    pwm5.write(90);
    pwm6.attach(pwm_pins[6]);
    pwm6.write(90);
    pwm7.attach(pwm_pins[7]);
    pwm7.write(90);
    pwm8.attach(pwm_pins[8]);
    pwm8.write(90);
    pwm9.attach(pwm_pins[9]);
    pwm9.write(90);
    pwm10.attach(pwm_pins[10]);
    pwm10.write(90);
    pwm11.attach(pwm_pins[11]);
    pwm11.write(90);
}

void map_interrupts() {
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
    setup_pin_modes();
    map_interrupts();

    // wait for arduino to boot
    delay(1000);

    spi.beginSlave();

    pinMode(BOARD_SPI2_NSS_PIN, INPUT);
}

void reset_self() {
    // PWMs to neutral
    pwm0.write(90);
    pwm1.write(90);
    pwm2.write(90);
    pwm3.write(90);
    pwm4.write(90);
    pwm5.write(90);
    pwm6.write(90);
    pwm7.write(90);
    pwm8.write(90);
    pwm9.write(90);
    pwm10.write(90);
    pwm11.write(90);

    // reset encoder vars
    for (uint8_t i=0; i<8; i++) {
        enc_count[i] = 0;
        enc_ab[i] = 0;
    }

    // LEDs off
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);

    // solenoides off
    for (uint8_t i=0; i<8; i++) {
        digitalWrite(solenoid_pins[i], LOW);
    }
}

void reset_encoder_count(uint8_t encoder_num) {
    // sanity check
    if (encoder_num > 7)
        return;
    
    enc_count[encoder_num] = 0;
}

void set_pwm_val(uint8_t pwm_chan, uint8_t val) {
    // sanity check
    if (pwm_chan > 11)
        return;

    uint16 pulseWidth = map(val, 0, 255, 544, 2400);

    if (pwm_chan == 0)
        pwm0.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 1)
        pwm1.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 2)
        pwm2.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 3)
        pwm3.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 4)
        pwm4.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 5)
        pwm5.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 6)
        pwm6.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 7)
        pwm7.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 8)
        pwm8.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 9)
        pwm9.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 10)
        pwm10.writeMicroseconds(pulseWidth);
    else if (pwm_chan == 11)
        pwm11.writeMicroseconds(pulseWidth);
}

void set_solenoid_val(uint8_t sol_chan, uint8_t state) {
    // sanity check
    if (sol_chan > 7)
        return;

    digitalWrite(solenoid_pins[sol_chan], state);
}

void set_led_state(uint8_t led_state) {
    if (led_state == 1) {
        // not connected
        digitalWrite(RED_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    } else if (led_state == 2) {
        // disabled
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
    } else if (led_state == 3) {
        // enabled
        digitalWrite(RED_LED, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
    } else {
        // error state
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    }
}

void set_controller_state() {
    // pwm
    for (int i=0; i<12; i++) {
        set_pwm_val(i, spi.read());
    }
    // solenoid
    for (int i=0; i<8; i++) {
        set_solenoid_val(i, spi.read());
    }
    // leds
    set_led_state(spi.read());
}

void transmit_encoder_count(uint8_t encoder_num) {
    // sanity check
    if (encoder_num < 8) {
        int32_t val = enc_count[encoder_num];

        // feed out 32 bit int as bytes
        spi.write((byte)((val >> 24) & 0xFF));
        spi.write((byte)((val >> 16) & 0xFF));
        spi.write((byte)((val >> 8) & 0xFF));
        spi.write((byte)(val & 0xFF));
    } else {
        // invalid encoder, return 0
        spi.write(0);
        spi.write(0);
        spi.write(0);
        spi.write(0);
    }
}

void loop() {
    // read opcode from SPI
    uint8_t cmd = spi.read();

    if (cmd == 1) {
        set_controller_state();
    } else if (cmd == 2) {
        transmit_encoder_count(spi.read());
    } else if (cmd == 3) {
        reset_encoder_count(spi.read());
    } else if (cmd == 4) {
        reset_self();
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
