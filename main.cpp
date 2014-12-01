// gorgon coprocessor main

#include <wirish/wirish.h>
#include "libraries/Servo/Servo.h"


/* gorgon pin defines */
const uint8_t interrupt_pins[16] __FLASH__ = { 19, 18, 20, 44, 22, 21, 24, 23, 43, 27, 40, 41, 2, 17, 34, 35 };
const uint8_t solenoid_pins[8] __FLASH__ = { 45, 42, 36, 37, 38, 7, 6, 39 };
const uint8_t pwm_pins[12] __FLASH__ = { 16, 15, 32, 47, 11, 10, 9, 8, 5, 4, 3, 33 };
bool pmw_attached[] = { false, false, false, false, false, false, false, false, false, false, false, false };

// hold encoder counts
volatile int32_t enc_count[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// counts per second vars
uint16_t enc_cps_accuracy[] = { 16, 16, 16, 16, 16, 16, 16, 16 };
float enc_cps[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
volatile int32_t rolling_cps[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long last_cps_calc_millis[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

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
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[0]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[1]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[0] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[0] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int1 A or B rising/falling edge
void int1_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[2]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[3]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[1] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[1] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int2 A or B rising/falling edge
void int2_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[4]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[5]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[2] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[2] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int3 A or B rising/falling edge
void int3_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[6]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[7]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[3] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[3] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int4 A or B rising/falling edge
void int4_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[8]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[9]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[4] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[4] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int5 A or B rising/falling edge
void int5_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[10]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[11]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[5] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[5] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int6 A or B rising/falling edge
void int6_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[12]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[13]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[6] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[6] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
}

// triggered on int7 A or B rising/falling edge
void int7_trigger() {
    static unsigned char enc_ab = 0;

    uint8_t state_bits = 0;
    if (digitalRead(interrupt_pins[14]) == HIGH)
        state_bits = 0x01;
    if (digitalRead(interrupt_pins[15]) == HIGH)
        state_bits |= 0x02;

    enc_ab = enc_ab << 2;                           // move the old data left two places
    enc_ab |= (state_bits & 0x03);                  // OR in the two new bits
    enc_count[7] += enc_table[(enc_ab & 0x0F)];     // get the change from the 16 entry table
    rolling_cps[7] += enc_table[(enc_ab & 0x0F)];   // keep track of our cps count
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

void reset_self() {
    // PWMs detached
    if (pmw_attached[0])
        pwm0.detach();
    if (pmw_attached[1])
        pwm1.detach();
    if (pmw_attached[2])
        pwm2.detach();
    if (pmw_attached[3])
        pwm3.detach();
    if (pmw_attached[4])
        pwm4.detach();
    if (pmw_attached[5])
        pwm5.detach();
    if (pmw_attached[6])
        pwm6.detach();
    if (pmw_attached[7])
        pwm7.detach();
    if (pmw_attached[8])
        pwm8.detach();
    if (pmw_attached[9])
        pwm9.detach();
    if (pmw_attached[10])
        pwm10.detach();
    if (pmw_attached[11])
        pwm11.detach();

    for (uint8_t i=0; i<12; i++)
        pmw_attached[i] = false;

    // reset encoder vars
    unsigned long current_millis = millis();
    for (uint8_t i=0; i<8; i++) {
        enc_count[i] = 0;

        enc_cps_accuracy[i] = 16;
        enc_cps[i] = 0;
        rolling_cps[i] = 0;
        last_cps_calc_millis[i] = current_millis;
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


void setup() {
    // coprocessor mapping
    setup_pin_modes();
    map_interrupts();

    // wait for arduino to boot
    delay(1000);

    spi.beginSlave();
    pinMode(BOARD_SPI2_NSS_PIN, INPUT_PULLUP);

    // reset for good measure
    reset_self();
}



void reset_encoder_count(uint8_t encoder_num) {
    // sanity check
    if (encoder_num > 7)
        return;
    
    enc_count[encoder_num] = 0;
}

void set_encoder_accuracy() {
    uint8_t encoder_num = spi.read();

    // sanity check
    if (encoder_num > 7)
        return;

    uint8_t hibyte = spi.read();
    uint8_t lobyte = spi.read();

    enc_cps_accuracy[encoder_num] = ((hibyte << 8) & 0xFF) | (lobyte & 0xFF);
}

void attach_pwm(uint8_t pwm_chan) {
    // sanity check
    if (pwm_chan > 11)
        return;
    else if (pmw_attached[pwm_chan])
        return;

    if (pwm_chan == 0) {
        pwm0.attach(pwm_pins[pwm_chan]);
        pwm0.write(90);
    } else if (pwm_chan == 1) {
        pwm1.attach(pwm_pins[pwm_chan]);
        pwm1.write(90);
    } else if (pwm_chan == 2) {
        pwm2.attach(pwm_pins[pwm_chan]);
        pwm2.write(90);
    } else if (pwm_chan == 3) {
        pwm3.attach(pwm_pins[pwm_chan]);
        pwm3.write(90);
    } else if (pwm_chan == 4) {
        pwm4.attach(pwm_pins[pwm_chan]);
        pwm4.write(90);
    } else if (pwm_chan == 5) {
        pwm5.attach(pwm_pins[pwm_chan]);
        pwm5.write(90);
    } else if (pwm_chan == 6) {
        pwm6.attach(pwm_pins[pwm_chan]);
        pwm6.write(90);
    } else if (pwm_chan == 7) {
        pwm7.attach(pwm_pins[pwm_chan]);
        pwm7.write(90);
    } else if (pwm_chan == 8) {
        pwm8.attach(pwm_pins[pwm_chan]);
        pwm8.write(90);
    } else if (pwm_chan == 9) {
        pwm9.attach(pwm_pins[pwm_chan]);
        pwm9.write(90);
    } else if (pwm_chan == 10) {
        pwm10.attach(pwm_pins[pwm_chan]);
        pwm10.write(90);
    } else if (pwm_chan == 11) {
        pwm11.attach(pwm_pins[pwm_chan]);
        pwm11.write(90);
    }

    // update state
    pmw_attached[pwm_chan] = true;
}

void detach_pwm(uint8_t pwm_chan) {
    // sanity check
    if (pwm_chan > 11)
        return;
    else if (!pmw_attached[pwm_chan])
        return;

    if (pwm_chan == 0) {
        pwm0.detach();
    } else if (pwm_chan == 1) {
        pwm1.detach();
    } else if (pwm_chan == 2) {
        pwm2.detach();
    } else if (pwm_chan == 3) {
        pwm3.detach();
    } else if (pwm_chan == 4) {
        pwm4.detach();
    } else if (pwm_chan == 5) {
        pwm5.detach();
    } else if (pwm_chan == 6) {
        pwm6.detach();
    } else if (pwm_chan == 7) {
        pwm7.detach();
    } else if (pwm_chan == 8) {
        pwm8.detach();
    } else if (pwm_chan == 9) {
        pwm9.detach();
    } else if (pwm_chan == 10) {
        pwm10.detach();
    } else if (pwm_chan == 11) {
        pwm11.detach();
    }

    // update state
    pmw_attached[pwm_chan] = false;
}

void set_pwm_val(uint8_t pwm_chan, uint8_t val) {
    // sanity check
    if (pwm_chan > 11)
        return;
    else if (!pmw_attached[pwm_chan])
        return;
    else if (val == 255)
        return;

    uint16 pulseWidth = map(val, 0, 254, 544, 2400);

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

    digitalWrite(solenoid_pins[sol_chan], state > 0 ? HIGH : LOW);
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
    } else if (led_state == 4) {
        // error state
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    } else if (led_state == 5) {
        // error state
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
    } else if (led_state == 6) {
        // error state
        digitalWrite(RED_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    } else {
        // error state
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
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
        spi.transfer((byte)((val >> 24) & 0xFF));
        spi.transfer((byte)((val >> 16) & 0xFF));
        spi.transfer((byte)((val >> 8) & 0xFF));
        spi.transfer((byte)(val & 0xFF));
    } else {
        // invalid encoder, return 0
        spi.transfer(0);
        spi.transfer(0);
        spi.transfer(0);
        spi.transfer(0);
    }
}

void transmit_encoder_cps(uint8_t encoder_num) {
    // sanity check
    if (encoder_num < 8) {
        union {
            float f;
            uint8_t b[4];
        } u;
        u.f = enc_cps[encoder_num];

        // feed out 32 bit int as bytes
        spi.transfer(u.b[0]);
        spi.transfer(u.b[1]);
        spi.transfer(u.b[2]);
        spi.transfer(u.b[3]);
    } else {
        // invalid encoder, return 0
        spi.transfer(0);
        spi.transfer(0);
        spi.transfer(0);
        spi.transfer(0);
    }
}

void loop() {
    if (!spi.isData()) {
        // run through encoder cps calculations
        for (int i=0; i<8; i++) {
            if ((abs(rolling_cps[i]) >= enc_cps_accuracy[i]) || (millis() - last_cps_calc_millis[i]) > 1000) {
                // time to calculate counts per second
                unsigned long current_millis = millis();
                enc_cps[i] = ((1000.0/(float)(current_millis - last_cps_calc_millis[i])) * (float)rolling_cps[i]);
                last_cps_calc_millis[i] = current_millis;
                rolling_cps[i] = 0;
            }
        }
    }

    // wait for 'activate' command (0xFF, 0x7F)
    if (spi.read() != 0xFF)
        return;
    if (spi.read() != 0x7F)
        return;

    // read opcode from SPI
    uint8_t cmd = spi.read();

    if (cmd == 1) {
        set_controller_state();
    } else if (cmd == 2) {
        transmit_encoder_count(spi.transfer(0xFF));
    } else if (cmd == 3) {
        reset_encoder_count(spi.transfer(0xFF));
    } else if (cmd == 4) {
        // get encoder CPS
        transmit_encoder_cps(spi.transfer(0xFF));
    } else if (cmd == 5) {
        // config encoder CPS accuracy
        set_encoder_accuracy();
    } else if (cmd == 6) {
        // PWM attach
        attach_pwm(spi.transfer(0xFF));
    } else if (cmd == 7) {
        // PWM detach
        detach_pwm(spi.transfer(0xFF));
    } else if (cmd == 8) {
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
