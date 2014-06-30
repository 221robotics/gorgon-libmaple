#include <wirish/wirish.h>
HardwareSPI spi(2);

uint64_t encoder_count = 0;


void setup() {
    /* Set up the LED to blink  */
    pinMode(BOARD_LED_PIN, OUTPUT);

    digitalWrite(BOARD_LED_PIN, HIGH);

    spi.beginSlave();
}

void reset_micro() {
    // TODO: reset micro
}

uint8_t* get_ptr_to_encoder_count(uint8_t encoder_num) {
    // TODO: actually get real encoder count
    return (uint8_t*)&encoder_count;
}

void reset_encoder_count(uint8_t encoder_num) {
    // TODO: implement
}

void set_pwm_val(uint8_t pwm_num, uint8_t pwm_val) {
    // TODO: implement
}

void set_solenoid_val(uint8_t sol_num, bool sol_on) {
    // TODO: implement
}

void set_led_state(uint8_t state) {
    // TODO: implement
}

void on_heartbeat() {
    // TODO: implement
}

void loop() {
    uint8_t cmd = spi.read();
    
    switch(cmd) {
        case 0x01:  /* RESET COPROCESSOR */
            reset_micro();
            break;
        case 0x02:  /* GET ENCODER COUNT */
            // return long long w/ count
            spi.write(get_ptr_to_encoder_count(spi.read()), 8);
            break;
        case 0x03:  /* RESET ENCODER COUNT */
            reset_encoder_count(spi.read());
            break;
        case 0x04:  /* SET PWM */
            set_pwm_val(spi.read(), spi.read());
            break;
        case 0x05:  /* SET SOLENOID */
            set_solenoid_val(spi.read(), spi.read() != 0 ? true : false);
            break;
        case 0x06:  /* SET CONTROLLER STATE */
            set_led_state(spi.read());
            break;
        case 0x07:  /* HEARTBEAT */
            on_heartbeat();
            break;
        default:
            // something bad has happened
            reset_micro();
            break;
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
