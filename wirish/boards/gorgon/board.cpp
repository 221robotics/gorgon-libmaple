/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file   wirish/boards/gorgon/board.cpp
 * @author Eric Barch <ericb@ericbarch.com>
 * @brief  Gorgon board file.
 */

#include <board/board.h>

#include <libmaple/gpio.h>
#include <libmaple/timer.h>

#include <wirish/wirish_debug.h>
#include <wirish/wirish_types.h>

/* Since we want the Serial Wire/JTAG pins as GPIOs, disable both SW
 * and JTAG debug support, unless configured otherwise. */
void boardInit(void) {
    disableDebugPorts();
}


// Pin map: this lets the basic I/O functions (digitalWrite(),
// analogRead(), pwmWrite()) translate from pin numbers to STM32
// peripherals.
//
// PMAP_ROW() lets us specify a row (really a struct stm32_pin_info)
// in the pin map. Its arguments are:
//
// - GPIO device for the pin (GPIOA, etc.)
// - GPIO bit for the pin (0 through 15)
// - Timer device, or NULL if none
// - Timer channel (1 to 4, for PWM), or 0 if none
// - ADC device, or NULL if none
// - ADC channel, or ADCx if none
extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {

    /* Top header */

    {GPIOB,   NULL, NULL, 11, 0, ADCx}, /* D0/PB11 */
    {GPIOB,   NULL, NULL, 10, 0, ADCx}, /* D1/PB10 */
    {GPIOB,   NULL, NULL,  2, 0, ADCx}, /* D2/PB2 */
    {GPIOB, TIMER3, ADC1,  0, 3,    8}, /* D3/PB0 */
    {GPIOA, TIMER3, ADC1,  7, 2,    7}, /* D4/PA7 */
    {GPIOA, TIMER3, ADC1,  6, 1,    6}, /* D5/PA6 */
    {GPIOA,   NULL, ADC1,  5, 0,    5}, /* D6/PA5 */
    {GPIOA,   NULL, ADC1,  4, 0,    4}, /* D7/PA4 */
    {GPIOA, TIMER2, ADC1,  3, 4,    3}, /* D8/PA3 */
    {GPIOA, TIMER2, ADC1,  2, 3,    2}, /* D9/PA2 */
    {GPIOA, TIMER2, ADC1,  1, 2,    1}, /* D10/PA1 */
    {GPIOA, TIMER2, ADC1,  0, 1,    0}, /* D11/PA0 */
    {GPIOC,   NULL, NULL, 15, 0, ADCx}, /* D12/PC15 */
    {GPIOC,   NULL, NULL, 14, 0, ADCx}, /* D13/PC14 */
    {GPIOC,   NULL, NULL, 13, 0, ADCx}, /* D14/PC13 */

    /* Bottom header */

    {GPIOB, TIMER4, NULL,  7, 2, ADCx}, /* D15/PB7 */
    {GPIOB, TIMER4, NULL,  6, 1, ADCx}, /* D16/PB6 */
    {GPIOB,   NULL, NULL,  5, 0, ADCx}, /* D17/PB5 */
    {GPIOB,   NULL, NULL,  4, 0, ADCx}, /* D18/PB4 */
    {GPIOB,   NULL, NULL,  3, 0, ADCx}, /* D19/PB3 */
    {GPIOA,   NULL, NULL, 15, 0, ADCx}, /* D20/PA15 */
    {GPIOA,   NULL, NULL, 14, 0, ADCx}, /* D21/PA14 */
    {GPIOA,   NULL, NULL, 13, 0, ADCx}, /* D22/PA13 */
    {GPIOA,   NULL, NULL, 12, 0, ADCx}, /* D23/PA12 */
    {GPIOA, TIMER1, NULL, 11, 4, ADCx}, /* D24/PA11 */
    {GPIOA, TIMER1, NULL, 10, 3, ADCx}, /* D25/PA10 */
    {GPIOA, TIMER1, NULL,  9, 2, ADCx}, /* D26/PA9 */
    {GPIOA, TIMER1, NULL,  8, 1, ADCx}, /* D27/PA8 */
    {GPIOB,   NULL, NULL, 15, 0, ADCx}, /* D28/PB15 */
    {GPIOB,   NULL, NULL, 14, 0, ADCx}, /* D29/PB14 */
    {GPIOB,   NULL, NULL, 13, 0, ADCx}, /* D30/PB13 */
    {GPIOB,   NULL, NULL, 12, 0, ADCx}, /* D31/PB12 */
    {GPIOB, TIMER4, NULL,  8, 3, ADCx}, /* D32/PB8 */
    {GPIOB, TIMER3, ADC1,  1, 4,    9}, /* D33/PB1 */

    /* Gorgon STM32F103R8 Additions */

    {GPIOC,   NULL, ADC1,  0, 0,   10}, /* D34/PC0 */
    {GPIOC,   NULL, ADC1,  1, 0,   11}, /* D35/PC1 */
    {GPIOC,   NULL, ADC1,  2, 0,   12}, /* D36/PC2 */
    {GPIOC,   NULL, ADC1,  3, 0,   13}, /* D37/PC3 */
    {GPIOC,   NULL, ADC1,  4, 0,   14}, /* D38/PC4 */
    {GPIOC,   NULL, ADC1,  5, 0,   15}, /* D39/PC5 */
    {GPIOC,   NULL, NULL,  6, 0, ADCx}, /* D40/PC6 */
    {GPIOC,   NULL, NULL,  7, 0, ADCx}, /* D41/PC7 */
    {GPIOC,   NULL, NULL,  8, 0, ADCx}, /* D42/PC8 */
    {GPIOC,   NULL, NULL,  9, 0, ADCx}, /* D43/PC9 */
    {GPIOC,   NULL, NULL, 10, 0, ADCx}, /* D44/PC10 */
    {GPIOC,   NULL, NULL, 11, 0, ADCx}, /* D45/PC11 */
    {GPIOC,   NULL, NULL, 12, 0, ADCx}, /* D46/PC12 */
    {GPIOB, TIMER4, NULL,  9, 4, ADCx}, /* D47/PB9 */
};

// Array of pins you can use for pwmWrite(). Keep it in Flash because
// it doesn't change, and so we don't waste RAM.
extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    3, 4, 5, 8, 9, 10, 11, 15, 16, 32, 33, 47 // OLD: 25, 26, 27
};

// Array of pins you can use for analogRead().
extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 33, 34, 35, 36, 37, 38, 39
};

//#define USB_DP 23
//#define USB_DM 24

// Array of pins that the board uses for something special. Other than
// the button and the LED, it's usually best to leave these pins alone
// unless you know what you're doing.
extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {

};
