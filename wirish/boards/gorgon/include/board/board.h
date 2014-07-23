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
 * @file   wirish/boards/gorgon/include/board/board.h
 * @author Eric Barch <ericb@ericbarch.com>
 * @brief  Gorgon board header.
 *
 * See wirish/boards/maple/include/board/board.h for more information
 * on these definitions.
 */

#ifndef _BOARD_GORGON_H_
#define _BOARD_GORGON_H_

/* 72 MHz -> 72 cycles per microsecond. */
#define CYCLES_PER_MICROSECOND    72
#define SYSTICK_RELOAD_VAL     71999 /* takes a cycle to reload */

/* Pin number for the built-in button. */
#define BOARD_BUTTON_PIN          -1

/* Pin number for the built-in LED. */
#define BOARD_LED_PIN             -1

/* Number of USARTs/UARTs whose pins are broken out to headers. */
#define BOARD_NR_USARTS           3

/* USART pin numbers. */
#define BOARD_USART1_TX_PIN       26
#define BOARD_USART1_RX_PIN       25
#define BOARD_USART2_TX_PIN       9
#define BOARD_USART2_RX_PIN       8
#define BOARD_USART3_TX_PIN       1
#define BOARD_USART3_RX_PIN       0

/* Number of SPI ports broken out to headers. */
#define BOARD_NR_SPI              2

/* SPI pin numbers. */
#define BOARD_SPI1_NSS_PIN        7
#define BOARD_SPI1_MOSI_PIN       4
#define BOARD_SPI1_MISO_PIN       5
#define BOARD_SPI1_SCK_PIN        6
#define BOARD_SPI2_NSS_PIN        31
#define BOARD_SPI2_MOSI_PIN       28
#define BOARD_SPI2_MISO_PIN       29
#define BOARD_SPI2_SCK_PIN        30

/* Total number of GPIO pins that are broken out to headers and
 * intended for use. This includes pins like the LED, button, and
 * debug port (JTAG/SWD) pins. */
#define BOARD_NR_GPIO_PINS        48

/* Number of pins capable of PWM output. */
#define BOARD_NR_PWM_PINS         12

/* Number of pins capable of ADC conversion. */
#define BOARD_NR_ADC_PINS         16

/* Number of pins already connected to external hardware.  For Maple,
 * these are just BOARD_LED_PIN, BOARD_BUTTON_PIN, and the debug port
 * pins (see below). */
#define BOARD_NR_USED_PINS         0

/* Debug port pins. */
#define BOARD_JTMS_SWDIO_PIN      22
#define BOARD_JTCK_SWCLK_PIN      21
#define BOARD_JTDI_PIN            20
#define BOARD_JTDO_PIN            19
#define BOARD_NJTRST_PIN          18

/* USB configuration.  BOARD_USB_DISC_DEV is the GPIO port containing
 * the USB_DISC pin, and BOARD_USB_DISC_BIT is that pin's bit. */
//#define BOARD_USB_DISC_DEV        GPIOB
//#define BOARD_USB_DISC_BIT        9

/* Pin aliases: these give the GPIO port/bit for each pin as an
 * enum. These are optional, but recommended. They make it easier to
 * write code using low-level GPIO functionality. */
enum {
    PB11, PB10, PB2, PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0, PC15, PC14,
    PC13, PB7, PB6, PB5, PB4, PB3, PA15, PA14, PA13, PA12, PA11, PA10, PA9,
    PA8, PB15, PB14, PB13, PB12, PB8, PB1, PC0, PC1, PC2, PC3, PC4, PC5, PC6,
    PC7, PC8, PC9, PC10, PC11, PC12, PB9
};

#endif
