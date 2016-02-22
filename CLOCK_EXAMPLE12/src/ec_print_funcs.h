/*****************************************************************************
 *
 * \file
 *
 * \brief Strings and integers print module for debug purposes.
 *
 * Copyright (c) 2009-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


#ifndef _EC_PRINT_FUNCS_H_
#define _EC_PRINT_FUNCS_H_

/**
 * \defgroup group_avr32_utils_print_funcs USART Debug strings
 *
 * This driver adds functionality to print debug strings and data through a dedicated USART. It supports writing both single
 * characters, strings, and integer values in both decimal and hexadecimal form.
 *
 * \{
 */

#include <avr32/io.h>
#include "board.h"


/*! \name USART Settings for the Debug Module
 */
//! @{
//! @}

/*
 * Debug UART (We can make this whatever we want, choosing: 115200, N, 8, 1)
 */

#  define ECDBG_USART               (&AVR32_USART0)
#  define ECDBG_USART_RX_PIN        AVR32_USART0_RXD_0_1_PIN
#  define ECDBG_USART_RX_FUNCTION   AVR32_USART0_RXD_0_1_FUNCTION
#  define ECDBG_USART_TX_PIN        AVR32_USART0_TXD_0_1_PIN
#  define ECDBG_USART_TX_FUNCTION   AVR32_USART0_TXD_0_1_FUNCTION
#  define ECDBG_USART_IRQ           AVR32_USART0_IRQ
#  define ECDBG_USART_BAUDRATE      115200

/*
 * Display UART: 2400, 1 (parity), 8 (data), 1 (stop)
 */

#  define DISPLAY_USART               (&AVR32_USART1)
#  define DISPLAY_USART_RX_PIN        AVR32_USART1_RXD_0_1_PIN
#  define DISPLAY_USART_RX_FUNCTION   AVR32_USART1_RXD_0_1_FUNCTION
#  define DISPLAY_USART_TX_PIN        AVR32_USART1_TXD_0_1_PIN
#  define DISPLAY_USART_TX_FUNCTION   AVR32_USART1_TXD_0_1_FUNCTION
#  define DISPLAY_USART_IRQ           AVR32_USART1_IRQ
#  define DISPLAY_USART_BAUDRATE      4800 //fudge 9apr15 TODO fix this the actual baud rate is 2400, for some reason had to put this to 4800 to get 2400




/*! \name VT100 Common Commands
 */
//! @{
#define CLEARSCR          "\x1B[2J\x1B[;H"    //!< Clear screen.
#define CLEAREOL          "\x1B[K"            //!< Clear end of line.
#define CLEAREOS          "\x1B[J"            //!< Clear end of screen.
#define CLEARLCR          "\x1B[0K"           //!< Clear line cursor right.
#define CLEARLCL          "\x1B[1K"           //!< Clear line cursor left.
#define CLEARELN          "\x1B[2K"           //!< Clear entire line.
#define CLEARCDW          "\x1B[0J"           //!< Clear cursor down.
#define CLEARCUP          "\x1B[1J"           //!< Clear cursor up.
#define GOTOYX            "\x1B[%.2d;%.2dH"   //!< Set cursor to (y, x).
#define INSERTMOD         "\x1B[4h"           //!< Insert mode.
#define OVERWRITEMOD      "\x1B[4l"           //!< Overwrite mode.
#define DELAFCURSOR       "\x1B[K"            //!< Erase from cursor to end of line.
#define CRLF              "\r\n"              //!< Carriage Return + Line Feed.
//! @}

/*! \name VT100 Cursor Commands
 */
//! @{
#define CURSON            "\x1B[?25h"         //!< Show cursor.
#define CURSOFF           "\x1B[?25l"         //!< Hide cursor.
//! @}

/*! \name VT100 Character Commands
 */
//! @{
#define NORMAL            "\x1B[0m"           //!< Normal.
#define BOLD              "\x1B[1m"           //!< Bold.
#define UNDERLINE         "\x1B[4m"           //!< Underline.
#define BLINKING          "\x1B[5m"           //!< Blink.
#define INVVIDEO          "\x1B[7m"           //!< Inverse video.
//! @}

/*! \name VT100 Color Commands
 */
//! @{
#define CL_BLACK          "\033[22;30m"       //!< Black.
#define CL_RED            "\033[22;31m"       //!< Red.
#define CL_GREEN          "\033[22;32m"       //!< Green.
#define CL_BROWN          "\033[22;33m"       //!< Brown.
#define CL_BLUE           "\033[22;34m"       //!< Blue.
#define CL_MAGENTA        "\033[22;35m"       //!< Magenta.
#define CL_CYAN           "\033[22;36m"       //!< Cyan.
#define CL_GRAY           "\033[22;37m"       //!< Gray.
#define CL_DARKGRAY       "\033[01;30m"       //!< Dark gray.
#define CL_LIGHTRED       "\033[01;31m"       //!< Light red.
#define CL_LIGHTGREEN     "\033[01;32m"       //!< Light green.
#define CL_YELLOW         "\033[01;33m"       //!< Yellow.
#define CL_LIGHTBLUE      "\033[01;34m"       //!< Light blue.
#define CL_LIGHTMAGENTA   "\033[01;35m"       //!< Light magenta.
#define CL_LIGHTCYAN      "\033[01;36m"       //!< Light cyan.
#define CL_WHITE          "\033[01;37m"       //!< White.
//! @}


/*! \brief Sets up DBG_USART with 8N1 at DBG_USART_BAUDRATE.
 *
 * \param pba_hz PBA clock frequency (Hz).
 */
extern void init_ecdbg_rs232(long pba_hz);
extern void init_display_rs232(long pba_hz);


/*! \brief Sets up DBG_USART with 8N1 at a given baud rate.
 *
 * \param baudrate Baud rate to set DBG_USART to.
 * \param pba_hz PBA clock frequency (Hz).
 */
extern void init_ecdbg_rs232_ex(unsigned long baudrate, long pba_hz);
extern void init_display_rs232_ex(unsigned long baudrate, long pba_hz);

/*! \brief Prints a string of characters to DBG_USART.
 *
 * \param str The string of characters to print.
 */
extern void print_ecdbg(const char *str);
extern void print_display(const char *str);

/*! \brief Prints a character to DBG_USART.
 *
 * \param c The character to print.
 */
extern void print_ecdbg_char(int c);
extern void print_display_char(int c);

/*! \brief Prints an integer to DBG_USART in a decimal representation.
 *
 * \param n The integer to print.
 */
extern void print_ecdbg_ulong(unsigned long n);
extern void print_display_ulong(unsigned long n);

/*! \brief Prints a char to DBG_USART in an hexadecimal representation.
 *
 * \param n The char to print.
 */
extern void print_ecdbg_char_hex(unsigned char n);
extern void print_display_char_hex(unsigned char n);

/*! \brief Prints a short integer to DBG_USART in an hexadecimal representation.
 *
 * \param n The short integer to print.
 */
extern void print_ecdbg_short_hex(unsigned short n);
extern void print_display_short_hex(unsigned short n);

/*! \brief Prints an integer to DBG_USART in an hexadecimal representation.
 *
 * \param n The integer to print.
 */
extern void print_dbg_hex(unsigned long n);
extern void print_dbg_hex(unsigned long n);

/*! \brief Prints a string of characters to a given USART.
 *
 * \param usart Base address of the USART instance to print to.
 * \param str The string of characters to print.
 */
extern void print(volatile avr32_usart_t *usart, const char *str);

/*! \brief Prints a character to a given USART.
 *
 * \param usart Base address of the USART instance to print to.
 * \param c The character to print.
 */
extern void print_char(volatile avr32_usart_t *usart, int c);

/*! \brief Prints an integer to a given USART in a decimal representation.
 *
 * \param usart Base address of the USART instance to print to.
 * \param n The integer to print.
 */
extern void print_ulong(volatile avr32_usart_t *usart, unsigned long n);

/*! \brief Prints a char to a given USART in an hexadecimal representation.
 *
 * \param usart Base address of the USART instance to print to.
 * \param n The char to print.
 */
extern void print_char_hex(volatile avr32_usart_t *usart, unsigned char n);

/*! \brief Prints a short integer to a given USART in an hexadecimal
 *         representation.
 *
 * \param usart Base address of the USART instance to print to.
 * \param n The short integer to print.
 */
extern void print_short_hex(volatile avr32_usart_t *usart, unsigned short n);

/*! \brief Prints an integer to a given USART in an hexadecimal representation.
 *
 * \param usart Base address of the USART instance to print to.
 * \param n The integer to print.
 */
extern void print_hex(volatile avr32_usart_t *usart, unsigned long n);

/**
 * \}
 */

#endif  // _EC_PRINT_FUNCS_H_