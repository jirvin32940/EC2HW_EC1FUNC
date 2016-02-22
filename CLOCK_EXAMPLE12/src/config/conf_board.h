/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/** Definition of TWI interrupt ID on board. */
#define BOARD_TWIHS_IRQn          TWIHS2_IRQn
#define BOARD_TWIHS_Handler    TWIHS2_Handler

/** Configure TWI2 pins */
#define CONF_BOARD_TWIHS0

/** Enable Com Port. */
#define CONF_BOARD_UART_CONSOLE

/* USART0 module is used in SYNC. mode. */
#define CONF_BOARD_USART_RXD
#define CONF_BOARD_USART_TXD
#define CONF_BOARD_USART_RTS
//#define CONF_BOARD_USART_CTS

/** Configure PWM pins used to control LEDs on board */
#define CONF_BOARD_PWM_LED0
//jsi 15feb16 i don't think we want this #define CONF_BOARD_PWM_LED1

#define ECLAVE_PSUPPLY_ONn	PIO_PA2_IDX
#define ECLAVE_LED_OEn		PIO_PA1_IDX
#define ECLAVE_MFP			PIO_PA0_IDX	//set to 1 for 1X, set to 0 for 4X
#define ECLAVE_SOLENOID		PIO_PA21_IDX
#define ECLAVE_DOORSW1		PIO_PC13_IDX
#define ECLAVE_DOORSW2		PIO_PC14_IDX
#define ECLAVE_COL3			PIO_PC17_IDX
#define ECLAVE_COL2			PIO_PC18_IDX
#define ECLAVE_COL1			PIO_PC19_IDX
#define ECLAVE_ROW3			PIO_PC20_IDX
#define ECLAVE_ROW2			PIO_PC21_IDX
#define ECLAVE_ROW1			PIO_PC22_IDX


#endif /* CONF_BOARD_H_INCLUDED */
