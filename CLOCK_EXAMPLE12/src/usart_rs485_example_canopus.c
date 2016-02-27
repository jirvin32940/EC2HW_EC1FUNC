/**
 * \file
 *
 * \brief USART RS485 example for SAM.
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

/**
 *  \mainpage USART RS485 Example with PDC
 *
 *  \par Purpose
 *
 *  The USART RS485 Example demonstrates how to use USART in RS485
 *  mode.
 *
 *  \par Requirements
 *
 *  This package can be used with same70_xplained_pro/samv71_xplained_ulta boards. Before running, make sure
 *  to connect two boards with RS485 lines. Match each paired pins of two
 *  boards respectively with A to A, B to B and FGND to FGND.
 *
 *  Please refer to the board user guide for the details of RS485 jumper
 *  settings.
 *
 *  \par Description
 *
 *  This example connects two boards through RS485 interface. One board acts
 *  as the transmitter and the other one as the receiver. It is determined by
 *  the sequence that the two applications started.
 *
 *  In any case, the application sends a sync character at running to seek a
 *  receiver. If the acknowledgement is received in a short time, it will act
 *  as the transmitter and then send a full frame text to the receiver.
 *
 *  The earlier started board will act automatically as the receiver due to no
 *  acknowledgement received. The receiver will wait until sync character is
 *  received. Then it sends the acknowledgement and waits for the full frame
 *  sent by the transmitter. At the end of reception, it prints out message
 *  through UART interface to assert that the whole process succeeds.
 *
 *  \par Usage
 *
 *  -# Build the program and download it into the two evaluation boards.
 *  -# Connect a serial cable to the UART port for each evaluation kit.
 *  -# On the computer, open and configure a terminal application for each board
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start application from two boards in sequence. Make sure the second board
 *     should NOT be started unless the first board has run to wait for the
 *     synchronizing character. The output message in later section would
 *     describe this.
 *
 *  -# In the terminal window, the following text should appear:
 *     \code
	-- USART RS485 Example --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *  -# The consequent messages will indicate the boards' behavior.
 *
 *     -  The earlier started board will output the message below to indicate it
 *     is waiting for a synchronization character:
 *     \code
	-I- Receiving sync character.
\endcode
 *     -  If it receives a sync character and prepare to receive a frame, it
 *     will print out the message below:
 *     \code
	-I- Start receiving!
\endcode
 *     -  After successfully receives a frame, the board will output the
 *     following message to indicate that the whole process succeeds.
 *     \code
	-I- Received successfully!
\endcode
 *     -  The later started one will act as transmitter, and if it receives an
 *     acknowledgement character successfully, it will output the following
 *     message and start transmitting:
 *     \code
	-I- Start transmitting!
\endcode
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <string.h>
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"
#include "pca9952.h"
#include "serial_id_ds2411.h"
#include "afec.h"
#include "timer.h"


/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE         2000

/** Size of the buffer. */
#define PDC_BUF_SIZE        BUFFER_SIZE

/** Acknowledge time out. */
#define TIMEOUT             (1000)

/** Character to synchronize with the other end. */
#define SYNC_CHAR            0x11

/** Character to acknowledge receipt of the sync char. */
#define ACK_CHAR             0x13

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK   0xffffffff

/** System tick frequency in Hz. */
#define SYS_TICK_FREQ        1000

#define STRING_EOL    "\r"
#define STRING_HEADER "-- USART RS485 Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** State of the USART. */
typedef enum st_usart_state {
	INITIALIZED,
	TRANSMITTING,
	RECEIVING,
	RECEIVED,
	TRANSMITTED
} usart_state_t;

/** Global usart state. */
volatile usart_state_t g_state = INITIALIZED;

/** Tick Counter in unit of ms. */
volatile uint32_t g_ul_tick_count;

/** Transmit buffer. Pure ASCII text. */
uint8_t g_uc_transmit_buffer[BUFFER_SIZE] = "DESCRIPTION of this example: \r\n \
 **************************************************************************\r\n\
 *  This application gives an example of how to use USART in RS485 mode.\r\n\
 *  RS-485 is a standard defining the electrical characteristics of drivers \r\n\
 *  and receivers for use in balanced digital multipoint systems. The standard \r\n\
 *  is published by the ANSI TIA/EIA. \r\n\
 *  \r\n\
 *  This example connects two boards through RS485 interface. One board acts \r\n\
 *  as the transmitter and the other one as the receiver. It is determined by \r\n\
 *  the sequence the two applications started. The earlier started board will \r\n\
 *  act automatically as the receiver due to no acknowledgement received. The \r\n\
 *  receiver will wait until sync character is received. Then it sends the \r\n\
 *  acknowledgement and waits for the full frame sent by the transmitter. \r\n\
 *  At the end of reception, it prints out message through UART interface to \r\n\
 *  assert that the whole process succeeds.\r\n\
 **************************************************************************\r\n\
 END of DESCRIPTION \r\n\
 ";

/** Receive buffer. */
uint8_t g_uc_receive_buffer[BUFFER_SIZE];

/** Pointer to receive buffer base. */
uint8_t *p_revdata = &g_uc_receive_buffer[0];
/** count number for received data. */
uint32_t g_ulcount = 0;


/* Global ul_ms_ticks in milliseconds since start of application */
volatile uint32_t ul_ms_ticks = 0;

/**
 * \brief Wait for the given number of microseconds (using the ul_ms_ticks generated
 * by the SAM microcontroller system tick).
 *
 * \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = ul_ms_ticks;
	while ((ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks) {
	}
}



struct CONTROLS controls;

struct {

	uint8_t doorsw1;
	uint8_t doorsw2;

	uint8_t last_doorsw1;
	uint8_t last_doorsw2;	

	uint8_t col3;
	uint8_t col2;
	uint8_t col1;
	uint8_t row3;
	uint8_t row2;
	uint8_t row1;
		
	uint8_t last_col3;
	uint8_t last_col2;
	uint8_t last_col1;
	uint8_t last_row3;
	uint8_t last_row2;
	uint8_t last_row1;
	uint8_t keypad;
	
}status;

void toggle(uint8_t *var)
{
	*var  = (((*var)+1) & 1);
}



/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event.
 *  Increment the ul_ms_ticks counter.
 */
void SysTick_Handler(void)
{
	g_ul_tick_count++;
	ul_ms_ticks++; //jsi 6feb16
	
	timerTickCount++;
	timerTickCount &= MAX_TICK; // force rollover at this count to avoid confusion detecting rollover with the MSbit set
	if (timerTickCount == 0)
	{
		rollover = 1;
	}
	
	if ((timerTickCount % TICKS_PER_SEC) == 0)
	{
		process_timers();
	}

	
	if (controls.buzzer_enable)
	{
		if (controls.buzzer_cycle == CYCLE_ON)
		{
			if (controls.buzzer_dur_count++ > controls.buzzer_on_dur)
			{
				if (controls.buzzer_repeat_count++ >= controls.buzzer_repeat)
				{
					controls.buzzer_enable = 0;
					pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
				}
				else
				{
					controls.buzzer_dur_count = 0;
					controls.buzzer_cycle = CYCLE_OFF;
					pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
				}
			}
		}
		else
		{
			if (controls.buzzer_dur_count++ > controls.buzzer_off_dur)
			{
				controls.buzzer_dur_count = 0;
				controls.buzzer_cycle = CYCLE_ON;
				pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL);
			}
		}
	}
	
	if (controls.solenoid_enable)
	{
		controls.solenoid_count++;
		
		if (controls.solenoid_cycle == CYCLE_ON)
		{
			if (controls.solenoid_count > SOLENOID_ON_COUNT)
			{
				controls.solenoid_count = 0;
				controls.solenoid_cycle = CYCLE_OFF;
				controls.solenoid_enable = 0; //solenoid is a one-shot
				ioport_set_pin_level(ECLAVE_SOLENOID, IOPORT_PIN_LEVEL_LOW);
			}
		}
		else
		{
			if (controls.solenoid_count > SOLENOID_OFF_COUNT)
			{
				controls.solenoid_count = 0;
				controls.solenoid_cycle = CYCLE_ON;
				ioport_set_pin_level(ECLAVE_SOLENOID, IOPORT_PIN_LEVEL_HIGH);
			}
			
		}
	}
}

/**
 *  \brief Get the tick count value.
 *
 */
static uint32_t get_tick_count(void)
{
	return g_ul_tick_count;
}

/**
 *  \brief Wait for some time in ms.
 *
 */
static void wait(volatile uint32_t ul_ms)
{
	uint32_t ul_start;
	uint32_t ul_current;

	ul_start = g_ul_tick_count;
	do {
		ul_current = g_ul_tick_count;
	} while (ul_current - ul_start < ul_ms);
}

/**
 *  \brief Handler for USART interrupt.
 *
 */
void USART_Handler(void)
{
	uint32_t ul_status;
	uint8_t uc_char;

	/* Read USART status. */
	ul_status = usart_get_status(BOARD_USART);

	/*transmit interrupt rises*/
	if(ul_status & (US_IER_TXRDY | US_IER_TXEMPTY)) {
		usart_disable_interrupt(BOARD_USART, (US_IER_TXRDY | US_IER_TXEMPTY));
	}

	/*receive interrupt rise, store character to receiver buffer*/
	if((g_state == RECEIVING) && (usart_read(BOARD_USART, (uint32_t *)&uc_char) == 0)) {
		*p_revdata++ = uc_char;
		g_ulcount++;
		if(g_ulcount >= BUFFER_SIZE) {
			g_state = RECEIVED;
			usart_disable_interrupt(BOARD_USART, US_IER_RXRDY);
		}
	}
}

/**
 *  \brief USART RS485 mode configuration.
 *
 *  Configure USART in RS485 mode, asynchronous, 8 bits, 1 stop bit,
 *  no parity, 256000 bauds and enable its transmitter and receiver.
 */
void configure_usart(void)
{
	const sam_usart_opt_t usart_console_settings = {
		BOARD_USART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		/* This field is only used in IrDA mode. */
		0
	};

	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(BOARD_ID_USART);

	/* Configure USART in RS485 mode. */
//jsi 7feb16 we want rs232 not rs485 for our application	usart_init_rs485(BOARD_USART, &usart_console_settings,
//jsi 7feb16 we want rs232 not rs485 for our application			sysclk_get_cpu_hz());
			
	usart_init_rs232(BOARD_USART, &usart_console_settings, sysclk_get_cpu_hz());

	/* enable transmitter timeguard, 4 bit period delay. */
	usart_set_tx_timeguard(BOARD_USART, 4);

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);

	/* Enable TX & RX function. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(USART_IRQn);
}

/**
 *  Configure system tick to generate an interrupt every 1us. Note that this was 1ms in the example code. jsi 11feb16
 */
void configure_systick(void)
{
	uint32_t ul_flag;

	ul_flag = SysTick_Config(sysclk_get_cpu_hz()/SYS_TICK_FREQ);
	if (ul_flag) {
		puts("-F- Systick configuration error\r");
		while (1) {
		}
	}
}

/**
 *  Configure UART for debug message output.
 */
void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
* \brief transmit data.
*
* \param *p_buff  data to be transmitted
* \param ulsize size of all data.
*
*/
uint8_t func_transmit(const uint8_t *p_buff, uint32_t ulsize)
{
	Assert(p_buff);

	while(ulsize > 0) {
		if(0 == usart_write(BOARD_USART, *p_buff)){
			usart_enable_interrupt(BOARD_USART, US_IER_TXRDY | US_IER_TXEMPTY);
			ulsize--;
			p_buff++;
		}
	}

	while(!usart_is_tx_empty(BOARD_USART)) {
		;  /*waiting for transmit over*/
	}

	return 0;
}

/**
 * \brief Dump buffer to uart.
 *
 */
static void dump_info(char *p_buf, uint32_t ul_size)
{
	uint32_t ul_i = 0;

	while ((ul_i < ul_size) && (p_buf[ul_i] != 0)) {
		printf("%c", p_buf[ul_i++]);
	}
}


/*
 * BELOW: Carry over from EC GEN I code 31jan16 Modified for EC Gen II
 */

	
enum {
	IDX_READY,
	IDX_CLEAN,
	IDX_CLEANING,
	IDX_DIRTY,
	IDX_ERROR,
	IDX_SHELF1,
	IDX_SHELF2,
	IDX_SHELF3,
	IDX_SHELF4,
	IDX_CLEAR
	
};


volatile U16 adc_current_conversion;


//TBD different scheme now #define EC_DOOR_LATCHED (!gpio_get_pin_value(ECLAVE_DOOR_LATCH)) //12apr15 this is the correct sense for the equipment going to the show


enum {
	SHELF_INACTIVE,
	SHELF_ACTIVE
};

void twi_init(void);
void twi_init(void)
{
	twihs_options_t opt;

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(ID_TWIHS0);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = TWIHS_CLK; //400KHz

	if (twihs_master_init(TWIHS0, &opt) != TWIHS_SUCCESS) {
		while (1) {
			/* Capture error */
		}
	}
}




/*
 * ABOVE: Carry over from EC GEN I code 31jan16 Modified for EC Gen II
 */


/*
 * Analog conversion...Bluesense0..3
 */

/** The conversion data is done flag */

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
#define MAX_DIGITAL     (4095UL)



uint32_t ul_vol;

bool is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value[4] = {0, 0, 0, 0};
volatile uint32_t g_ul_last_value[4] = {0, 0, 0, 0};


uint8_t afecSel[4] = {AFEC1, AFEC0, AFEC1, AFEC1};
uint8_t adcCh[4] = 	{AFEC_CHANNEL_9, AFEC_CHANNEL_4, AFEC_CHANNEL_4, AFEC_CHANNEL_5};

/**
 * \brief AFEC interrupt callback function.
 */

/**
 * \brief AFEC0 DRDY interrupt callback function.
 */

uint32_t g_afec0_sample_data;
uint32_t g_afec1_sample_data;

static void afec0_data_ready(void)
{
	g_afec0_sample_data = afec_get_latest_value(AFEC0);
	is_conversion_done = true;
}

static void afec1_data_ready(void)
{
	g_afec1_sample_data = afec_get_latest_value(AFEC1);
	is_conversion_done = true;
}



void init_adc(void)
{
	struct afec_config afec_cfg;
	struct afec_ch_config afec_ch_cfg;

	
	afec_enable(AFEC0);
	afec_enable(AFEC1);

	afec_get_config_defaults(&afec_cfg);
	afec_cfg.resolution = AFEC_12_BITS;
	afec_init(AFEC0, &afec_cfg);
	afec_init(AFEC1, &afec_cfg);
	
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_3;
	
	afec_ch_set_config(AFEC1, AFEC_CHANNEL_9, &afec_ch_cfg);
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_4, &afec_ch_cfg);
	afec_ch_set_config(AFEC1, AFEC_CHANNEL_4, &afec_ch_cfg);
	afec_ch_set_config(AFEC1, AFEC_CHANNEL_5, &afec_ch_cfg);

	afec_set_trigger(AFEC0, AFEC_TRIG_SW);
	afec_set_trigger(AFEC1, AFEC_TRIG_SW);

	afec_set_callback(AFEC0, AFEC_INTERRUPT_DATA_READY, afec0_data_ready, 1);
	afec_set_callback(AFEC1, AFEC_INTERRUPT_DATA_READY, afec1_data_ready, 1);

// Got this calibration code from a SAM4E example, supposedly code compatible with the SAME70, but this doesn't build
//	afec_start_calibration(AFEC0);
//	while((afec_get_interrupt_status(AFEC0) & AFEC_ISR_EOCAL) != AFEC_ISR_EOCAL);
	
//	afec_start_calibration(AFEC1);
//	while((afec_get_interrupt_status(AFEC1) & AFEC_ISR_EOCAL) != AFEC_ISR_EOCAL);
	
}


/*
 * PWM stuff for the buzzer
 */

/** PWM frequency in Hz */
#define PWM_FREQUENCY      2000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100 //jsi 15feb16 what should this be? is this related to pwm frequency somehow?
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    (PERIOD_VALUE/2)

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;


/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM0_Handler(void)
{
	static uint32_t ul_count = 0;  /* PWM counter value */
	static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */
	static uint8_t fade_in = 1;  /* LED fade in flag */

	uint32_t events = pwm_channel_get_interrupt_status(PWM0);

	/* Interrupt on PIN_PWM_LED0_CHANNEL */
	if ((events & (1 << PIN_PWM_LED0_CHANNEL)) ==
	(1 << PIN_PWM_LED0_CHANNEL)) {
		ul_count++;

		/* Fade in/out */
		if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
			/* Fade in */
			if (fade_in) {
				ul_duty++;
				if (ul_duty == PERIOD_VALUE) {
					fade_in = 0;
					}
				} else {
				/* Fade out */
				ul_duty--;
				if (ul_duty == INIT_DUTY_VALUE) {
					fade_in = 1;
				}
			}

			/* Set new duty cycle */
			ul_count = 0;
			g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
//jsi 16feb16			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, ul_duty);
					pwm_channel_update_duty(PWM0, &g_pwm_channel_led, (PERIOD_VALUE/2)); //jsi 16feb16 just fixed for now
//jsi 15feb16			g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
//jsi 15feb16			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, ul_duty);
		}
	}
}



void init_pwm(void)
{
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
	pwm_channel_disable(PWM0, PIN_PWM_LED1_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
	pwm_channel_init(PWM0, &g_pwm_channel_led);

	/* Enable channel counter event interrupt */
	pwm_channel_enable_interrupt(PWM0, PIN_PWM_LED0_CHANNEL, 0);

	/* Initialize PWM channel for LED1 */
	/* Period is center-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a high level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
	pwm_channel_init(PWM0, &g_pwm_channel_led);

	/* Disable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM0, PIN_PWM_LED1_CHANNEL, 0);

	/* Configure interrupt and enable PWM interrupt */
	NVIC_DisableIRQ(PWM0_IRQn);
	NVIC_ClearPendingIRQ(PWM0_IRQn);
	NVIC_SetPriority(PWM0_IRQn, 0);
	NVIC_EnableIRQ(PWM0_IRQn);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL);
//jsi 15feb16	pwm_channel_enable(PWM0, PIN_PWM_LED1_CHANNEL);

}


/**
 *  \brief usart_rs485 Application entry point.
 *
 *  Configure USART in RS485 mode. If the application starts earlier, it acts
 *  as a receiver. Otherwise, it should be a transmitter.
 *
 *  \return Unused (ANSI-C compatibility).
 */

#  define EXAMPLE_LED_GPIO    LED0_GPIO

