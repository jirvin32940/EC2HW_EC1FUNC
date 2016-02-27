/**
 * \file
 *
 * \brief Seal Shield Electroclave device sanitizer
 * based on Atmel's AVR UC3C CAN-LIN Loopback Demo and Atmel SAME70 RS485 and many other demos
 *
 * Copyright (c) 2011-2014 Atmel Corporation. All rights reserved.
 *
 * Copyright (c) 2015 Seal Shield. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *TMR
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
/*! \mainpage
 *
 * \section compilinfo Compilation Information
 * This software is written for GNU GCC for AVR32 and for IAR Embedded Workbench
 * for Atmel AVR32. Other compilers may or may not work.
 *
 * \section deviceinfo Device Information
 * All AVR32 AT32UC3C devices can be used.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#if 0 //these includes are from the Electroclave Gen I hardware project
#include "compiler.h"
#include "power_clocks_lib.h"
#include "gpio.h"
#include "ec_print_funcs.h" //8apr15 changed from print_funcs.h
#include "flashc.h"
#include "adcifa.h"
#include "twim.h"
#include "conf_pca9952.h" //6apr15
#include "pca9952.h" //7apr15
#include "conf_eclave.h"	//8apr15
#include "cycle_counter.h"		//8apr15	
#include "usart.h"				//9apr15
#include "serial_id_ds2411.h"	//9apr15
#include "flashc.h"				//2may15
#include "string.h"
#include "stdio.h"
#include "ctype.h"
#include "sysclk.h"
#include <pll.h>
#endif

#include <string.h>
#include <stdio.h>
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"
#include "pca9952.h"
#include "serial_id_ds2411.h"
#include "afec.h"
#include "ec_print_funcs.h" //8apr15 changed from print_funcs.h
#include "timer.h"
#include "ctype.h"
#include "arm_math.h"
#define USART_FAILURE                -1 //!< Failure because of some unspecified reason.


extern void mdelay(uint32_t ul_dly_ticks);
extern void udelay(uint32_t ul_dly_ticks);
extern void configure_console(void);
extern void configure_usart(void);
extern void configure_systick(void);
extern void init_pwm(void);
extern void twi_init(void);
extern void init_adc(void);

void button_beep_buzzer(void);
void print_ecdbg_num(unsigned int num);

unsigned char read_usage_struct(unsigned char sel);
unsigned char test_flash(unsigned char sel);

void add_new_led_board_sides_to_usage(void);

unsigned char calc_usage_csum(unsigned char sel);
void copy_usage_to_usage(unsigned char dst, unsigned char src);
void write_usage_to_flash(unsigned char sel);

void load_usage_indeces(void);

extern struct CONTROLS controls;

unsigned char minute_count(unsigned char * pMinuteBits);
void reset_minutes(unsigned char * pMinuteBits);
unsigned char inc_minutes(unsigned char * pMinuteBits);

unsigned char firstDoorOpenSinceIdle = 1;
unsigned char firstTimeThroughPCA9952 = 1;


unsigned char minPingPong; //used to toggle between 2 buffers each minute
unsigned char hourPingPong; //used to toggle between 2 regions of flash each hour

#define NO_LED_BOARD_PRESENT 0xFF
#define NUM_LED_BOARDS			5
#define NUM_LED_BOARD_SIDES		8
#define NUM_SHELVES				4	//Shelf 0 is board 0 bottom + board 1 top
									//Shelf 1 is board 1 bottom + board 2 top
									//Shelf 2 is board 2 bottom + board 3 top
									//Shelf 3 is board 3 bottom + board 4 top
									//Board 0 upper side and board 4 lower side are not used

unsigned char usageIdx[NUM_LED_BOARD_SIDES];

uint8_t scanKPResult = 0;

enum { BOTTOM, TOP};


typedef struct {
	
	unsigned char active;
	unsigned char tLedIdx;
	unsigned char bLedIdx;
	unsigned char devicesPresent;
	unsigned char present;

} SHELF;


typedef struct {
	
	unsigned char idFamily;
	unsigned char id[6];
	unsigned char idcsum;
	unsigned char present;
	unsigned char shelfIdx;
	unsigned char uSideIdx;
	unsigned char lSideIdx;
	unsigned char uSideShelfIdx;
	unsigned char lSideShelfIdx;
	
} LEDBRD;
	
typedef struct {
	
	unsigned char sanitizeMinutes;
	unsigned char ushdwIdx;
	unsigned char maxUsageReached;
	unsigned char shelfIdx;
	unsigned char boardIdx;
	
} LEDBRDSIDE;		


SHELF shelf[NUM_SHELVES];
LEDBRD ledBrd[NUM_LED_BOARDS];
LEDBRDSIDE ledBrdSide[NUM_LED_BOARD_SIDES];

unsigned long sanitizeMinutes;
unsigned long tmpSanitizeMinutes;
unsigned long displayTimerSeconds = 8;
unsigned char firstTimeSinceDoorLatched = 0;


enum {
	STATE_EC_IDLE,
	STATE_DOOR_OPEN,
	STATE_DOOR_AJAR,
	STATE_DOOR_LATCHED,
	STATE_START_SANITIZE,
	STATE_SANITIZE,
	STATE_START_CLEAN,
	STATE_CLEAN,
	STATE_CHASSIS_ERROR,
	STATE_SHUTDOWN_PROCESSES
};


/*
 * process_kpb() states
 */

enum {
	
	KPB_START,
	KPB_DIG1,
	KPB_DIG2,
	KPB_DIG3,
	KPB_DIG4	
};

/*
 * process_kpb() returns
 */
enum {
	
	KPB_CONTINUE,
	KPB_VALID,
	KPB_ERROR
};


/*
 * Bits packed ROW3, 2, 1, COL2, 1.
 * For example, if ROW3..1 is 0b111 and COL2..1 is 0b01 the packed bits are 0x1D and we recognize that as SW1
 */

#define KEYPAD_START	0x0D
#define KEYPAD_SW1		0x15
#define KEYPAD_SW2		0x19
#define KEYPAD_SW3		0x0E
#define KEYPAD_SW4		0x16
#define KEYPAD_SW5		0x1A

uint8_t scan_keypad(void)
{
	uint8_t tempKeypad, retKPB, col1, col2, col3, row1, row2, row3;
	static uint8_t last_retKPB = 0, repeatCountSetting = 5;
	static uint32_t repeatCount = 0;
	
	ioport_set_pin_dir(ECLAVE_COL3, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ECLAVE_COL2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ECLAVE_COL1, IOPORT_DIR_OUTPUT);

	ioport_set_pin_dir(ECLAVE_ROW3, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ECLAVE_ROW2, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ECLAVE_ROW1, IOPORT_DIR_INPUT);
	
	ioport_set_pin_level(ECLAVE_COL3, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(ECLAVE_COL2, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(ECLAVE_COL1, IOPORT_PIN_LEVEL_LOW);

	row3 = ioport_get_pin_level(ECLAVE_ROW3);
	row2 = ioport_get_pin_level(ECLAVE_ROW2);
	row1 = ioport_get_pin_level(ECLAVE_ROW1);


	ioport_set_pin_dir(ECLAVE_ROW3, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ECLAVE_ROW2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ECLAVE_ROW1, IOPORT_DIR_OUTPUT);

	ioport_set_pin_dir(ECLAVE_COL3, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ECLAVE_COL2, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ECLAVE_COL1, IOPORT_DIR_INPUT);
	
	ioport_set_pin_level(ECLAVE_ROW3, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(ECLAVE_ROW2, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(ECLAVE_ROW1, IOPORT_PIN_LEVEL_LOW);

	col3 = ioport_get_pin_level(ECLAVE_COL3);
	col2 = ioport_get_pin_level(ECLAVE_COL2);
	col1 = ioport_get_pin_level(ECLAVE_COL1);
	
	tempKeypad = ((row3 << 4) |
					(row2 << 3) |
					(row1 << 2) |
					(col2 << 1) |
					(col1));
					
	switch(tempKeypad)
	{
		case KEYPAD_START:
		case KEYPAD_SW1:
		case KEYPAD_SW2:
		case KEYPAD_SW3:
		case KEYPAD_SW4:
		case KEYPAD_SW5:
			if (last_retKPB == tempKeypad)
			{
				if (repeatCount++ == repeatCountSetting)
				{
					button_beep_buzzer();
					
					print_ecdbg("+--------+\r\n");
					switch (tempKeypad)
					{
						case KEYPAD_START:
							print_ecdbg("KPB START\r\n");
							break;
						case KEYPAD_SW1:
							print_ecdbg("KPB SW1\r\n");
							break;
						case KEYPAD_SW2:
							print_ecdbg("KPB SW2\r\n");
							break;
						case KEYPAD_SW3:
							print_ecdbg("KPB SW3\r\n");
							break;
						case KEYPAD_SW4:
							print_ecdbg("KPB SW4\r\n");
							break;
						case KEYPAD_SW5:
							print_ecdbg("KPB SW5\r\n");
							break;
					} //switch (tempKeypad)

					print_ecdbg("repeatCount: ");
					print_ecdbg_num(repeatCount);
					print_ecdbg("\r\n");

					last_retKPB = tempKeypad;
					scanKPResult = tempKeypad;
					return tempKeypad;
				}//if (repeatCount++ == 5)
		} //if (last_retKPB == tempKeypad)
		else {
			
			print_ecdbg("last_retKPB: ");
			print_ecdbg_num(last_retKPB);
			print_ecdbg(" tempKeypad: ");
			print_ecdbg_num(tempKeypad);
			print_ecdbg("\r\n");
			
			last_retKPB = tempKeypad;
			repeatCount = 0;	
		}
		break;
		
	} //switch (tempKeypad)

	return 0;
}


uint8_t kpbState = KPB_START;

uint8_t kpbValidCodes[4] = {KEYPAD_SW1, KEYPAD_SW2, KEYPAD_SW3, KEYPAD_SW4}; //just one valid code for now

uint8_t process_kpb(void)
{
	uint8_t kpb;
	
	if (kpbState > KPB_START)
	{
		if (timer_done(TMR_KEYPAD))
		{
			print_ecdbg("process_kpb() timeout (error)\r\n");
			end_timer(TMR_KEYPAD); //make sure everything is reset for this timer
			kpbState = KPB_START;
			return KPB_ERROR;
		}
	}
	
	if ((kpb = scan_keypad()) != 0)
	{
		switch(kpbState)
		{
			case KPB_START:
				if (kpbValidCodes[0] == kpb)
				{
					start_timer(TMR_KEYPAD, (15 * SECONDS));
					kpbState = KPB_DIG1;
					print_ecdbg("KPB State: DIG1\r\n");
					return KPB_CONTINUE;
				}
				break;
			case KPB_DIG1:
				if (kpbValidCodes[1] == kpb)
				{
					kpbState = KPB_DIG2;
					print_ecdbg("KPB State: DIG2\r\n");
					return KPB_CONTINUE;
				}
				break;
			case KPB_DIG2:
				if (kpbValidCodes[2] == kpb)
				{
					kpbState = KPB_DIG3;
					print_ecdbg("KPB State: DIG3\r\n");
					return KPB_CONTINUE;
				}
				break;
			case KPB_DIG3:
				if (kpbValidCodes[3] == kpb)
				{
					kpbState = KPB_START;
					print_ecdbg("KPB State: START\r\n");
					return KPB_VALID;
				}
				break;
		}
		
		kpbState = KPB_START;
		print_ecdbg("KPB State: START (ERROR)\r\n");
		return KPB_ERROR; //if we got here, we had a code but it didn't match what was in the list of valid codes
	}
	
	return KPB_CONTINUE; //if we are here, nothing happened or we are in the middle of debounce
}



unsigned char electroclaveState  = STATE_EC_IDLE; //start here unless we detect any errors on power up
unsigned char errorDisplayState = 0;
unsigned char displayChanged = 0;



/*
 * NOTE: Don't let these structs exceed 2K bytes or they will overrun the flash areas they are assigned to.
 */ 

#define NUM_SETS_LED_BOARD_SIDES	12	//should be enough for the lifetime of the unit
#define LED_BOARD_SIDE_STRUCT_SIZE	10	//bytes


void door_ajar_buzzer(void);
void door_ajar_buzzer(void)
{
	controls.buzzer_enable = 1;
	controls.buzzer_cycle = CYCLE_ON;
	controls.buzzer_on_dur = 1000;	//ms
	controls.buzzer_off_dur = 1000;	//ms
	controls.buzzer_repeat = 0xFF;	//forever essentially
	controls.buzzer_repeat_count = 0;
	controls.buzzer_dur_count = 0;
	pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL); 
}


void button_beep_buzzer(void)
{
	controls.buzzer_enable = 1;
	controls.buzzer_cycle = CYCLE_ON;
	controls.buzzer_on_dur = 100;	//ms
	controls.buzzer_off_dur = 0;	//ms
	controls.buzzer_repeat = 1;		//just one short beep
	controls.buzzer_repeat_count = 0;
	controls.buzzer_dur_count = 0;
	pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL);
}


/*
 * CHASSIS AND BOARD STATISTICS STORED IN FLASH
 *
 * Using the bottom-most flash inside the Atmel processor.
 * Processor flash is organized as 128 byte sectors, and rated for 100,000 erase cycles.
 *
 *	DATA										UPDATE FREQUENCY
 * --------------------------------------------------------------
 * -(0)Serial ID and flags						Serial IDs for each LED board logged when a board is plugged into the chassis.
 *												An entry is maintained for the board's top side, and another entry for the bottom side
 *												 *if* the board is not in an outer slot (0 or 4). A board in slot 0 will only have the
 *												 bottom side tracked, and a board in slot 4 will only have its top side tracked. (If the board
 *												 is later moved to another slot, the second side will also start being tracked at that point.)
 *												This structure has room for 96 entries: 12 complete sets of 8 LED board sides which should be 
 *												more than enough to cover the life of the product.
 *												This structure also tracks whether this slot in the structure has been filled, and whether
 *												max hour usage (2000 hours) has been reached.
 * -(1)Chassis sanitation cycles				Incremented on start of each sanitation cycle.
 * -(2)Usage hours								Each LED board side's sanitation usage hours are tracked in this structure. Like the Serial ID and flags
 *												 struct, it has 96 entries per buffer.
 * -(3)Usage in minutes + chassis san minutes	Similar structure to the usage hours structure, except here we only track usage minutes.
 *												The reason for breaking them out separately is because there are potentially 96 entries that need
 *												to be updated once per minute, and since there is no guarantee that the minutes will roll over to
 *												hours at the same time for each of the 96 entries, we need to create lots of buffers for this struct
 *												to make sure we don't exceed the flash max erase cycles of 100,000.
 * -(4)Configuration							User discretion. 2 copies at 100,000 erase cycles should be *more* than enough. Who is going to change 
 *												 initialDTE more than 200,000 times during the lifetime of the chassis? <famous last words.>
 */


/*
 * REGION 0 - SERIAL ID PLUS FLAGS
 * 
 * 96 LED board sides * 8 byte structure =  768 bytes = 768 bytes/128 bytes per sector = 6 sectors per set.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (96 LED board sides * (1 insertion + slot filled per side + 1 maxUsageReached per side) / 100,000 erase cycles) =  96*2 = 192 / 100,0000 erase cycles = 1 sets but we need at least 2 sets so that we always have one valid copy
 *
 */

typedef struct {
	
	unsigned char id[6];					//6 bytes - 48 bits

	unsigned int top_botn			: 1;	//top .=. 1, bottom .=. 0 side of the LED board (track them independently)
	unsigned int maxUsageReached	: 1;	//go/no-go flag
	unsigned int slotFilled			: 1;
	unsigned int					: 1;
	unsigned int					: 1;
	unsigned int					: 1;
	unsigned int					: 1;
	unsigned int					: 1;
	unsigned int csum				: 8;	//csum for entire struct in entry 0 only. Keep this 96 element struct to (6*128 byte) sectors.
	
} SERIAL_ID_AND_FLAGS;

SERIAL_ID_AND_FLAGS			sf[(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES)];

#define NUM_SERIAL_ID_SECTORS_PER_BUF	6
#define NUM_SERIAL_ID_BUFS_SECTORS		12

unsigned int sfFlashIdx = 0;


/*
 * REGION 1 - CHASSIS SANITATION CYCLES
 * 
 * Max value: 96 LED board sides * 2000 hours max per LED board * 3 cycles max per hour (20 minutes @aging of 0 or 100% intensity) = 576,000 = 0x8CA00
 * Bits required for max value: 20
 * Number of bits of checksum per entry, rounds up to the nearest byte - since compiler will for this to 4 bytes (round # of ints) use a byte for csum.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (576,000 / 100,000) =  ~6
 * This is very space inefficient but the minimum flash area we can write is 128 bytes (1 sector) at a time, so 4 bytes of data per sector it is. 
 *
 */

typedef struct {
	
	unsigned int	cycles	: 20;
	unsigned int	csum	: 8;
	unsigned int			: 4; //compiler forces this to 4 bytes anyway
	
}CHASSIS_SANITATION_CYCLES;

CHASSIS_SANITATION_CYCLES sanc;

#define NUM_SAN_CYCLE_BUFS_PER_SECTOR	1	//min flash write is one 128 byte sector. Space-wise this is inefficient, but what can we do?
#define NUM_SAN_CYCLE_BUFS_SECTORS		6	//we need at least two sectors so that if one is in the middle of an update when the power goes down another is still in tact

unsigned int sanCycleFlashIdx = 0;

/*
 * REGION 2 - USAGE HOURS
 * 
 * Max value: (2000 hours max per LED board side) = 0x7D0
 * Bits required for max value: 11, round up to 16 bits = 2 bytes
 * 96 entries per struct * 2 bytes = 192 bytes per struct which requires two 128-byte sectors per struct
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (2000*96)/100,000 = 192,000/100,000 = ~2
 * We need 2 sets * 2 sectors = 4 sectors.
 *
 */

typedef struct  
{
	unsigned short	hrs[(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES)];
	unsigned char	csum;

}USAGE_HOURS_SET;

USAGE_HOURS_SET h;

#define NUM_USAGE_HOURS_SECTORS_PER_BUF		2
#define NUM_USAGE_HOURS_BUFS_SECTORS		4

unsigned int hFlashIdx = 0;



/*
 * REGION 3 - USAGE MINUTES + CHASSIS SANITATION MINUTES
 * 
 * Here we only track minutes 0..59 which fits in 6 bits, rounded up to 8 bits = 1 byte.
 * 96 LED board sides * 1 byte =  96 bytes per set = 96/128 bytes per sector = ~1 sectors per set.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (96 LED board sides *2000 hours of sanitization * 60 minutes/hour / 100,000 erase cycles) =  116 sets = 116 sectors.
 * Chassis sanitation minutes takes up minimal space at 4 bytes per buffer (96 LED board sides * 2000 hours * 60 minutes = 11,520,000 = 0xafc800 easily fits in an unsigned long). We have plenty of room for that.
 */


typedef struct  
{
	unsigned char	mins[(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES)];
	unsigned long	sanMins;
	unsigned char	csum;

} USAGE_MINS_SET;

USAGE_MINS_SET m;

#define NUM_USAGE_MINS_BUFS_PER_SECTOR		1
#define NUM_USAGE_MINS_BUFS_SECTORS			116

unsigned int mFlashIdx = 0;



/*
 * REGION 4 - CONFIGURATION
 *
 * Currently just initialDTE. Plenty of room for expansion.
 */


typedef struct  
{
	unsigned char	initialDTE;
	unsigned char	csum;

} CONFIGURATION;

CONFIGURATION c;

#define NUM_CONFIG_BUFS_PER_SECTOR		1
#define NUM_CONFIG_BUFS_SECTORS			2

unsigned int configFlashIdx = 0;


enum {SE_PASS, SE_FAIL};

typedef struct 
{
	unsigned char	topdrive;
	unsigned int	botdrive;	
	unsigned int	flashArea;		
	unsigned char	ledBrdSerialIdCsum;
	unsigned char	ledBrdSideMaxUsage;	
	unsigned char	usageStructsFull;
	
}SYSERR;

SYSERR sysErr;

#define BIT(x) (1<<(x))

void init_sysErr(void);
void init_sysErr(void)
{
	memset(&sysErr, 0x00, sizeof(sysErr)); //Init everything to "PASS"
}

/*
 * ADC reading storage: for device detection
 */
int16_t bluesense_buf[4];


/*
 * Commands for LED display: we can only display the strings provided for by the display
 */

unsigned char CMD_READY[7] =	{0x55, 0xAA, 0x91, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAN[7] =	{0x55, 0xAA, 0x92, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEANING[7] = {0x55, 0xAA, 0x93, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_DIRTY[7] =	{0x55, 0xAA, 0x94, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_ERROR[7] =	{0x55, 0xAA, 0x95, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF1[7] =	{0x55, 0xAA, 0x96, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF2[7] =	{0x55, 0xAA, 0x97, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF3[7] =	{0x55, 0xAA, 0x98, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF4[7] =	{0x55, 0xAA, 0x99, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAR[7] =	{0x55, 0xAA, 0xCE, 0x00, 0x00, 0x00, 0x00}; //experiment 11apr15
	
unsigned char* cmdPtrArray[10] = {
	&CMD_READY[0],
	&CMD_CLEAN[0],
	&CMD_CLEANING[0],
	&CMD_DIRTY[0],
	&CMD_ERROR[0],
	&CMD_SHELF1[0],
	&CMD_SHELF2[0],
	&CMD_SHELF3[0],
	&CMD_SHELF4[0],
	&CMD_CLEAR[0]
};

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

uint8_t validKeypadCode = 0;
uint8_t startButtonPressed = 0;

#define EC_DOOR_LATCHED ((!ioport_get_pin_level(ECLAVE_DOORSW1)) && (!ioport_get_pin_level(ECLAVE_DOORSW2)))


enum {
	SHELF_INACTIVE,
	SHELF_ACTIVE
};


void display_text(unsigned char idx);
void display_text(unsigned char idx)
{
	for (int i = 0; i<7; i++)
	{
		putchar(((unsigned char) ((*(cmdPtrArray[idx]+i)))));
	}
	
}

void chassis_error(void);
void chassis_error(void)
{
	display_text(IDX_ERROR);
	print_ecdbg("Chassis error...shutting down.\r\n");
	
	while(1); //catastrophic error, just hang TODO: allow technician interface to work here possibly
	
}

void init_io(void);
void init_io(void)
{
	uint32_t ioFlags;

	ioport_set_pin_dir(ECLAVE_SERIAL_ID0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID0, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID1, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID2, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID3, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID3, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(ECLAVE_SERIAL_ID4, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ECLAVE_SERIAL_ID4, IOPORT_PIN_LEVEL_HIGH);
}


/* LED board side designations: 2 sides per shelf */
enum {
	LED_BRD_0_BOT,	//shelf 0
	LED_BRD_1_TOP,	//shelf 0
	LED_BRD_1_BOT,	//shelf 1
	LED_BRD_2_TOP,	//shelf 1
	LED_BRD_2_BOT,	//shelf 2
	LED_BRD_3_TOP,	//shelf 2
	LED_BRD_3_BOT,	//shelf 3
	LED_BRD_4_TOP	//shelf 3
};


void print_ecdbg_num(unsigned int num)
{
	char str[6];
	
	sprintf(str, "%d", num);	
	
	print_ecdbg(str);
}

/* One serial ID chip per board */
void read_led_board_serial_ids(void);
void read_led_board_serial_ids(void)
{
	/*
	 * Check for LED board presence by issuing a reset to the serial ID chip and checking for a response.
	 */
	
	SetSpeed(1); //1==standard speed, not overdrive 
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		ledBrd[i].present = !OWTouchReset(i);
		if (ledBrd[i].present)
		{
			print_ecdbg("LED board detected in slot ");
			print_ecdbg_num(i);
			print_ecdbg("\r\n");
		}
	}
	
	if (ledBrd[0].present && ledBrd[1].present)
	{
		shelf[0].present = 1;
		
		print_ecdbg("Shelf 0 present\r\n");
	}
	if (ledBrd[1].present && ledBrd[2].present)
	{
		shelf[1].present = 1;
		print_ecdbg("Shelf 1 present\r\n");
	}
	if (ledBrd[2].present && ledBrd[3].present)
	{
		shelf[2].present = 1;
		print_ecdbg("Shelf 2 present\r\n");
	}
	if (ledBrd[3].present && ledBrd[4].present)
	{
		shelf[3].present = 1;
		print_ecdbg("Shelf 3 present\r\n");
	}
	

	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		unsigned char acc = 0;
		
		if (ledBrd[i].present)
		{
			OWWriteByte(i, 0x33); //Read ID command
			
			ledBrd[i].idFamily = OWReadByte(i);
			
			acc = crc8_add(0x00, ledBrd[i].idFamily);
			
			for (int j=0; j<6; j++)
			{
				ledBrd[i].id[j] = OWReadByte(i);
				acc = crc8_add(acc, ledBrd[i].id[j]);
			}
			
			ledBrd[i].idcsum = OWReadByte(i);
			
			if (acc != ledBrd[i].idcsum)
			{
				sysErr.ledBrdSerialIdCsum |= BIT(i); //SE_FAIL;
				ledBrd[i].present = 0; //crc8 wasn't valid for this ID chip, don't trust the board
				print_ecdbg("Invalid serial ID checksum.\r\n");
				
				electroclaveState = STATE_CHASSIS_ERROR;
			}
		}
	}
}

/* LEDs we are using are rated for up to 2000 hours */
//TODO: Note part number here
enum {
	LED_BOARD_SIDE_PAST_LIFETIME_LIMIT,
	LED_BOARD_SIDE_WITHIN_LIFETIME_LIMIT
};

/* Each side of an LED board will get different usage */
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx);
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx)
{
	unsigned char idx;
	unsigned int hours;
	float32_t intensity = 0;
	float32_t tmpSanMinutes = 0;
	
	/*
	 * Find the record for this board's serial ID number, and check the usage hours and see if we
	 *	are past the 2000 hour mark. If we are, this board is considered un-usuable until it is
	 *	refurbished. 
	 */
	
	idx = ledBrdSide[sideIdx].ushdwIdx;
	
	
	hours = h.hrs[idx];

		

/*
 * Since we have to calculate the hours to see if the shelf is valid, finish out the calculations for the sanitizing time also. We'll need it later.
 */
//	intensity = ((0.00002 * hours * hours) - (0.0699 * hours) + 91.879);
#if 0 //debug 23feb16 this makes the board hang???
	intensity = 0.00002;
	intensity *= hours;
	intensity *= hours;
	intensity -= (0.0699 * hours);
	intensity += 91.879;
	
	tmpSanMinutes = c.initialDTE;
	tmpSanMinutes *= 100;
	tmpSanMinutes /= intensity;
	
	ledBrdSide[sideIdx].sanitizeMinutes = (unsigned char) tmpSanMinutes; //Shortest sanitize time is 20 minutes. Sanitize time increases as LED intensity drops with usage. Sanitize time is around 49 minutes when usage is at 2000 hours.

#endif 0 //debug 23feb16 this makes the board hang???	

//	ledBrdSide[sideIdx].sanitizeMinutes = 60; //DEBUG hard code to 1 minute per Christian 24jun15 take this out later
//	ledBrdSide[sideIdx].sanitizeMinutes = 255; //DEBUG hard code to 10 minutes to debug BOTDRIVE problem 31jul15 take this out later
//	ledBrdSide[sideIdx].sanitizeMinutes = 30; //DEBUG hard code to 30 minutes for sanitation tests 16jan16
	ledBrdSide[sideIdx].sanitizeMinutes = 1; //DEBUG hard code to 1 minute for trade show 25feb16


	if (hours < 2001)
	{
		return LED_BOARD_SIDE_WITHIN_LIFETIME_LIMIT;
	}
	else
	{
		return LED_BOARD_SIDE_PAST_LIFETIME_LIMIT;
//DEBUG 24jun15 need to function even with these errors for demo purposes		electroclaveState = STATE_CHASSIS_ERROR;
	}
}

/* Aggregate the information */
void check_led_brd_side_lifetimes(void);
void check_led_brd_side_lifetimes(void)
{
	unsigned char brdIdx;
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		if (ledBrd[brdIdx].present)
		{
			ledBrdSide[i].maxUsageReached = !check_led_brd_side_lifetime(i);	
		}	
	}
}

extern bool is_conversion_done;
extern uint32_t g_afec0_sample_data;
extern uint32_t g_afec1_sample_data;


int16_t adc_process_task(unsigned char shelfIdx);
int16_t adc_process_task(unsigned char shelfIdx)
{
	
	switch(shelfIdx)
	{
		case 0:
			afec_channel_enable(AFEC1, AFEC_CHANNEL_9);
			afec_start_software_conversion(AFEC1);
			is_conversion_done = false;
			while (is_conversion_done == false);
			bluesense_buf[shelfIdx] = g_afec1_sample_data;
			afec_channel_disable(AFEC1, AFEC_CHANNEL_9);
			break;
		case 1:
			afec_channel_enable(AFEC0, AFEC_CHANNEL_4);
			afec_start_software_conversion(AFEC0);
			is_conversion_done = false;
			while (is_conversion_done == false);
			bluesense_buf[shelfIdx] = g_afec0_sample_data;
			afec_channel_disable(AFEC0, AFEC_CHANNEL_4);
			break;
		case 2:
			afec_channel_enable(AFEC1, AFEC_CHANNEL_4);
			afec_start_software_conversion(AFEC1);
			is_conversion_done = false;
			while (is_conversion_done == false);
			bluesense_buf[shelfIdx] = g_afec1_sample_data;
			afec_channel_disable(AFEC1, AFEC_CHANNEL_4);
			break;
		case 3:
			afec_channel_enable(AFEC1, AFEC_CHANNEL_5);
			afec_start_software_conversion(AFEC1);
			is_conversion_done = false;
			while (is_conversion_done == false);
			bluesense_buf[shelfIdx] = g_afec1_sample_data;
			afec_channel_disable(AFEC1, AFEC_CHANNEL_5);
			break;		
	}
	
	return bluesense_buf[shelfIdx];
}




enum {
	NO_DEVICES_PRESENT,
	DEVICES_PRESENT
};

unsigned char check_shelf_for_devices(unsigned char shelfPosition);
unsigned char check_shelf_for_devices(unsigned char shelfPosition)
{
	uint16_t bluesense[8] = {0,0,0,0,0,0,0,0};
	unsigned long bluesenseAccumulated = 0;
	unsigned int bluesenseAvg = 0;
	
	char str[80];
	
	led_shelf(shelfPosition, LED_ON); //TODO: do we finish this task fast enough to not check the door latch in here? Can't have LEDs on if the door opens
	
	mdelay(100); //30may15 was 50ms, trying 100 to see if we can get more consistent
		
	//Read bluesense for this shelf

	for (int i=0; i<8; i++)
	{
		bluesense[i] = adc_process_task(shelfPosition);
		
		if (bluesense[i] & 0x8000) //don't try to average negative numbers
		{
			bluesenseAccumulated += 0;
		}
		else
		{
			bluesenseAccumulated += bluesense[i];
		}
	}
	
	bluesenseAvg = bluesenseAccumulated/8;
	
	led_shelf(shelfPosition, LED_OFF);
	
	memset(str,0x00, 80);

	for (int i=0; i<8; i++)
	{
		sprintf(str, "shelf %d: bluesense[%d]=0x%X\r\n", shelfPosition, i, bluesense[i]);
		print_ecdbg(str);
	}

	if ((bluesenseAvg < 0xC00) ||  (bluesenseAvg & 0x8000))//full range for 12 bit number is 0xFFF, but this number is 2's complement meaning it can (and it does) go negative
	{
		return DEVICES_PRESENT;
	}
	else
	{
		return NO_DEVICES_PRESENT;
	}
}

void check_shelves_for_devices(void);
void check_shelves_for_devices(void)
{
	for (int i=0; i<NUM_SHELVES; i++)
	{
		if (shelf[i].present)
		{
			shelf[i].devicesPresent = check_shelf_for_devices(i);
			
			if (shelf[i].devicesPresent)
			{
				print_ecdbg("Devices detected on shelf ");
				print_ecdbg_num(i);
				print_ecdbg("\r\n");
			}
		}
	}
}


void print_pca9952_errors(unsigned char sideSel, unsigned char eflag0, unsigned char eflag1);
void print_pca9952_errors(unsigned char sideSel, unsigned char eflag0, unsigned char eflag1)
{
	unsigned char bit;
	
	switch (sideSel)
	{
		case TOP:
			print_ecdbg("PCA9952 Error(s) on TOPDRIVE ");
			
			for (int i=0; i<8; i++)
			{
				bit = (1 << i);
				if (bit & eflag0)
				{
					print_ecdbg_num(i);
					print_ecdbg(" ");
					sysErr.topdrive |= BIT(i); //SE_FAIL
//DEBUG 24jun15 need to function even with these errors for demo purposes					electroclaveState = STATE_CHASSIS_ERROR;
				}
			}
			
			print_ecdbg("\r\n");
			
			if (eflag1 != 0)
			{
				print_ecdbg("ERROR on unused channels: PCA9952 - Controller board U7\r\n");
				electroclaveState = STATE_CHASSIS_ERROR;
			}
			
			break;

		case BOTTOM:
			print_ecdbg("PCA9952 Error(s) on BOTDRIVE ");
			
			for (int i=0; i<8; i++)
			{
				bit = (1 << i);
				if (bit & eflag0)
				{
					print_ecdbg_num(i);
					print_ecdbg(" ");
					sysErr.botdrive |= BIT(i); //SE_FAIL;
//DEBUG 24jun15 need to function even with these errors for demo purposes					electroclaveState = STATE_CHASSIS_ERROR;
				}
			}
			
			for (int i=0; i<4; i++)
			{
				bit = (1 << i);
				if (bit & eflag1)
				{
					print_ecdbg_num((i+8));
					print_ecdbg(" ");
					sysErr.botdrive |= BIT(i+8); //SE_FAIL;
//DEBUG 24jun15 need to function even with these errors for demo purposes					electroclaveState = STATE_CHASSIS_ERROR;
				}
			}
			
			print_ecdbg("\r\n");
			
			if ((eflag1 & 0xF0) != 0)
			{
				print_ecdbg("ERROR on unused channels: PCA9952 - Controller board U8\r\n");
				electroclaveState = STATE_CHASSIS_ERROR;
			}
			break;
	}
}

unsigned char numActiveShelves, numPresentShelves;

unsigned char topEflag0 = 0, topEflag1 = 0, botEflag0 = 0, botEflag1 = 0;

void test_led_driver_channels(void);
void test_led_driver_channels(void)
{
	unsigned char tmp1, tmp2, numShelvesPresent = 0;
	
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		if (shelf[i].present)
		{
			numShelvesPresent++;
		}
	}
	
	if (numShelvesPresent !=0)
	{
		//Tone down the current so all shelves can be turned on at once
		PCA9952_write_reg(LED_TOP, PCA9952_IREFALL, LED_TEST_DRIVER_CURRENT);
		PCA9952_write_reg(LED_BOTTOM, PCA9952_IREFALL, LED_TEST_DRIVER_CURRENT);

		for (int i=0; i<NUM_SHELVES; i++)
		{
			if (shelf[i].present)
			{
				led_shelf(i, LED_ON);
			}
		}
		
		udelay(100); //maybe need this while testing LED boards with resistors in place of real LEDs 31july2015

		PCA9952_write_reg(LED_TOP, PCA9952_MODE2, 0x40); //starts fault test
		PCA9952_write_reg(LED_BOTTOM, PCA9952_MODE2, 0x40); //starts fault test
		
		while (1)
		{
			tmp1 = PCA9952_read_reg(LED_TOP, PCA9952_MODE2);
			
			if ((tmp1 & 0x40) == 0)
			{
				topEflag0 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG0);
				topEflag1 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG1);
				
				if ((topEflag0 != 0) || (topEflag1 != 0))
				{
					if (!firstTimeThroughPCA9952)
					{
						print_pca9952_errors(TOP, topEflag0, topEflag1);
					}
				}
				
				break; //fault test for LED_TOP strings is complete
			}
			
		}
		
		while (1)
		{
			tmp2 = PCA9952_read_reg(LED_BOTTOM, PCA9952_MODE2);
			
			if ((tmp2 & 0x40) == 0)
			{
				botEflag0 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG0);
				botEflag1 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG1);
				
				if ((botEflag0 != 0) || (botEflag1 != 0))
				{
					if (!firstTimeThroughPCA9952)
					{
						print_pca9952_errors(BOTTOM, botEflag0, botEflag1);
					}
				}
				
				break; //fault test for LED_BOTTOM strings is complete
			}
			
		}
		
		for (int i=0; i<NUM_SHELVES; i++)
		{
			led_shelf(i, LED_OFF);
		}
		
		//Put driver current back to full power
		PCA9952_write_reg(LED_TOP, PCA9952_IREFALL, LED_DRIVER_CURRENT);
		PCA9952_write_reg(LED_BOTTOM, PCA9952_IREFALL, LED_DRIVER_CURRENT);
	}
	
	sysErr.topdrive = topEflag0;
	sysErr.botdrive = (botEflag1 << 8) | botEflag0;
	firstTimeThroughPCA9952 = 0;
}

void set_shelves_active_inactive(void);
void set_shelves_active_inactive(void)
{

	test_led_driver_channels();
	
	numActiveShelves = 0;
	numPresentShelves = 0;
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelf[i].active = SHELF_INACTIVE;
		
	}
	
	/* check shelf 0 */
	if (shelf[0].present &&
		(!ledBrdSide[LED_BRD_0_BOT].maxUsageReached) &&
		(!ledBrdSide[LED_BRD_1_TOP].maxUsageReached) )
	{
		numPresentShelves++;
		
		if (shelf[0].devicesPresent)
		{
			shelf[0].active = SHELF_ACTIVE;
			numActiveShelves++;
			print_ecdbg("Shelf 0 active\r\n");
		}
	}
	
	/* check shelf 1 */
	
	if (shelf[1].present &&
	(!ledBrdSide[LED_BRD_1_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_2_TOP].maxUsageReached) )
	{
		numPresentShelves++;
		
		if (shelf[1].devicesPresent)
		{
			shelf[1].active = SHELF_ACTIVE;
			numActiveShelves++;
			print_ecdbg("Shelf 1 active\r\n");
		}
	}
	
	/* check shelf 2 */
	
	if (shelf[2].present &&
	(!ledBrdSide[LED_BRD_2_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_3_TOP].maxUsageReached) )
	{
		numPresentShelves++;
		
		if (shelf[2].devicesPresent)
		{
			shelf[2].active = SHELF_ACTIVE;
			numActiveShelves++;
			print_ecdbg("Shelf 2 active\r\n");
		}
	}
	
	/* check shelf 3 */
	
	if (shelf[3].present &&
	(!ledBrdSide[LED_BRD_3_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_4_TOP].maxUsageReached) )
	{
		numPresentShelves++;
		
		if (shelf[3].devicesPresent)
		{
			shelf[3].active = SHELF_ACTIVE;
			numActiveShelves++;
			print_ecdbg("Shelf 3 active\r\n");
		}
	}
}

unsigned char num_active_shelves(void);
unsigned char num_active_shelves(void)
{
	return numActiveShelves;
}

unsigned char num_present_shelves(void);
unsigned char num_present_shelves(void)
{
	return numPresentShelves;
}


/*! \brief Initializes the MCU system clocks.
 */

volatile bool input_fft_view = false;
volatile bool output_fft_view = false;
volatile bool zoom_view = false;
volatile int32_t zoom_view_id;


unsigned char calc_sanitize_time(unsigned char shelfIdx);
unsigned char calc_sanitize_time(unsigned char shelfIdx)
{
	unsigned char uSideMinutes, lSideMinutes, minutes, boardIdx, sideIdx;
	
	boardIdx = shelf[shelfIdx].tLedIdx;							//top board in the shelf
	sideIdx = ledBrd[boardIdx].lSideIdx;						//lower side of the top board
	lSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;
	

	boardIdx = shelf[shelfIdx].bLedIdx;							//bottom board in the shelf					
	sideIdx = ledBrd[boardIdx].uSideIdx;						//upper side of the bottom board
	uSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;

	minutes = (uSideMinutes >= lSideMinutes) ? uSideMinutes : lSideMinutes; //choose the sanitize time for the more worn-out leds
	
	return (minutes);
	
}


void door_latch_open_kill_all_shelves(void);
void door_latch_open_kill_all_shelves(void)
{
	led_shelf(0, LED_OFF);
	led_shelf(1, LED_OFF);
	led_shelf(2, LED_OFF);
	led_shelf(3, LED_OFF);
	
	print_ecdbg("Door latch opened, kill all shelves for safety.\r\n");
}


/*
 * 2 copies: one each for alternating minutes.
 * Need these areas of flash to erase and be written independently.
 * It's very easy for the power to get shut down during the 
 * one minute updates while the unit is sanitizing. Keeping these
 * 2 areas of flash erasing and updating independently ensures that at
 * least one buffer is intact if the other buffer gets corrupted.
 */




//! NVRAM data structure located in the flash array.
#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram0")))
#endif
static unsigned char serialIdAndFlagsFlash[(NUM_SERIAL_ID_BUFS_SECTORS * 128)]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM0"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram1")))
#endif
static unsigned char sanitationCyclesFlash[NUM_SAN_CYCLE_BUFS_SECTORS * 128]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM1"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram2")))
#endif
static unsigned char usageHoursFlash[(NUM_USAGE_HOURS_BUFS_SECTORS*128)]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM2"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram3")))
#endif
static unsigned char usageMinutesFlash[NUM_USAGE_MINS_BUFS_SECTORS * 128]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM3"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram4")))
#endif
static unsigned char configFlash[NUM_CONFIG_BUFS_SECTORS * 128]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM4"
#endif
;



unsigned char minute_count(unsigned char * pMinuteBits)
{
	unsigned char tmpMinBits, minuteCount = 0, bit;
	
	for (int i=0; i<8; i++)
	{
		tmpMinBits = *(pMinuteBits+i);
		bit = 0x80;
		for (int j=0; j<8; j++)
		{
			if ((bit & tmpMinBits) == 0)
			{
				minuteCount++;
			}
			
			bit >>= 1;
		}
	}
	
	return minuteCount;
}

unsigned char inc_minutes(unsigned char * pMinuteBits)
{
	unsigned char bit, invBit, tmpMinBits, minCount = 0;
	
	for (int i=0; i<8; i++)
	{
		bit = 0x80;
		tmpMinBits = (*(pMinuteBits+i));
		
		for (int j=0; j<8; j++)
		{
			minCount++;			
			if (bit & tmpMinBits)
			{
				invBit = (bit ^= 0xFF);
				tmpMinBits &= invBit;
				(*(pMinuteBits+i)) = tmpMinBits;
				return minCount;
			}
			bit >>= 1;
		}
	}
	
	return 0; //we should never get here, but we need this to avoid the warning
}


void reset_minutes(unsigned char * pMinuteBits)
{
	for (int i=0; i<8; i++)
	{
		*(pMinuteBits+i) = 0xFF; //Set to all ones: we will flip one bit from 1 to 0 each minute to save from having to erase flash every minute
	}	
	
}

#define STRINGS_MATCH 0


unsigned char usage_idx(unsigned char * idPtr, unsigned char top_botn);
unsigned char usage_idx(unsigned char * idPtr, unsigned char top_botn)
{
	unsigned char tmpBoardId[6];
	
	for (unsigned char i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		if (sf[i].slotFilled)
		{
			tmpBoardId[0] = *(idPtr+0);
			tmpBoardId[1] = *(idPtr+1);
			tmpBoardId[2] = *(idPtr+2);
			tmpBoardId[3] = *(idPtr+3);
			tmpBoardId[4] = *(idPtr+4);
			tmpBoardId[5] = *(idPtr+5);
			
			
			if (tmpBoardId[0] == sf[i].id[0]) {
				if (tmpBoardId[1] == sf[i].id[1]) {
					if (tmpBoardId[2] == sf[i].id[2]) {
						if (tmpBoardId[3] == sf[i].id[3]) {
							if (tmpBoardId[4] == sf[i].id[4]) {
								if (tmpBoardId[5] == sf[i].id[5]) {
									if (top_botn == sf[i].top_botn)
									{
										return (i); //found a match!

									} //check top_botn match
								} //tmpBoardId[5]
							} //tmpBoardId[4]
						} //tmpBoardId[3]
					} //tmpBoardId[2]
				} //tmpBoardId[1]
			} //tmpBoardId[0]
		} //if slotFilled (don't check against slots that haven't been assigned
	} //for each slot in ush
	
	return NO_LED_BOARD_PRESENT; //no match found
}


void load_usage_indeces(void)
{
	unsigned char top_botn, brdIdx;
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		if (ledBrd[brdIdx].present)
		{
			top_botn = i%2;
			
			usageIdx[i] = usage_idx(&ledBrd[brdIdx].id[0], top_botn); //TODO: should change this nomenclature to upper/lower, we are talking about board sides here, not which board in the shelf, be consistent
		}
		else
		{
			usageIdx[i] = NO_LED_BOARD_PRESENT;
		}
	}
}


enum{CHECKSUM_INVALID, CHECKSUM_VALID};


enum {SUCCESS, ERROR};

#if 0 //ignore for now 22feb16
unsigned char test_flash(unsigned char sel)
{
	volatile void* memPtr;
	unsigned char pattern[4] = {0x00, 0xAA, 0x55, 0xFF}, ubyte; //NOTE test 0xFF pattern last to essentially erase the flash
	unsigned char *ubPtr;
	unsigned long memSize;
	
	switch(sel)
	{
		case 0:
			memPtr = &serialIdAndFlagsFlash;
			memSize = NUM_SERIAL_ID_BUFS_SECTORS * 128;
			break;
		case 1:
			memPtr = &sanitationCyclesFlash;
			memSize = NUM_SAN_CYCLE_BUFS_SECTORS * 128;
			break;
		case 2:
			memPtr = &usageHoursFlash;
			memSize = NUM_USAGE_HOURS_BUFS_SECTORS * 128;
			break;
		case 3:
			memPtr = &usageMinutesFlash;
			memSize = NUM_USAGE_MINS_BUFS_SECTORS * 128;
			break;
	}

	for (unsigned char i=0; i<4; i++) //4 patterns to test
	{
		flashc_memset(memPtr, pattern[i], 8, memSize, true);
		
		ubPtr = (unsigned char*) memPtr;
		for (unsigned long j=0; j<memSize; j++)
		{
			ubyte = (*ubPtr);
			if (ubyte != pattern[i])
			{
				return ERROR;
			}
			ubPtr++;
		}
	}
	
	return SUCCESS;
}
#endif //22feb16 ignore for now

unsigned char calc_region_checksum(unsigned char sel);
unsigned char calc_region_checksum(unsigned char sel)
{
	unsigned char csum = 0;

	switch(sel)
	{
		case 0: //serial ID and flags
			csum = 0;
			for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
			{
				csum += sf[i].id[0];
				csum += sf[i].id[1];
				csum += sf[i].id[2];
				csum += sf[i].id[3];
				csum += sf[i].id[4];
				csum += sf[i].id[5];
				csum += sf[i].maxUsageReached;
				csum += sf[i].slotFilled;
				csum += sf[i].top_botn;
			}
			csum = ((csum ^ 0xFF) & 0xFF);
			break;

		case 1: //san cycles
			csum = ((sanc.cycles ^ 0xFF) & 0xFF);
			break;

		case 2: //usage hours
			csum = 0;
			for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
			{
				csum += h.hrs[i];
			}
			csum = ((csum ^ 0xFF) & 0xFF);
			break;

		case 3: //usage mins
			csum = 0;
			for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
			{
				csum += m.mins[i];
			}
			csum += m.sanMins;
			csum = ((csum ^ 0xFF) & 0xFF);
			break;
		case 4: //configuration
			csum = 0;
			csum += c.initialDTE;
			csum = ((csum ^ 0xFF) & 0xFF);
			break;
	}
	
	return csum;	
}

#if 0 //22feb16 ignore for now
unsigned char eval_region(unsigned char sel);
unsigned char eval_region(unsigned char sel)
{
	SERIAL_ID_AND_FLAGS				tmpSf[(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES)];
	CHASSIS_SANITATION_CYCLES		tmpSanc;
	USAGE_HOURS_SET					tmpH;
	USAGE_MINS_SET					tmpM;
	CONFIGURATION					tmpC;
	
	unsigned char					csum;
	long							flashOffset;
	long							tmpFlashOffset;
	unsigned char					retVal = 0; //NOT GOOD
	

	unsigned long tmpHours, uHours, tmpMinutes, uMinutes, tmpSlotsFilled, uSlotsFilled;
	
	print_ecdbg("eval_region() ");
	
	switch (sel)
	{
		case 0: //serial ID and flags
			
			print_ecdbg("region 0 - serial ID and flags\r\n");
			
			memset(&tmpSf, 0x00, sizeof(sf));
			
			for (unsigned int i=0; i<(NUM_SERIAL_ID_BUFS_SECTORS / NUM_SERIAL_ID_SECTORS_PER_BUF); i++)
			{
				flashOffset =  (i* 128 * NUM_SERIAL_ID_SECTORS_PER_BUF);
				tmpFlashOffset = flashOffset + (unsigned long)serialIdAndFlagsFlash;
				memcpy(&sf, (const void*) tmpFlashOffset, sizeof(sf));
				
				csum = calc_region_checksum(0);

				if (csum == sf[0].csum) //checksum is good
				{
					print_ecdbg("good csum\r\n");
					
					retVal = 1; //we have at least one good copy

					tmpSlotsFilled = 0;
					uSlotsFilled = 0;
				
					for (int j=0; j<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); j++)
					{
						tmpSlotsFilled += tmpSf[j].slotFilled;
						uSlotsFilled += sf[j].slotFilled;
					}
				
					if (uSlotsFilled > tmpSlotsFilled)
					{
						memcpy(&tmpSf, &sf, sizeof(sf));
						sfFlashIdx = i; //this is the new best copy
						
						print_ecdbg("sfFlashIdx ");
						print_ecdbg_num(sfFlashIdx);
						print_ecdbg("\r\n");
					}

				}
			}
			
			if (retVal == 1)
			{
				memcpy(&sf, &tmpSf, sizeof(sf));
			}
			break;

		case 1: //san cycles
			memset(&tmpSanc, 0x00, sizeof(sanc));

			print_ecdbg("region 1 - sanitation cycles\r\n");

			for (unsigned int i=0; i<(NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS); i++)
			{
				flashOffset = (i * 128 * NUM_SAN_CYCLE_BUFS_PER_SECTOR);
				
				tmpFlashOffset = flashOffset + (unsigned long) sanitationCyclesFlash;
				memcpy(&sanc, (const void*) tmpFlashOffset, sizeof(sanc));
				
				csum = calc_region_checksum(1);
				
				if (csum == sanc.csum) //checksum is good
				{
					print_ecdbg("good csum\r\n");

					retVal = 1; //we have at least one good copy
					
					if (sanc.cycles > tmpSanc.cycles)
					{
						memcpy(&tmpSanc, &sanc, sizeof(sanc));
						sanCycleFlashIdx = i; //this is the new best copy

						print_ecdbg("sanCycleFlashIdx ");
						print_ecdbg_num(sanCycleFlashIdx);
						print_ecdbg("\r\n");
					}
				}
			}
			if (retVal == 1)
			{
				memcpy(&sanc, &tmpSanc, sizeof(sanc));
			}
			break;
			
		case 2: //usage hours
			memset(&tmpH, 0x00, sizeof(h));

			print_ecdbg("region 2 - usage hours\r\n");

			for (unsigned int i=0; i<(NUM_USAGE_HOURS_BUFS_SECTORS / NUM_USAGE_HOURS_SECTORS_PER_BUF); i++)
			{
				flashOffset = (i * 128 * NUM_USAGE_HOURS_SECTORS_PER_BUF);
				
				tmpFlashOffset = flashOffset + (unsigned long) usageHoursFlash;
				
				memcpy(&h, (const void*) tmpFlashOffset, sizeof(h));
				
				csum = calc_region_checksum(2);
				
				if (csum == h.csum) //checksum is good
				{
					print_ecdbg("good csum\r\n");

					retVal = 1; //we have at least one good copy
					
					tmpHours = 0;
					uHours = 0;
					
					for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
					{
						tmpHours += tmpH.hrs[i];
						uHours += h.hrs[i];
					}
					
					if (uHours > tmpHours)
					{
						memcpy(&tmpH, &h, sizeof(h));
						hFlashIdx = i; //this is the new best copy
						
						print_ecdbg("hFlashIdx ");
						print_ecdbg_num(hFlashIdx);
						print_ecdbg("\r\n");

					}
				}
			}
			if (retVal == 1)
			{
				memcpy(&h, &tmpH, sizeof(h));
			}
			break;

		case 3: //usage minutes
			memset(&tmpM, 0x00, sizeof(m));
			
			print_ecdbg("region 3 - usage minutes\r\n");
			
			for (unsigned int i=0; i<(NUM_USAGE_MINS_BUFS_PER_SECTOR * NUM_USAGE_MINS_BUFS_SECTORS); i++)
			{
				flashOffset = (i * 128 * NUM_USAGE_MINS_BUFS_PER_SECTOR);
				
				tmpFlashOffset = flashOffset + (unsigned long) usageMinutesFlash;
				
				memcpy(&m, (const void*) tmpFlashOffset, sizeof(m));
				
				csum = calc_region_checksum(3);
				
				if (csum == m.csum) //checksum is good
				{
					print_ecdbg("good csum\r\n");

					retVal = 1; //we have at least one good copy
					
					tmpMinutes = 0;
					uMinutes = 0;
					
					for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
					{
						//TODO: I should be checking the serial ID and flags struct to see if the slot is filled, but i don't have a good way of syncing sf and m right now. In the meantime, just make sure that m.mins[i] is 0 if not used.
						tmpMinutes += tmpM.mins[i];
						uMinutes += m.mins[i];
					}
					
					if (uMinutes > tmpMinutes)
					{
						memcpy(&tmpM, &m, sizeof(m));
						mFlashIdx = i; //this is the new best copy

						print_ecdbg("mFlashIdx ");
						print_ecdbg_num(mFlashIdx);
						print_ecdbg("\r\n");
					}
				}
			}
			if (retVal == 1)
			{
				memcpy(&m, &tmpM, sizeof(m));
			}
			break;
		case 4: //configuration
			memset(&tmpC, 0x00, sizeof(c));

			print_ecdbg("region 4 - configuration\r\n");

			for (unsigned int i=0; i<(NUM_CONFIG_BUFS_PER_SECTOR * NUM_CONFIG_BUFS_SECTORS); i++)
			{
				flashOffset = (i * 128 * NUM_CONFIG_BUFS_PER_SECTOR);
			
				tmpFlashOffset = flashOffset + (unsigned long) configFlash;
				memcpy(&c, (const void*) tmpFlashOffset, sizeof(c));
			
				csum = calc_region_checksum(4);
			
				if (csum == c.csum) //checksum is good
				{
					print_ecdbg("good csum\r\n");

					retVal = 1; //we have at least one good copy
				
					memcpy(&tmpC, &c, sizeof(c));
					configFlashIdx = i; //no good eval criteria for initialDTE: user could increase or decrease it. Therefore, always store 2 copies so both copies will be the same.

					print_ecdbg("configFlashIdx ");
					print_ecdbg_num(configFlashIdx);
					print_ecdbg("\r\n");
				}
			}
			if (retVal == 1)
			{
				memcpy(&c, &tmpC, sizeof(c));
			}
			break;
		
	}
	
	return retVal;
}

unsigned char write_region_to_flash(unsigned char sel, unsigned char idx, unsigned char csum);
unsigned char write_region_to_flash(unsigned char sel, unsigned char idx, unsigned char csum)
{
	unsigned long tmpFlashOffset, flashOffset;
	unsigned char tmpIdx;
	
	if (idx == 0xFF) //use the default system index
	{
		switch(sel)
		{
			case 0: //serial ID and flags
				tmpIdx = sfFlashIdx;
				break;
			case 1: //sanitation cycles
				tmpIdx = sanCycleFlashIdx;
				break;
			case 2: //usage hours
				tmpIdx = hFlashIdx;
				break;
			case 3: //usage minutes
				tmpIdx = mFlashIdx;
				break;
			case 4: //configuration
				tmpIdx = configFlashIdx;
				break;
		}
	}
	else //use the specific index passed to this function
	{
		tmpIdx = idx;
	}
	
	switch (sel)
	{
		case 0: //serial ID and flags
			//NOTE: this is not as parameterized as it should be, only good for 2 sectors, but good enough for now. 
			
			sf[0].csum = csum;
			flashOffset = tmpIdx * 128 * NUM_SERIAL_ID_SECTORS_PER_BUF;
			tmpFlashOffset = flashOffset + (unsigned long) serialIdAndFlagsFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &sf, sizeof(sf), true);
			break;

		case 1: //san cycles
			//NOTE: this is not as parameterized as it should be, only good for 2 sectors, but good enough for now.
			
			sanc.csum = csum;
			flashOffset = tmpIdx * 128 * NUM_SAN_CYCLE_BUFS_PER_SECTOR; //one sector per buf
			tmpFlashOffset = flashOffset + (unsigned long) sanitationCyclesFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &sanc, sizeof(sanc), true);
			break;

		case 2: //usage hours
			h.csum = csum;
			flashOffset = tmpIdx * 128 * NUM_USAGE_HOURS_SECTORS_PER_BUF;
			tmpFlashOffset = flashOffset + (unsigned long) usageHoursFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &h, sizeof(h), true);
			break;
			
		case 3: //usage minutes
			//NOTE: this is not as parameterized as it should be, but good enough for now.
			m.csum = csum;
			flashOffset = tmpIdx * 128 * NUM_USAGE_MINS_BUFS_PER_SECTOR;
			tmpFlashOffset = flashOffset + (unsigned long) usageMinutesFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &m, sizeof(m), true);
			break;

		case 4: //configuration
			c.csum = csum;
			flashOffset = tmpIdx * 128 * NUM_CONFIG_BUFS_PER_SECTOR; //one sector per buf
			tmpFlashOffset = flashOffset + (unsigned long) configFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &c, sizeof(c), true);
			break;
	}
	
	return SUCCESS;	
}

void write_bad_region_to_flash(unsigned char sel, unsigned char idx);
void write_bad_region_to_flash(unsigned char sel, unsigned char idx)
{
	unsigned long tmpFlashOffset, flashOffset;
	unsigned char tmpIdx;

	USAGE_MINS_SET tmpM;
	
	memset(&tmpM, 0x00, sizeof(m)); //just zero it out and don't give it a good checksum
	
	if (sel != 3)
	{
		return; //we only do this for the minutes region
	}
	
	if (idx == 0xFF) //use the default system index
	{
		switch(sel)
		{
			case 3: //usage minutes
				tmpIdx = mFlashIdx;
				break;
		}
	}
	else //use the specific index passed to this function
	{
		tmpIdx = idx;
	}
	
	switch (sel)
	{
		case 3: //usage minutes
			flashOffset = tmpIdx * 128 * NUM_USAGE_MINS_BUFS_PER_SECTOR;
			tmpFlashOffset = flashOffset + (unsigned long) usageMinutesFlash;
			flashc_memcpy((volatile void*)tmpFlashOffset, &tmpM, sizeof(m), false); //don't erase, it would just cause the flash to wear out twice as fast
			break;
	}
	
}


void copy_region_to_another_sector(unsigned char sel);
void copy_region_to_another_sector(unsigned char sel)
{
	unsigned char tmpIdx, csum;
	
	switch (sel)
	{
		case 0: //serial ID and flags
			if (sfFlashIdx < ((NUM_SERIAL_ID_BUFS_SECTORS/NUM_SERIAL_ID_SECTORS_PER_BUF)/2))
			{
				tmpIdx = sfFlashIdx + ((NUM_SERIAL_ID_BUFS_SECTORS/NUM_SERIAL_ID_SECTORS_PER_BUF)/2);
			}
			else
			{
				tmpIdx = sfFlashIdx - ((NUM_SERIAL_ID_BUFS_SECTORS/NUM_SERIAL_ID_SECTORS_PER_BUF)/2);
			}
			
			csum = calc_region_checksum(0);
			write_region_to_flash(0, tmpIdx, csum);
			break;

		case 1: //san cycles
			if (sanCycleFlashIdx < ((NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS)/2))
			{
				tmpIdx = sanCycleFlashIdx + ((NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS)/2);
			}
			else
			{
				tmpIdx = sanCycleFlashIdx - ((NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS)/2);
			}
			csum = calc_region_checksum(1);
			write_region_to_flash(1, tmpIdx, csum);
			break;

		case 2: //usage hours
			if (hFlashIdx < ((NUM_USAGE_HOURS_BUFS_SECTORS/NUM_USAGE_HOURS_SECTORS_PER_BUF)/2))
			{
				tmpIdx = hFlashIdx + ((NUM_USAGE_HOURS_BUFS_SECTORS/NUM_USAGE_HOURS_SECTORS_PER_BUF)/2);
			}
			else
			{
				tmpIdx = hFlashIdx - ((NUM_USAGE_HOURS_BUFS_SECTORS/NUM_USAGE_HOURS_SECTORS_PER_BUF)/2);
			}
			csum = calc_region_checksum(2);
			write_region_to_flash(2, tmpIdx, csum);
			break;

		case 3: //usage minutes
			tmpIdx = mFlashIdx + (NUM_USAGE_MINS_BUFS_SECTORS/2);
			if (tmpIdx < (NUM_USAGE_MINS_BUFS_SECTORS/2))
			{
				tmpIdx += (NUM_USAGE_MINS_BUFS_SECTORS/2); //wrap if necessary
			}
			else
			{
				tmpIdx -= (NUM_USAGE_MINS_BUFS_SECTORS/2); //wrap if necessary
			}
			csum = calc_region_checksum(3);
			write_region_to_flash(3, tmpIdx, csum);
			break;

		case 4: //configuration
			if (configFlashIdx < ((NUM_CONFIG_BUFS_PER_SECTOR * NUM_CONFIG_BUFS_SECTORS)/2))
			{
				tmpIdx = configFlashIdx + ((NUM_CONFIG_BUFS_PER_SECTOR * NUM_CONFIG_BUFS_SECTORS)/2);
			}
			else
			{
				tmpIdx = configFlashIdx - ((NUM_CONFIG_BUFS_PER_SECTOR * NUM_CONFIG_BUFS_SECTORS)/2);
			}
			csum = calc_region_checksum(4);
			write_region_to_flash(4, tmpIdx, csum);
			break;
	}
	
}


/*
 * Since minutes just go from 0..59 and then wrap around, we can't use the strategy we use on other structs for
 * determining the optimum minutes sector (picking the highest value) so we have to do something to disrupt prior
 * entries which is intentionally write a bad checksum to flash. 
 */
void disrupt_prior_m_sector(void);
void disrupt_prior_m_sector(void)
{
	unsigned char tmpIdx;

	if (mFlashIdx > 0)
	{
		tmpIdx = mFlashIdx - 1;
	}
	else
	{
		tmpIdx = NUM_USAGE_MINS_BUFS_SECTORS - 1; //backwards wrap if necessary
	}

	write_bad_region_to_flash(3, tmpIdx);
	
}

#endif //22feb16 ignore for now

#define USAGE_FULL 0xFF

unsigned char find_first_open_usage_slot(void);
unsigned char find_first_open_usage_slot(void)
{
	for (unsigned int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		
		if (!sf[i].slotFilled)
		{
			return i;
		}
	}
	
	print_ecdbg("No more room for LED board info. Cannot track minute usage for additional boards.\r\n");
	
	sysErr.usageStructsFull = FAIL;
	electroclaveState = STATE_CHASSIS_ERROR;

	
	return USAGE_FULL; //Error, no open slots
}

void add_new_led_board_sides_to_usage(void)

{
	unsigned char firstOpenSlot, slotAssignment, brdIdx, top_botn;
	
	//NOTE that load_usage_indeces() must have been run already for this function to work. 
	// i.e., usageIdx[][] must be populated.
	

	firstOpenSlot = find_first_open_usage_slot();
	
	slotAssignment = firstOpenSlot;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		top_botn = (i%2) ? TOP : BOTTOM; //odd sides are top, even sides are bottom
		
		if ((ledBrd[brdIdx].present) && (usageIdx[i] == NO_LED_BOARD_PRESENT)) //TODO: do I need the NO_LED_BOARD_PRESENT check? this should always be open at this point
		{
			strncpy((char*)&sf[slotAssignment].id[0], (char*)&ledBrd[brdIdx].id[0],6);
			
			sf[slotAssignment].top_botn = top_botn;
			
			sf[slotAssignment].slotFilled = 1;

			usageIdx[i] = slotAssignment++;

		}
	}
}

unsigned char calc_usage_csum(unsigned char sel)
{
	unsigned char csum = 0;
	
/*
 * Lots of ways to checksum this struct, don't over-think it
 */

	for (unsigned char i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		
		csum += sf[i].id[0];
		csum += sf[i].id[1];
		csum += sf[i].id[2];
		csum += sf[i].id[3];
		csum += sf[i].id[4];
		csum += sf[i].id[5];
		
		csum += sf[i].maxUsageReached;
		csum += sf[i].top_botn;
		csum += sf[i].slotFilled;

	}
	
	return csum;
}


unsigned long calc_usage_current_led_boards(unsigned char sel);
unsigned long calc_usage_current_led_boards(unsigned char sel)
{
	unsigned long minutes = 0; 
	unsigned char idx;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		if (usageIdx[i] != NO_LED_BOARD_PRESENT)
		{
			idx = usageIdx[i];

			minutes += m.mins[idx];

		
//TODO: come back to this if we fix the flash with no erase problem			minutes += minute_count(&usageShdw[sel].u[idx].minuteBits[0]);
		}
	}
	
	return minutes;
}

void increment_ledBoard_usage_min(void);


void inc_sanMins(void);
void inc_sanMins(void)
{
	m.sanMins++;
	
	//the rest of the update of the struct, calc'ing the csum, writing to flash etc will happen when the usage minutes get updated
}

void inc_sanCycles(void);
void inc_sanCycles(void)
{
	sanc.cycles++;
	sanCycleFlashIdx++;
	if (sanCycleFlashIdx >= (NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS))
	{
		sanCycleFlashIdx = 0;
	}
	sanc.csum = calc_region_checksum(1);
//skip for now 22feb16	write_region_to_flash(1, 0xFF, sanc.csum);
}

void store_config(void);
void store_config(void)
{
	/* initialDTE set in the serial user interface */
	configFlashIdx++;
	if (configFlashIdx >= (NUM_CONFIG_BUFS_PER_SECTOR * NUM_CONFIG_BUFS_SECTORS))
	{
		configFlashIdx = 0;
	}
	c.csum = calc_region_checksum(4);
//skip for now 22feb16	write_region_to_flash(4, 0xFF, c.csum);
}

void increment_ledBoard_usage_min(void)
{
	unsigned char idx;
	unsigned char topLEDboardLowerSideIdx;
	unsigned char bottomLEDboardUpperSideIdx;
	unsigned char topUIdx;
	unsigned char bottomUIdx;
	unsigned char hourRollover = 0;
	
	inc_sanMins();

	for (unsigned char i=0; i<NUM_SHELVES; i++) //check every active shelf
	{
		if (shelf[i].active == SHELF_ACTIVE)
		{
			topLEDboardLowerSideIdx = ledBrd[shelf[i].tLedIdx].lSideIdx;
			bottomLEDboardUpperSideIdx = ledBrd[shelf[i].bLedIdx].uSideIdx;
			
			topUIdx = ledBrdSide[topLEDboardLowerSideIdx].ushdwIdx;
			bottomUIdx = ledBrdSide[bottomLEDboardUpperSideIdx].ushdwIdx;
			
			for (unsigned char k=0; k<2; k++) //for each board side in the shelf
			{
				switch (k)
				{
					case 0:
						idx = topUIdx;
						break;
					case 1:
						idx = bottomUIdx;
						break;

				}

				m.mins[idx] = m.mins[idx] + 1;
				if (m.mins[idx] > 59)
				{
					m.mins[idx] = 0;
					hourRollover++; //count number of board sides that had hours rollover this pass for the current hourPingPong selection
					h.hrs[idx] = h.hrs[idx] + 1;
						
					if ((h.hrs[idx]) >= 2000) //2000 hours * 60 minutes per hour
					{
						sf[idx].maxUsageReached = 1; //And...we're done. Reached 2000 hours.
					}
				}//if ((minutes %60) == 0)
			} //for each board side in the shelf (k)
		} //if (shelf[i].active)
	} //for (i=0; i<NUM_SHELVES; i++)
	
	mFlashIdx++;
	if (mFlashIdx >= NUM_USAGE_MINS_BUFS_SECTORS)
	{
		mFlashIdx = 0;
	}
	m.csum = calc_region_checksum(3);
//skip for now 22feb16	write_region_to_flash(3, 0xFF, m.csum);
//skip for now 22feb16	copy_region_to_another_sector(3);
//skip for now 22feb16	disrupt_prior_m_sector();

	if (hourRollover)
	{
		hFlashIdx++;
		if (hFlashIdx >= (NUM_USAGE_HOURS_BUFS_SECTORS/NUM_USAGE_HOURS_SECTORS_PER_BUF))
		{
			hFlashIdx = 0;
		}
		h.csum = calc_region_checksum(2);
//skip for now 22feb16		write_region_to_flash(2, 0xFF, h.csum);

		hourRollover = 0; //reset for next pass
	}
}



void init_shelf_n_ledBrd_structs(void);
void init_shelf_n_ledBrd_structs(void)
{
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelf[i].present = 0;
		shelf[i].devicesPresent = 0;
		shelf[i].active = 0;
	}
	
	shelf[0].tLedIdx = 0;
	shelf[0].bLedIdx = 1;
	shelf[1].tLedIdx = 1;
	shelf[1].bLedIdx = 2;
	shelf[2].tLedIdx = 2;
	shelf[2].bLedIdx = 3;
	shelf[3].tLedIdx = 3;
	shelf[3].bLedIdx = 4;
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		ledBrd[i].present = 0;
	}
	
	ledBrd[0].uSideIdx = 0xFF;
	ledBrd[0].lSideIdx = 0;
	ledBrd[1].uSideIdx = 1;
	ledBrd[1].lSideIdx = 2;
	ledBrd[2].uSideIdx = 3;
	ledBrd[2].lSideIdx = 4;
	ledBrd[3].uSideIdx = 5;
	ledBrd[3].lSideIdx = 6;
	ledBrd[4].uSideIdx = 7;
	ledBrd[4].lSideIdx = 0xFF;

	ledBrd[0].uSideShelfIdx = 0xFF;
	ledBrd[1].uSideShelfIdx = 0;
	ledBrd[2].uSideShelfIdx = 1;
	ledBrd[3].uSideShelfIdx = 2;
	ledBrd[4].uSideShelfIdx = 3;
 
	ledBrd[0].lSideShelfIdx = 0;
	ledBrd[1].lSideShelfIdx = 1;
	ledBrd[2].lSideShelfIdx = 2;
	ledBrd[3].lSideShelfIdx = 3;
	ledBrd[4].lSideShelfIdx = 0xFF;
	

	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].maxUsageReached = 0;
		ledBrdSide[i].sanitizeMinutes = 0;
		ledBrdSide[i].ushdwIdx = 0xFF;
	}
	
	ledBrdSide[0].boardIdx = 0;
	ledBrdSide[1].boardIdx = 1;
	ledBrdSide[2].boardIdx = 1;
	ledBrdSide[3].boardIdx = 2;
	ledBrdSide[4].boardIdx = 2;
	ledBrdSide[5].boardIdx = 3;
	ledBrdSide[6].boardIdx = 3;
	ledBrdSide[7].boardIdx = 4;
	

	ledBrdSide[0].shelfIdx = 0;
	ledBrdSide[1].shelfIdx = 0;
	ledBrdSide[2].shelfIdx = 1;
	ledBrdSide[3].shelfIdx = 1;
	ledBrdSide[4].shelfIdx = 2;
	ledBrdSide[5].shelfIdx = 2;
	ledBrdSide[6].shelfIdx = 3;
	ledBrdSide[7].shelfIdx = 3;
	
	for (int j=0; j<NUM_LED_BOARD_SIDES; j++)
	{
		usageIdx[j] = NO_LED_BOARD_PRESENT;
	}

}


void load_usageIdx_to_ledBrdSide(void);
void load_usageIdx_to_ledBrdSide(void)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].ushdwIdx = usageIdx[i];
	}
}


void init_led_board_info(void);
void init_led_board_info(void)
{
	unsigned char regionGood[5];
	unsigned char csum;

#if 0 //23feb16 force this flash region eval to be bad, we want the defaults loaded in the structures, put this back in later
	for (int i=0; i<5; i++)
	{
		regionGood[i] = eval_region(i);
	}
	
	if (regionGood[0] && regionGood[1] && regionGood[2] && regionGood[3] && regionGood[4])
	{
		print_ecdbg("All 5 flash regions have good data sets.\r\n");

		load_usage_indeces();
		
		add_new_led_board_sides_to_usage();
		load_usageIdx_to_ledBrdSide();

		//serial ID and flags
		csum = calc_region_checksum(0);
//skip for now 22feb16		write_region_to_flash(0, 0xFF, csum);
//skip for now 22feb16		copy_region_to_another_sector(0);

		//san cycles
		csum = calc_region_checksum(1);
//skip for now 22feb16		write_region_to_flash(1,  0xFF, csum);
//skip for now 22feb16		copy_region_to_another_sector(1);

		//usage hours
		csum = calc_region_checksum(2);
//skip for now 22feb16		write_region_to_flash(2,  0xFF, csum);
//skip for now 22feb16		copy_region_to_another_sector(2);

		//usage minutes
		csum = calc_region_checksum(3);
//skip for now 22feb16		write_region_to_flash(3,  0xFF, csum);
//skip for now 22feb16		copy_region_to_another_sector(3);

		//configuration
		csum = calc_region_checksum(4);
//skip for now 22feb16		write_region_to_flash(4,  0xFF, csum);
//skip for now 22feb16		copy_region_to_another_sector(4);

	}
	else
	{
#endif //23feb16 force this flash region eval to be bad, we want the defaults loaded in the structures, put this back in later
		memset(&sf, 0x00, sizeof(sf));		//serial id's and flags
		memset(&sanc, 0x00, sizeof(sanc));	//total chassis sanitation cycles
		memset(&h, 0x00, sizeof(h));		//usage hours
		memset(&m, 0x00, sizeof(m));		//usage minutes
		memset(&c, 0x00, sizeof(c));		//configuration
		c.initialDTE = 30; //changed to 30 minutes for a sanitation test for new LEDs 16jan16

		for (int i=0; i<5; i++)
		{
#if 0 //22feb16 skip this for now			
			if (test_flash(i) == ERROR)
			{
				print_ecdbg("Flash area ERROR: region ");
				print_ecdbg_num(i);
				print_ecdbg("\r\n");
				sysErr.flashArea |= BIT(i); //SE_FAIL;
				chassis_error();
			}
#endif
		}
		add_new_led_board_sides_to_usage();
		load_usageIdx_to_ledBrdSide();

		for (int i=0; i<5; i++)
		{
			unsigned char csum;
			csum = calc_region_checksum(i);
//skip for now 22feb16			write_region_to_flash(i,  0xFF, csum);
//skip for now 22feb16			copy_region_to_another_sector(i);
		}
#if 0 //23feb16 force this flash region eval to be bad, we want the defaults loaded in the structures, put this back in later
	} //if-else
#endif //23feb16 force this flash region eval to be bad, we want the defaults loaded in the structures, put this back in later	
}


void show_sw_version(void);
void show_sw_version(void)
{
	print_ecdbg("\r\n*---------------------------------------------------*\r\n");
	print_ecdbg(    "ELECTROCLAVE\r\nCopyright (c) 2016 Seal Shield, Inc. \r\n");
	print_ecdbg(    "Hardware Version: Classic +++ Software Version: 0.078\r\n");

}

void show_chassis_status_info(void);
void show_chassis_status_info(void)
{
	char pStr[80];
	unsigned char uSideIdx, lSideIdx, uSideUsageIdx, lSideUsageIdx;
	unsigned char sanMinutesMax = 0, sanMinutesMin = 0xFF, sanMinutesUpper, sanMinutesLower, uMins, lMins;
	unsigned int uHrs, lHrs;
	
	print_ecdbg("\r\n***INSTALLED LED BOARDS***\r\n\r\n");
	
	print_ecdbg(" LED | LED BOARD    |   UPPER SIDE     |   LOWER SIDE    \r\n");
	print_ecdbg("SLOT |    ID        | HRS:MIN    DTE   | HRS:MIN    DTE   \r\n");
	print_ecdbg("----------------------------------------------------------\r\n");
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if (ledBrd[i].present)
		{
			uSideIdx = ledBrd[i].uSideIdx;
			lSideIdx = ledBrd[i].lSideIdx;
			
			if (uSideIdx != NO_LED_BOARD_PRESENT)
			{
				uSideUsageIdx = ledBrdSide[uSideIdx].ushdwIdx;	
				ledBrdSide[uSideIdx].maxUsageReached = !check_led_brd_side_lifetime(uSideIdx);
				sanMinutesUpper = ledBrdSide[uSideIdx].sanitizeMinutes;
				uHrs = h.hrs[uSideUsageIdx];
				uMins = m.mins[uSideUsageIdx];
			}
			else
			{
				uHrs = 0;
				uMins = 0;
				sanMinutesUpper = 0;
			}
			
			if (lSideIdx != NO_LED_BOARD_PRESENT)
			{
				lSideUsageIdx = ledBrdSide[lSideIdx].ushdwIdx;	
				ledBrdSide[lSideIdx].maxUsageReached = !check_led_brd_side_lifetime(lSideIdx);
				sanMinutesLower = ledBrdSide[lSideIdx].sanitizeMinutes;
				lHrs = h.hrs[lSideUsageIdx];
				lMins = m.mins[lSideUsageIdx];
			}
			else
			{
				lHrs = 0;
				lMins = 0;
				sanMinutesLower = 0;
			} 
			
			
			sprintf(pStr, "%2d     %02X%02X%02X%02X%02X%02X  %04d:%02d     %02d     %04d:%02d     %02d\r\n", 
				i, 
				ledBrd[i].id[0], ledBrd[i].id[1], ledBrd[i].id[2], ledBrd[i].id[3], ledBrd[i].id[4], ledBrd[i].id[5],
				uHrs, uMins,
				sanMinutesUpper,
				lHrs, lMins,
				sanMinutesLower);
			print_ecdbg(pStr);
			

			/* 
			 * Determine the min and max sanitize times for the LED boards that are currently installed
			 */
			if ((sanMinutesMax < sanMinutesUpper) && (sanMinutesUpper != 0))
			{
				sanMinutesMax = sanMinutesUpper;
			}
			if ((sanMinutesMax < sanMinutesLower) && (sanMinutesLower != 0))
			{
				sanMinutesMax = sanMinutesLower;
			}
			if ((sanMinutesMin > sanMinutesUpper) && (sanMinutesUpper != 0))
			{
				sanMinutesMin = sanMinutesUpper;
			}
			if ((sanMinutesMin > sanMinutesLower) && (sanMinutesLower != 0))
			{
				sanMinutesMin = sanMinutesLower;
			}
		}
	}
	
	print_ecdbg("MAX DTE: ");
	print_ecdbg_num(sanMinutesMax);
	print_ecdbg(" MIN DTE: ");
	print_ecdbg_num(sanMinutesMin);
	print_ecdbg("\r\n");
	
	print_ecdbg("TOTAL SANITIZE TIME: ");
	if ((m.sanMins/60) < 10)
	{
		print_ecdbg("0"); //print leading 0 if we need it
	}
	print_ecdbg_num((m.sanMins/60));
	print_ecdbg(":");

	if ((m.sanMins%60) < 10)
	{
		print_ecdbg("0"); //print leading 0 if we need it
	}
	print_ecdbg_num((m.sanMins%60));

	print_ecdbg("  TOTAL SANITIZE CYCLES: ");
	print_ecdbg_num(sanc.cycles);

	print_ecdbg("\r\n");
	
}


void show_chassis_sysErr(void);
void show_chassis_sysErr(void)
{
	char str[80];
	

	print_ecdbg("\r\n***SYSTEM TESTS***\r\n\r\n");


/*
 *	LED Driver: Top
 */
	sprintf(str, "LED Driver: TOP (0..7)                 ");
	
	for (int i=0; i<8; i++)
	{
		if ((sysErr.topdrive & BIT(i)))
		{
			strcat(str,"F ");			
//DEBUG 24jun15 need to function even with these errors for demo purposes			electroclaveState = STATE_CHASSIS_ERROR;
		}
		else
		{
			strcat(str,"P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 *	LED Driver: Bottom
 */
	sprintf(str, "LED Driver: BOTTOM (0..11)             ");
	
	for (int i=0; i<12; i++)
	{
		if ((sysErr.botdrive & BIT(i)))
		{
			strcat(str,"F ");			
//DEBUG 24jun15 need to function even with these errors for demo purposes			electroclaveState = STATE_CHASSIS_ERROR;
		}
		else
		{
			strcat(str,"P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 *	Flash
 */

	sprintf(str, "Flash (0..4)                           ");
	
	for (int i=0; i<5; i++)
	{
		if ((sysErr.flashArea & BIT(i)) == SE_FAIL)
		{
			strcat(str, "F ");
			electroclaveState = STATE_CHASSIS_ERROR;
		}
		else
		{
			strcat(str, "P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 * LED board serial ID checksums
 */	
	sprintf(str, "LED Board Serial ID Checksums (0..4)   ");
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if ((sysErr.ledBrdSerialIdCsum & BIT(i)) == SE_FAIL)
		{
			strcat(str, "F ");
			electroclaveState = STATE_CHASSIS_ERROR;
		}
		else
		{
			strcat(str, "P ");
		}
	}

	print_ecdbg(str);
	print_ecdbg("\r\n");
	

/*
 * LED Board Side Max Usage Reached
 */
	sprintf(str, "LED Board Side Max Usage (0..7)        ");
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		if (ledBrdSide[i].maxUsageReached)
		{
			strcat(str, "F ");
//DEBUG 24jun15 need to function even with these errors for demo purposes			electroclaveState = STATE_CHASSIS_ERROR;
		}
		else
		{
			strcat(str, "P ");
		}
	}

	print_ecdbg(str);
	print_ecdbg("\r\n");
	

/*
 * Usage Struct Full
 */

	sprintf(str, "Usage Struct Open Slots                ");
	if (sysErr.usageStructsFull == SE_FAIL)
	{
		strcat(str, "F \r\n");
		electroclaveState = STATE_CHASSIS_ERROR;
	}
	else
	{
		strcat(str, "P \r\n");
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");

}

void show_chassis_all_LED_boards(void);
void show_chassis_all_LED_boards(void)
{
	char str[80];
	int i = 0;

	print_ecdbg("\r\n***LED BOARDS MASTER LIST***\r\n\r\n");
	
	while(1)
	{
		if (sf[i].slotFilled)
		{
			sprintf(str, "%2d) %02X%02X%02X%02X%02X%02X ", i,
			sf[i].id[0],sf[i].id[1],sf[i].id[2],sf[i].id[3],sf[i].id[4],sf[i].id[5]);
			
			if (sf[i].top_botn)
			{
				strcat(str, " TOP\r\n");
			}
			else
			{
				strcat(str, " BOT\r\n");
			}
			
			print_ecdbg(str);
		}
		else
		{
			break; //LED boards are stored contiguously, so if we hit a blank spot we are done with the entries in the list
		}
		i++;
		
	}
	
	print_ecdbg("\r\n\r\n");

}

void show_help_and_prompt(void);
void show_help_and_prompt(void)
{
	print_ecdbg("Type 'H' for help.\r\n\r\n");
}

char cmd[20];
unsigned char cmdIdx = 0;

void service_ecdbg_input(void);
void service_ecdbg_input(void)
{
	int rx_char;
	unsigned int tmpNewDte;
	unsigned char tryToChangeDte = 0;
	
	
	if (usart_is_rx_ready(BOARD_USART)) {
		usart_read(BOARD_USART, (uint32_t *)&rx_char);
	}
	else
	{
		return;
	}


	if (rx_char == USART_FAILURE)
	{
//26may15 why are we getting this? ignore for now		usart_write_line(ECDBG_USART, "UART error\r\n");
		return;
	}
	if (rx_char == '\x03')
	{
		return;
	}
	
	if ((rx_char < 0x0a) || (rx_char > 0x7a))
	{
		return; //completely out of range, ignore
	}
	
	
	if ((rx_char == 0x0d) ||							//carriage return
		(rx_char == 0x0a) ||							//line feed
		(rx_char == 0x20) ||							//space
		((rx_char >= 0x30) && (rx_char <= 0x39)) ||		//decimal number
		((rx_char >= 0x41) && (rx_char <= 0x5a)) ||		//upper case alpha
		((rx_char >= 0x61) && (rx_char <= 0x7a)))		//lower case alpha
	{
		if (rx_char == 0x50)
		{
			return; //TODO: this is kludgey...whenever we print to the debug port we rx a 'P' (0x50), just ignore them for now.
		}
	}
	else
	{
		return;
	}


	cmd[cmdIdx++] = rx_char;
	
	print_ecdbg(&rx_char);
	if (rx_char == '\r')
	{ 
		if (cmdIdx == 2)
		{
			switch(cmd[0])
			{
				case 'H':
				case 'h':
					print_ecdbg("\r\n**-----------------**\r\n");
					print_ecdbg("  Electroclave HELP\r\n");
					print_ecdbg("**-----------------**\r\n");
					print_ecdbg("    H        - This help menu\r\n");
					print_ecdbg("    D        - Show current DTE setting\r\n");
					print_ecdbg("    D  xx    - Change initial DTE to xx minutes where 2 >= xx >= 59.\r\n");
					print_ecdbg("    S        - System status\r\n");
					print_ecdbg("**-----------------**\r\n");
					print_ecdbg(">");
					break;
				case 'D':
				case 'd':
					print_ecdbg("Initial DTE set to: ");
					print_ecdbg_num(c.initialDTE);
					print_ecdbg(" minutes.\r\n>");
					break;
				case 'S':
				case 's':
					show_sw_version();
					show_chassis_status_info();
					show_chassis_sysErr();
					show_chassis_all_LED_boards();
					show_help_and_prompt();
					break;
				case 'K':
				case 'k':
					print_ecdbg("Valid Keypad Code\r\n");
					validKeypadCode = 1;
					break;
				case 'T':
				case 't':
					print_ecdbg("Start button pressed\r\n");
					startButtonPressed = 1;
					break;
			}
		}
		else if (cmd[1] == ' ')
		{
			if ((cmd[0] == 'D') || (cmd[0] == 'd'))
			{
				if (cmdIdx == 4)
				{
					if (isdigit(cmd[2]))
					{
						tmpNewDte = cmd[2] - 0x30;
						tryToChangeDte = 1;
					}					
				}
				else if (cmdIdx == 5)
				{
					if (isdigit(cmd[2]) && (isdigit(cmd[3])))
					{
						tmpNewDte = (cmd[2]-0x30) * 10;
						tmpNewDte += (cmd[3] - 0x30);
						tryToChangeDte = 1;
					}
				}
				if (tryToChangeDte)
				{
					if ((tmpNewDte < 60) && (tmpNewDte > 1))
					{
						print_ecdbg("Initial DTE now set to: ");
						print_ecdbg_num(tmpNewDte);
						print_ecdbg("\r\n>");
						
						c.initialDTE = tmpNewDte;
						
						store_config();
						store_config(); //do this twice to store it in both buffers to make extra sure we got it
						
					}
					else
					{
						print_ecdbg("Error. Initial DTE not modified. \r\n");
						print_ecdbg("Must be a value between 2 and 59.\r\n>");
					}
				}
			}
		}
		
		// Add a LF and consider this as the end of the line.
		print_ecdbg("\r\n>");
		cmdIdx = 0;
		return;
	}
}

#define EXAMPLE_LED_GPIO    LED0_GPIO


/*! \brief Main File Section:
 *          - Initialization (CPU, TWI, Usart,...)
 */
int main(void){
	static unsigned char displayIdx = 0;
	char mainStr[80];
	uint8_t kpbResult;
	uint8_t firstTimeSinceShutdown = 1;
	
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	init_io();
	
	/* 1ms tick. */
	configure_systick();

	/* Configure UART for blue scrolling display */
	configure_console();

	/* Configure USART. */
	configure_usart();

	init_sysErr();
	
	init_shelf_n_ledBrd_structs();
	read_led_board_serial_ids();
		
//make this ecII jsi 7feb16	gpio_set_pin_high(ECLAVE_LED_OEn); //make sure outputs are disabled at the chip level

	/*
	 * Enable transmitter here, and disable receiver first, to avoid receiving
	 * characters sent by itself. It's necessary for half duplex RS485.
	 */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	show_sw_version();

	ioport_set_pin_level(ECLAVE_LED_OEn, IOPORT_PIN_LEVEL_HIGH); //make sure outputs are disabled at the chip level

	
	init_led_board_info();

	twi_init();
	PCA9952_init();
	test_led_driver_channels();


	show_chassis_status_info();
	show_chassis_sysErr();
	show_chassis_all_LED_boards();
	show_help_and_prompt();
	
	ioport_set_pin_level(ECLAVE_LED_OEn, IOPORT_PIN_LEVEL_LOW); //...and we are live!
	ioport_set_pin_level(ECLAVE_PSUPPLY_ONn, IOPORT_PIN_LEVEL_LOW);


	init_pwm();
	
	init_adc();
	
	controls.buzzer_enable = 0;
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL); //for the love of christ turn this off


	// Print Startup Message
	display_text(IDX_READY);
	

	start_timer(TMR_DEBUG, ((1*SECONDS)/2));


	// Main loop
	while (true) 
	{
		switch(electroclaveState)
		{
			case STATE_EC_IDLE:
				if (EC_DOOR_LATCHED && firstTimeSinceShutdown) {
					controls.buzzer_enable = 0;
					pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
					ioport_set_pin_level(EXAMPLE_LED_GPIO, IOPORT_PIN_LEVEL_LOW);
					print_ecdbg("Door latch detected\r\n");
					firstTimeSinceDoorLatched = 1;
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
					firstDoorOpenSinceIdle = 1;
					firstTimeSinceShutdown = 0;
				}

				if (kpbResult == KPB_VALID)
//jsi debug 25feb16 temporarily leave this out to debug the rest of the state machine				if (validKeypadCode)
				{
					controls.solenoid_enable = true;
					start_timer(TMR_DOOR_OPEN, (15 * SECONDS)); //TODO: change to 2 minutes for real product
					electroclaveState = STATE_DOOR_OPEN;
					print_ecdbg("STATE_DOOR_OPEN\r\n");
					validKeypadCode = 0; //jsi 25feb16 debug
					firstTimeSinceDoorLatched = 1;
				}
				break;
				
			case STATE_DOOR_OPEN:
				if (timer_done(TMR_DOOR_OPEN))
				{
					door_ajar_buzzer();
					electroclaveState = STATE_DOOR_AJAR;
					print_ecdbg("STATE_DOOR_AJAR\r\n");

				}
				if (scanKPResult == KEYPAD_START)
//jsi 25feb16 debug				if (startButtonPressed)
				{
					if (EC_DOOR_LATCHED)
					{
						end_timer(TMR_DOOR_OPEN);
						electroclaveState = STATE_DOOR_LATCHED;
						print_ecdbg("STATE_DOOR_LATCHED\r\n");
						startButtonPressed = 0; //jsi 25feb16 debug
					}
				}
				break;
			
			case STATE_DOOR_AJAR:
				if (EC_DOOR_LATCHED)
				{
					controls.buzzer_enable = false;
					pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
					electroclaveState = STATE_EC_IDLE;
					print_ecdbg("STATE_EC_IDLE\r\n");
				}
				break;				
				
			case STATE_DOOR_LATCHED:
#if 0 //EC1 25feb16
				if (validKeypadCode) {
					controls.solenoid_enable = 1;
					print_ecdbg("Valid keypad code detected\r\n");
					electroclaveState = STATE_VALID_KEYPAD_CODE;
					validKeypadCode = 0; //reset
				}
#endif //EC1 25feb16
				if (firstTimeSinceDoorLatched)
				{
					check_led_brd_side_lifetimes();
					check_shelves_for_devices();
					set_shelves_active_inactive();
				
					firstTimeSinceDoorLatched = 0;
				}

				if (num_active_shelves() != 0) {
					electroclaveState = STATE_START_SANITIZE;
					print_ecdbg("STATE_START_SANITIZE\r\n");
					print_ecdbg("Sanitizing\r\n");
					//13jun15					display_text(IDX_CLEAR);
					//13jun15					cpu_delay_ms(500, EC_CPU_CLOCK_FREQ);
					display_text(IDX_CLEANING);
					start_timer(TMR_DISPLAY, (8 * SECONDS));
				}
				else if (num_present_shelves() != 0){
					electroclaveState = STATE_EC_IDLE;
					print_ecdbg("STATE_EC_IDLE\r\n");
					print_ecdbg("At least one shelf is present, but no devices to be cleaned.\r\n");
					display_text(IDX_READY);
				}
				else
				{
					//DEBUG 24jun15 need to function even with these errors for demo purposes					electroclaveState = STATE_CHASSIS_ERROR;
					print_ecdbg("No shelves, or shelves are past lifetime\r\n");
					display_text(IDX_ERROR);
				}
				break;
				
			case STATE_START_SANITIZE:
//13jun15				display_text(IDX_CLEAR);
//13jun15				cpu_delay_ms(500, EC_CPU_CLOCK_FREQ); //half second TODO: figure out why this is here and get rid of it, don't like to just hang for no reason, especially when we need to be monitoring the door latch
				
				displayIdx = 0xFF; //this means not assigned yet
				sanitizeMinutes = 0;
				for (int i = 0; i<NUM_SHELVES; i++) {
					if (shelf[i].active == SHELF_ACTIVE) {
						tmpSanitizeMinutes = calc_sanitize_time(i);
						
						if (tmpSanitizeMinutes > sanitizeMinutes)
						{
							sanitizeMinutes = tmpSanitizeMinutes;
						}
						
						led_shelf(i, LED_ON);
						
						if (displayIdx == 0xFF)
						{
							displayIdx = i; //set this to the first active shelf if this is the first active shelf encountered
						}
					}
				}
				
				
//16jan16 #if 0 //DEBUG: set this to seconds not minutes so we can debug this logic faster 11may15
				start_timer(TMR_SANITIZE, (sanitizeMinutes*MINUTES));
//16jan16 #endif
//16jan16 we really want minutes right now				cpu_set_timeout((sanitizeMinutes * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &sanitizeTimer); //DEBUG take this out when done debugging logic, put it back to minutes 11may15
				
				sprintf(mainStr, "sanitizeMinutes: %ld tmpSanitizeMinutes: %ld\r\n", sanitizeMinutes, tmpSanitizeMinutes);
				print_ecdbg(mainStr);


				inc_sanCycles();

				
//DEBUG 11may15 do this once per second for debug				cpu_set_timeout((60 * cpu_ms_2_cy(1000,EC_CPU_CLOCK_FREQ)), &oneMinuteTimer); //one minute for the usage statistics
				start_timer(TMR_ONE_MINUTE, (1*SECONDS)); //once per second for debug
				electroclaveState = STATE_SANITIZE;
				print_ecdbg("STATE_SANITIZE\r\n");

				break;
				
			case STATE_SANITIZE:
				/*
    			 * Manage the display
				 */
				if (timer_done(TMR_DISPLAY))
				{
					end_timer(TMR_DISPLAY);
					switch (displayIdx)
					{
						case 0:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF1);
							break;
						case 1:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF2);
							break;
						case 2:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF3);
							break;
						case 3:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF4);
							break;
					}
					
					while (1)
					{
						if (++displayIdx >= NUM_SHELVES)
						{
							displayIdx = 0; //12apr15 wrap around
						}
						
						if (shelf[displayIdx].active)
						{
							break; //this shelf is active, we don't need to look for another one
						}
						
					}

					start_timer(TMR_DISPLAY, displayTimerSeconds * SECONDS);

					//NOTE we need to be careful here, we need to be able to shut off the shelf LEDs the *instant* the door latch opens, this is important for safety
					//this means we need as little logic between turning the shelf on and turning it off so we can react as quickly as possible to the door latch
				}

				/*
    			 * Manage storing usage statistics to flash
				 */
				if (timer_done(TMR_ONE_MINUTE))
				{
					end_timer(TMR_ONE_MINUTE);
					
					increment_ledBoard_usage_min(); //increments usage minutes for active shelves only
					
//DEBUG 11may15 set to one second for debug					cpu_set_timeout(cpu_ms_2_cy(60000, EC_CPU_CLOCK_FREQ), &oneMinuteTimer); //one minute for the usage statistics
					start_timer(TMR_ONE_MINUTE, (1 * SECONDS));
				}
				/*
    			 * Manage the sanitizer timer
				 */
				if (timer_done(TMR_SANITIZE)) {
					
					for (int i=0; i< NUM_SHELVES; i++)
					{
						led_shelf(i, LED_OFF); //turn off every shelf. (doesn't hurt to make sure that even non-active shelves are off.)
					}
					end_timer(TMR_SANITIZE);
					print_ecdbg("Shelf clean\r\n");
					electroclaveState = STATE_START_CLEAN;
					print_ecdbg("STATE_START_CLEAN\r\n");
				}
				
				break;
				
			case STATE_START_CLEAN:
				display_text(IDX_CLEAN);
				electroclaveState = STATE_CLEAN;
				print_ecdbg("STATE_CLEAN\r\n");
#if 0 //DEBUG do this in seconds to debug logic 11may15				
				cpu_set_timeout((20 * 60 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &cleanTimer);
#endif
//DEBUG 24jun15 change to 60 seconds for demo, put this line back in later				cpu_set_timeout((20 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &cleanTimer); //DEBUG 11may15 

//				cpu_set_timeout((60 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &cleanTimer); //DEBUG 24jun15 change to 60 seconds for demo, remove later
				start_timer(TMR_CLEAN, (10 * SECONDS));
				break;	
				
			case STATE_CLEAN:
				if (timer_done(TMR_CLEAN)) {
					end_timer(TMR_CLEAN);
					electroclaveState = STATE_START_SANITIZE;	
					print_ecdbg("STATE_START_SANITIZE\r\n");
				}
				break;
				
			
			case STATE_CHASSIS_ERROR:
				//Shutdown all processes that could harm the user or equipment if the door is open
				for (int i=0; i< NUM_SHELVES; i++)
				{
					led_shelf(i, LED_OFF); //turn off every shelf. (doesn't hurt to make sure that even non-active shelves are off.)
				}
				
				
				if (timer_done(TMR_ERROR_DISPLAY))
				{
					end_timer(TMR_ERROR_DISPLAY);

					while(1)
					{
						switch(errorDisplayState)
						{
							case 0:
								display_text(IDX_ERROR);
								displayChanged = 1;
								start_timer(TMR_ERROR_DISPLAY, (8 * SECONDS));
								errorDisplayState = 1;
								break;
							case 1:
								if ((ledBrdSide[LED_BRD_0_BOT].maxUsageReached || ledBrdSide[LED_BRD_1_TOP].maxUsageReached) ||
									(sysErr.topdrive & BIT(0)) || (sysErr.topdrive & BIT(1)) ||
									(sysErr.botdrive & BIT(0)) || (sysErr.botdrive & BIT(1)) || (sysErr.botdrive & BIT(2)))
								{
									display_text(IDX_SHELF1);
									displayChanged = 1;
									start_timer(TMR_ERROR_DISPLAY, (8 * SECONDS));
								}
								errorDisplayState = 2;
								break;
							case 2:
								if ((ledBrdSide[LED_BRD_1_BOT].maxUsageReached || ledBrdSide[LED_BRD_2_TOP].maxUsageReached) ||
									(sysErr.topdrive & BIT(2)) || (sysErr.topdrive & BIT(3)) ||
									(sysErr.botdrive & BIT(3)) || (sysErr.botdrive & BIT(4)) || (sysErr.botdrive & BIT(5)))
								{
									display_text(IDX_SHELF2);
									displayChanged = 1;
									start_timer(TMR_ERROR_DISPLAY, (8 * SECONDS));
								}
								errorDisplayState = 3;
								break;
							case 3:
								if ((ledBrdSide[LED_BRD_2_BOT].maxUsageReached || ledBrdSide[LED_BRD_3_TOP].maxUsageReached) ||
									(sysErr.topdrive & BIT(4)) || (sysErr.topdrive & BIT(5)) ||
									(sysErr.botdrive & BIT(6)) || (sysErr.botdrive & BIT(7)) || (sysErr.botdrive & BIT(8)))
								{
									display_text(IDX_SHELF3);
									displayChanged = 1;
									start_timer(TMR_ERROR_DISPLAY, (8 * SECONDS));
								}
								errorDisplayState = 4;
								break;
							case 4:
								if ((ledBrdSide[LED_BRD_3_BOT].maxUsageReached || ledBrdSide[LED_BRD_4_TOP].maxUsageReached) ||
									(sysErr.topdrive & BIT(6)) || (sysErr.topdrive & BIT(7)) ||
									(sysErr.botdrive & BIT(9)) || (sysErr.botdrive & BIT(10)) || (sysErr.botdrive & BIT(11)))
								{
									display_text(IDX_SHELF4);
									displayChanged = 1;
									start_timer(TMR_ERROR_DISPLAY, (8 * SECONDS));
								}
								errorDisplayState = 0;
								break;
							default:
								errorDisplayState = 0;
								break;
						} //switch(errorDisplayState)
						
						if (displayChanged)
						{
							displayChanged = 0;
							break; //get out of while loop and wait until we need to update the display again
						}
						
					} //while(1)

				} //if (timer_done(TMR_ERROR_DISPLAY))
				break;
				
			case STATE_SHUTDOWN_PROCESSES:
				//Shutdown all processes that could harm the user or equipment if the door is open
				for (int i=0; i< NUM_SHELVES; i++)
				{
					led_shelf(i, LED_OFF); //turn off every shelf. (doesn't hurt to make sure that even non-active shelves are off.)
				}
				if (timer_done(TMR_DIRTY))
				{
					end_timer(TMR_DIRTY);
					electroclaveState = STATE_EC_IDLE;
					print_ecdbg("STATE_EC_IDLE\r\n");
				}
				firstTimeSinceShutdown = 1;
				break;
		} //switch(electroclaveState)
		
		/*
		 * This check overrides everything going on in the state machine, if the user opens the door,
		 * shut down all processes for safety
		 */
		if (((kpbResult = process_kpb()) == KPB_VALID)  && (electroclaveState != STATE_EC_IDLE))
		{
			controls.solenoid_enable = true;
			start_timer(TMR_DOOR_OPEN, (15 * SECONDS)); //TODO: change to 2 minutes for real product


			if (firstDoorOpenSinceIdle)
			{
				door_latch_open_kill_all_shelves();

//13jun15				display_text(IDX_CLEAR);
//13jun15				cpu_delay_ms(500, EC_CPU_CLOCK_FREQ);
				switch (electroclaveState)
				{
					case STATE_START_SANITIZE:
					case STATE_SANITIZE:
						display_text(IDX_DIRTY);
						electroclaveState = STATE_SHUTDOWN_PROCESSES;
						print_ecdbg("STATE_SHUTDOWN_PROCESSES\r\n");
						print_ecdbg("Door latch opened, shutting down all processes\r\n");
						start_timer(TMR_DIRTY, (8 * SECONDS));
						break;
						
					case STATE_START_CLEAN:
					case STATE_CLEAN:
						display_text(IDX_CLEAN);
						electroclaveState = STATE_SHUTDOWN_PROCESSES;
						print_ecdbg("STATE_SHUTDOWN_PROCESSES\r\n");
						print_ecdbg("Door latch opened, shutting down all processes\r\n");
						break;
						
					case STATE_CHASSIS_ERROR:
						display_text(IDX_ERROR);
						break;
					
					default:
						display_text(IDX_READY);
						electroclaveState = STATE_SHUTDOWN_PROCESSES;
						print_ecdbg("STATE_SHUTDOWN_PROCESSES\r\n");
						print_ecdbg("Door latch opened, shutting down all processes\r\n");
						break;
				}

				firstDoorOpenSinceIdle = 0;
				
			}
		} //if (!EC_DOOR_LATCHED)
		
		if (timer_done(TMR_DEBUG))
		{
			end_timer(TMR_DEBUG);
			start_timer(TMR_DEBUG, ((1 * SECONDS)/2));
			ioport_toggle_pin_level(EXAMPLE_LED_GPIO);
		}
		
		service_ecdbg_input();

	} //while(true)
	
} //main
