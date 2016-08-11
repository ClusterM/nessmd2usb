#include "defines.h"
#include <avr/io.h>
#include <util/delay.h>
#include "gamepad.h"
#include "bits.h"

#ifdef NES_ENABLED
void init_nes_gamepad()
{
	NES_PORT_DDR |= 1<<NES_LATCH_PIN; // Latch, output
	NES_PORT_DDR |= 1<<NES_CLOCK_PIN; // Clock, output
	NES_PORT_DDR &= ~(1<<NES_DATA_PIN); // Data, input
	NES_PORT_PORT |= 1<<NES_DATA_PIN; // Data, pull-up
#ifdef NES_SECOND_ENABLED	
	NES_PORT_DDR &= ~(1<<NES_DATA_PIN2); // Data, input
	NES_PORT_PORT |= 1<<NES_DATA_PIN2; // Data, pull-up
#endif
#ifdef NES_THIRD_ENABLED	
	NES_PORT_DDR &= ~(1<<NES_DATA_PIN3); // Data, input
	NES_PORT_PORT |= 1<<NES_DATA_PIN3; // Data, pull-up
#endif
#ifdef NES_FORTH_ENABLED	
	NES_PORT_DDR &= ~(1<<NES_DATA_PIN4); // Data, input
	NES_PORT_PORT |= 1<<NES_DATA_PIN4; // Data, pull-up
#endif
}

uint32_t get_nes_gamepad()
{
	uint32_t gamepad_data = 0;
	NES_PORT_PORT &= ~(1<<NES_LATCH_PIN); // Latch
	_delay_us(10);
	int b;
	for (b = 0; b < 8; b++)
	{
		NES_PORT_PORT &= ~(1<<NES_CLOCK_PIN); // Clock
		_delay_us(10);
		gamepad_data |= (((NES_PORT_PIN>>NES_DATA_PIN)&1)<<b);
#ifdef NES_SECOND_ENABLED
		gamepad_data |= (uint32_t)(((NES_PORT_PIN>>NES_DATA_PIN2)&1)<<b) << 8;
#endif
#ifdef NES_THIRD_ENABLED
		gamepad_data |= (uint32_t)(((NES_PORT_PIN>>NES_DATA_PIN3)&1)<<b) << 16;
#endif
#ifdef NES_FORTH_ENABLED
		gamepad_data |= (uint32_t)(((NES_PORT_PIN>>NES_DATA_PIN4)&1)<<b) << 24;
#endif
		NES_PORT_PORT |= 1<<NES_CLOCK_PIN; // Clock
		_delay_us(10);
	}		
	NES_PORT_PORT |= 1<<NES_LATCH_PIN; // Latch
	return gamepad_data;
}
#endif

uint32_t get_nes_gamepad_decoded(void)
{
	return ~get_nes_gamepad();
}

#ifdef SNES_ENABLED
void init_snes_gamepad()
{
	SNES_PORT_DDR |= 1<<SNES_LATCH_PIN; // Latch, output
	SNES_PORT_DDR |= 1<<SNES_CLOCK_PIN; // Clock, output
	SNES_PORT_DDR &= ~(1<<SNES_DATA_PIN); // Data, input
	SNES_PORT_PORT |= 1<<SNES_DATA_PIN; // Data, pull-up
}

uint16_t get_snes_gamepad()
{
	uint16_t gamepad_data = 0;
	SNES_PORT_PORT &= ~(1<<SNES_LATCH_PIN); // Latch
	int b;
	for (b = 0; b < 16; b++)
	{
		SNES_PORT_PORT &= ~(1<<SNES_CLOCK_PIN); // Clock
		_delay_us(10);
		gamepad_data |= ((uint16_t)((SNES_PORT_PIN>>SNES_DATA_PIN)&1)<<b);
		SNES_PORT_PORT |= 1<<SNES_CLOCK_PIN; // Clock
		_delay_us(10);
	}		
	SNES_PORT_PORT |= 1<<SNES_LATCH_PIN; // Latch
	return gamepad_data;
}
#endif

#ifdef N64_ENABLED
void init_n64_gamepad()
{
	TCCR0 |= _BV(CS00); // Timer 
	N64_PORT_DDR &= ~(1<<N64_DATA_PIN); // Input
	N64_PORT_PORT &= ~(1<<N64_DATA_PIN); // No pull-up (using external resistor)
}

int get_n64_gamepad(uint8_t* data)
{
	int b, bit;
	N64SEND_0; N64SEND_0; N64SEND_0; N64SEND_0; N64SEND_0; N64SEND_0; N64SEND_0; N64SEND_1; N64SEND_STOP;
	for (b = 0; b < 4; b++)
	{
		data[b] = 0;
		for (bit = 0; bit < 8; bit++)
		{		
			TCNT0 = 0;
			while (!N64SIGNAL) if (TCNT0 >= 0xF0) return 0;
			TCNT0 = 0;
			while(N64SIGNAL) if (TCNT0 >= 0xF0) return 0;
			data[b] = data[b]<<1;
			if (TCNT0 < 0x24 * F_CPU / 20000000UL) data[b] |= 1;
		}
	}
	return 1;
}
#endif

#ifdef SMD_ENABLED
void init_smd_gamepad()
{
	SMD_SELECT_PORT_DDR |= 1<<SMD_SELECT_PIN; // Select, output
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA0_PIN); // Data 0, input
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA1_PIN); // Data 1, input
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA2_PIN); // Data 2, input
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA3_PIN); // Data 3, input
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA4_PIN); // Data 4, input
	SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA5_PIN); // Data 5, input
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA0_PIN; // Data 0, pull-up
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA1_PIN; // Data 1, pull-up
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA2_PIN; // Data 2, pull-up
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA3_PIN; // Data 3, pull-up
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA4_PIN; // Data 4, pull-up
	SMD_DATA_PORT_PORT |= 1<<SMD_DATA5_PIN; // Data 5, pull-up
#ifdef SMD_SECOND_ENABLED	
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA0_PIN2); // Data 0, input
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA1_PIN2); // Data 1, input
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA2_PIN2); // Data 2, input
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA3_PIN2); // Data 3, input
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA4_PIN2); // Data 4, input
	SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA5_PIN2); // Data 5, input
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA0_PIN2; // Data 0, pull-up
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA1_PIN2; // Data 1, pull-up
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA2_PIN2; // Data 2, pull-up
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA3_PIN2; // Data 3, pull-up
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA4_PIN2; // Data 4, pull-up
	SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA5_PIN2; // Data 5, pull-up
#endif
}

uint32_t get_smd_gamepad()
{
	uint8_t gamepad_data_low = 0xFF;
	uint8_t gamepad_data_high = 0xFF;
	uint8_t gamepad_data_low2 = 0xFF;
	uint8_t gamepad_data_high2 = 0xFF;
	SMD_SELECT_PORT_PORT &= ~(1<<SMD_SELECT_PIN); // Select - low
	_delay_us(50);
	gamepad_data_low = ((SMD_DATA_PORT_PIN>>SMD_DATA0_PIN)&1) 
		| (((SMD_DATA_PORT_PIN>>SMD_DATA1_PIN)&1)<<1) 
		| (((SMD_DATA_PORT_PIN>>SMD_DATA2_PIN)&1)<<2)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA3_PIN)&1)<<3)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA4_PIN)&1)<<4)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA5_PIN)&1)<<5);
#ifdef SMD_SECOND_ENABLED
	gamepad_data_low2 = ((SMD_DATA_PORT_PIN2>>SMD_DATA0_PIN2)&1) 
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA1_PIN2)&1)<<1) 
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA2_PIN2)&1)<<2)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA3_PIN2)&1)<<3)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA4_PIN2)&1)<<4)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA5_PIN2)&1)<<5);
#endif
	SMD_SELECT_PORT_PORT |= 1<<SMD_SELECT_PIN; // Select - high
	_delay_us(50);
	gamepad_data_high = ((SMD_DATA_PORT_PIN>>SMD_DATA0_PIN)&1) 
		| (((SMD_DATA_PORT_PIN>>SMD_DATA1_PIN)&1)<<1) 
		| (((SMD_DATA_PORT_PIN>>SMD_DATA2_PIN)&1)<<2)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA3_PIN)&1)<<3)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA4_PIN)&1)<<4)
		| (((SMD_DATA_PORT_PIN>>SMD_DATA5_PIN)&1)<<5);
#ifdef SMD_SECOND_ENABLED
	gamepad_data_high2 = ((SMD_DATA_PORT_PIN2>>SMD_DATA0_PIN2)&1) 
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA1_PIN2)&1)<<1) 
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA2_PIN2)&1)<<2)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA3_PIN2)&1)<<3)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA4_PIN2)&1)<<4)
		| (((SMD_DATA_PORT_PIN2>>SMD_DATA5_PIN2)&1)<<5);
#endif
	return ((uint32_t)gamepad_data_high2<<24) | ((uint32_t)gamepad_data_low2<<16) | ((uint32_t)gamepad_data_high<<8) | gamepad_data_low;
}
#endif

#ifdef SMD_USE_DENDY9_PIN
void init_dendy_9pin(uint8_t n)
{ // 3, 4, 6
	if (!n)
	{
		SMD_DATA_PORT_DDR &= ~(1<<SMD_DATA1_PIN); // Data 1 aka data, input
		SMD_DATA_PORT_DDR |= 1<<SMD_DATA2_PIN; // Data 2 aka latch, output
		SMD_DATA_PORT_DDR |= 1<<SMD_DATA3_PIN; // Data 3 aka clock, output
		SMD_DATA_PORT_DDR |= 1<<SMD_DATA4_PIN; // Data 4 aka VCC, output
		SMD_DATA_PORT_PORT |= 1<<SMD_DATA1_PIN; // Data 1 aka data, pull-up
		SMD_DATA_PORT_PORT |= 1<<SMD_DATA2_PIN; // Data 2 aka latch, hi
		SMD_DATA_PORT_PORT |= 1<<SMD_DATA3_PIN; // Data 3 aka clock, hi
		SMD_DATA_PORT_PORT |= 1<<SMD_DATA4_PIN; // Data 4 aka VCC, hi
	} else {
#ifdef SMD_SECOND_ENABLED
		SMD_DATA_PORT_DDR2 &= ~(1<<SMD_DATA1_PIN2); // Data 1 aka data, input
		SMD_DATA_PORT_DDR2 |= 1<<SMD_DATA2_PIN2; // Data 2 aka latch, output
		SMD_DATA_PORT_DDR2 |= 1<<SMD_DATA3_PIN2; // Data 3 aka clock, output
		SMD_DATA_PORT_DDR2 |= 1<<SMD_DATA4_PIN2; // Data 4 aka VCC, output
		SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA1_PIN2; // Data 1 aka data, pull-up
		SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA2_PIN2; // Data 2 aka latch, hi
		SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA3_PIN2; // Data 3 aka clock, hi
		SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA4_PIN2; // Data 4 aka VCC, hi
#endif
	}
}

uint8_t get_dendy_9pin(uint8_t n)
{
	uint8_t gamepad_data = 0;
	if (!n)
	{
		SMD_DATA_PORT_PORT &= ~(1<<SMD_DATA2_PIN); // Data 2 aka latch, low
		_delay_us(10);
		int b;
		for (b = 0; b < 8; b++)
		{
			SMD_DATA_PORT_PORT &= ~(1<<SMD_DATA3_PIN); // Data 3 aka clock, low
			_delay_us(10);
			gamepad_data |= (((SMD_DATA_PORT_PIN>>SMD_DATA1_PIN)&1)<<b);
			SMD_DATA_PORT_PORT |= 1<<SMD_DATA3_PIN; // Data 3 aka clock, hi
			_delay_us(10);
		}		
		SMD_DATA_PORT_PORT |= 1<<SMD_DATA2_PIN; // Data 2 aka latch, hi
	} else {
#ifdef SMD_SECOND_ENABLED
		SMD_DATA_PORT_PORT2 &= ~(1<<SMD_DATA2_PIN2); // Data 2 aka latch, low
		_delay_us(10);
		int b;
		for (b = 0; b < 8; b++)
		{
			SMD_DATA_PORT_PORT2 &= ~(1<<SMD_DATA3_PIN2); // Data 3 aka clock, low
			_delay_us(10);
			gamepad_data |= (((SMD_DATA_PORT_PIN2>>SMD_DATA1_PIN2)&1)<<b);
			SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA3_PIN2; // Data 3 aka clock, hi
			_delay_us(10);
		}		
		SMD_DATA_PORT_PORT2 |= 1<<SMD_DATA2_PIN2; // Data 2 aka latch, hi
#endif
	}
	return gamepad_data;
}
#endif

uint32_t get_smd_gamepad_decoded(void)
{
	uint32_t result = 0;
	uint8_t smd_detected[2] = {0, 0};
	uint8_t b, c, d;	
	for (c = 0; c < 4; c++)
	{
		uint32_t smd_gamepad_data = get_smd_gamepad();
		for (d = 0; d < 2; d++) // for each controller
		{			
			if ((smd_gamepad_data & 0b00001111) || (c < 2)) // 3-button mode
			{
				for (b = 0; b <= 13; b++)
				{
					if (!((smd_gamepad_data>>b)&1))
					{
						switch (b)
						{
							case 0: // Up
								set_bit(result, 8 + d*16);
								break;
							case 1: // Down
								set_bit(result, 9 + d*16);
								break;
							case 2: // always low
							case 3:
								smd_detected[d] = 1;
								break;
							case 4: // A
								set_bit(result, 0 + d*16);
								break;
							case 5: // Start
								set_bit(result, 6 + d*16);
								break;
							case 10: // Left
								set_bit(result, 10 + d*16);
								break;
							case 11: // Right
								set_bit(result, 11 + d*16);
								break;
							case 12: // B
								set_bit(result, 1 + d*16);
								break;
							case 13: // C
								set_bit(result, 2 + d*16);
								break;
						}
					}
				}
			} else { // 6-button mode
				for (b = 4; b <= 11; b++)
				{
					if (!((smd_gamepad_data>>b)&1))
					{
						switch (b)
						{
							case 4: // A
								set_bit(result, 0 + d*16);
								break;
							case 5: // Start
								set_bit(result, 6 + d*16);
								break;
							case 8: // Z
								set_bit(result, 5 + d*16);
								break;
							case 9: // Y
								set_bit(result, 4 + d*16);
								break;
							case 10: // X
								set_bit(result, 3 + d*16);
								break;
							case 11: // Mode
								set_bit(result, 7 + d*16);
								break;
						}
					}
				}
			}
			smd_gamepad_data >>= 16;
		}
	}
	if (!smd_detected[0] || !smd_detected[1]) // SMD gamepad is not connected?
	{
#ifdef SMD_USE_DENDY9_PIN
		// so maybe it's 9-pin dendy gamepad?
		for (d = 0; d < 2; d++)
		{
			if (!smd_detected[d])
			{
				init_dendy_9pin(d);
				_delay_us(50);
				uint32_t dendy_data = ~get_dendy_9pin(d);
				result &= ~(0xFFFFUL << (d*16));
				result |= ((dendy_data & 0x0F) | ((dendy_data & 0xF0) << 4)) << (16*d);
			}
		}
		init_smd_gamepad(); // back to SMD mode
#endif
	}

	return result;
}

#ifdef DUALSHOCK_ENABLED
void init_dualshock_gamepad()
{
	DUALSHOCK_PORT_DDR |= (1<<DUALSHOCK_COMMAND_PIN); // Command pin - output
	DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_COMMAND_PIN); // Command pin - login high
	DUALSHOCK_PORT_DDR &= ~(1<<DUALSHOCK_DATA_PIN);   // Data pin - input
	DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_DATA_PIN);   // Data pin - pull-up
	DUALSHOCK_PORT_DDR |= (1<<DUALSHOCK_CLOCK_PIN);   // Clock - output
	DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_CLOCK_PIN);  // Clock - logic high	
	DUALSHOCK_ATTENTION_DDR |= (1<<DUALSHOCK_ATTENTION_PIN); // Attention - output
	DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN); // Attention - logic high	
#ifdef DUALSHOCK_SECOND_ENABLED
	DUALSHOCK_ATTENTION_DDR |= (1<<DUALSHOCK_ATTENTION_PIN2); // Attention 2 - output
	DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN2); // Attention 2 - logic high	
#endif
	/*
	DUALSHOCK_PORT_DDR &= ~(1<<DUALSHOCK_ACK_PIN);   // Ack pin - input
	DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_ACK_PIN);   // Ack pin - pull-up
	*/
}

int dualshock_command(uint8_t* command, uint8_t* data, int length, uint8_t controller_number)
{
	if (!controller_number)
		DUALSHOCK_ATTENTION_PORT &= ~(1<<DUALSHOCK_ATTENTION_PIN); // Attention!
	else
		DUALSHOCK_ATTENTION_PORT &= ~(1<<DUALSHOCK_ATTENTION_PIN2); // Attention!
	_delay_us(20);
	int b, bit;
	for (b = 0; b < length; b++) // Each byte...
	{
		data[b] = 0;
		for (bit = 0; bit < 8; bit++)
		{
			if ((command[b] >> bit) & 1) // 1?
				DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_COMMAND_PIN); // 1!
			else DUALSHOCK_PORT_PORT &= ~(1<<DUALSHOCK_COMMAND_PIN); // 0!			
			DUALSHOCK_PORT_PORT &= ~(1<<DUALSHOCK_CLOCK_PIN);  // Clock - logic low
			_delay_us(20);
			if ((DUALSHOCK_PORT_PIN >> DUALSHOCK_DATA_PIN) & 1) // Reading data... 1?
				data[b] |= (1<<bit); // 1!
			DUALSHOCK_PORT_PORT |= (1<<DUALSHOCK_CLOCK_PIN);  // Clock - logic high			
			_delay_us(20);
		}
		if (b == 1 && data[1] == 0xFF) // Alternative device detection
		{
			if (!controller_number)	
				DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN);  // No attention...
			else
				DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN2);  // No attention...
			return 0;
		}
		/*
		if (b<length-1) // Waiting for ACK
		{
			int t;
			for (t = 0; t < 50; t++)
			{			
				if (!((DUALSHOCK_PORT_PIN >> DUALSHOCK_ACK_PIN)&1)) // ACK reveived
				{
					ok = 1;
					break;
				}
				_delay_us(1);
			}
			if ((b < 2) && !ok) return 0; // No ACK in first two bytes? Aboooort! Saving time
		}
		*/
	}
	if (!controller_number)	
		DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN);  // No attention...
	else
		DUALSHOCK_ATTENTION_PORT |= (1<<DUALSHOCK_ATTENTION_PIN2);  // No attention...
	_delay_us(20);
	return 1;
}

int get_dualshock_gamepad(uint8_t* data, int size, uint8_t motor_small, uint8_t motor_large, uint8_t controller_number) // pointer to uint8_t[21], number of bytes to request, vibration...
{
	static uint8_t dualshock_configered[2] = {0, 0};

	uint8_t command_query[21] = {0x01, 0x42, 0, motor_small, motor_large, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	if (!dualshock_command(command_query, data, size, controller_number))
	{
		dualshock_configered[controller_number] = 0;
		return 0;
	}
	if (!dualshock_configered[controller_number]) // Need to reconfigure dualshock
	{
		uint8_t command_config_mode[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
		if (!dualshock_command(command_config_mode, data, sizeof(command_config_mode), controller_number)) return 0;
		uint8_t command_analog_mode[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
		if (!dualshock_command(command_analog_mode, data, sizeof(command_analog_mode), controller_number)) return 0;
		uint8_t command_config_motors[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
		if (!dualshock_command(command_config_motors, data, sizeof(command_config_motors), controller_number)) return 0;
		uint8_t command_config_pressure[9] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
		if (!dualshock_command(command_config_pressure, data, sizeof(command_config_pressure), controller_number)) return 0;
		uint8_t command_config_mode_exit[8] = {0x01, 0x43, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
		if (!dualshock_command(command_config_mode_exit, data, sizeof(command_config_mode_exit), controller_number)) return 0;
		dualshock_configered[controller_number] = 1;
		if (!dualshock_command(command_query, data, size, controller_number)) return 0;
	}
	return 1;
}
#endif
