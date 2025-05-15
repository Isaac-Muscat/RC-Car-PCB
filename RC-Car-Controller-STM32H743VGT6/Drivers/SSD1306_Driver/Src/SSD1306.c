/*
 * SSD1306.c
 *
 *  Created on: May 15, 2025
 *      Author: Loochis
 */

#include "SSD1306.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------
const uint8_t SSD1306_INITCMDS[] = {
	18,                                                       // number of initializers
	0, DISPLAY_OFF,                                           // 0xAE = Set Display OFF
	1, SET_MUX_RATIO, 63,                                     // 0xA8 - 64MUX for 128 x 64 version
	                                                          	  //      - 32MUX for 128 x 32 version
	1, MEMORY_ADDR_MODE, 0x00,                                // 0x20 = Set Memory Addressing Mode
	                                                          	  // 0x00 - Horizontal Addressing Mode
	                                                          	  // 0x01 - Vertical Addressing Mode
	                                                          	  // 0x02 - Page Addressing Mode (RESET)
	2, SET_COLUMN_ADDR, START_COLUMN_ADDR, END_COLUMN_ADDR,   // 0x21 = Set Column Address, 0 - 127
	2, SET_PAGE_ADDR, START_PAGE_ADDR, END_PAGE_ADDR,         // 0x22 = Set Page Address, 0 - 7
	0, SET_START_LINE,                                        // 0x40
	1, DISPLAY_OFFSET, 0x80,                                  // 0xD3
	0, SEG_REMAP_OP,                                          // 0xA0 / remap 0xA1
	0, COM_SCAN_DIR_OP,                                       // 0xC0 / remap 0xC8
	1, COM_PIN_CONF, 0x12,                                    // 0xDA, 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
	                                                          	  //       0x12 - for 128 x 64 version
	                                                          	  //       0x02 - for 128 x 32 version
	1, SET_CONTRAST, 0x7F,                                    // 0x81, 0x7F - reset value (max 0xFF)
	0, DIS_ENT_DISP_ON,                                       // 0xA4
	0, DIS_NORMAL,                                            // 0xA6
	1, SET_OSC_FREQ, 0x80,                                    // 0xD5, 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
	1, SET_PRECHARGE, 0xc2,                                   // 0xD9, higher value less blinking
	                                                          	  // 0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
	1, VCOM_DESELECT, 0x20,                                   // Set V COMH Deselect, reset value 0x22 = 0,77xUcc
	1, SET_CHAR_REG, 0x14,                                    // 0x8D, Enable charge pump during display on
	0, DISPLAY_ON                                             // 0xAF = Set Display ON
};

uint8_t SSD1306_SendCommand(SSD1306_HandleTypeDef *hssd, uint8_t command) {
	uint8_t composite[2] = {COMMAND, command};
	if (HAL_I2C_Master_Transmit(hssd->i2c_handle, (hssd->address) << 1, composite, 2, 100))
		return ERROR;
	return SUCCESS;
}


uint8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd) {
	// Some control variables
	uint16_t n_commands = SSD1306_INITCMDS[0];
	uint16_t n_arguments;
	uint16_t cmd_idx = 1;

	// Read the init sequence
	while (n_commands--) {
	        // Get no. of Args
	        n_arguments = SSD1306_INITCMDS[cmd_idx];
	        cmd_idx++;

	        // Send initial command
	        if (SSD1306_SendCommand(hssd, SSD1306_INITCMDS[cmd_idx])) return cmd_idx;
	        cmd_idx++;

	        // Send argumemts
	        while (n_arguments--) {
	            if (SSD1306_SendCommand(hssd, SSD1306_INITCMDS[cmd_idx])) return cmd_idx;
	            cmd_idx++;
	        }
	    }
	    return 0;
}

uint8_t SSD1306_Clear(SSD1306_HandleTypeDef *hssd) {
	hssd->str_cursor = 1;							// Reset the cursor to top-left
	memset(hssd->vram, 0x00, CACHE_SIZE_MEM + 1);	// set all bytes to 0
	return 0;
}


uint8_t SSD1306_Update(SSD1306_HandleTypeDef *hssd) {
	hssd->vram[0] = DATA_STREAM; // Identify the outgoing data as a stream
	return HAL_I2C_Master_Transmit_DMA(hssd->i2c_handle, (hssd->address) << 1, hssd->vram, CACHE_SIZE_MEM + 1);
}
