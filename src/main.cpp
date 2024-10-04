/**********************************************************************
*                                                                     *
* AVR program to respond to the Davis Vantage green dot Console data  *
* logger challenge.  The program size is such that it should work     *
* with ATTINY85 and ATTINY45 (not tested) devices.                    *
*                                                                     *
* The code is based on work and code done by Torkel M. Jodalen and    *
* others which is documkented here:                                   *
*       https://www.annoyingdesigns.com/meteo/DavisSPI.pdf            *
*                                                                     *
* Additional information and the original ATTINY85 hex file can be    *
* found here.  USe the .txt file as the .hex file appears empty.      *
* https://github.com/SpiceWire/davis-vantage-pro2-datalogger-emulator *
*                                                                     *
***********************************************************************/

/*
AVR Memory Usage
----------------
Device: ATTINY85

Program:     870 bytes
(.text + .data + .bootloader)

Data:        133 bytes
(.data + .bss + .noinit)
*/


#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
//#include <avr/interrupt.h>

#define CMD_STATUS      0xD7  // Chip status command
#define CMD_STATUS_REV  0xEB  // Chip status command
#define CMD_SECURITY    0x77  // Dump security register command
#define CMD_DEVICEID    0x9F  // Manufacturer & Device ID command
#define RESPONSE_STATUS 0x8C  // Response to CMD_STATUS

#define SS   _BV(PB4)         // create byte representation of PORTB bit 4
#define POCI _BV(PB1)
#define OVFL _BV(USIOIF)      // create byte representation os USISR bit 6

uint8_t cmd = 0x0;

// Static look-up table for security register
// Stored in program flash to reduce SRAM usage
const uint8_t lookup[256] PROGMEM = {
  0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x21, 0x25, 0x29, 0x2D, 0x31, 0x35, 0x39, 0x3D,
  0x46, 0x42, 0x4E, 0x4A, 0x56, 0x52, 0x5E, 0x5A, 0x67, 0x63, 0x6F, 0x6B, 0x77, 0x73, 0x7F, 0x7B,
  0x8C, 0x88, 0x84, 0x80, 0x9C, 0x98, 0x94, 0x90, 0xAD, 0xA9, 0xA5, 0xA1, 0xBD, 0xB9, 0xB5, 0xB1,
  0xCA, 0xCE, 0xC2, 0xC6, 0xDA, 0xDE, 0xD2, 0xD6, 0xEB, 0xEF, 0xE3, 0xE7, 0xFB, 0xFF, 0xF3, 0xF7,
  0x18, 0x1C, 0x10, 0x14, 0x08, 0x0C, 0x00, 0x04, 0x39, 0x3D, 0x31, 0x35, 0x29, 0x2D, 0x21, 0x25,
  0x5E, 0x5A, 0x56, 0x52, 0x4E, 0x4A, 0x46, 0x42, 0x7F, 0x7B, 0x77, 0x73, 0x6F, 0x6B, 0x67, 0x63,
  0x94, 0x90, 0x9C, 0x98, 0x84, 0x80, 0x8C, 0x88, 0xB5, 0xB1, 0xBD, 0xB9, 0xA5, 0xA1, 0xAD, 0xA9,
  0xD2, 0xD6, 0xDA, 0xDE, 0xC2, 0xC6, 0xCA, 0xCE, 0xF3, 0xF7, 0xFB, 0xFF, 0xE3, 0xE7, 0xEB, 0xEF,
  0x31, 0x35, 0x39, 0x3D, 0x21, 0x25, 0x29, 0x2D, 0x10, 0x14, 0x18, 0x1C, 0x00, 0x04, 0x08, 0x0C,
  0x77, 0x73, 0x7F, 0x7B, 0x67, 0x63, 0x6F, 0x6B, 0x56, 0x52, 0x5E, 0x5A, 0x46, 0x42, 0x4E, 0x4A,
  0xBD, 0xB9, 0xB5, 0xB1, 0xAD, 0xA9, 0xA5, 0xA1, 0x9C, 0x98, 0x94, 0x90, 0x8C, 0x88, 0x84, 0x80,
  0xFB, 0xFF, 0xF3, 0xF7, 0xEB, 0xEF, 0xE3, 0xE7, 0xDA, 0xDE, 0xD2, 0xD6, 0xCA, 0xCE, 0xC2, 0xC6,
  0x29, 0x2D, 0x21, 0x25, 0x39, 0x3D, 0x31, 0x35, 0x08, 0x0C, 0x00, 0x04, 0x18, 0x1C, 0x10, 0x14,
  0x6F, 0x6B, 0x67, 0x63, 0x7F, 0x7B, 0x77, 0x73, 0x4E, 0x4A, 0x46, 0x42, 0x5E, 0x5A, 0x56, 0x52,
  0xA5, 0xA1, 0xAD, 0xA9, 0xB5, 0xB1, 0xBD, 0xB9, 0x84, 0x80, 0x8C, 0x88, 0x94, 0x90, 0x9C, 0x98,
  0xE3, 0xE7, 0xEB, 0xEF, 0xF3, 0xF7, 0xFB, 0xFF, 0xC2, 0xC6, 0xCA, 0xCE, 0xD2, 0xD6, 0xDA, 0xDE
};

uint8_t response[128];                      // pre-built response to validation request

uint8_t calculate(int byteno) {
  //
  // Helper function for init _response
  //
  int index = byteno + 64;                  // 64-127 for device ID
  index = response[index] + byteno;         // Index into the lookup matrix
  index = index & 0xff;                     // Avoid index out of range
  return(pgm_read_byte(&(lookup[index])));  // Return "encoded byte" from lookup table
  }

void init_response(void) {
  //
  // Create a random device ID and generate the security challenge in a memory
  // buffer.  Push the buffer to the console when requested
  //
    for (int i=64;i<128;i++) {
        response[i] = rand() & 0x7f;
      }
  
    for (int i=3;i<64;i++) {
        response[i] = calculate(i);
      }

    // Fill in a serial number(?), bytes 0-2
    response[0] = 0x9a; // 9A 2F 21 are values found in an original
    response[1] = 0x2f; // logger. This may or may not be a device
    response[2] = 0x21; // serial number
}

void init_registers() {
  //Config PortB.0 = Input  PB0 = MOSI / PICO
  //Config PortB.1 = Output PB1 = MISO / POCI
  //Config PortB.2 = Input  PB2 = Clock
  //Config PortB.4 = Input  PB4 = SS

  DDRB = POCI;                      //Set direction for PB1 / POCI
  USICR=(1<<USIWM0) | (1<<USICS1);  // Configure USI to 3 wire SPI Mode 0
}

//void wait_for_ss(void) {
//  while (PORTB & SS) { /* wait for Select to go low */ }/
//}

bool chip_select(void) {
  return(!(PORTB & SS));  /* wait for Select to go low */
}

void wait_for_spi(void) {
  while( !(USISR & OVFL)) {/* wait for SPI byte ready*/}
  cmd = USIDR;
}

void reset_ovfl(void) {
  USISR = OVFL;   // reset USIOIE flag (counter overflow / byte ready)
}

int main() {

  init_response();
  init_registers();
 
  while(true) {
    
    //wait_for_ss();
    while (chip_select()) {
    
      wait_for_spi();                 // wait for a SPI byte from console
                                      // byte returned in cmd

      if (cmd == CMD_STATUS) {        // 0xD7
        USIDR = RESPONSE_STATUS;      // 0x8C
        reset_ovfl();
      } 
    
      else if (cmd == CMD_SECURITY){  // 0x77
        // first receive / send three 0x00 bytes to emulate
        // the "expected" flash memory chip
        for (int i=0;i<3;i++) {
          USIDR = 0x00;
          reset_ovfl();
          wait_for_spi();
        }

        // next send the 128 byte security challenge response
        // created earlier
        for (int i=0;i<128;i++) {
          USIDR = response[i];
          reset_ovfl();
          wait_for_spi();
        }      
      } // end of cmd == CMD_SECURITY
    } // end of while chip_select
  } // end of while true / outer loop
}