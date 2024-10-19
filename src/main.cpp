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
*                                                                     *
***********************************************************************/

/*

AVR Memory Usage
----------------
Device: attiny85

Program:     950 bytes (11.6% Full)
(.text + .data + .bootloader)

Data:        134 bytes (26.2% Full)
(.data + .bss + .noinit)

*/

#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define CMD_STATUS      0xD7  // Chip status command
#define RESPONSE_STATUS 0x8C  // Response to CMD_STATUS
#define CMD_SECURITY    0x77  // Dump security register command
#define CMD_PLL         0xD2  // PLL loop, 12 byte packets
#define CMD_DEVICEID    0x9F  // Manufacturer & Device ID command

#define SS   _BV(PB4)         // create byte representation of PORTB bit 4
#define POCI _BV(PB1)
#define OVFL _BV(USIOIF)      // create byte representation os USISR bit 6

//bool logged_on = false;
volatile bool byte_ready = false;   // signal from Interrupt handler that a SPI byte is ready
volatile uint8_t cmd = 0x0;         // byte returned from SPI interrupt handler

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

/*
* This code will use a fixed security response instead of a randoom response
* Both methods appear to work but the randon response is "cooler" :-)
*/
/*
const uint8_t SECURITY_REGISTER_DATA[128] = {
0x80, 0x2D, 0x22, 0x6F, 0x52, 0x6F, 0x98, 0xA9, 0x21, 0x25, 0x5E, 0x2D, 0x2D, 0x31, 0xD2, 0x39,
0x18, 0x1C, 0x63, 0x0C, 0x31, 0x21, 0x2D, 0x39, 0x90, 0xDE, 0x94, 0x6F, 0x6B, 0x77, 0x73, 0x7F,
0x63, 0x8C, 0x88, 0x84, 0x80, 0x9C, 0x98, 0x94, 0x90, 0xAD, 0xA9, 0xA5, 0xA1, 0xBD, 0xB9, 0xB5,
0xB1, 0xCA, 0xCE, 0xC2, 0xC6, 0xDA, 0xDE, 0xD2, 0xD6, 0xEB, 0xEF, 0xE3, 0xE7, 0xFB, 0xFF, 0xF3,
0x0B, 0x02, 0x16, 0x17, 0x11, 0x15, 0x1F, 0x22, 0x00, 0x00, 0x46, 0x00, 0xFF, 0xFF, 0xAC, 0xFF,
0x30, 0x30, 0x4D, 0x32, 0x36, 0x39, 0x37, 0x31, 0x0F, 0x1C, 0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

void init_fixed_response(void) {
  
  for (int i=0;i<128;i++){
    response[i] = SECURITY_REGISTER_DATA[i];
  }
}
*/

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

  DDRB  = (1<<PB3);                                   // PB3 is debug pin
  PORTB = (1<<PB1) | (1<<PB2) | (1<<PB4);            // Pullups

  USICR = (1<<USIWM0) | (1<<USICS1) | (1<< USIOIE);  // Configure USI to 3 wire SPI Mode 0
  USIDR = 0;
}

void wait_for_ss(void) {
  while (PINB & SS) { /* wait for chip select to go low */ }
}

bool chip_select(void) {
  return(!(PINB & SS));  /* wait for chip select to go low */
}

void reset_ovfl(void) {
  USISR = OVFL;   // reset USIOIE flag (counter overflow / byte ready), clear clk counter
}

void wait_for_byte(void) {
  while( !byte_ready) {/* wait for SPI byte ready*/}
}

ISR(USI_OVF_vect) {
  // Interrupt service routine for SPI clock overflow / a byte is ready
  //   save the result in cmd
  //   set the byte ready flag for the main function
  //   clear the interrupt condition
  //PORTB |= (1 << PB3);  // debug
  cmd = USIDR;
  byte_ready = true;
  USISR = 0b01000000;
  //PORTB &= ~(1 << PB3); // debug
}

int main() {

  cli();                      // disable interrupts during setup
  // init_fixed_response();   // initialiaze the fixed response buffer
  init_response();            // initialize the random security response buffer
  init_registers();           // initialize SPI mode and interrupts
  sei();                      // enable SPI OVF interrupts

  while(true) {               // loop forever
    wait_for_ss();            // Wait for Chip Select to go low / selected
                              //  ......... waiting ......... then
    reset_ovfl();             //  reset ovfl interrupt and initialize clk counter
    DDRB |= (1 << PB1);       //  set miso pin to output while SS is low
    byte_ready = false;       //  clear byte ready flag, prepare for next SPI byte

    while (chip_select()) {   // while SS is low / selected
      //PORTB |= (1 << PB3);  // debug
      wait_for_byte();        // wait for next SPI byte

        /************************************************************* 
        *  This code processes PLL data but is not necessary as the *
        *  default else block below essentially does the same thing *
        * ***********************************************************/
        /*
        if (cmd == CMD_PLL) {         // 0xD2 signifies a 12 byte PLL packet
          // echo back 12 bytes
          //PORTB |= (1<<PB3);        // debug
          for (int i = 0;i<12;i++){
            wait_for_byte();
            USIDR=cmd;
            byte_ready = false;
          }
          //PORTB &= ~(1 << PB3);     // debug
        }
        */

      if (cmd == CMD_STATUS) {        // 0xDC
        // PORTB |= (1<<PB3);         // debug: set pin PB3
        USIDR = RESPONSE_STATUS;      // return 0x8C which is presumably "all good"
        byte_ready = false;           // get ready for next byte interrupt
        // PORTB &= ~(1 << PB3);      // debug: reset ping PB3
      } 
    
      else if (cmd == CMD_SECURITY){  // 0x77
        // first receive / send three 0x00 bytes to emulate
        // the "expected" flash memory chip
        // PORTB |= (1<<PB3);         // debug:
        for (int i=0;i<3;i++) {
          wait_for_byte();
          USIDR = 0x00;
          byte_ready = false;
        }
        // next send the 128 byte security challenge response
        // created earlier
        for (int i=0;i<128;i++) {
          wait_for_byte();
          USIDR = response[i];
          byte_ready = false;;
        }
        // PORTB &= ~(1 << PB3);      // debug:
      } // end of cmd == CMD_SECURITY

      else{                           // just echo back for all other bytes 
        USIDR = cmd;
        byte_ready = false;
      }    

    } /****** end of while chip_select *****/
    
    // PORTB &= ~(1 << PB3); // debug
    // This is absolutely required othewrwise the console generates a PLL not locked error
    DDRB &= ~(1<<PB1);  // Set MISO to input / Hi-Z
    PORTB |= (1<<PB1);  // with pullup

  } /**** end of while true / outer loop *****/
} /**** end of main *****/