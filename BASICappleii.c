/*
Copyright (c) 2015, Damian Peckett <damian@pecke.tt>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

All firmware is property of Apple Computer and is recreated here for 
historical purposes only.
*/

#include <avr/pgmspace.h>

#include <ctype.h>
//#include <stdbool.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <uzebox.h>


#include "data/hack-font.inc"

#include "data/scancodes.inc"
#include "data/roms.inc"
#include "screen.h"


//HACKITY HACK HACK...



#ifndef NO_SPI_RAM
	#include <spiram.h>

	uint16_t spiram_cursor = 0;
	uint16_t spiram_state = 0;

	uint8_t SpiRamCursorInit(){
		spiram_cursor = 0xFFFC;//reset vector
		spiram_state = 0;
		if(!SpiRamInit())
			return 0;
		SpiRamSeqReadStart(0, (uint16_t)0xFFFC);
		return 1;
	}

	uint8_t SpiRamCursorRead(uint16_t addr){
		if(spiram_state){//in a sequential write?
			SpiRamSeqWriteEnd();
			asm("nop");asm("nop");
			SpiRamSeqReadStart(0, addr);
			asm("nop");asm("nop");
			spiram_state = 0;
			spiram_cursor = addr+1;
			return SpiRamSeqReadU8();
		}
		if(spiram_cursor != addr){//current sequential read position doesn't match?
			SpiRamSeqReadEnd();
			asm("nop");asm("nop");
			SpiRamSeqReadStart(0, addr);
			asm("nop");asm("nop");
			spiram_cursor = addr+1;
			return SpiRamSeqReadU8();
		}
		spiram_cursor++;
		return SpiRamSeqReadU8();
	}

	void SpiRamCursorWrite(uint16_t addr, uint8_t val){
		if(!spiram_state){//in a sequential read?
			SpiRamSeqReadEnd();
			asm("nop");asm("nop");
			SpiRamSeqWriteStart(0, addr);
			spiram_state = 1;
			spiram_cursor = addr+1;
			asm("nop");asm("nop");
			SpiRamSeqWriteU8(val);
			return;
		}
		if(spiram_cursor != addr){//current sequential write position doesn't match?
			SpiRamSeqWriteEnd();
			asm("nop");asm("nop");
			SpiRamSeqWriteStart(0, addr);
			spiram_cursor = addr+1;
			asm("nop");asm("nop");
			SpiRamSeqWriteU8(val);
			return;
		}
		spiram_cursor++;
		SpiRamSeqWriteU8(val);
	}
#endif



extern volatile unsigned int joypad1_status_lo,joypad2_status_lo;
extern volatile unsigned int joypad1_status_hi,joypad2_status_hi;
#define Wait200ns() asm volatile("lpm\n\tlpm\n\t");
#define Wait100ns() asm volatile("lpm\n\t");


// Address Modes
#define AD_IMP	0x01
#define AD_A 	0x02
#define AD_ABS 	0x03
#define AD_ABSX	0x04
#define AD_ABSY	0x05
#define AD_IMM	0x06
#define AD_IND	0x07
#define AD_INDX	0x08
#define AD_INDY	0x09
#define AD_REL	0x0A
#define AD_ZPG	0x0B
#define AD_ZPGX	0x0C
#define AD_ZPGY	0x0D

// SR Flag Modes
#define FL_NONE 0x00
#define FL_Z 	0x20
#define FL_ZN 	0xA0
#define FL_ZNC	0xB0
#define FL_ZC 	0x30
#define FL_ALL	0xF0

//Unimplemented ops
#define UNDF	0x00

//Other constants
#define SR_FIXED_BITS 0x20
#define SR_CARRY      0x01
#define SR_ZERO       0x02
#define SR_INT        0x04
#define SR_DEC        0x08
#define SR_BRK        0x10
#define SR_OVER       0x40
#define SR_NEG        0x80

//Stack pointer base address
#define STP_BASE       0x100

//high nibble SR flags, low nibble address mode
const unsigned char flags[] PROGMEM = {
	AD_IMP, AD_INDX, UNDF, UNDF, UNDF, FL_ZN|AD_ZPG, FL_ZNC|AD_ZPG, UNDF, AD_IMP, FL_ZN|AD_IMM, FL_ZNC|AD_A, UNDF, UNDF, FL_ZN|AD_ABS, FL_ZNC|AD_ABS, UNDF,
	AD_REL, FL_ZN|AD_INDY, UNDF, UNDF, UNDF, FL_ZN|AD_ZPGX, FL_ZNC|AD_ZPGX, UNDF, AD_IMP, FL_ZN|AD_ABSY, UNDF, UNDF, UNDF, FL_ZN|AD_ABSX, FL_ZNC|AD_ABSX, UNDF,
	AD_ABS, FL_ZN|AD_INDX, UNDF, UNDF, FL_Z|AD_ZPG, FL_ZN|AD_ZPG, FL_ZNC|AD_ZPG, UNDF, AD_IMP, FL_ZN|AD_IMM, FL_ZNC|AD_A, UNDF, FL_Z|AD_ABS, FL_ZN|AD_ABS, FL_ZNC|AD_ABS, UNDF,
	AD_REL, FL_ZN|AD_INDY, UNDF, UNDF, UNDF, FL_ZN|AD_ZPGX, FL_ZNC|AD_ZPGX, UNDF, AD_IMP, FL_ZN|AD_ABSY, UNDF, UNDF, UNDF, FL_ZN|AD_ABSX, FL_ZNC|AD_ABSX, UNDF,
	AD_IMP, FL_ZN|AD_INDX, UNDF, UNDF, UNDF, FL_ZN|AD_ZPG, FL_ZNC|AD_ZPG, UNDF, AD_IMP, FL_ZN|AD_IMM, FL_ZNC|AD_A, UNDF, AD_ABS, FL_ZN|AD_ABS, FL_ZNC|AD_ABS, UNDF,
	AD_REL, FL_ZN|AD_INDY, UNDF, UNDF, UNDF, FL_ZN|AD_ZPGX, FL_ZNC|AD_ZPGX, UNDF, AD_IMP, FL_ZN|AD_ABSY, UNDF, UNDF, UNDF, FL_ZN|AD_ABSX, FL_ZNC|AD_ABSX, UNDF,
	AD_IMP, FL_ALL|AD_INDX, UNDF, UNDF, UNDF, FL_ALL|AD_ZPG, FL_ZNC|AD_ZPG, UNDF, FL_ZN|AD_IMP, FL_ALL|AD_IMM, FL_ZNC|AD_A,UNDF, AD_IND, FL_ALL|AD_ABS, FL_ZNC|AD_ABS, UNDF,
	AD_REL, FL_ALL|AD_INDY, UNDF, UNDF, UNDF, FL_ALL|AD_ZPGX, FL_ZNC|AD_ZPGX, UNDF, AD_IMP, FL_ALL|AD_ABSY, UNDF, UNDF, UNDF, FL_ALL|AD_ABSX, FL_ZNC|AD_ABSX, UNDF,
	UNDF, AD_INDX, UNDF, UNDF, AD_ZPG, AD_ZPG, AD_ZPG, UNDF, FL_ZN|AD_IMP, UNDF, FL_ZN|AD_IMP, UNDF, AD_ABS, AD_ABS, AD_ABS, UNDF,
	AD_REL, AD_INDY, UNDF, UNDF, AD_ZPGX, AD_ZPGX, AD_ZPGY, UNDF, FL_ZN|AD_IMP, AD_ABSY, AD_IMP, UNDF, UNDF, AD_ABSX, UNDF, UNDF,
	FL_ZN|AD_IMM, FL_ZN|AD_INDX, FL_ZN|AD_IMM, UNDF, FL_ZN|AD_ZPG, FL_ZN|AD_ZPG, FL_ZN|AD_ZPG, UNDF, FL_ZN|AD_IMP, FL_ZN|AD_IMM, FL_ZN|AD_IMP, UNDF, FL_ZN|AD_ABS, FL_ZN|AD_ABS, FL_ZN|AD_ABS, UNDF,
	AD_REL, FL_ZN|AD_INDY, UNDF, UNDF, FL_ZN|AD_ZPGX, FL_ZN|AD_ZPGX, FL_ZN|AD_ZPGY, UNDF, AD_IMP, FL_ZN|AD_ABSY, FL_ZN|AD_IMP, UNDF, FL_ZN|AD_ABSX, FL_ZN|AD_ABSX, FL_ZN|AD_ABSY, UNDF,
	FL_ZNC|AD_IMM, FL_ZNC|AD_INDX, UNDF, UNDF, FL_ZNC|AD_ZPG, FL_ZNC|AD_ZPG, FL_ZN|AD_ZPG, UNDF, FL_ZN|AD_IMP, FL_ZNC|AD_IMM, FL_ZN|AD_IMP, UNDF, FL_ZNC|AD_ABS, FL_ZNC|AD_ABS,	FL_ZN|AD_ABS, UNDF,
	AD_REL, FL_ZNC|AD_INDY, UNDF, UNDF, UNDF, FL_ZNC|AD_ZPGX, FL_ZN|AD_ZPGX, UNDF, AD_IMP, FL_ZNC|AD_ABSY, UNDF, UNDF, UNDF, FL_ZNC|AD_ABSX, FL_ZN|AD_ABSX, UNDF,
	FL_ZNC|AD_IMM, FL_ALL|AD_INDX, UNDF, UNDF, FL_ZNC|AD_ZPG, FL_ALL|AD_ZPG, FL_ZN|AD_ZPG, UNDF, FL_ZN|AD_IMP, FL_ALL|AD_IMM, AD_IMP, UNDF, FL_ZNC|AD_ABS, FL_ALL|AD_ABS,	FL_ZN|AD_ABS, UNDF,
	AD_REL, FL_ALL|AD_INDY, UNDF, UNDF, UNDF, FL_ALL|AD_ZPGX, FL_ZN|AD_ZPGX, UNDF, AD_IMP, FL_ALL|AD_ABSY, UNDF, UNDF, UNDF, FL_ALL|AD_ABSX, FL_ZN|AD_ABSX, UNDF
};




uint8_t cassette_read_state();
short cassette_read_transition();
uint8_t cassette_read_block(unsigned short A1, unsigned short A2);
void cassette_begin();





// CPU registers
unsigned short PC;
unsigned char STP = 0xFD, A = 0x00, X = 0x00, Y = 0x00, SR = SR_FIXED_BITS;

//Execution variables
unsigned char opcode, opflags;
unsigned short argument_addr;

//Temporary variables for flag generation
unsigned char value8;
unsigned short value16, value16_2, result;






// keyboard scan buffer
unsigned short keyboard_data[3] = {0, 0, 0};
unsigned char keyboard_buf_indx = 0, keyboard_mbyte = 0;
uint8_t shift_enabled = 0;

// In apple II scancode format
volatile unsigned char keymem = 0;

unsigned char keyboard_read() {
  return keymem;
}

void keyboard_strobe() {
  keymem&=0x7F;
}

// clock must be on digital 3
void keyboard_begin() {
/*
  pinMode(3, INPUT_PULLUP);
  pinMode(KEYBD_DATA_PIN, INPUT_PULLUP);
  attachInterrupt(1, keyboard_bit, FALLING);
*/
}

void keyboard_bit() {
/*
  if(digitalRead(KEYBD_DATA_PIN))keyboard_data[2] |= _BV(keyboard_buf_indx);
  else keyboard_data[2] &= ~(_BV(keyboard_buf_indx));

  if(++keyboard_buf_indx == 11) {
    // Ignore parity checks for now
    keyboard_data[2] = (keyboard_data[2]>>1)&0xFF;
    
    // extended keys
    if(keyboard_data[2] == 0xF0 || keyboard_data[2] == 0xE0) keyboard_mbyte = 1;
    else {
      //decrement counter for multibyte commands
      if(keyboard_mbyte) keyboard_mbyte--;
      // multibyte command is finished / normal command, process it
      if(!keyboard_mbyte) {
        if(keyboard_data[1] != 0xF0 && keyboard_data[1] != 0xE0) {
          //Standard keys
          if(keyboard_data[2] == 0x12 || keyboard_data[2] == 0x59) shift_enabled = true; //shift modifiers
          else keymem = pgm_read_byte_near(scancode_to_apple+keyboard_data[2]+((shift_enabled)?0x80:0x00));
        } else if(keyboard_data[0] != 0xF0 && keyboard_data[1] == 0xE0) {
          //Extended keys
          if(keyboard_data[2] == 0x6B) keymem = 0x95; //back key
          if(keyboard_data[2] == 0x74) keymem = 0x88; //forward key
          // Power management keys, hardware reset
          if(keyboard_data[2] == 0x37) {
            // enable watchdog with min timeout
            // wait until reset
            wdt_enable(WDTO_15MS);
            for(;;);            
          }
        } else if(keyboard_data[1] == 0xF0 && (keyboard_data[2] == 0x12 || keyboard_data[2] == 0x59)) shift_enabled = 0;  
      }      
    }

    //shuffle buffer
    keyboard_data[0] = keyboard_data[1];
    keyboard_data[1] = keyboard_data[2];
    keyboard_buf_indx = 0;
  }
*/
}























unsigned char ram[1024];
// Free memory for storing BASIC programs
unsigned char basic[768];
unsigned char basic_global[256];

unsigned char read8(unsigned short address) {
  unsigned char page = address>>8;
  if(page < 0x04) {
    return ram[address];
  } else if(page < 0x08) {
    return screenRead(address);
  } else if (page < 0x10) {
    return basic[address-0x800];
  } else if(page == 0xBE){
    return basic_global[address-0xBE00];
  }else if(page >= 0xD0 && page <= 0xD7){
   return pgm_read_byte_near(D0ROM+address-0xD000);
  }else if (page >= 0xE0) {//Integer BASIC+Sweet16
    return pgm_read_byte_near(rom+address-0xE000);
  } else {
    // Keyboard Data
    if(address == 0xC000) return keyboard_read();
    // Keyboard Strobe
    if(address == 0xC010) keyboard_strobe();
    //if(address == 0xC030) speaker_toggle();// Speaker toggle
    return SpiRamCursorRead(address);//return 0;
  }
}

unsigned short read16(unsigned short address) {
  return (unsigned short)read8(address) | (((unsigned short)read8(address+1))<<8);
}

void write8(unsigned short address, unsigned char value) {
  unsigned char page = address>>8;
  if(page < 0x04) {
    ram[address] = value;
  } else if(page >= 0x04 && page < 0x08) {
    screenWrite(address, value);
  } else if (page >= 0x08 && page < 0x10) {
    basic[address-0x800] = value;
  } else {
    // Keyboard Strobe
    if(address == 0xC010) keyboard_strobe();
    //if(address == 0xC030) speaker_toggle();// Speaker toggle
    SpiRamCursorWrite(address, value);
  }
}

void write16(unsigned short address, unsigned short value) {
   write8(address, value&0x00FF);
   write8(address+1, (value>>8)&0x00FF);
}




// Hook routines for the apple II monitor program
// Used to trick the apple code into working on this hardware
// Ideally should patch the ROM itself, will do in future.
void program_hooks(unsigned short addr) {
  // hook screen scroll, monitor command
  if(addr == 0xFC70) {
    screenScroll();
    PC = 0xFC95;
  } 
  // hook cassette write commnand
  else if (addr == 0xFECD || addr == 0xFECF) {
    // Header length
    cassette_header((addr==0xFECD)?64:0xA);///??
    // Write Data Block
    cassette_write_block(read16(0x3C), read16(0x3E));
    // Emulate counter behaviour
    write16(0x003C, read16(0x3E));
    PC = 0xFEF5;
  }
  // hook cassette read command
  else if (addr == 0xFEFD) {
      // Read Data Block
      uint8_t success = cassette_read_block(read16(0x3C), read16(0x3E));
      // Emulate counter behaviour
      write16(0x003C, read16(0x3E));
      if(success) PC = 0xFF3A; 
      else PC = 0xFF2D;
  }
}























#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void cassette_header(unsigned short periods) {
/*
  // Header Tone
  for(int i = 0; i < periods*128; ++i) {
    digitalWrite(SPEAKER_PIN, HIGH);
    delayMicroseconds(650);
    digitalWrite(SPEAKER_PIN, LOW);
    delayMicroseconds(650);
  }
  // Sync pulse, one half cycle at 2500hz and then 2000hz
  // 2500 hz
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds(200);
  // 2000 hz
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds(250);
*/
}

void cassette_write_byte(unsigned char val) {
/*
  // Shift it out, MSB first
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x80) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x80) ? 500 : 250);
  //bit 6
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x40) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x40) ? 500 : 250);
  //bit 5
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x20) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x20) ? 500 : 250);
  //bit 4
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x10) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x10) ? 500 : 250);
  //bit 3
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x08) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x08) ? 500 : 250);
  //bit 2
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x04) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x04) ? 500 : 250);  
  //bit 1
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x02) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x02) ? 500 : 250);  
  //bit 0
  digitalWrite(SPEAKER_PIN, HIGH);
  delayMicroseconds((val&0x01) ? 500 : 250);  
  digitalWrite(SPEAKER_PIN, LOW);
  delayMicroseconds((val&0x01) ? 500 : 250);
*/
}

void cassette_write_block(unsigned short A1, unsigned short A2) {
/*
  unsigned char checksum = 0xFF, val = 0;
  for(unsigned short addr = A1; addr <= A2; ++addr) {
    val = read8(addr);
    cassette_write_byte(val);
    checksum ^= val;
  }
  cassette_write_byte(checksum);
  // High idle for a little while, make sure all bits cleared
  digitalWrite(SPEAKER_PIN, HIGH);
  delay(10);
  digitalWrite(SPEAKER_PIN, LOW);
*/
}

// Used to track center voltage

////////////////////float cassette_center_voltage = 512;

// implement zero crossing detector
uint8_t cassette_read_state() {
/*
  static uint8_t zerocross_state = 0;
  // get value
  short adc = (analogRead(CASSETTE_READ_PIN) - (short)cassette_center_voltage);
  // bias drift correction
  cassette_center_voltage += adc*0.05f;
  // ~7mv hysteresis
  if(zerocross_state && adc < -7) zerocross_state = 0;
  else if(!zerocross_state && adc > 7) zerocross_state = true;  
  return zerocross_state;
*/
}

// figure out the duration of zero crossing
short cassette_read_transition() {
/*
  unsigned long start_time;
  static uint8_t last = 0;
  uint8_t cur = last;
  // loop until state transition
  for(start_time = micros();cur == last;) cur = cassette_read_state();
  // update transition tracking
  last = cur;
  //return duration of transition us
  return micros() - start_time;
*/
}

// Based loosely on steve wozniaks original algorithm
uint8_t cassette_read_block(unsigned short A1, unsigned short A2) {
/*
  short bitperiod;
  unsigned char val, checksum = 0xFF, datachecksum = 0x00;
  
  // Calibrate the zero crossing detector
  for(short i = 0; i < 10000; ++i) cassette_read_state();
  
  // find tape in edge
  cassette_read_transition();
  cassette_read_transition();
  
  // Small delay to allow things to settle
  delay(500);
  
  //wait for sync bit, short zero
  while(cassette_read_transition() > 300);
  
  // skip second cycle of sync bit
  cassette_read_transition();
  
  // start reading data  
  for(unsigned short addr = A1; addr <= A2; ++addr) {
    // zero our byte of memory
    val = 0;
    for(unsigned char i = 8; i != 0; --i) {
      bitperiod = (cassette_read_transition() + cassette_read_transition()) / 2;
      if(bitperiod > 300) val |= _BV(i-1);
    }
    // write byte to memory
    write8(addr, val);
    // update checksum
    checksum ^= val;
  }
  
  // Read checksum
  for(unsigned char i = 8; i != 0; --i) {
    bitperiod = (cassette_read_transition() + cassette_read_transition()) / 2;
    if(bitperiod > 300) datachecksum |= _BV(i-1);
  }
  
  //return whether the data passes error checking
  return (datachecksum == checksum);
*/
}

void cassette_begin() {
/*
  // ADC prescale, 77khz
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  // internal pullup on analog pin
  digitalWrite(CASSETTE_READ_PIN, HIGH);
  // use 1.1v internal analog reference
  analogReference(INTERNAL);
*/
}












void setflags() {
  // Mask out affected flags
  switch(opflags&0xF0) {
    case 0xA0: SR&=0x7D; break;
    case 0xB0: SR&=0x7C; break;
    case 0x30: SR&=0xFC; break;
    case 0xF0: SR&=0x3C; break;
    case 0x20: SR&=0xFD; break;
  }

  // Set various status flags
  if(opflags&0x80) SR |= (result&0x0080); //negative
  if(opflags&0x20) SR |= (((result&0xFF) == 0)?0x02:0); //zero
  if(opflags&0x10) SR |= ((result&0xFF00)?0x01:0); //carry
  if(opflags&0x40) SR |= ((result^((unsigned short)A))&(result^value16)&0x0080)>>1; 
}

// Stack functions
void push16(unsigned short pushval) {
  write8(STP_BASE + (STP--), (pushval>>8)&0xFF);
  write8(STP_BASE + (STP--), pushval&0xFF);
}

void push8(unsigned char pushval) {
  write8(STP_BASE + (STP--), pushval);
}

unsigned short pull16() {
  value16 = read8(STP_BASE + (++STP)) | ((unsigned short)read8(STP_BASE + (++STP))<< 8);
  return value16;
}

unsigned char pull8() {
  return read8(STP_BASE + (++STP));
}

void run() {

  
////////////////////////////  for(;;) {
    // Routines for hooking apple ][ monitor routines
    program_hooks(PC);
    
    // Get opcode / addressing mode
    opcode = read8(PC++);
    opflags = pgm_read_byte_near(flags+opcode);
  
    // Addressing modes
    switch(opflags&0x0F) {
      case AD_IMP: case AD_A: argument_addr = 0xFFFF; break;
      case AD_ABS:
        argument_addr = read16(PC);
        PC += 2;
        break;
      case AD_ABSX:
        argument_addr = read16(PC) + (unsigned short)X;
        PC += 2;
        break;
      case AD_ABSY:
        argument_addr = read16(PC) + (unsigned short)Y;
        PC += 2;
        break;
      case AD_IMM:
        argument_addr = PC++;
        break;
      case AD_IND:
        argument_addr = read16(PC);
        value16 = (argument_addr&0xFF00) | ((argument_addr+1)&0x00FF); // Page wrap
        argument_addr = (unsigned short)read8(argument_addr) | ((unsigned short)read8(value16) << 8);
        PC+=2;
        break;
      case AD_INDX:
        argument_addr = ((unsigned short)read8(PC++) + (unsigned short)X)&0xFF;
        value16 = (argument_addr&0xFF00) | ((argument_addr+1)&0x00FF); // Page wrap
        argument_addr = (unsigned short)read8(argument_addr) | ((unsigned short)read8(value16) << 8);
        break;
      case AD_INDY:
        argument_addr = (unsigned short)read8(PC++);
        value16 = (argument_addr&0xFF00) | ((argument_addr+1)&0x00FF); // Page wrap
        argument_addr = (unsigned short)read8(argument_addr) | ((unsigned short)read8(value16) << 8);
        argument_addr += Y;
        break;
      case AD_REL:
        argument_addr = (unsigned short)read8(PC++);
        argument_addr |= ((argument_addr&0x80)?0xFF00:0);
        break;
      case AD_ZPG:
        argument_addr = (unsigned short)read8(PC++);
        break;
      case AD_ZPGX:
        argument_addr = ((unsigned short)read8(PC++) + (unsigned short)X)&0xFF;
        break;
      case AD_ZPGY:
        argument_addr = ((unsigned short)read8(PC++) + (unsigned short)Y)&0xFF;
        break;
      }

      //opcodes
      switch(opcode) {
        //ADC
        case 0x69: case 0x65: case 0x75:
        case 0x6D: case 0x7D: case 0x79:
        case 0x61: case 0x71:
          value16 = (unsigned short)read8(argument_addr);
          result = (unsigned short)A + value16 + (unsigned short)(SR&SR_CARRY);
          setflags();
          A = result&0xFF;
          break;
        //AND
        case 0x29: case 0x25: case 0x35:
        case 0x2D: case 0x3D: case 0x39:
        case 0x21: case 0x31:
          result = A&read8(argument_addr);
          A = result&0xFF;
          setflags();
          break;
        //ASL A
        case 0x0A:
          value16 = (unsigned short)A;
          result = value16<<1;
          setflags();
          A = result&0xFF;
          break;
        //ASL
        case 0x06: case 0x16: case 0x0E: 
        case 0x1E:
          value16 = read8(argument_addr);
          result = value16<<1;
          setflags();
          write8(argument_addr, result&0xFF);
          break;
        //BCC
        case 0x90:
          if(!(SR&SR_CARRY)) PC += argument_addr;
          break;
        //BCS
        case 0xB0:
          if((SR&SR_CARRY)) PC += argument_addr;
          break;
        //BEQ
        case 0xF0:
          if((SR&SR_ZERO)) PC += argument_addr;
          break;
        //BNE
        case 0xD0:
          if(!(SR&SR_ZERO)) PC += argument_addr;
          break;
        //BIT
        case 0x24: case 0x2C:
          value8 = read8(argument_addr);
          result = A & value8;
          setflags();
          SR = (SR&0x3F) | (value8&0xC0);
          break;
        //BMI
        case 0x30:
          if((SR&SR_NEG)) PC += argument_addr;
          break;
        //BPL
        case 0x10:
          if(!(SR&SR_NEG)) PC += argument_addr;
          break;
        //BRK
        case 0x00:
          PC++;
          push16(PC);
          push8(SR|SR_BRK);
          SR|=SR_INT;
          PC = read16(0xFFFE);
          break;
        //BVC
        case 0x50:
          if(!(SR&SR_OVER)) PC += argument_addr;
          break;
        //BVS
        case 0x70:
          if(SR&SR_OVER) PC += argument_addr;
          break;
        //CLC
        case 0x18:
          SR&=0xFE;
          break;
        //CLD
        case 0xD8:
          SR&=0xF7;
          break;
        //CLI
        case 0x58:
          SR&=0xFB;
          break;
        //CLV
        case 0xB8:
          SR&=0xBF;
          break;
        //CMP
        case 0xC9: case 0xC5: case 0xD5:
        case 0xCD: case 0xDD: case 0xD9:
        case 0xC1: case 0xD1:
          value16 = ((unsigned short)read8(argument_addr)) ^ 0x00FF;
          result = (unsigned short)A + value16 + (unsigned short)1;
          setflags();
          break;
        //CPX
        case 0xE0: case 0xE4: case 0xEC:
          value16 = ((unsigned short)read8(argument_addr)) ^ 0x00FF;
          result = (unsigned short)X + value16 + (unsigned short)1;
          setflags();
          break;
        //CPY
        case 0xC0: case 0xC4: case 0xCC:
          value16 = ((unsigned short)read8(argument_addr)) ^ 0x00FF;
          result = (unsigned short)Y + value16 + (unsigned short)1;
          setflags();
          break;
        //DEC
        case 0xC6: case 0xD6: case 0xCE: 
        case 0xDE:
          value16 = (unsigned short)read8(argument_addr);
          result = value16 - 1;
          setflags();
          write8(argument_addr, result&0xFF);
          break;
        //DEX
        case 0xCA:
          result = --X;
          setflags();
          break;
        //DEY
        case 0x88:
          result = --Y;
          setflags();
          break;
        //EOR
        case 0x49: case 0x45: case 0x55:
        case 0x4D: case 0x5D: case 0x59:
        case 0x41: case 0x51:
          value8 = read8(argument_addr);
          result = A^value8;
          setflags();
          A = result&0xFF;
          break;
        //INC
        case 0xE6: case 0xF6: case 0xEE:
        case 0xFE:
          value16 = (unsigned short)read8(argument_addr);
          result = value16 + 1;
          setflags();
          write8(argument_addr, result&0xFF);	
          break;
        //INX
        case 0xE8:
          result = ++X;
          setflags();
          break;
        //INY
        case 0xC8:
          result = ++Y;
          setflags();	
          break;
        //JMP
        case 0x4C: case 0x6C:
          PC = argument_addr;
          break;
        //JSR
        case 0x20:
          push16(PC-1);
          PC = argument_addr;
          break;
        //LDA
        case 0xA9: case 0xA5: case 0xB5:
        case 0xAD: case 0xBD: case 0xB9:
        case 0xA1: case 0xB1:
          A = read8(argument_addr);
          result = A;
          setflags();
          break;
        //LDX
        case 0xA2: case 0xA6: case 0xB6:
        case 0xAE: case 0xBE:
          X = read8(argument_addr);
          result = X;
          setflags();
          break;
        //LDY
        case 0xA0: case 0xA4: case 0xB4:
        case 0xAC: case 0xBC:
          Y = read8(argument_addr);
          result = Y;
          setflags();
          break;
        //LSR A
        case 0x4A:
          value8 = A;
          result = value8 >> 1;
          result |= (value8&0x1)?0x8000:0;
          setflags();
          A = result&0xFF;
          break;
        //LSR
        case 0x46: case 0x56: case 0x4E:
        case 0x5E:
          value8 = read8(argument_addr);
          result = value8 >> 1;
          result |= (value8&0x1)?0x8000:0;
          setflags();
          write8(argument_addr, result&0xFF);
          break;
        //NOP
        case 0xEA:
          break;
        //ORA
        case 0x09: case 0x05: case 0x15:
        case 0x0D: case 0x1D: case 0x19:
        case 0x01: case 0x11:
          value8 = read8(argument_addr);
          result = A | value8;
          setflags();
          A = result&0xFF;
          break;
        //PHA
        case 0x48:
          push8(A);
          break;
        //PHP
        case 0x08:
          push8(SR|SR_BRK);
          break;
        //PLA
        case 0x68:
          result = pull8();
          setflags();
          A = result;
          break;
       //PLP
      case 0x28:
        SR = pull8() | SR_FIXED_BITS;
        break;
      //ROL A
      case 0x2A:
        value16 = (unsigned short)A;
        result = (value16 << 1) | (SR&SR_CARRY);
        setflags();	
        A = result&0xFF;
        break;
      //ROL
      case 0x26: case 0x36: case 0x2E: 
      case 0x3E:
        value16 = (unsigned short)read8(argument_addr);
        result = (value16 << 1) | (SR&SR_CARRY);
        setflags();
        write8(argument_addr, result&0xFF);
        break;
      //ROR A
      case 0x6A:
        value16 = (unsigned short)A;
        result = (value16 >> 1) | ((SR&SR_CARRY) << 7);
        result |= (value16&0x1)?0x8000:0;
        setflags();
        A = result&0xFF;
        break;
      //ROR
      case 0x66: case 0x76: case 0x6E: 
      case 0x7E:
        value16 = (unsigned short)read8(argument_addr);
        result = (value16 >> 1) | ((SR&SR_CARRY) << 7);
        result |= (value16&0x1)?0x8000:0;
        setflags();
        write8(argument_addr, result&0xFF);
        break;
      //RTI
      case 0x40:
        SR = pull8();
        PC = pull16();
        break;
      //RTS
      case 0x60:
        PC = pull16() + 1;
        break;
      //SBC
      case 0xE9: case 0xE5: case 0xF5:
      case 0xED: case 0xFD: case 0xF9:
      case 0xE1: case 0xF1:
        value16 = ((unsigned short)read8(argument_addr)) ^ 0x00FF;
        result = (unsigned short)A + value16 + (unsigned short)(SR&SR_CARRY);
        setflags();
        A = result&0xFF;
        break;
      //SEC
      case 0x38:
        SR |= SR_CARRY;
        break;
      //SED
      case 0xF8:
        SR |= SR_DEC;
        break;
      //SEI
      case 0x78:
        SR |= SR_INT;
        break;
      //STA
      case 0x85: case 0x95: case 0x8D:
      case 0x9D: case 0x99: case 0x81:
      case 0x91:
        write8(argument_addr, A);
        break;
      //STX
      case 0x86: case 0x96: case 0x8E:
        write8(argument_addr, X);
        break;
      //STY
      case 0x84: case 0x94: case 0x8C:
        write8(argument_addr, Y);
        break;
      //TAX
      case 0xAA:
        X = A;
        result = A;
        setflags();
        break;
      //TAY
      case 0xA8:
        Y = A;
        result = A;
        setflags();
        break;
      //TSX
      case 0xBA:
        X = STP;
        result = STP;
        setflags();
        break;
      //TXA
      case 0x8A:
        A = X;
        result = X;
        setflags();
        break;
      //TXS
      case 0x9A:
        STP = X;
        result = X;
        setflags();
        break;
      //TYA
      case 0x98:
        A = Y;
        result = Y;
        setflags();
        break;
      }
  ////////////////////  }
}

unsigned char is_up=0, shift = 0, mode = 0;
u8 kb_readByte(u8 command){
	static u8 state=0;
	u8 data=0;
	unsigned char i,c;

	if(state==0){//ready to transmit condition 
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_CLOCK_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_LATCH_PIN);
		Wait200ns();Wait200ns();Wait200ns();Wait200ns();Wait200ns();
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);
		Wait200ns();Wait200ns();Wait200ns();Wait200ns();Wait200ns();
		state = (command == KB_SEND_END ? 0:1);
	}

	for(i=0;i<8;i++){//read data
		data<<=1;

		if(command&0x80)
			JOYPAD_OUT_PORT |= _BV(JOYPAD_LATCH_PIN);		
		else
			JOYPAD_OUT_PORT &= ~(_BV(JOYPAD_LATCH_PIN));

		JOYPAD_OUT_PORT &= ~(_BV(JOYPAD_CLOCK_PIN));//pulse clock pin		
		command<<=1;
		Wait100ns();
		if((JOYPAD_IN_PORT&(1<<JOYPAD_DATA2_PIN))!=0)
			data |= 1;
		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);		
	}
	JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));	

	if(!data)
		return 0;

	uint8_t sc = data;



	//enter=0x5a

	if (!is_up)// Last data received was the up-key identifier
	{
		switch (sc)
		{
			case 0xF0 :// The up-key identifier
				is_up = 1;
				break;
			case 0x12 :// Left SHIFT
				shift = 1;
				break;

			case 0x59 :// Right SHIFT
				shift = 1;
				break;
			case 0x05 :// F1
				if(mode == 0)
				mode = 1;// Enter scan code mode
				if(mode == 2)
				mode = 3;// Leave scan code mode
				break;
			default:
				if(mode == 0 || mode == 3)// If ASCII mode
				{
					if(!shift)// If shift not pressed,
					{ // do a table look-up
					//	for(i = 0; pgm_read_byte(&(unshifted[i][0]))!=sc && pgm_read_byte(&(unshifted[i][0])); i++);
					//	if (pgm_read_byte(&(unshifted[i][0])) == sc) {
					//		c=pgm_read_byte(&(unshifted[i][1]));
					//		return c;////debug_char2(c);
					//		//if(sc==0x5a)debug_hex(c);
					//	}
return pgm_read_byte(&scancode_to_apple[sc]);
					} else {// If shift pressed
return pgm_read_byte(&scancode_to_apple[sc+0x80]);
					//	for(i = 0; pgm_read_byte(&(shifted[i][0]))!=sc && pgm_read_byte(&(shifted[i][0])); i++);
					//	if (pgm_read_byte(&(shifted[i][0])) == sc) {
					//		c=pgm_read_byte(&(shifted[i][1]));
					//		return c;/////debug_char2(c);
					//		//if(sc==0x5a)debug_hex(c);
					//	}
					}
				} else{ // Scan code mode
					return sc;//debug_hex(sc);// Print scan code
				}
				break;
		}
	} else {
		is_up = 0;// Two 0xF0 in a row not allowed
		switch (sc)
		{
			case 0x12 :// Left SHIFT
				shift = 0;
				break;
			case 0x59 :// Right SHIFT
				shift = 0;
				break;
			case 0x05 :// F1
				if(mode == 1)
				mode = 2;
				if(mode == 3)
				mode = 0;
				break;
		}
	}
	return 0;
}





void main(){
SpiRamCursorInit();
  // Load the reset vector
  PC = read16(0xFFFC);
  STP = 0xFD;

	SetTileTable(hack_font);
	//  Serial.begin(115200);
	clearScreen();
	//speaker_begin();
	cassette_begin();
	keyboard_begin();
	///sei();
int frame = 0;
	while(1){
		run();
		if(GetVsyncFlag()){
			ClearVsyncFlag();
			uint8_t t = kb_readByte(KB_SEND_END);
			if(t)
				keymem = t;
		}
/*
		PrintHexByte(10,20,opcode);
		PrintHexByte(10,22,PC>>8);
		PrintHexByte(12,22,PC);

		PrintHexByte(10,11,read8(0xFE84));
		PrintHexByte(12,11,read8(0xFE85));

		PrintHexByte(10,12,read8(0xFE86));
		PrintHexByte(12,12,read8(0xFE87));
*/
	}

}
