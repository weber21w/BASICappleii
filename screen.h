extern unsigned char vram[];

void writeCharacter(unsigned char row, unsigned char col, unsigned char val) {
row += 10;//HACK
vram[(row*VRAM_TILES_H)+col] = val;
}

unsigned char readCharacter(unsigned char row, unsigned char col) {
return vram[(row*VRAM_TILES_H)+col];

 // unsigned long transaction_begin;
 // unsigned char buf[4] = {0xFA, 0, 0};
 // buf[1] = col;
 // buf[2] = row;
  //////////////Serial.write(buf, 3);
  /////////////////////////////for(transaction_begin = millis(); !Serial.available(); millis() < (transaction_begin+50));
  /////////////////return Serial.read();

}

void clearScreen() {
for(uint16_t i=0;i<VRAM_SIZE;i++)
	vram[i] = ' '+64;
  unsigned char cmd = 0xFC;
  /////////Serial.write(&cmd, 1);
////  delay(20);
}

void screenScroll() {
  //unsigned char cmd = 0xFB;
  ////////////Serial.write(&cmd, 1);
  /////delay(20);
uint16_t voff = 0;
for(uint8_t y=0;y<VRAM_TILES_V-1;y++){
	for(uint8_t x=0;x<VRAM_TILES_H;x++){
		vram[voff] = vram[voff+VRAM_TILES_H];
		voff++;
	}
}
for(uint8_t x=0;x<VRAM_TILES_H;x++)
	vram[voff++] = ' '+64;

}

// squash all the modes for this demo
unsigned char appleToGhettoVGA(unsigned char apple) {
  //////////////////if(apple >= 0xC0) return apple-0x40;
  ///////else return apple&0x7F;
return apple&0x7F;
}

// Squash into normal mode
unsigned char ghettoVGAToApple(unsigned char ghettoVga) {
  if(ghettoVga >= 0x40 && ghettoVga <= 0x7f) return ghettoVga; 
  else return ghettoVga|0x80;
}

// Extract row, column addressing from woz's interlacing mode
static unsigned char g_row = 0, g_col = 0;
void decodeScreenAddress(unsigned short address) {
  unsigned char block_of_lines = (address>>7) - 0x08;
  unsigned char block_offset = (address&0x00FF) - ((block_of_lines&0x01) ? 0x80 : 0x00);
  if(block_offset < 0x28) {
    g_row = block_of_lines;
    g_col = block_offset;
  } else if(block_offset < 0x50) {
    g_row = block_of_lines + 8;
    g_col = block_offset-0x28;
  } else {
    g_row = block_of_lines + 16;
    g_col = block_offset-0x50;
  }
}

unsigned char screenRead(unsigned short address) {
    //Find our row/column
  decodeScreenAddress(address);
  return readCharacter(g_row, g_col);//ghettoVGAToApple(readCharacter(g_row, g_col));
}

void screenWrite(unsigned short address, unsigned char value) {
  //Find our row/column
  decodeScreenAddress(address);
  // If not bell character
  if(value != 0x87) writeCharacter(g_row, g_col, value);//appleToGhettoVGA(value));
}
