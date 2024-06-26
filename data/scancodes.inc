//dongle commands
#define KB_SEND_KEY				0x00
#define KB_SEND_END				0x01
#define KB_SEND_DEVICE_ID		0x02
#define KB_SEND_FIRMWARE_REV	0x03
#define KB_RESET				0x7F


// Unshifted characters
const unsigned char unshifted[][2] PROGMEM = {
	{0x0d,9},
	{0x0e,'`'},
	{0x15,'q'},
	{0x16,'1'},
	{0x1a,'z'},
	{0x1b,'s'},
	{0x1c,'a'},
	{0x1d,'w'},
	{0x1e,'2'},
	{0x21,'c'},
	{0x22,'x'},
	{0x23,'d'},
	{0x24,'e'},
	{0x25,'4'},
	{0x26,'3'},
	{0x29,' '},
	{0x2a,'v'},
	{0x2b,'f'},
	{0x2c,'t'},
	{0x2d,'r'},
	{0x2e,'5'},
	{0x31,'n'},
	{0x32,'b'},
	{0x33,'h'},
	{0x34,'g'},
	{0x35,'y'},
	{0x36,'6'},
	{0x3a,'m'},
	{0x3b,'j'},
	{0x3c,'u'},
	{0x3d,'7'},
	{0x3e,'8'},
	{0x41,','},
	{0x42,'k'},
	{0x43,'i'},
	{0x44,'o'},
	{0x45,'0'},
	{0x46,'9'},
	{0x49,'.'},
	{0x4a,'/'},
	{0x4b,'l'},
	{0x4c,';'},
	{0x4d,'p'},
	{0x4e,'-'},
	{0x52,'\''},
	{0x54,'['},
	{0x55,'='},
	{0x5a,13},
	{0x5b,']'},
	{0x5d,'\\'},
	{0x66,8},
	{0x69,'1'},
	{0x6b,'4'},
	{0x6c,'7'},
	{0x70,'0'},
	{0x71,'.'},
	{0x72,'2'},
	{0x73,'5'},
	{0x74,'6'},
	{0x75,'8'},
	{0x79,'+'},
	{0x7a,'3'},
	{0x7b,'-'},
	{0x7c,'*'},
	{0x7d,'9'},
	{0,0}
};

// Shifted characters
const unsigned char shifted[][2] PROGMEM= {
	{0x0d,9},
	{0x0e,'~'},
	{0x15,'Q'},
	{0x16,'!'},
	{0x1a,'Z'},
	{0x1b,'S'},
	{0x1c,'A'},
	{0x1d,'W'},
	{0x1e,'@'},
	{0x21,'C'},
	{0x22,'X'},
	{0x23,'D'},
	{0x24,'E'},
	{0x25,'$'},
	{0x26,'#'},
	{0x29,' '},
	{0x2a,'V'},
	{0x2b,'F'},
	{0x2c,'T'},
	{0x2d,'R'},
	{0x2e,'%'},
	{0x31,'N'},
	{0x32,'B'},
	{0x33,'H'},
	{0x34,'G'},
	{0x35,'Y'},
	{0x36,'^'},
	{0x3a,'M'},
	{0x3b,'J'},
	{0x3c,'U'},
	{0x3d,'&'},
	{0x3e,'*'},
	{0x41,'<'},
	{0x42,'K'},
	{0x43,'I'},
	{0x44,'O'},
	{0x45,')'},
	{0x46,'('},
	{0x49,'>'},
	{0x4a,'?'},
	{0x4b,'L'},
	{0x4c,':'},
	{0x4d,'P'},
	{0x4e,'_'},
	{0x52,'\"'},
	{0x54,'{'},
	{0x55,'+'},
	{0x5a,13},
	{0x5b,'}'},
	{0x5d,'|'},
	{0x66,8},
	{0x69,'1'},
	{0x6b,'4'},
	{0x6c,'7'},
	{0x70,'0'},
	{0x71,'>'},
	{0x72,'2'},
	{0x73,'5'},
	{0x74,'6'},
	{0x75,'8'},
	{0x79,'+'},
	{0x7a,'3'},
	{0x7b,'-'},
	{0x7c,'*'},
	{0x7d,'9'},
	{0,0}
};




const unsigned char scancode_to_apple[] PROGMEM = {
 //$0    $1    $2    $3    $4    $5    $6    $7    $8    $9    $A    $B    $C    $D    $E    $F
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //$00
  0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0xB1, 0x00, 0x00, 0x00, 0xDA, 0xD3, 0xC1, 0xD7, 0xB2, 0x00, //$10
  0x00, 0xC3, 0xD8, 0xC4, 0xC5, 0xB4, 0xB3, 0x00, 0x00, 0xA0, 0xD6, 0xC6, 0xD4, 0xD2, 0xB5, 0x00, //$20
  0x00, 0xCE, 0xC2, 0xC8, 0xC7, 0xD9, 0xB6, 0x00, 0x00, 0x00, 0xCD, 0xCA, 0xD5, 0xB7, 0xB8, 0x00, //$30
  0x00, 0xAC, 0xCB, 0xC9, 0xCF, 0xB0, 0xB9, 0x00, 0x00, 0xAE, 0xAF, 0xCC, 0xBB, 0xD0, 0xAD, 0x00, //$40
  0x00, 0x00, 0xA7, 0x00, 0x00, 0xBD, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x00, 0x00, 0x00, 0x00, 0x00, //$50
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0xB1, 0x00, 0xB4, 0xB7, 0x00, 0x00, 0x00, //$60
  0xB0, 0xAE, 0xB2, 0xB5, 0xB6, 0xB8, 0x9B, 0x00, 0x00, 0xAB, 0xB3, 0xAD, 0xAA, 0xB9, 0x00, 0x00,  //$70
  // High mirror, shift modified keys
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //$80 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0xA1, 0x00, 0x00, 0x00, 0xDA, 0xD3, 0xC1, 0xD7, 0xC0, 0x00, //$90 1
  0x00, 0xC3, 0xD8, 0xC4, 0xC5, 0xA4, 0xA3, 0x00, 0x00, 0xA0, 0xD6, 0xC6, 0xD4, 0xD2, 0xA5, 0x00, //$A0 2
  0x00, 0xCE, 0xC2, 0xC8, 0xC7, 0xD9, 0xDE, 0x00, 0x00, 0x00, 0xCD, 0xCA, 0xD5, 0xA6, 0xAA, 0x00, //$B0 3
  0x00, 0xBC, 0xCB, 0xC9, 0xCF, 0xA9, 0xA8, 0x00, 0x00, 0xBE, 0xBF, 0xCC, 0xBA, 0xD0, 0xAD, 0x00, //$C0 4 
  0x00, 0x00, 0xA2, 0x00, 0x00, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x00, 0x00, 0x00, 0x00, 0x00, //$D0 5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0xB1, 0x00, 0xB4, 0xB7, 0x00, 0x00, 0x00, //$E0 6
  0xB0, 0xAE, 0xB2, 0xB5, 0xB6, 0xB8, 0x9B, 0x00, 0x00, 0xAB, 0xB3, 0xAD, 0xAA, 0xB9, 0x00, 0x00  //$F0 7
};