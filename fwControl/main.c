/*********************************************************************
 *               Ground Noise Control 
 * .board: Versa2.0
 *  Antoine Rousseau @ metalu.net - feb.2019
 *********************************************************************/

#define BOARD Versa2

#include <fruit.h>
#include <analog.h>
#include <i2c_master.h>
#include <ht16k33.h>

#include "../protocol.h"

t_delay mainDelay;
char oldStart = 0;
char oldStop = 0;
char oldHome = 0;

unsigned char error = ERROR_NONE;
unsigned char state = STATE_IDLE;
int pos;
unsigned char turns;

int speed;

int loopCount = 0;
char doQuery = 1;

ht16k33 ledDisplay;
t_delay inhibScreen = 0;

void setup(void) {	
//----------- Setup ----------------
	fruitInit();

	pinModeDigitalOut(LED); 		// set the LED pin mode to digital out
	digitalClear(LED);				// clear the LED
	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms

	pinModeDigitalIn(STARTSW);
	pinModeDigitalIn(STOPSW);
	pinModeDigitalIn(HOMESW);

//----------- Analog setup ----------------
	analogInit();		// init analog module
	analogSelect(0,SPEEDPOT);	// assign connector K1 to analog channel 0

	// setup I2C master:
	i2cm_init(I2C_MASTER, I2C_SLEW_ON, FOSC/400000/4-1);
	
	// setup ht16k33
	ht16k33_init(&ledDisplay, 0x70);
	ht16k33_setBrightness(&ledDisplay, 6);
		// print "----":
	ht16k33_writeDigitRaw(&ledDisplay, 0, 0x40);
	ht16k33_writeDigitRaw(&ledDisplay, 1, 0x40);
	ht16k33_writeDigitRaw(&ledDisplay, 2, 0x00); // colon
	ht16k33_writeDigitRaw(&ledDisplay, 3, 0x40);
	ht16k33_writeDigitRaw(&ledDisplay, 4, 0x40);
	ht16k33_writeDisplay(&ledDisplay);
}

void print(int n, unsigned char dots)
{
	ht16k33_printNumber(&ledDisplay, n, 10);
	ht16k33_writeDigitRaw(&ledDisplay, 2, dots);
	ht16k33_writeDisplay(&ledDisplay);
}

void sendSpeed()
{
	unsigned char frbuf[6];
	unsigned int finalSpeed = ((unsigned long)speed * CENTER_SPEED / 1000);
	frbuf[0] = 'C';
	frbuf[1] = MOTOR_ID;
	frbuf[2] = 'S';
	frbuf[3] = finalSpeed>>8;
	frbuf[4] = finalSpeed&255;
	frbuf[5] = '\n';
	fraiseSendBroadcast(frbuf, 6);
	delayStart(inhibScreen, 2000000);
}

void speedService()
{
	static char up = 0;
	// pot[0 -> 1024] => speed[900 -> 1100]
	int tmpspeed = 898 + (((long)(analogGet(0) >> ANALOG_FILTER) * 206) >> 10);
	int tmp_seconds;
	
	if(tmpspeed > 1100) tmpspeed = 1100;
	if(tmpspeed < 900) tmpspeed = 900;
	
	if(up) {
		if(tmpspeed < speed - 2) {
			up = 0;
		}
		else if(tmpspeed <= speed) return;
	} else {
		if(tmpspeed > speed + 2) {
			up = 1;
		}
		else if(tmpspeed >= speed) return;
	}
	speed = tmpspeed;
//	print(speed, 0);
	tmp_seconds = 1330000UL/speed;
	print((tmp_seconds/60)*100 + (tmp_seconds%60), 2);
	sendSpeed();
}

void doStart()
{
	unsigned char frbuf[5];
	frbuf[0] = 'C';
	frbuf[1] = MOTOR_ID;
	frbuf[2] = 'G';
	frbuf[3] = 'O';
	frbuf[4] = '\n';
	fraiseSendBroadcast(frbuf, 5);
}

void doStop()
{
	unsigned char frbuf[8];
	frbuf[0] = 'C';
	frbuf[1] = MOTOR_ID;
	frbuf[2] = 'A';
	frbuf[3] = 'B';
	frbuf[4] = 'O';
	frbuf[5] = 'R';
	frbuf[6] = 'T';
	frbuf[7] = '\n';
	fraiseSendBroadcast(frbuf, 8);
}

void doHome()
{
	unsigned char frbuf[8];
	frbuf[0] = 'C';
	frbuf[1] = MOTOR_ID;
	frbuf[2] = 'H';
	frbuf[3] = 'O';
	frbuf[4] = 'M';
	frbuf[5] = 'E';
	frbuf[6] = '\n';
	fraiseSendBroadcast(frbuf, 7);
}

void switchesService()
{
	char tmpStart = !digitalRead(STARTSW);
	char tmpStop = !digitalRead(STOPSW);
	char tmpHome = !digitalRead(HOMESW);
	
	if(tmpStart != oldStart) {
		oldStart = tmpStart;
		doStart();
	}
	if(tmpStop != oldStop) {
		oldStop = tmpStop;
		doStop();
	}
	if(tmpHome != oldHome) {
		oldHome = tmpHome;
		doHome();
	}
}

void queryMotorStatus()
{
	unsigned char frbuf[5];
	frbuf[0] = 'C';
	frbuf[1] = MOTOR_ID;
	frbuf[2] = 'U';
	frbuf[3] = '\n';
	fraiseSendBroadcast(frbuf, 4);
}

void loop() {
// ---------- Main loop ------------
	fraiseService();	// listen to Fraise events
	analogService();	// analog management routine

	if(delayFinished(mainDelay)) // when mainDelay triggers :
	{
		loopCount++;
		delayStart(mainDelay, 5000); 	// re-init mainDelay
		analogSend();		// send analog channels that changed
		printf("Cs %d %d %d\n",oldHome, oldStop, oldStart);
		fraiseService();
		speedService();
		fraiseService();
		switchesService();
		fraiseService();
		if(loopCount == 5) {
			loopCount =0;
			if(doQuery) queryMotorStatus();
		}
	}
}

// Receiving

void fraiseReceiveChar() // receive text
{
	unsigned char c;
	
	c=fraiseGetChar();
	if(c=='L'){		//switch LED on/off 
		c=fraiseGetChar();
		digitalWrite(LED, c!='0');		
	}
	else if(c=='E') { 	// echo text (send it back to host)
		printf("C");
		c = fraiseGetLen(); 			// get length of current packet
		while(c--) printf("%c",fraiseGetChar());// send each received byte
		putchar('\n');				// end of line
	}
	else if(c=='Q'){		//switch doQuery on/off 
		c=fraiseGetChar();
		doQuery = c!='0';
	}
}

void fraiseReceiveBroadcast() // receive broadcast
{
	unsigned char c;
	
	c=fraiseGetChar();
	if(c != CONTROL_ID) return; // adress verification
	c=fraiseGetChar();
	if(c == 'U') {
		state = fraiseGetChar();
		error = fraiseGetChar();
		pos = fraiseGetInt();
		turns = fraiseGetChar();
	}
	if((!inhibScreen) || delayFinished(inhibScreen)) {
		inhibScreen = 0;
		print((unsigned long)pos*9999/ROT_RAMP_HIPOS, 0);
	}
}

void fraiseReceive() // receive raw
{
	unsigned char c;
	c=fraiseGetChar();
	
	switch(c) {
		case 1: ht16k33_setBrightness(&ledDisplay, fraiseGetChar()); break;
	}
}

