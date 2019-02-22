/*********************************************************************
 *               Ground Noise Motor 
 * .board: StepGN (FraiseStep 1.2 + 16MHz quartz)
 *  Antoine Rousseau @ metalu.net - feb.2019
 *********************************************************************/

#define BOARD StepGN

#include <fruit.h>
#include <analog.h>
#include <dcmotor.h>
#include <ramp.h>
#include <eeparams.h>

#include "../protocol.h"

//t_ramp posRamp;
t_delay mainDelay;

#ifdef _DCMOTOR_H_
#define USE_MOTORS
#endif

#ifdef USE_MOTORS
DCMOTOR_DECLARE(A);
DCMOTOR_DECLARE(B);
#endif

long int speed = 0;
int targetSpeed = 0;
#define SPEED_FILTER 6

int transHomePWM = -1024;
int rotHomePWM = 200;

unsigned char error = ERROR_NONE;
unsigned char state = STATE_IDLE;
unsigned char mode = MODE_MANUAL;

//char startSw = 0;
//char stopSw = 0;
char homeSw = 0;
//char dirSw = 0;
//char modeSw = 0;

//-------------  Timer1 macros :  ---------------------------------------- 
//prescaler=PS fTMR1=FOSC/(4*PS) nbCycles=0xffff-TMR1init T=nbCycles/fTMR1=(0xffff-TMR1init)*4PS/FOSC
//TMR1init=0xffff-(T*FOSC/4PS) ; max=65536*4PS/FOSC : 
//ex: PS=8 : T=0.01s : TMR1init=0xffff-15000
//Maximum 1s !!
#define	TMR1init(T) (0xffff-((T*FOSC)/32000)) //ms ; maximum: 8MHz:262ms 48MHz:43ms 64MHz:32ms
#define	TMR1initUS(T) (0xffff-((T*FOSC)/32000000)) //us ; 
#define InitTimer(T) do{ TMR1H=TMR1init(T)/256 ; TMR1L=TMR1init(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define InitTimerUS(T) do{ TMR1H=TMR1initUS(T)/256 ; TMR1L=TMR1initUS(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define TimerOut() (PIR1bits.TMR1IF)

void highInterrupts()
{
	if(PIR1bits.TMR1IF) {
		DCMOTOR_CAPTURE_SERVICE(A);
		DCMOTOR_CAPTURE_SERVICE(B);
		InitTimerUS(10);
	}
}

void setup(void) {	
//----------- Setup ----------------
	fruitInit();
			
	pinModeDigitalOut(LED); 	// set the LED pin mode to digital out
	digitalClear(LED);		// clear the LED
	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms

//----------- Analog setup ----------------
	analogInit();		// init analog module
	analogSelect(0, MAC);	// assign MotorA current sense to analog channel 0
	analogSelect(1, MBC);	// assign MotorB current sense to analog channel 1

//----------- dcmotor setup ----------------

#ifdef USE_MOTORS
    dcmotorInit(A);
    dcmotorInit(B);
    pinModeDigitalIn(ROT_ZERO);
    pinModeDigitalIn(TRANS_LOSW);
    pinModeDigitalIn(TRANS_HISW);
#endif
	
	//rampInit(&posRamp);
	
//#define HW_PARAMS	
#ifdef HW_PARAMS
	DCMOTOR(A).Setting.PosWindow = 6;
	DCMOTOR(A).Setting.PwmMin = 50;
	//DCMOTOR(D).Setting.PosErrorGain = 6;
	//DCMOTOR(D).Setting.onlyPositive = 0;
	
	DCMOTOR(A).PosRamp.maxSpeed = 1800;
	DCMOTOR(A).PosRamp.maxAccel = 2400;
	DCMOTOR(A).PosRamp.maxDecel = 2400;
	//rampSetPos(&DCMOTOR(C).PosRamp, 0);

	DCMOTOR(A).PosPID.GainP = 40;
	DCMOTOR(A).PosPID.GainI = 1;
	DCMOTOR(A).PosPID.GainD = 0;
	DCMOTOR(A).PosPID.MaxOut = 1023;

	//DCMOTOR(C).VolVars.homed = 0;
#else
	EEreadMain();
#endif
	
	DCMOTOR(B).Setting.reversed = 1;

	T1CON=0b00110011;//src=fosc/4,ps=8,16bit r/w,on.
	PIE1bits.TMR1IE=1;  //1;
	IPR1bits.TMR1IP=1;
}

void testRotZero()
{
	static unsigned char oldRotZero = 0;
	unsigned char rotZero = 0;
	
	if(digitalRead(ROT_ZERO) == ROT_ZERO_LEVEL)
		rotZero = 1;
		
	if(rotZero == oldRotZero) return;
	oldRotZero = rotZero;
	
	/*if(state == STATE_RUNNING) {
		if(oldRotZero == 0) return;	
		if(rampGetPos(&(DCMOTOR(A).PosRamp)) < ROT_PULSES_PER_TURN / 2) return; // avoid false zero before half of the turn

		turns++;
		
		if(turns >= NB_TURNS) {
			state = STATE_IDLE;
			DCMOTOR(A).Vars.PWMConsign = 0;
			DCMOTOR(A).Setting.Mode = 0;
			return;
		}
		
		rampSetPosMoving(&(DCMOTOR(A).PosRamp), 0);
		DCMOTOR(A).VolVars.Position = 0;
		rampSetPosMoving(&(DCMOTOR(B).PosRamp), 0);
		DCMOTOR(B).VolVars.Position = 0;
		
		if(turns == (NB_TURNS - 1)) rampGoto(&(DCMOTOR(A).PosRamp), ROT_PULSES_PER_TURN);
		else rampGoto(&(DCMOTOR(A).PosRamp), ROT_PULSES_PER_TURN * 2);
	}
	else*/
	#if 0
	if((state == STATE_HOMING) && oldRotZero) {
		DCMOTOR(A).VolVars.Position = 0;
		rampSetPos(&(DCMOTOR(A).PosRamp), 0);
		DCMOTOR(A).VolVars.homed = 1;
		DCMOTOR(A).Vars.PWMConsign = 0;
	}
	#endif
}

void testTransEnds()
{
	if((digitalRead(TRANS_HISW) == TRANS_SWLEVEL) && (DCMOTOR(B).Vars.PWMConsign > 0)) {
		DCMOTOR(B).Vars.PWMConsign = 0;
		DCMOTOR(B).Setting.Mode = 0;
	}
		
	if((digitalRead(TRANS_LOSW) == TRANS_SWLEVEL) && (DCMOTOR(B).Vars.PWMConsign < 0)) {
		DCMOTOR(B).Vars.PWMConsign = 0;
		DCMOTOR(B).Setting.Mode = 0;
	}

	/*if((state == STATE_RUNNING) && (turns >= NB_TURNS) && (digitalRead(TRANS_HISW) == TRANS_SWLEVEL)) {
		state = STATE_IDLE;
	}
	else */

	if((state == STATE_RUNNING) 
	&& (digitalRead(TRANS_HISW) == TRANS_SWLEVEL)
	&& (mode == MODE_AUTO)) {
		state = STATE_HOMING;
		DCMOTOR(A).Vars.PWMConsign = 0; // stop rotation
		DCMOTOR(A).Setting.Mode = 0;
		DCMOTOR(B).Vars.PWMConsign = transHomePWM;
		DCMOTOR(B).Setting.Mode = 0;
	}
		
	if((state == STATE_HOMING) 
	/*&& (DCMOTOR(A).VolVars.homed == 1) */
	&& (digitalRead(TRANS_LOSW) == TRANS_SWLEVEL)) {
		state = STATE_RUNNING;
		DCMOTOR(A).VolVars.Position = 0;
		DCMOTOR(B).VolVars.Position = 0;
		//rampSetPos(&(DCMOTOR(B).PosRamp), 0);
		DCMOTOR(A).VolVars.homed = 1;
		DCMOTOR(B).VolVars.homed = 1;
		/*DCMOTOR(A).Vars.SpeedConsign = speed>>SPEED_FILTER; 
		DCMOTOR(A).Setting.Mode = 1;*/
	}
}
	
void sendStatus()
{
	unsigned char frbuf[10];
	frbuf[0] = 'B';
	frbuf[1] = CONTROL_ID;
	frbuf[2] = 'U';
	frbuf[3] = state;
	frbuf[4] = error;
//#define RTNPOS (DCMOTOR_GETPOS(B)>>4)
#define RTNPOS (rampGetPos(&(DCMOTOR(A).PosRamp)))
	frbuf[5] = (RTNPOS >> 8) & 255;
	frbuf[6] = RTNPOS & 255;
	//frbuf[7] = turns & 255;
	frbuf[8] = '\n';
	fraiseSendBroadcast(frbuf, 9);
}

int loopCount = 0;
char doSendStatus = 0;

#define DCMOTOR_UPDATE_ASYM2_(motID) do{ \
	DCMOTOR_FORMATPWM(motID);\
	dcmotor_vabs = dcmotor_v < 0 ? 1023 + dcmotor_v : dcmotor_v; \
	SET_PWM(MOT##motID##_PWM, dcmotor_vabs); \
	if(dcmotor_v < 0) { digitalSet(M##motID##1);}\
	else { digitalClear(M##motID##1);}\
 } while(0)
#define DCMOTOR_UPDATE_ASYM2(motID) CALL_FUN(DCMOTOR_UPDATE_ASYM2_,motID)

void loop() {
// ---------- Main loop ------------
	fraiseService();	// listen to Fraise events
	analogService();	// analog management routine

	if(delayFinished(mainDelay)) // when mainDelay triggers :
	{
		delayStart(mainDelay, 10000); 	// re-init mainDelay
		analogSend();		// send analog channels that changed
		fraiseService();
#ifdef USE_MOTORS
		speed = speed - (speed>>SPEED_FILTER) + targetSpeed;

		if(state == STATE_RUNNING) {
			DCMOTOR(A).Vars.SpeedConsign = speed>>SPEED_FILTER;
			DCMOTOR(A).Setting.Mode = 1;
		}
		
		if(homeSw) {
			DCMOTOR(B).Vars.PWMConsign = transHomePWM;
			DCMOTOR(B).Setting.Mode = 0;
		} else {
			if(state == STATE_RUNNING) {
				DCMOTOR(B).Vars.SpeedConsign = -1*ROT2TRANS(speed>>SPEED_FILTER);
				DCMOTOR(B).Setting.Mode = 1;
			} else {
				//DCMOTOR(B).Vars.PWMConsign = 0;
				//DCMOTOR(B).Setting.Mode = 0;
			}
		}

		testRotZero();
		testTransEnds();
		DCMOTOR_COMPUTE(A,ASYM2);
		/*if(state == STATE_RUNNING)
			rampGoto(&DCMOTOR(B).PosRamp, ROT2TRANS(rampGetPos(&DCMOTOR(A).PosRamp)));*/
		
		DCMOTOR_COMPUTE(B,ASYM2);
		
		fraiseService();
		
		/*if(	(state == STATE_RUNNING)
		&&	(rampGetPos(&DCMOTOR(A).PosRamp) == DCMOTOR(A).PosRamp.destPos)
		) {
			state = STATE_IDLE; // FINISHED
		}*/
#endif
		//rampCompute(&(DCMOTOR(A).PosRamp));
		//printf("Cr %ld %d %d\n", /*rampGetPos(&(DCMOTOR(A).PosRamp))*/RTNPOS, /*(DCMOTOR(A).PosRamp).speed*/ state, DCMOTOR(A).VolVars.homed);
		loopCount++;
		if(loopCount == 100) {
			loopCount = 0;
			if(doSendStatus) sendStatus();
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
	else
	if(c=='E') { 	// echo text (send it back to host)
		printf("C");
		c = fraiseGetLen(); 			// get length of current packet
		while(c--) printf("%c",fraiseGetChar());// send each received byte
		putchar('\n');				// end of line
	}
	else if(c=='W') { 	// WRITE: save eeprom
		if((fraiseGetChar() == 'R') && (fraiseGetChar() == 'I') && (fraiseGetChar() == 'T') && (fraiseGetChar() == 'E'))
			EEwriteMain();
	}
	else if(c=='s'){		//switch doSendStatus on/off 
		c=fraiseGetChar();
		doSendStatus = c!='0';
	}
	else if(c=='U') { // get status
		sendStatus();
	}
}

void fraiseReceiveCharBroadcast() // receive broadcast text
{
	unsigned char c;
	
	c=fraiseGetChar();
	if(c != MOTOR_ID) return; // adress verification
	
	c=fraiseGetChar();
	if(c=='L'){		//switch LED on/off 
		c=fraiseGetChar();
		digitalWrite(LED, c!='0');		
	}
	else if(c=='G') {	// GO
		if(fraiseGetChar() != 'O') return;
/*//#define DEBUGA
#ifdef DEBUGA
		if(state == STATE_RUNNING) return;
		DCMOTOR(A).VolVars.Position = 0;
		rampSetPos(&(DCMOTOR(A).PosRamp), 0);
		DCMOTOR(A).VolVars.homed = 1;
		DCMOTOR(B).VolVars.Position = 0;
		rampSetPos(&(DCMOTOR(B).PosRamp), 0);
		DCMOTOR(B).VolVars.homed = 1;
#else
		if(state != STATE_HOMED) return;
#endif
		//turns = 0;*/
		state = STATE_RUNNING;
		/*rampGoto(&(DCMOTOR(A).PosRamp), ROT_RAMP_HIPOS );
		DCMOTOR(A).Setting.Mode = 2;
		DCMOTOR(B).Setting.Mode = 2;*/
		
	}
	else if(c=='H') {	// HOME
		if(fraiseGetChar() != 'O') return;
		if(fraiseGetChar() != 'M') return;
		if(fraiseGetChar() != 'E') return;
		
		homeSw = fraiseGetChar() - '0';
		//if(state != STATE_IDLE) return;

		/*state = STATE_HOMING;

		if(digitalRead(ROT_ZERO) != ROT_ZERO_LEVEL) {
			DCMOTOR(A).VolVars.homed = 0;
			if(DCMOTOR_GETPOS(A) > ROT_PULSES_PER_TURN)
				DCMOTOR(A).Vars.PWMConsign = rotHomePWM;
			else DCMOTOR(A).Vars.PWMConsign = -rotHomePWM;
			DCMOTOR(A).Setting.Mode = 0;
		} else {
			DCMOTOR(A).VolVars.Position = 0;
			rampSetPos(&(DCMOTOR(A).PosRamp), 0);
			DCMOTOR(A).VolVars.homed = 1;
		}

		if(digitalRead(TRANS_LOSW) != TRANS_SWLEVEL)
			DCMOTOR(B).Vars.PWMConsign = transHomePWM;
		DCMOTOR(B).Setting.Mode = 0;
		DCMOTOR(B).VolVars.homed = 0;*/
		
	}
	else if(c=='M') {	// MOD
		if(fraiseGetChar() != 'O') return;
		if(fraiseGetChar() != 'D') return;
		
		mode = fraiseGetChar() - '0';
	}
	else if(c=='A') {	// ABORT
		if(fraiseGetChar() != 'B') return;
		if(fraiseGetChar() != 'O') return;
		if(fraiseGetChar() != 'R') return;
		if(fraiseGetChar() != 'T') return;

		//if(state == STATE_IDLE) return;

		state = STATE_IDLE;
		DCMOTOR(A).Vars.PWMConsign = 0;
		DCMOTOR(A).Setting.Mode = 0;
		DCMOTOR(B).Vars.PWMConsign = 0;
		DCMOTOR(B).Setting.Mode = 0;
	}
	else if(c=='U') { // get status
		sendStatus();
	}
	else if(c=='S') { //SPEED
		//(DCMOTOR(A).PosRamp).maxSpeed = fraiseGetInt();
		targetSpeed = fraiseGetInt();
	}
}

void fraiseReceive() // receive raw
{
	unsigned char c;
	c=fraiseGetChar();
	
	switch(c) {
#ifdef USE_MOTORS
	    case 120 : DCMOTOR_INPUT(A) ; break;
	    case 121 : DCMOTOR_INPUT(B) ; break;
#endif
//		case 200 : rampInput(&posRamp);
	}
}

void EEdeclareMain()
{
	//rampDeclareEE(&(DCMOTOR(A).PosRamp));
	DCMOTOR_DECLARE_EE(A);
	DCMOTOR_DECLARE_EE(B);
}
