#ifndef _STEP1_2_IO_
#define _STEP1_2_IO_

#define MAENPORT B
#define MAENBIT 3
#define MAENAN 9

#define MAEN2 KZ1

#define MA1PORT A
#define MA1BIT 4
#define MA1AN 2

#define MA2PORT B
#define MA2BIT 5
#define MA2PWM 3
#define MA2SETUP_PWM() do{PSTR3CON=0; PSTR3CONbits.STR3A=1;} while(0)

#define MOTA_CONFIG() do{MA2SETUP_PWM();} while(0)
#define MOTA_PWM MA2PWM

#define MASENSEPORT A
#define MASENSEBIT 	0
#define MASENSEAN 	0

#define MAIPORT C
#define MAIBIT 1
#define MAIPWM 2
#define MAISETUP_PWM() do{PSTR2CON=0; PSTR2CONbits.STR2A=1;} while(0)

#define MACPORT A
#define MACBIT 3
#define MACAN 3

//--------------------------------

#define MBENPORT B
#define MBENBIT 4
#define MBENAN 11
#define MBENPWM 1
#define MBENSETUP_PWM() do{PSTR1CON=0; PSTR1CONbits.STR1D=1;} while(0)

#define MBEN2 KZ1

#define MB1PORT B
#define MB1BIT 0
#define MB1AN 12
#define MB1PWM 4
#define MB1SETUP_PWM() do{} while(0)

#define MB2PORT B
#define MB2BIT 1
#define MB2AN 10
#define MB2PWM 1
#define MB2SETUP_PWM() do{PSTR1CON=0; PSTR1CONbits.STR1C=1;} while(0)

#define MBSENSEPORT A
#define MBSENSEBIT 	1
#define MBSENSEAN 	1

#define MBIPORT C
#define MBIBIT 2
#define MBIAN 14
#define MBIPWM 1
#define MBISETUP_PWM() do{PSTR1CON=0; PSTR1CONbits.STR1A=1;} while(0)

#define MBCPORT A
#define MBCBIT 2
#define MBCAN 2

#define MOTB_CONFIG() do{MB2SETUP_PWM();} while(0)
#define MOTB_PWM MB2PWM

//--------------------------------

//#define LEDPORT A
//#define LEDBIT 	7

//--------------------------------

#define K1PORT 	A
#define K1BIT 	5
#define K1AN	4

#define K2PORT	C
#define K2BIT 	3
#define K2AN	15

#define K3PORT	C
#define K3BIT 	4
#define K3AN	16

#define K4PORT	C
#define K4BIT 	5
#define K4AN	17

#define K5PORT	C
#define K5BIT 	6
#define K5AN	18

#define K6PORT	C
#define K6BIT 	7
#define K6AN	19
#define K6PWM	3
#define K6SETUP_PWM() do{PSTR3CON=0; PSTR3CONbits.STR3B=1;} while(0)

#define K7PORT	B
#define K7BIT 	2
#define K7AN	8
#define K7PWM	1
#define K7SETUP_PWM() do{PSTR1CON=0; PSTR1CONbits.STR1B=1;} while(0)
#define K7INT	2

#endif
