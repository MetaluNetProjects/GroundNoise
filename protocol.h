#ifndef _PROTOCOL_H
#define _PROTOCOL_H
enum {
	STATE_IDLE = 0,
	STATE_HOMING,
	STATE_HOMED,
	STATE_RUNNING,
//	STATE_END
};

enum {
	ERROR_NONE = 0,
	ERROR_NO_ROT_ZERO, // rot zero wasn't detected
	ERROR_NO_TRANS_HOME, // trans home wasn't detected
};

enum {
	MODE_MANUAL = 0,
	MODE_AUTO
};

// for broadcast fruit-to-fruit communication:
#define MOTOR_ID 100
#define CONTROL_ID 200

#define ROT_PULSES_PER_TURN 4021UL //2010
#define TRANS_PULSES_PER_TURN 1690UL //1690
#define NB_TURNS 65
#define RAMPDIV_POW 4

#define ROT_RAMP_HIPOS ((ROT_PULSES_PER_TURN * NB_TURNS) >> RAMPDIV_POW)
#define TRANS_RAMP_HIPOS ((TRANS_PULSES_PER_TURN * NB_TURNS) >> RAMPDIV_POW)
#define ROT2TRANS(rot_pulses) (((long)(rot_pulses) * 27)>>6) // 27/64 ~= 1690/4021

#define CENTER_SPEED 1000

#define MAXSPEED 8200

// 1000 = 0.5tr/mn = 120s
// 8500 = 4.25tr/mn = 14s


#endif

