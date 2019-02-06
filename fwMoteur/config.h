#ifndef _CONFIG_H_
#define _CONFIG_H_

#define ANALOG_FILTER 3 
#define RAMP_UINCPOW 13 // 1 increment = 1024 milli-increments (mincs) = 1024x1024 micro-increments (uincs)
#define RAMP_VPOW 6
#define RAMP_TO_POS_POW RAMP_UINCPOW-4

// rotation motor:
#define MOTA_END KZ1
#define MOTA_ENDLEVEL 0
#define MOTA_A K1
#define MOTA_B K2

// translation motor:
#define MOTB_END KZ1
#define MOTB_ENDLEVEL 0
#define MOTB_A K3
#define MOTB_B K4

#define TRANS_LOSW K5
#define TRANS_HISW K6
#define TRANS_SWLEVEL 0

#define ROT_ZERO K7
#define ROT_ZERO_LEVEL 0

#define LED MAI

#endif // _CONFIG_H_

