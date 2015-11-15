#ifndef _NAZABUTLER_H
#define _NAZABUTLER_H 
#include <stdint.h>

//! Return compass heading or GPS heading depending on speed (>= 2m/s)
double getHeading();

// global variables
extern int receiver_rssi;
extern int flightmode;
extern uint16_t rc_inputs[];

extern int roll_pwm;
extern int pitch_pwm;

extern float pitch_rad;

extern uint16_t battery_voltage;

// Defines for APM/NAVLINK
#define FLIGHTMODE_ACRO_MODE       1
#define FLIGHTMODE_ATTI_MODE       2 
#define FLIGHTMODE_GPS_MODE        5 
#define FLIGHTMODE_FAIL_MODE       6


//! Convert from sbus values to Mavlink raw values (1000=0, 2000=100%)
static inline uint16_t sbusToPPM(uint16_t sbus)
{
  // 400 -> 1000, 1600 -> 2000
  return (sbus - 400) *5 / 6+1000;
}

static inline uint16_t sbusToPercent(uint16_t sbus)
{
  return sbusToPPM(sbus) -1000/2;
}

#endif

