//! Return compass heading or GPS heading depending on speed (>= 2m/s)
double getHeading();

//! Convert from sbus values to Mavlink raw values (1000=0, 2000=100%)
uint16_t sbusToPPM(uint16_t sbus);

// global variables
extern int receiver_rssi;
extern int flightmode;
extern uint16_t rc_inputs[];

extern int roll_pwm;
extern int pitch_pwm;

extern float pitch_rad;

// Defines for APM/NAVLINK
#define FLIGHTMODE_ACRO_MODE       1
#define FLIGHTMODE_ATTI_MODE       2 
#define FLIGHTMODE_GPS_MODE        5 
#define FLIGHTMODE_FAIL_MODE       6


