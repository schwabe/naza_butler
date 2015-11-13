// Pin 3 => S.Port Telemetry
// Pin 7 => Naza GPS
// Pin 9 => SBus

#define NazaSerial Serial3
#define SportSerial FrSkySportSingleWireSerial::SERIAL_1
#define sbusSerial Serial2 
#define DebugSerial Serial

#define MODECHANNEL 7


// Defines for APM/NAVLINK
#define FLIGHTMODE_ACRO_MODE       1
#define FLIGHTMODE_ATTI_MODE       2 
#define FLIGHTMODE_GPS_MODE        5 
#define FLIGHTMODE_FAIL_MODE       6


// SBUS values (including these values), tested via Naza Light software assitent

#define GPS_HIGH 1568
#define GPS_LOW 1537


// (For valaues 1012-1013, the assistent software jumps between Failsafe and Atti)
#define ATTI_HIGH 1011
#define ATTI_LOW 1040

#define MANUAL_LOW 481
#define MANUAL_HIGH 512


