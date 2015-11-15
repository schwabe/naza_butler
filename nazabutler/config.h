// Pin 3 => S.Port Telemetry
// Pin 7 => Naza GPS
// Pin 9 => SBus

// 115200 8N1. RX only
#define NazaSerial Serial3

// 100000 8N1, inverted, half-duplex
#define SportSerial FrSkySportSingleWireSerial::SERIAL_1

// 100000 8E2, inverted, RX only
#define sbusSerial Serial2 

// 57600 8N1, TX Only
#define MavlinkSerial altSerial

// USB Port, emulated
#define DebugSerial Serial

#define THROTTLECHANNEL 3
#define MODECHANNEL 7

// sbus value to consider being armed
#define THROTTLEARMED 600

#define HOME_SET_PRECISION         0.2 //Precision to wait for 10s before setting altitude home (don't set lower than 0.1)
#define HOME_SET_AUTO_TIMEOUT      90 //Home will be automatically set after 90s



//Artificial horizon
#define PITCH_GAIN                 4.3 //Set Pitch Naza gain to 20 in Naza GUI
#define PITCH_LEVEL                1500
//#define PITCH_INVERT               //Uncomment if pitch output is reversed (if horizon is not moving up when copter has nose down)

#define ROLL_GAIN                  4.3 //Set Roll Naza gain to 20 in Naza GUI 
#define ROLL_LEVEL                 1500
//#define ROLL_INVERT                //Uncomment if roll output is reversed (if horizon (/) is not the opposite (\) of the copter position)



// SBUS values (including these values), tested via Naza Light software assitent

#define GPS_HIGH 1568
#define GPS_LOW 1537


// (For valaues 1012-1013, the assistent software jumps between Failsafe and Atti)
#define ATTI_HIGH 1011
#define ATTI_LOW 1040

#define MANUAL_LOW 481
#define MANUAL_HIGH 512




