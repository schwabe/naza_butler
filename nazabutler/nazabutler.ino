#include "config.h"


#include "FrSkySportTelemetry.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorSp2Uart.h"

#include "NazaDecoderLib.h"
#include <ADC.h>

#define SERIAL_9BIT_SUPPORT
#include "FUTABA_SBUS.h"

#include <AltSoftSerial.h>

#include "nazabutler.h"
#include "analog.h"
#include "ppm.h"
#include "PulsePosition_ftm2.h"

FrSkySportSensorGps gps;                               // Create GPS sensor with default ID

FrSkySportTelemetry telemetry;                         // Create telemetry object
//FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
FrSkySportSensorSp2uart sp2uart;

PulsePositionInput rollPPM;
PulsePositionInput pitchPPM;


FUTABA_SBUS sBus;

int flightmode;
uint16_t rc_inputs[16];

unsigned long fix_time=0;
static float gps_altitude_last=0;
static int alt_Home_m=-1000; // not set

void checkPosition()
{
   unsigned long currtime=millis();
  //Home can be set when GPS is 3D fix, and altitude is not changing (within 20cm) for more than 10s
  if(  (NazaDecoder.getFixType() < 3) || (abs(gps_altitude_last - NazaDecoder.getGpsAlt()) > HOME_SET_PRECISION ) ) {
    fix_time = currtime + 10000;
    gps_altitude_last = NazaDecoder.getGpsAlt();
  } 
  
  #if defined(HOME_SET_AUTO_TIMEOUT)
  if( (currtime/1000) > HOME_SET_AUTO_TIMEOUT ) {
    fix_time = 0;
  }
  #endif

  //Set home altitude if not already set, and 3D fix and copter moving for more than 500ms
  if( (alt_Home_m == -1000) && (currtime > fix_time) ) {
    alt_Home_m = NazaDecoder.getGpsAlt();
  } 
}

double getHeading()
{
  // more than 2m/s
  if (NazaDecoder.getFixType() >= NazaDecoderLib::FIX_3D && NazaDecoder.getSpeed() >= 2)
    return NazaDecoder.getCog();
  else
    return NazaDecoder.getHeadingNc();
}

void setup()
{
  
  NazaSerial.begin(115200);
#ifdef DEBUG
  DebugSerial.begin(115200);
#endif

  rollPPM.begin(25);
  pitchPPM.begin(32);

  sBus.begin();
telemetry.begin(SportSerial, &gps, &sp2uart);

 MavlinkSerial.begin(57600);
 setupAnalogSensors();
 // setupPPM();

 pinMode(LED_BUILTIN, OUTPUT);
}


void checkSbusState()
{
    sBus.UpdateChannels();
//    for (int i=1;i<=16;i++) {
//      DebugSerial.printf("S%d: %d ", i , sBus.Channel(i));
//    }
//    DebugSerial.printf(" \n");

    auto oldMode = flightmode;
    auto modeChannel = sBus.Channel(MODE_CHANNEL);
    if (modeChannel >= GPS_LOW &&  modeChannel <= GPS_HIGH) {
      flightmode = FLIGHTMODE_GPS_MODE;
    } else if (modeChannel >= ATTI_LOW &&  modeChannel <= ATTI_HIGH) {
      flightmode = FLIGHTMODE_ATTI_MODE;
    } else if (modeChannel >= MANUAL_LOW && modeChannel <= MANUAL_HIGH) {
      flightmode = FLIGHTMODE_ACRO_MODE;
    } else {
      flightmode = FLIGHTMODE_FAIL_MODE;
    }
    sBus.toChannels=0;
    if (flightmode != oldMode)
      DebugSerial.printf("Flightmode (%d) changed to %d\n", modeChannel, flightmode);

    for (int i=0;i<16;i++) {
      rc_inputs[i]=sBus.Channel(i+1);
    }
  
}

void loop()
{
  auto currtime=millis();

  static unsigned long analogSensors = 0;
  if (currtime > analogSensors) {
    analogSensors= currtime+200;
    readAnalogSensors();
    // Blink with some weird pattern :)
    digitalWrite(LED_BUILTIN, (currtime >> 12) & 0x1);
    sp2uart.setData(battery_voltage/1000.0f, getReceiverRSSI());

//    DebugSerial.printf("roll_ppm: %d, pitch_ppm: %d\n", roll_ppm, pitch_ppm);
  }
    
  if(Serial3.available())
  {
    char c = Serial3.read();
    uint8_t decodedMessage = NazaDecoder.decode(c);
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
#ifdef DEBUG_GPS
        DebugSerial.print("Lat: "); Serial.print(NazaDecoder.getLat(), 7);
        DebugSerial.print(", Lon: "); Serial.print(NazaDecoder.getLon(), 7);
        DebugSerial.print(", Alt: "); Serial.print(NazaDecoder.getGpsAlt(), 7);
        DebugSerial.print(", Fix: "); Serial.print(NazaDecoder.getFixType());
        DebugSerial.print(", Sat: "); Serial.println(NazaDecoder.getNumSat());
#endif

        checkPosition();

        if (alt_Home_m != -1000) {
            // We just assume that heading is recent enough
            gps.setData(NazaDecoder.getLat(), NazaDecoder.getLon(), NazaDecoder.getGpsAlt(),
                        NazaDecoder.getSpeed(), getHeading(),
                        NazaDecoder.getYear(), NazaDecoder.getMonth(),NazaDecoder.getDay(),
                        NazaDecoder.getHour(), NazaDecoder.getMinute(), NazaDecoder.getSecond());
        }
        break;
      case NAZA_MESSAGE_COMPASS:
          // DebugSerial.print("Heading: "); Serial.println(NazaDecoder.getHeadingNc(), 2);
        // void setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
        
        break;
      case NAZA_MESSAGE_NONE:
        //ignore, no full message yet
        break;
    }
  }
  
  
  auto chans =pitchPPM.available();
  if (chans >=1) {
    pitch_ppm = pitchPPM.read(1);;
    for (int i=2;i<=chans;i++) {
      pitchPPM.read(i);
      //DebugSerial.printf("P%d: %.2f ", i,);
    }
  }

  chans =rollPPM.available();
  if (chans >=1) {
    roll_ppm = rollPPM.read(1);
    for (int i=2;i<=chans;i++) {
      rollPPM.read(i);
      //DebugSerial.printf("R%d: %.2f ", i, rollPPM.read(i));
    }
    //DebugSerial.printf(" %d\n",chans);
  }

  
  sBus.FeedLine();
  if (sBus.toChannels) {
    checkSbusState();
  }
  
  telemetry.send();
  sendMavlinkMessages();
}


