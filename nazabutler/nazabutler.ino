#include "config.h"
#include "nazabutler.h"

#include "FrSkySportTelemetry.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorFlvss.h"

#include "NazaDecoderLib.h"

#define SERIAL_9BIT_SUPPORT
#include "FUTABA_SBUS.h"

#include <AltSoftSerial.h>
//#include <PulsePosition.h>

FrSkySportSensorGps gps;                               // Create GPS sensor with default ID

FrSkySportTelemetry telemetry;                         // Create telemetry object
FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID

//PulsePositionInput ppmSum;

FUTABA_SBUS sBus;

int flightmode;
int receiver_rssi;
uint16_t rc_inputs[16];

unsigned long fix_time=0;
static float gps_altitude_last=0;
static int alt_Home_m=-1000; // not set

extern AltSoftSerial altSerial;

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
  telemetry.begin(SportSerial, &gps);
  NazaSerial.begin(115200);
#ifdef DEBUG
  DebugSerial.begin(115200);
#endif
// ppmSum.begin(23); 
 sBus.begin();

 MavlinkSerial.begin(57600);
}


void checkSbusState()
{
    sBus.UpdateChannels();
#if 0
    for (int i=1;i<=16;i++) {
      DebugSerial.printf("S%d: %d ", i , sBus.Channel(i));
    }
    DebugSerial.printf(" \n");
#endif

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
      DebugSerial.printf("Flightmode changed to %d", flightmode);

    for (int i=0;i<16;i++) {
      rc_inputs[i]=sBus.Channel(i+1);
    }
  
}

void loop()
{
  if(Serial3.available())
  {
    char c = Serial3.read();
    uint8_t decodedMessage = NazaDecoder.decode(c);
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        DebugSerial.print("Lat: "); Serial.print(NazaDecoder.getLat(), 7);
        DebugSerial.print(", Lon: "); Serial.print(NazaDecoder.getLon(), 7);
        DebugSerial.print(", Alt: "); Serial.print(NazaDecoder.getGpsAlt(), 7);
        DebugSerial.print(", Fix: "); Serial.print(NazaDecoder.getFixType());
        DebugSerial.print(", Sat: "); Serial.println(NazaDecoder.getNumSat());

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
        //DebugSerial.print("Heading: "); Serial.println(NazaDecoder.getHeadingNc(), 2);
        //    void setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
        
        break;
      case NAZA_MESSAGE_NONE:
        //ignore, no full message yet
        break;
    }
  }
  
/*
  auto chans =ppmSum.available();
  if (chans >=1) {
    for (int i=1;i<=chans;i++) {
      DebugSerial.printf("C%d: %.2f ", i, ppmSum.read(i));
    }
    DebugSerial.printf(" %d\n",chans);
  }
*/


  sBus.FeedLine();
  if (sBus.toChannels) {
    checkSbusState();
  }
  
  telemetry.send();
  sendMavlinkMessages();
 
}


