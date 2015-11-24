#include <Arduino.h>
#include "config.h"

#include "ppm.h"


unsigned long pin11h = 0;

unsigned long pin12h = 0;

int roll_ppm;
int pitch_ppm;


static inline void do_ppm_len(uint16_t pin, unsigned long & lasth, int & retval) 
{
  auto now = micros();
  auto val = digitalRead(pin);
  if (val == HIGH) {
    lasth = now;
    return;
  } 
  
  auto ppm = now - lasth;
  if (false && (ppm < 400 || ppm > 4200))
    // ignore
    return;
  else
    retval = ppm;
}
    

static unsigned long lastisr=0;

static void ppm_isr()
{
  if (lastisr == micros())
    return;
  lastisr = micros();
  micros();

  do_ppm_len(11, pin11h, roll_ppm);
  do_ppm_len(12, pin12h, pitch_ppm);
  
  if (roll_ppm !=0 || pitch_ppm !=0)
    DebugSerial.printf("p11: %d p12: %d\n", roll_ppm, pitch_ppm);

}




void setupPPM()
{
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  attachInterrupt(11, ppm_isr, CHANGE);
  attachInterrupt(12, ppm_isr, CHANGE);
}



void loopPPM(){

  
}
