#include <ADC.h>
#include "config.h"

ADC *adc = new ADC();

static uint16_t receiver_rssi;
uint16_t battery_voltage=3412;
static unsigned int num_lipo_cells=0;
static int internal_temp;


void setupAnalogSensors()
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // Temperature Input
  pinMode(38, INPUT);

  adc->setAveraging(5); // set number of averages
  adc->setResolution(12); // set bits of resolution
  


  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADC_ADACK_2_4, ADC_ADACK_4_0, ADC_ADACK_5_2 and ADC_ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->setConversionSpeed(ADC_LOW_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_LOW_SPEED); // change the sampling speed
  
}

void  guess_cell_number() {
  for (int i=1;i<=6;i++) {
    auto single_cell = battery_voltage/i;
    if (single_cell > 3300 && single_cell < 4300) {
      num_lipo_cells = i;
      return;
    }
  }
}



float getReceiverRSSI()
{
  // /4096*100
  return receiver_rssi/40.96f;
}

 void readAnalogSensors()
 {
   uint32_t a0 = adc->analogRead(A0);
   receiver_rssi = adc->analogRead(A1);

   auto temp = adc->analogRead(38);
   if (temp != internal_temp)
     DebugSerial.printf("Temp %d, rssi %d\n", temp, receiver_rssi);
   
   internal_temp = temp;
   
   // 1,65544
   battery_voltage= a0* 28000/4635;
   if (num_lipo_cells == 0 && millis() > 1000){
     guess_cell_number();
   }
   
   

 }

  

int estimatepower(){
	int remaining=5;
    
    auto single_cell = battery_voltage/num_lipo_cells;
    if(single_cell>3350)remaining=10;
	if(single_cell>3430)remaining=20;
	if(single_cell>3500)remaining=30;
	if(single_cell>3570)remaining=40;
	if(single_cell>3630)remaining=50;
	if(single_cell>3700)remaining=60;
	if(single_cell>3770)remaining=70;
	if(single_cell>3830)remaining=80;
	if(single_cell>3900)remaining=90;
	if(single_cell>4050)remaining=100;
	return remaining;
}
