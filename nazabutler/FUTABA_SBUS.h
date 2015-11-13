
#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <Arduino.h>


#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define BAUDRATE 100000
//------------------------------------
 // f�r Pro Micro,Mega und Leonardo andere Serial1 Serial2 Serial3 
//#define sbusSerial Serial // f�r Nano Micro Mini da nur 1 Serielle vorhanden 
//ATmega 32U4 hat interne USB an D+ D- als Serial, damit USART TX Rx frei als Serial1
//--------------------------------------
#define ALL_CHANNELS  // falls alle 16 Kan�le


class FUTABA_SBUS
{
	public:
		uint8_t sbusData[25];
		int16_t channels[18];
		int16_t servos[18];
		uint8_t  failsafe_status;
		int sbus_passthrough;
		int toChannels;
		void begin(void);
		int16_t Channel(uint8_t ch);
		uint8_t DigiChannel(uint8_t ch);
		void Servo(uint8_t ch, int16_t position);
		void DigiServo(uint8_t ch, uint8_t position);
		uint8_t Failsafe(void);
		void PassthroughSet(int mode);
		int PassthroughRet(void);
		void UpdateServos(void);
		void UpdateChannels(void);
		void FeedLine(void);
	private:
		uint8_t byte_in_sbus;
		uint8_t bit_in_sbus;
		uint8_t ch;
		uint8_t bit_in_channel;
		uint8_t bit_in_servo;
		uint8_t inBuffer[25];
		int bufferIndex;
		uint8_t inData;
		int feedState;

};

#endif
