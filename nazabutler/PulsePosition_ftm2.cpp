/* PulsePosition Library for Teensy 3.1
 * High resolution input and output of PPM encoded signals
 * http://www.pjrc.com/teensy/td_libs_PulsePosition.html
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy
 * boards.  Please support PJRC's efforts to develop open source software by
 * purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "PulsePosition_ftm2.h"


// Timing parameters, in microseconds.


// The shortest time allowed between any 2 rising edges.  This should be at
// least double TX_PULSE_WIDTH.
#define TX_MINIMUM_SIGNAL   300.0

// The longest time allowed between any 2 rising edges for a normal signal.
#define TX_MAXIMUM_SIGNAL  2500.0

// The default signal to send if nothing has been written.
#define TX_DEFAULT_SIGNAL  1500.0

// When transmitting with a single pin, the minimum space signal that marks
// the end of a frame.  Single wire receivers recognize the end of a frame
// by looking for a gap longer than the maximum data size.  When viewing the
// waveform on an oscilloscope, set the trigger "holdoff" time to slightly
// less than TX_MINIMUM_SPACE, for the most reliable display.  This parameter
// is not used when transmitting with 2 pins.
#define TX_MINIMUM_SPACE   5000.0

// The minimum total frame size.  Some servo motors or other devices may not
// work with pulses the repeat more often than 50 Hz.  To allow transmission
// as fast as possible, set this to the same as TX_MINIMUM_SIGNAL.
#define TX_MINIMUM_FRAME  20000.0

// The length of all transmitted pulses.  This must be longer than the worst
// case interrupt latency, which depends on how long any other library may
// disable interrupts.  This must also be no more than half TX_MINIMUM_SIGNAL.
// Most libraries disable interrupts for no more than a few microseconds.
// The OneWire library is a notable exception, so this may need to be lengthened
// if a library that imposes unusual interrupt latency is in use.
#define TX_PULSE_WIDTH      100.0

// When receiving, any time between rising edges longer than this will be
// treated as the end-of-frame marker.
#define RX_MINIMUM_SPACE   3500.0


// convert from microseconds to I/O clock ticks
#if defined(KINETISK)
#define CLOCKS_PER_MICROSECOND ((double)F_BUS / 1000000.0)
#elif defined(KINETISL)
#define CLOCKS_PER_MICROSECOND ((double)F_PLL / 2000000.0)
#endif
#define TX_MINIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MINIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MAXIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MAXIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_DEFAULT_SIGNAL_CLOCKS  (uint32_t)(TX_DEFAULT_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_SPACE_CLOCKS   (uint32_t)(TX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_FRAME_CLOCKS   (uint32_t)(TX_MINIMUM_FRAME * CLOCKS_PER_MICROSECOND)
#define TX_PULSE_WIDTH_CLOCKS     (uint32_t)(TX_PULSE_WIDTH * CLOCKS_PER_MICROSECOND)
#define RX_MINIMUM_SPACE_CLOCKS   (uint32_t)(RX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)


#define FTM2_SC_VALUE (FTM_SC_TOIE | FTM_SC_CLKS(1) | FTM_SC_PS(0))

#if defined(KINETISK)
#define CSC_CHANGE(reg, val)         ((reg)->csc = (val))
#define CSC_INTACK(reg, val)         ((reg)->csc = (val))
#define CSC_CHANGE_INTACK(reg, val)  ((reg)->csc = (val))
#define FRAME_PIN_SET()              *framePinReg = 1
#define FRAME_PIN_CLEAR()            *framePinReg = 0
#elif defined(KINETISL)
#define CSC_CHANGE(reg, val)         ({(reg)->csc = 0; while ((reg)->csc); (reg)->csc = (val);})
#define CSC_INTACK(reg, val)         ((reg)->csc = (val) | FTM_CSC_CHF)
#define CSC_CHANGE_INTACK(reg, val)  ({(reg)->csc = 0; while ((reg)->csc); (reg)->csc = (val) | FTM_CSC_CHF;})
#define FRAME_PIN_SET()              *(framePinReg + 4) = framePinMask
#define FRAME_PIN_CLEAR()            *(framePinReg + 8) = framePinMask
#endif


// FMT2_C0SC => 32
// FMT2_C1SC => 25

void ftm2_isr(void)
{
	if (FTM2_SC & 0x80) {
		#if defined(KINETISK)
		FTM2_SC = FTM2_SC_VALUE;
		#elif defined(KINETISL)
		FTM2_SC = FTM2_SC_VALUE | FTM_SC_TOF;
		#endif
		PulsePositionInput::overflow_count++;
		PulsePositionInput::overflow_inc = true;
	}

    
	// TODO: this could be efficient by reading FTM2_STATUS
	uint8_t maskin = PulsePositionInput::channelmask;
	if ((maskin & 0x01) && (FTM2_C0SC & 0x80)) PulsePositionInput::list[0]->isr();
	if ((maskin & 0x02) && (FTM2_C1SC & 0x80)) PulsePositionInput::list[1]->isr();

	PulsePositionInput::overflow_inc = false;
}

// some explanation regarding this C to C++ trickery can be found here:
// http://forum.pjrc.com/threads/25278-Low-Power-with-Event-based-software-architecture-brainstorm?p=43496&viewfull=1#post43496

uint16_t PulsePositionInput::overflow_count = 0;
bool PulsePositionInput::overflow_inc = false;
uint8_t PulsePositionInput::channelmask = 0;
PulsePositionInput * PulsePositionInput::list[8];

PulsePositionInput::PulsePositionInput(void)
{
  //cscEdge = 0b0100 0100;
  cscEdge = 0b1001100;
}

PulsePositionInput::PulsePositionInput(int polarity)
{
	cscEdge = (polarity == FALLING) ? 0b01001000 : 0b01000100;
}


bool PulsePositionInput::begin(uint8_t pin)
{
	uint32_t channel;
	volatile void *reg;

	if (FTM2_MOD != 0xFFFF || (FTM2_SC & 0x7F) != FTM2_SC_VALUE) {
		FTM2_SC = 0;
		FTM2_CNT = 0;
		FTM2_MOD = 0xFFFF;
		FTM2_SC = FTM2_SC_VALUE;
		#if defined(KINETISK)
		FTM2_MODE = 0;
		#endif
	}
	switch (pin) {
	  case  25: channel = 1; reg = &FTM2_C1SC; break;
	  case  32: channel = 0; reg = &FTM2_C0SC; break;
	  default:
		return false;
	}
	prev = 0;
	write_index = 255;
	available_flag = false;
	ftm = (struct ftm_channel_struct *)reg;
	list[channel] = this;
	channelmask |= (1<<channel);
	*portConfigRegister(pin) = PORT_PCR_MUX(3);
	CSC_CHANGE(ftm, cscEdge); // input capture & interrupt on rising edge
	NVIC_SET_PRIORITY(IRQ_FTM2, 32);
	NVIC_ENABLE_IRQ(IRQ_FTM2);
	return true;
}

void PulsePositionInput::isr(void)
{
	uint32_t val, count;

	val = ftm->cv;
	CSC_INTACK(ftm, cscEdge); // input capture & interrupt on rising edge
	count = overflow_count;
	if (val > 0xE000 && overflow_inc) count--;
	val |= (count << 16);
	count = val - prev;
	prev = val;
#if 0
     Serial.print(val, HEX);
     Serial.print("  ");
     Serial.print(count);
    Serial.print("/");
     Serial.println(RX_MINIMUM_SPACE_CLOCKS);
#endif
     
     if ( count >= RX_MINIMUM_SPACE_CLOCKS) {
		if (write_index < 255) {
			for (int i=0; i < write_index; i++) {
				pulse_buffer[i] = pulse_width[i];
			}
			total_channels = write_index;
			available_flag = true;
		}
		write_index = 0;
	} else {
		if (write_index < PULSEPOSITION_MAXCHANNELS) {
			pulse_width[write_index++] = count;
		}
	}
}

int PulsePositionInput::available(void)
{
	uint32_t total;
	bool flag;

	__disable_irq();
	flag = available_flag;
	total = total_channels;
	__enable_irq();
	if (flag) return total;
	return -1;
}

float PulsePositionInput::read(uint8_t channel)
{
	uint32_t total, index, value=0;

	if (channel == 0) return 0.0;
	index = channel - 1;
	__disable_irq();
	total = total_channels;
	if (index < total) value = pulse_buffer[index];
	if (channel >= total) available_flag = false;
	__enable_irq();
	return (float)value / (float)CLOCKS_PER_MICROSECOND;
}

