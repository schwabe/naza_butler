# Naza Butler project #

A small project that uses a Teensy 3.2/3.1 as mediator between

- Frsky S.Port (X4R/X6R/X8R)
- Minim OSD
- Naza GPS module
- SBus (also X4R/X6R/X8R)

The setup is currently mostly hardcoded to my own F450 quadrocopter.

Of some interest for other project might the PulsePosition_ftm2.h/cpp files that
implement a PWM/PPM decoder for a servo signal on the FTM2 timer. The FTM0 in this
project is already used by the AltSerial port.

The project originally started since the [BagaOSD](https://code.google.com/p/bagaosd/) did not support SBus and S.Port, which use inverted Serial signals.

The project contains code from Pawelsky (Naza GPS and FrSky telemetry). 


