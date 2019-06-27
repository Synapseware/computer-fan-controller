computer-fan-controller
=======================

This project is used to control a 3 or 4 pin computer case fan.  It supports 3 schemes:

- 4 pin PWM control with rotation sensing
- 3 pin voltage control with rotation sensing
- 2 pin temperature proportional control

_Note: All modes use a temperature sensor to control fan speed_

I2C or SPI bus control

# Theory
Upon startup, the controller circuit sets the fan voltage to maximum and monitors the tachometer input.  Once the signal is stable (1.5 to 2.0 seconds later), it will attempt to use the 4 pin PWM control signal by ramping the fan down to 50%.  If the tachometer shows a proportional slowdown, the controller assumes 4 pin mode.  If no speed change, then the fan assumes 3 pin mode and drops the voltage to 8v, and again monitors the tachometer.  If a signal is detected, the controller will gradually lower the voltage to the fan so it can discover the minimum working voltage.  If the controller detects no signal from the tachometer, it assumes a 2 pin fan and will monitor the current draw to detect fan rotation.

# Control schemes
This is a rough overview of the 3 control schemes.

## 4 pin PWM control
The 4 pin PWM control uses the following scheme:
- Maximum voltage (100% duty cycle)
- PWM control signal
- Tachometer monitoring

## 3 pin voltage control
The 3 pin control is voltage based
- Variable voltage via a DC-DC buck converter
- Tachometer monitoring

## 2 pin voltage control
- Variable voltage via a DC-DC buck converter
- Current monitoring for stall detection

## Automatic mode monitoring
The controller employs a monitoring scheme that can detect when a fan is disconnected or replaced with a similar or different type, requiring a change in control schemes

# Hardware Modules

## ADC
3 channels are needed on the ADC, one for reading the fan voltage, and one for reading the fan current draw.

## Timers
3 timers are needed for the various PWM and monitoring loops.

- TCD5 is used to generate a 25kHz PWM signal for the DC-DC converter, and for the PWM output for 4-pin PWM control mode
- TCC5 is used to monitor the tachometer signal and count rotation events
- TCC4 is used to drive the system monitoring event loop

# Hardware I/O
## ADC
The ADC resides on Port A:

- PA0 - Aref input - should be tied to the mcu's supply rail and decoupled
- PA1 - Current sensor
- PA2 - Voltage sensor
- PA3 - Temperature sensor

## PWM Timer
The timer/counter TCD5, resides on Port D:

- PD4 - PWM output for CCA register
- PD5 - PWM output for CCB register
