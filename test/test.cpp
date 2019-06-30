#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fstream>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>


using namespace std;

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

short PWM_PERIOD = 160;
short _fanRawCurrent = 0;
short _fanRawVoltage = 0;
short _tmpRawValue = 0;

// ----------------------------------------------------------------------------
// Returns the fan current in amps
float getCurrent(void)
{
    // The max voltage from the resistor divider for the DC-DC power supply
    // is chosen to be close to 3.3v, which is the ADC's reference voltage.
    // That's why the getVoltage() and getCurrent() functions have similar
    // values for the ADC factors.
    // The transfer function from the OpAmp output is:
    // Vin/Rin = (Vout-Vin)/Rf ... or ...
    // Av = 1 + (Rf/Rin)
    // At 0.255V, the load resistor is at maximum power load.  At Pmax,
    // the current is as follows:
    // Imax = sqrt(P/R) = sqrt(0.5/.13) ~= 1.96
    // Vmax = sqrt(P/R)(R) = sqrt(0.5/.13)(.13) ~= 255mV
    // Vout = (Vmax)(Av) = (0.255)(12.4) ~= 3.161
    // ADC = (Vin)(4096)/Vref => (3.161)(4096)/3.3 ~= 3923
    // ADC factor = 3923/1.96 ~= 2001.5
    // ADC xfer function:
    //       = 3923 / 2001.5
    //       = 1.96
 
    // Map the ADC sample to the fans' current
    return (float) _fanRawCurrent / 2001.5;
}

// ----------------------------------------------------------------------------
// Returns the fan voltage in volts
float getVoltage(void)
{
    // The max voltage from the resistor divider for the DC-DC power supply
    // is chosen to be close to 3.3v, which is the ADC's reference voltage.
    // That's why the getVoltage() and getCurrent() functions have similar
    // values for the ADC factors.
    // The voltage transfer function is:
    // Vout = Vin(R2/(R1+R2))
    //      R1 = 9.1k
    //      R2 = 3.3k
    // Vout = (12)((3.3k)/(3.3k+9.1k))
    //      = 3.194
    // ADC  = (Vin)(4096)(Vref)
    //      = (3.194)(4096)/(3.3)
    //      = 3964
    // ADC factor = 3964/3.194 ~= 1241
    // Resistor divider factor = Vin/Vout
    //      = 12/3.194
    //      = 3.757
    // Vout = ADC/ADCf*12/3.194

    // Map the ADC sample to the fans' voltage
    return (float) _fanRawVoltage / 1241.0 * 3.757;
}

// ----------------------------------------------------------------------------
// Returns the temperature in degrees Celcius
float getTemperature(void)
{
    // Calibrated for a TMP35 temperature sensor, which is an analog temperature
    // sensor calibrated in degrees Celcius, with a range of -40°C to +125°C.
    // 250mV @ 25°C
    // 10°C ≤ T A ≤ 125°C, 10mV/°C
    // Using similar ADC input range as the other analog inputs, mapping the
    // temperature sensor through an op-amp with gain of about 2.47, 25C ~= 0.6175
    // Max temp is 125C, so (1.25)(2.47) = 3.0875
    // ADC  = (Vin)(4096)/Vref
    //      = (3.0875)(4096)/3.3
    //      = 3832
    // ADC factor = 3832/3.0875 ~= 1241.0
    // Xfer function
    // t/C  = ADC / ADC factor / gain * 100
    //      = 3832 / 1241.0 / 2.47 * 100
    //      = 3832 / 1241.0 / 247
    //      ~= 125
    return (float) _tmpRawValue / 1241.0 / 247.0 * 10000;
}

// ----------------------------------------------------------------------------
// Maps a temperature value, in °C, to a duty cycle value for the DC-DC supply
uint16_t mapTemperatureToPwm(float temperature)
{
    // 0-15 => 30%
    // 15-80 => 30% - 100%
    // 30%-100% => 48 - 160
    // y = mx + b
    // m = (y-y')/(x-x')
    // m = (100-30)/(70-15)
    // m ~= 1.3
    // y = 1.3x + b
    // 30 = (1.3)(15) + b
    // b = 10.5
    // y = 1.3x + 11

    // min temperature = 15/C
    // min fan speed = 30%
    // max temperature = 70/C
    // max fan speed = 100%

    float percentDutyCycle = temperature * 1.3 + 11;
    if (percentDutyCycle < 30.0)
        return 0.30 * PWM_PERIOD;

    if (percentDutyCycle > 100.0)
        return PWM_PERIOD - 1;

    // linear map
    return PWM_PERIOD * percentDutyCycle / 100;
}

// ----------------------------------------------------------------------------
// Sets the DC-DC voltage
void setFanVoltage(float voltage)
{
    // Ideally this would be done via a PID controller.  But for now, we'll
    // have to figure out some way to set a target voltage and then monitor
    // it for accuracy.  Different loads on the buck converter will result
    // in different final voltages, so no mapping exists between the PWM
    // duty cycle and the target voltage.
    // For simplicty, assume 12.0v = 100%, 6.0v = 50%
    uint16_t dutyCycle = 0;
    if (voltage > 1)
    {
        dutyCycle = (uint16_t) ((voltage / 12.0) * PWM_PERIOD)-1;
    }

    // capture max
    if (dutyCycle > PWM_PERIOD-1)
    {
        dutyCycle = PWM_PERIOD-1;
    }

    //TCD5.PWM_CTRL_VOLT = dutyCycle;
    cout << "Target voltage " << voltage << " yields duty cycle of " << dutyCycle << endl;
}

int main(int argc, char* argv[])
{
    /*
	cout << "Mapping 0/C to 125/C" << endl;

	for (int i = 0; i <= 125; i++)
	{
		short dutyCycle = mapTemperatureToPwm(i);
		cout << i << "/C => " << dutyCycle << " duty cycle" << endl;
	}

    while (1)
    {
        cout << "Enter ADC value for current sensor: ";
        cin >> _fanRawCurrent;
        if (!_fanRawCurrent)
            break;

        cout << "Current: " << getCurrent() << "Amps" << endl;
    }

    while (1)
    {
        cout << "Enter ADC value for voltage sensor: ";
        cin >> _fanRawVoltage;
        if (!_fanRawVoltage)
            break;

        cout << "Current: " << getVoltage() << "Volts" << endl;
    }

    while (1)
    {
        cout << "Enter ADC value for temperature: ";
        cin >> _tmpRawValue;
        if (!_tmpRawValue)
            break;

        cout << "Temperature: " << getTemperature() << "°C" << endl;
    }
    */

    float voltage = 0.0;
    while (1)
    {
        cout << "Enter DC-DC converter voltage for duty cycle mapping: ";
        cin >> voltage;

        setFanVoltage(voltage);
    }

	cout << "Done";
}

