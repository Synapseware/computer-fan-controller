#include "ctrl.h"


// Globals
volatile uint16_t   _fanRawCurrent  = 0;
volatile uint16_t   _fanRawVoltage  = 0;
volatile uint16_t   _tmpRawValue    = 0;
volatile uint16_t   _tachRawValue   = 0;
volatile uint8_t    _monitorTick    = 0;
volatile uint8_t    _discardADC     = 1;


// -----------------------------------------------------------------------------
// Configures the MCU flags
void ConfigureCore(void)
{
    // start the internal 32MHz oscillator
    XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);

    // set CPU clock to 32MHz RC osc.
    XMEGACLK_SetCPUClockSource(CLOCK_SRC_INT_RC32MHZ);

	// Enable low, med, and high level interrupts in the PMIC
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

// ----------------------------------------------------------------------------
// Initializes the ADC to read on 2 channels
void ConfigureADC(void)
{
    // The ADC is used to sample the DC-DC voltage, and the voltage across
    // the current sense resistor for the fan.
    // Pin A1 is for voltage
    // Pin A2 is for current

    // get the ADCA calibration data from the NVM controller
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    XMEGACLK_CCP_Write(&NVM.CTRLA, NVM_CMDEX_bm);
    ADCA.CAL = pgm_read_word(PRODSIGNATURES_ADCACAL0);
    //ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
    NVM.CMD = 0;

    // See "Alternate Pin Functions" in the datasheet around page 59

    PORTA.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;

    // disable the ADC
    ADCA.CTRLA = 0;

    // Configure for Freerunning, 12-bit
    ADCA.CTRLB = ADC_CURRLIMIT_NO_gc |
                 ADC_RESOLUTION_12BIT_gc |
                 ADC_FREERUN_bm;

    // Configure with external Vref @ PORT A0
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;

    // Don't use events
    ADCA.EVCTRL = 0;

    // Configure prescaler
    ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;

    // Configure ADC for 1x gain and single-ended input
    ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc |
                    ADC_CH_INPUTMODE_SINGLEENDED_gc;

    // Select the ADC channel
    ADCA.CH0.MUXCTRL = ADC_CH_TEMP;

    // Enable interrupts on completed conversions
    ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc |
                       ADC_CH_INTLVL_LO_gc;

    // Enable and start the ADC
    ADCA.CTRLA = ADC_START_bm | ADC_ENABLE_bm;
}

// ----------------------------------------------------------------------------
// Initializes the tachometer monitor, which requires a timer.  TCC5 is
// used to count RPS from the fan.
void ConfigureTachometer(void)
{
	// Initialize a timer to count fan rotation signals for a second.
	// Fans that have a speed sensor need to have the rotation signal
	// divided by 2, since there are 2 pulses per rotation
    // There are 3 timers:
    //  TCC4, TCC5, TCD5
    // We'll use TCC5 to capture the tachometer signal

    // Need to configure TCC5 to count external events, then watch TCC5 for a
    // set/known period of time to get an estimate of the fan's RPM

    // Setup event system to trigger on edge change for pin C5
    // Capture on pin PC5
    PORTC.DIRCLR = PIN5_bm;
    PORTC.PIN5CTRL = PORT_ISC_RISING_gc | PORT_OPC_PULLUP_gc;
    EVSYS.CH5MUX = EVSYS_CHMUX_PORTC_PIN5_gc;

    // Reset timer
    TCC5.CTRLA = 0;
    TCC5.CTRLB = 0;
    TCC5.CTRLC = 0;

    // Configure TCC5 for event counting...

    // Configure for Input Capture for external control via an event channel
    TCC5.CTRLD = TC_EVACT_UPDOWN_gc | TC_EVSEL_CH0_gc;

    // Enable input capture
    TCC5.CTRLE = TC_CCAMODE_CAPT_gc;

    // Clock from event channel 0
    TCC5.CTRLA = TC_CLKSEL_EVCH0_gc;
}

// ----------------------------------------------------------------------------
// Initializes the 2 PWM control signals, which are driven by TCD5
void ConfigurePWM(void)
{
	// Initialize a PWM timer for 25kHz and output on different ports.
	// One PWM port will be used to control the DC-DC convert, while
	// the other is used to control a 4-pin PWM fan

    // Disable the timer
    TCD5.CTRLA = 0;

    // Single-slope PWM
    TCD5.CTRLB = TC_WGMODE_SINGLESLOPE_gc;

    TCD5.CTRLC = 0;
    TCD5.CTRLD = 0;

    // Enable capture/compare mode output
    TCD5.CTRLE = TC_CCAMODE_COMP_gc | TC_CCBMODE_COMP_gc;

    TCD5.CTRLA = PWM_CLK_SEL;
    TCD5.PER = PWM_PERIOD-1;

    // Set the control scheme to none.
    ControlPWM(SCHEME_NONE);
}

// ----------------------------------------------------------------------------
// Initializes the hardware monitor, which uses the RTC
void ConfigureMonitor(void)
{
    // enable the 32kHz RC clock
    XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32KHZ);
    CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
    while((RTC.STATUS & RTC_SYNCBUSY_bm));

    RTC_Initialize(1024, 0, 20, RTC_PRESCALER_DIV1_gc);
    RTC_SetIntLevels(RTC_OVFINTLVL_HI_gc, RTC_COMPINTLVL_LO_gc);
}

// ----------------------------------------------------------------------------
// initialize all the hardware on the system
void init(void)
{
    cli();

	// configure CPU core
	ConfigureCore();

    // Initialize PWM drivers
    ConfigurePWM();

    // configures the ADC module
    ConfigureADC();

    // Configure tachometer capturing timer
    //ConfigureTachometer();

	// Initialize fan monitoring
    ConfigureMonitor();

    sei();
}

// ----------------------------------------------------------------------------
// Sets the initial control states
void ControlPWM(uint8_t mode)
{
	// CCA is connected to the voltage converter
	// CCB is connected to the PWM output for 4-pin mode
	switch (mode)
	{
		case SCHEME_NONE:
			// Disable all PWM output pins (high-z)
            PORTD.DIRCLR = PWM_OUT_VOLT | PWM_OUT_PWM;
            TCD5.PWM_CTRL_VOLT = 0;        // PIN4
            TCD5.PWM_CTRL_PWM = 0;         // PIN5
			break;
		case SCHEME_PWM:
            // 4-pin PWM control:
            // Max DC-DC voltage
            // PWM output
            // Start fan PWM signal at 50%
            PORTD.DIRCLR = PWM_OUT_VOLT;    // PIN4
            PORTD.DIRSET = PWM_OUT_PWM;     // PIN5
            TCD5.PWM_CTRL_VOLT = PWM_PERIOD-1;
            TCD5.PWM_CTRL_PWM = PWM_PERIOD-1;
            break;
		case SCHEME_POWER:
            // Current and voltage have the same PWM control scheme:
            // Voltage PWM is used to control fan speed, while
            // fan PWM output is disabled (high-Z)
            PORTD.DIRSET = PWM_OUT_VOLT;    // PIN4
            PORTD.DIRCLR = PWM_OUT_PWM;     // PIN5
            TCD5.PWM_CTRL_VOLT = PWM_PERIOD-1;
            TCD5.PWM_CTRL_PWM = 0;
			break;
    }
}

// ----------------------------------------------------------------------------
// Sets the duty cycle on the PWM output signal for 4-pin PWM mode
void setPwmOutput(uint16_t dutyCycle)
{
    // 
    TCD5.PWM_CTRL_PWM = dutyCycle;
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

    TCD5.PWM_CTRL_VOLT = dutyCycle;
}

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
// Return the fan RPM
uint16_t getTachometer(void)
{
    // The raw tachometer value is derived from a 1 second sample frequency.
    // RPM's are estimated from this value. The tachometer output from the fan
    // toggles twice per rotation, so we need to multiply the raw value by 60/2.
    return _tachRawValue * 30;
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
// Completes a fan monitoring pass, including 
void ProcessMonitor(void)
{
    static uint8_t    monitorMode       = MONITOR_NONE;
    static uint8_t	  delay				= 0;

    // Steps for monitoring a fan:
    // On start-up:
    // - Set voltage to max
    // - Check for current usage
    // 		if no current, check again
    // - Check for tachometer data
    // 		if no tachometer data, assume proportional temp + current monitor
    // - If tachometer, adjust PWM output by 50%
    // 		if tachometer decreases, assume 4-pin PWM control loop
    // - If no tachometer change, assume 3-pin DC voltage control
    // 		start reducing voltage by 0.5v steps and monitor tachometer to
    // 		establish fan lower-bounds of speed
    // While monitoring a fan, if the current load drops to 0, restart the
    // monitoring startup process

	// allow skips
	if (delay)
	{
		delay--;
		return;
	}

    float current = getCurrent();
    float voltage = getVoltage();
    float temperature = getTemperature();
    uint16_t rpms = getTachometer();

    mapTemperatureToPwm(58.0);

    return;

    switch (monitorMode)
	{
        // The initial monitoring state
		case MONITOR_NONE:
			ControlPWM(SCHEME_PWM);
            monitorMode = MONITOR_STARTUP;
			break;

        // Begins the fan observation process
        case MONITOR_STARTUP:
            // Check for RPM signal
            if (rpms < 10)
            {
                // assume no RPM signal is present, switch mode to proportional
                // control
                ControlPWM(SCHEME_POWER);
                monitorMode = MONITOR_POWER;
            }

            monitorMode = MONITOR_RPM_INIT;
            break;

        // Monitor the RPM signal and determine if 4-pin or 3-pin mode is required
        case MONITOR_RPM_INIT:
            break;

        // RPM signal not found, so assume proportial temperature control
        case MONITOR_POWER:
            // Check current usage - if 0, assume startup mode
            if (current < MIN_CURRENT)
            {
                // No power usage found, assume startup role
                monitorMode = MONITOR_NONE;
                break;
            }
            break;
	}
}

// ----------------------------------------------------------------------------
int main(void)
{
    PORTC.DIRSET = PIN2_bm;
    PORTC.OUTCLR = PIN2_bm;
    PORTD.DIRSET = PWM_OUT_PWM;

    init();

    /*
    while(1)
    {
        PORTD.OUTTGL = PWM_OUT_PWM;
    }
    */

    ControlPWM(SCHEME_PWM);

    PORTD.DIRSET = PWM_OUT_VOLT | PWM_OUT_PWM;

    TCD5.PWM_CTRL_VOLT = PWM_PERIOD * 0.1;
    TCD5.PWM_CTRL_PWM = PWM_PERIOD * 0.7;

    // wait here
    while(1)
    {
        if (!_monitorTick)
            continue;

        // looks like it's mapping 0.17 to 1.31 volts
        float temperature = getTemperature();
        uint16_t dutyCycle = mapTemperatureToPwm(temperature);
        setPwmOutput(dutyCycle);
    }

    //uint16_t dutyCycle = mapTemperatureToPwm(58.0);
    setPwmOutput(100);

    // wait for a monitor tick after init
    while (_monitorTick)
    {
        ControlPWM(SCHEME_PWM);
        break;
    }

    // main loop
    while(1)
    {
        if (!_monitorTick)
            continue;

        _monitorTick = 0;

        // do-monitoring
        ProcessMonitor();
    }
}

// ----------------------------------------------------------------------------
// ADC sample complete interrupt handler
// Automatically switches the ADC channel
// after sampling, and discards bad results
ISR(ADCA_CH0_vect)
{
    if (_discardADC)
    {
        _discardADC = 0;
        return;
    }

    switch (ADCA.CH0.MUXCTRL)
    {
        case ADC_CH_CURRENT:
            _fanRawCurrent = ADCA.CH0.RES;
            _discardADC = 1;
            ADCA.CH0.MUXCTRL = ADC_CH_VOLTAGE;
            break;
        case ADC_CH_VOLTAGE:
            _fanRawVoltage = ADCA.CH0.RES;
            _discardADC = 1;
            ADCA.CH0.MUXCTRL = ADC_CH_TEMP;
            break;
        case ADC_CH_TEMP:
            _tmpRawValue = ADCA.CH0.RES;
            _discardADC = 1;
            ADCA.CH0.MUXCTRL = ADC_CH_CURRENT;
            break;
    }
}

// ----------------------------------------------------------------------------
// RTC overflow interrupt handler for the Monitor
ISR(RTC_OVF_vect)
{
	// Flag a monitoring loopA
	_monitorTick = 1;

    //PORTD.OUTTGL = PWM_OUT_PWM;
    PORTC.OUTSET = PIN2_bm;

    // capture the raw counter value
    _tachRawValue = TCC5.CNT;

    // Restart the tachometer counter
    TCC5.CTRLGSET = TC_CMD_RESTART_gc;
}

ISR(RTC_COMP_vect)
{
    PORTC.OUTCLR = PIN2_bm;
}