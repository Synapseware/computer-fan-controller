#include "ctrl.h"


// Globals
volatile uint16_t   _fanSpeed       = 0;
volatile uint16_t   _fanCurrent     = 0;
volatile uint16_t   _fanRawCurrent  = 0;
volatile uint16_t   _fanVoltage     = 0;
volatile uint16_t   _fanRawVoltage  = 0;
volatile uint16_t   _tempValue      = 0;
volatile uint16_t   _tempRawValue   = 0;
volatile uint8_t    _controlScheme  = SCHEME_NONE;
volatile uint8_t    _monitorMode    = MONITOR_NONE;
volatile uint8_t    _monitorTick    = 0;
volatile uint8_t    _discardADC     = 1;


// -----------------------------------------------------------------------------
// Configures the MCU flags
void ConfigureCore(void)
{
	// Start the PLL and configure for 32MHz CPU clock
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);

	// set the main clock source to the PLL
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	// Enable lov, med and high level interrupts in the PMIC
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

    PORTA.DIRCLR = PIN1_bm | PIN2_bm;

    // disable the ADC
    ADCA.CTRLA = 0;

    // Configure for Freerunning, 12-bit
    ADCA.CTRLB = ADC_CURRLIMIT_NO_gc |
                 ADC_RESOLUTION_12BIT_gc;

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
    ADCA.CH0.MUXCTRL = ADC_CH_CURRENT;

    // No interrupts (for now)
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
    TCD5.CTRLE = TC_CCAMODE_COMP_gc;

    TCD5.CTRLA = PWM_PRESCALE;
    TCD5.PER = PWM_PERIOD;

    // Set the control scheme to none.
    ControlPWM(SCHEME_NONE);
}

// ----------------------------------------------------------------------------
// Initializes the hardware monitor, which uses Timer2.  TCC4 ISR
// is handled in assembly for low overhead
void ConfigureMonitor(void)
{
    // TCxx is setup to prescale by ....
    // second event loops.
    // The ISR is handled by assembly language for minimum overhead.  The ISR
    // toggles a global variable, _monitorTick, which when set to 1 initiates
    // a control loop.
    // Configure the prescaler (32MHz / 1024 / 31250 = 1) and set the
    // compare A value to 1 for 1 second interrupts
    TCC4.CTRLA = TC_CLKSEL_DIV1024_gc;
    TCC4.PER = 31250-1;
    TCC4.CCA = 1;

    // Normal mode, 16 bit
    TCC4.CTRLB = TC_BYTEM_NORMAL_gc | TC_WGMODE_NORMAL_gc;

    // Normal
    TCC4.CTRLC = 0;

    // No event system connections
    TCC4.CTRLD = 0;

    // No compare/capture
    TCC4.CTRLE = 0;

    // Enable high priority interrupt on CCA
    TCC4.INTCTRLA = 0;
    TCC4.INTCTRLB = TC_CCAINTLVL_HI_gc;
}

// ----------------------------------------------------------------------------
// initialize all the hardware on the system
void init(void)
{
	// configure CPU core
	ConfigureCore();

    // configures the ADC module
    ConfigureADC();

    // Configure tachometer capturing timer
    ConfigureTachometer();

    // Initialize PWM drivers
    ConfigurePWM();

	// Initialize fan monitoring
    ConfigureMonitor();
}

// ----------------------------------------------------------------------------
// Sets the DC-DC PWM control signal
void ControlPWM(uint8_t mode)
{
	// There are 2 control schemes for the PWM signal:
	switch (mode)
	{
		case SCHEME_NONE:
			// Disable all PWM output pins (high-z)
            PORTD.DIRCLR = PWM_OUT_VOLT | PWM_OUT_FAN;
            TCD5.CCA = 0;
            TCD5.CCB = 0;
			break;
		case SCHEME_PWM:
            // 4-pin PWM control:
            // Max DC-DC voltage
            // PWM output
            // Start fan PWM signal at 50%
            PORTD.DIRSET = PWM_OUT_VOLT | PWM_OUT_FAN;
            TCD5.CCA = PWM_PERIOD;
            TCD5.CCB = PWM_PERIOD >> 2;
            break;
		case SCHEME_CURR:
        case SCHEME_VOLT:
            // Current and voltage have the same PWM control scheme:
            // Voltage PWM is used to control fan speed, while
            // fan PWM output is disabled (high-Z)
            PORTD.DIRSET = PWM_OUT_VOLT;
            PORTD.DIRCLR = PWM_OUT_FAN;
            TCD5.CCA = PWM_PERIOD;
            TCD5.CCB = 0;
			break;
    }
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
// Completes a fan monitoring pass, including 
void ProcessMonitor(void)
{
    // Implement awesome fan control sheme state engine here!  Yeeeah!
}

// ----------------------------------------------------------------------------
int main(void)
{
    init();

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
            _tempRawValue = ADCA.CH0.RES;
            _discardADC = 1;
            ADCA.CH0.MUXCTRL = ADC_CH_CURRENT;
            break;
    }
}

// ----------------------------------------------------------------------------
// Timer1 Compare A interrupt handler
ISR(TCC4_CCA_vect)
{
    // Flag a monitoring cycle
    _monitorTick = 1;

    // clear the interrupt flag when done
    TCC4.INTFLAGS = TC4_CCAIF_bm;
}
