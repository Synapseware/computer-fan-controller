#include "main.h"
#include "adc.h"


// Globals
volatile uint16_t   _fanSpeed       = 0;
volatile uint8_t    _fanCurrent     = 0;
volatile uint8_t    _fanRawCurrent  = 0;
volatile uint8_t    _fanVoltage     = 0;
volatile uint8_t    _fanRawVoltage  = 0;
volatile uint8_t    _controlScheme  = SCHEME_NONE;
volatile uint8_t    _monitorMode    = MONITOR_NONE;
volatile uint8_t    _monitorTick    = 0;
volatile uint8_t    _discardADC     = 1;


// ----------------------------------------------------------------------------
// Initializes the ADC to read the temperature sensor and
// the current sensor used to detect fan stall
void initSensors()
{
    // The ADC is 10-bit (0-1023).  The reference voltage will be set
    // to , and the raw ADC transfer function is:
    //        Vin * 1024
    // ADC = ------------
    //          Vref
    // Vref = 5.0

    ADMUX   =   (0<<REFS1)  |   // Use internal supply, Vref disconnected
                (0<<REFS0)  |   // 
                (0<<ADLAR)  |   // Right-adjust result
                (0<<MUX3)   |   // Start on channel 0
                (0<<MUX2)   |   // 
                (0<<MUX1)   |   // 
                (0<<MUX0);      // 

    ADCSRA  =   (1<<ADEN)   |   // Bit 7 – ADEN: ADC Enable
                (1<<ADSC)   |   // Bit 6 – ADSC: ADC Start Conversion
                (1<<ADATE)  |   // Bit 5 – ADATE: ADC Auto Trigger Enable
                (0<<ADIF)   |   // Bit 4 – ADIF: ADC Interrupt Flag
                (1<<ADIE)   |   // Bit 3 – ADIE: ADC Interrupt Enable
                (1<<ADPS2)  |   // ADC prescaler of 64 (8MHz/64=125kHz)
                (1<<ADPS1)  |   // 
                (0<<ADPS0);     // 

    ADCSRB  =   (0<<ACME)   |   // 
                (0<<ADTS2)  |   // 
                (0<<ADTS1)  |   // 
                (0<<ADTS0);     // 

    DIDR0   =   (0<<ADC5D)  |   // 
                (0<<ADC4D)  |   // 
                (0<<ADC3D)  |   // 
                (0<<ADC2D)  |   // 
                (0<<ADC1D)  |   // 
                (0<<ADC0D);     // 

    DDRC    =   (DDRC & 0xFC);  // Mask out the ADC0 and ADC1
}

// ----------------------------------------------------------------------------
// Initializes the tachometer monitor, which requires a timer.  Timer1 is
// used to count RPS from the fan.
void initTachometer()
{
    TCCR1A  =   (0<<COM1A1) |
                (0<<COM1A0) |
                (0<<COM1B1) |
                (0<<COM1B0) |
                (0<<WGM11)  |
                (0<<WGM10);

    TCCR1B  =   (0<<ICNC1)  |
                (0<<ICES1)  |
                (0<<WGM13)  |
                (0<<WGM12)  |
                (0<<CS12)   |
                (0<<CS11)   |
                (0<<CS10);

    TCCR1C  =   (0<<FOC1A)  |
                (0<<FOC1B);


    TIMSK1  =   (0<<ICIE1)  |
                (0<<OCIE1B) |
                (0<<OCIE1A) |
                (0<<TOIE1);

    OCR1A   =   0xffff;
    OCR1B   =   0xffff;
}

// ----------------------------------------------------------------------------
// Initializes the 2 PWM control signals, which are driven by Timer0
void initPWM()
{
    TCCR0A  =   (0<<COM0A1) |
                (0<<COM0A0) |
                (0<<COM0B1) |
                (0<<COM0B0) |
                (0<<WGM01)  |
                (0<<WGM00);

    TCCR0B  =   (0<<FOC0A)  |
                (0<<FOC0B)  |
                (0<<WGM02)  |
                (0<<CS02)   |
                (0<<CS01)   |
                (0<<CS00);

    TIMSK0  =   (0<<OCIE0B) |
                (0<<OCIE0A) |
                (0<<TOIE0);

    OCR0A   =   0xFF;
    OCR0B   =   0xFF;    
}

// ----------------------------------------------------------------------------
// Initializes the hardware monitor, which uses Timer2.  Timer2 ISR
// is handled in assembly for low overhead
void initMonitor()
{
    // Timer2 is setup to prescale by 8MHz / 256 / 250 / 125 for 1
    // second event loops.
    // The ISR is handled by assembly language for minimum overhead.  The ISR
    // toggles a global variable, _monitorTick, which when set to 1 initiates
    // a control loop.

    TCCR2A  =   (0<<COM2A1) |   // Disconnect OC0A
                (0<<COM2A0) |   // 
                (0<<COM2B1) |   // Disconnect OC0B
                (0<<COM2B0) |   // 
                (0<<WGM21)  |   // CTC 010
                (1<<WGM20);     // CTC 010

    TCCR2B  =   (0<<FOC2A)  |   // No force
                (0<<FOC2B)  |   // 
                (0<<WGM22)  |   // CTC 010
                (1<<CS22)   |   // CLK/256
                (1<<CS21)   |   // 
                (0<<CS20);      // 

    TIMSK2  =   (0<<OCIE2B) |   // 
                (1<<OCIE2A) |   // Enable ISR on compare A
                (0<<TOIE2);     // 

    OCR2A   =   250;            // 8MHz / 256 / 250 = 125 Hz
    OCR2B   =   0xFF;
}

// ----------------------------------------------------------------------------
// initialize all the hardware on the system
void init()
{
    initSensors();

    initTachometer();

    initPWM();

    initMonitor();
}

// ----------------------------------------------------------------------------
// Sets the DC-DC PWM control signal
void controlPWM(uint8_t mode)
{

}

// ----------------------------------------------------------------------------
// Returns the fan current in amps
float getCurrent()
{
    // The transfer function from the OpAmp output is:
    // Vin/Rin = (Vout-Vin)/Rf ... or ...
    // Av = 1 + (Rf/Rin)
    // Which in this case, is 18.447
    // Imax = sqrt(P/R) = sqrt(0.5/.13) ~= 1.96
    // Vmax = sqrt(P/R)(R) = sqrt(0.5/.13)(.13) ~= 255mV
    // Vout = (Vmax)(Av) = (0.255)(18.447) ~= 4.704
    // ADC = (Vin)(1024)/Vref => (4.704)(1024)/5.0 ~= 963
    // ADC factor = 963/1.96 ~= 491
    // ADC xfer function:
    //  Ifan = ADC / 491.3
    //       = 963 / 491.3
    //       = 1.96
    // ADC = (Ifan)(491)
    // Ifan = ADC/491 in amps
    // Result = Ifan * 100
    // So, at Imax, return value would be:
    // ADC/491*100 = 196
    // To get better integer math, just do:
    // ADC * 100 / 491
    // 963 * 100 / 491 = 196 (integer result)

    // Map the ADC sample to the fans' current
    return (float) _fanRawCurrent / 491.0;
}

// ----------------------------------------------------------------------------
// Returns the fan voltage in volts
float getVoltage()
{
    // the voltage transfer function is:
    // Vout = Vin(R2/(R1+R2))
    // Vout = (12)(5.6k)/(10k+5.6k)
    //      = 4.307
    // ADC  = (Vin)(1024)(Vref)
    //      = (4.307)(1024)/(5.0)
    //      = 882
    // ADC factor = 882/4.307 ~= 205
    // Resistor divider factor = Vin/Vout
    //      = 12/4.307
    //      = 2.786
    // Vout = ADC/ADCf*12/4.307

    // Map the ADC sample to the fans' voltage
    return (float) _fanRawVoltage / 205.0 * 2.768;
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


    }
}

// ----------------------------------------------------------------------------
// ADC sample complete interrupt handler
// Automatically switches the ADC channel
// after sampling, and discards bad results
ISR(ADC_vect)
{
    if (_discardADC)
    {
        _discardADC = 0;
        return;
    }

    switch (ADMUX)
    {
        case ADC_CURRENT:
            _fanRawCurrent = ADC;
            _discardADC = 1;
            ConfigureADCChannel(ADC_VOLTAGE);
            break;
        case ADC_VOLTAGE:
            _fanRawVoltage = ADC;
            _discardADC = 1;
            ConfigureADCChannel(ADC_CURRENT);
            break;
    }
}

// ----------------------------------------------------------------------------
// Timer1 Compare A interrupt handler
ISR(TIMER1_COMPA_vect)
{

}
