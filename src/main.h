#ifndef __MAIN_H__
#define __MAIN_H__


// AVR basics
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>


#define false 0
#define true 1
#define bool uint8_t
#define uint_reg_t uint8_t


#include "board.h"
#include "drivers/ADC.h"
#include "drivers/ClockManagement.h"


//--------------------------------------------------
// Board constants
#define ADC_CHANNEL     1
#define ADC_SMOOTHING   4

// ADC monitors
#define ADC_CH_CURRENT  ADC_CH_MUXPOS_PIN1_gc
#define ADC_CH_VOLTAGE  ADC_CH_MUXPOS_PIN2_gc
#define ADC_CH_TEMP     ADC_CH_MUXPOS_PIN3_gc

#define ADC_OVER_LIMIT  975


// PWM outputs
#define PWM_OUT_VOLT    PIN4_bm
#define PWM_OUT_FAN     PIN5_bm

// PWM controls
#define PWM_PRESCALE    TC45_CLKSEL_DIV8_gc
//                      ((F_CPU / 8 / 25000)-1) = 159 (0-159 = 160)
#define PWM_PERIOD      159

// Control schemes
#define SCHEME_NONE     0
#define SCHEME_PWM      1
#define SCHEME_CURR     2
#define SCHEME_VOLT     3

// Monitoring modes
#define MONITOR_NONE    0
#define MONITOR_AMPS    1
#define MONITOR_VOLTS   2
#define MONITOR_ERROR   255





//-------------------------------------------------
// Functions
void ConfigureCore(void);
void ConfigureADC(void);
void ConfigureTachometer(void);
void ConfigurePWM(void);
void ConfigureMonitor(void);
void ControlPWM(uint8_t mode);
float getCurrent(void);
float getVoltage(void);
void ProcessMonitor(void);

#endif
