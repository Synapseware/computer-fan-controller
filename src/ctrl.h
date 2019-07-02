#ifndef __CTRL_H__
#define __CTRL_H__


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
#include "drivers/rtc_driver.h"


//--------------------------------------------------
// Board constants
#define ADC_CHANNEL         1
#define ADC_SMOOTHING       4

// ADC monitors
#define ADC_CH_CURRENT      ADC_CH_MUXPOS_PIN1_gc
#define ADC_CH_VOLTAGE      ADC_CH_MUXPOS_PIN2_gc
#define ADC_CH_TEMP         ADC_CH_MUXPOS_PIN3_gc

// PWM outputs
#define PWM_OUT_VOLT        PIN4_bm
#define PWM_OUT_PWM         PIN5_bm
#define PWM_CTRL_VOLT       CCA
#define PWM_CTRL_PWM        CCB

// PWM controls
#define PWM_CLK_SEL         TC_CLKSEL_DIV8_gc
//                          ((F_CPU / 8 / 25000)-1) = 159 (0-159 = 160)
#define PWM_PERIOD          160

// Use the 32.786kHz RC OSC divided by 1024 internally
#define MONITOR_CLOCK       CLK_RTCSRC_RCOSC_gc
#define MONITOR_PRESCALE	RTC_PRESCALER_DIV1_gc
#define MONITOR_PERIOD      (32768/1024/32)
#define MONITOR_ISR_LVL		RTC_OVFINTLVL_HI_gc

// Control schemes for PWM output states
#define SCHEME_NONE         0
#define SCHEME_PWM          1
#define SCHEME_POWER        2

// Monitoring modes
#define MONITOR_NONE        0
#define MONITOR_STARTUP     1
#define MONITOR_POWER       2
#define MONITOR_RPM_INIT    3


// Minimum current is 10mA
#define MIN_CURRENT         0.010



//-------------------------------------------------
// Functions
void ConfigureCore(void);
void ConfigureADC(void);
void ConfigureTachometer(void);
void ConfigurePWM(void);
void ConfigureMonitor(void);
void ControlPWM(uint8_t mode);
void setPwmOutput(uint16_t dutyCycle);
void setFanVoltage(float voltage);
uint8_t isFanConnected(void);
float getVoltage(void);
float getTemperature(void);
uint16_t getTachometer(void);
void ProcessMonitor(void);

#endif
