#ifndef __MAIN_H__
#define __MAIN_H__


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>


//--------------------------------------------------
// Board constants
#define ADC_CHANNEL     1
#define ADC_SMOOTHING   4

// ADC monitors
#define ADC_CURRENT     0
#define ADC_VOLTAGE     1

#define ADC_OVER_LIMIT  975


// PWM outputs
#define PWM_OUT_VOLT    OC0A
#define PWM_OUT_FAN     OC0B

#define TIMER2_DELAY    125

// Control schemes
#define SCHEME_NONE     0
#define SCHEME_PROP     2
#define SCHEME_VOLT     3
#define SCHEME_PWM      4

// Monitoring modes
#define MONITOR_NONE    0
#define MONITOR_AMPS    1
#define MONITOR_VOLTS   2
#define MONITOR_ERROR   255


// Miscellaneous
#define LED_DBG         PB0

#define SPI_CLK         PB5
#define SPI_DI          PB4
#define SPI_DO          PB3
#define SPI_CS          PB2
#define SPI_DDR         DDRB
#define SPI_PORT        PORTB


//-------------------------------------------------


#endif
