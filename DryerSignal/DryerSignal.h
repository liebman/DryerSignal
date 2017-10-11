#ifndef _DryerSignal_H_
#define _DryerSignal_H_

#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "Logger.h"

#define USE_PWM
//#define TEST_SOUND

#define SAMPLE_THRESHOLD  5      // if we read higher than this its on!

#define COUNT_TO_ARMED    10     // 10 seconds with current will arm
#define MISS_TO_IDLE      2      // miss 2 seconds of zero current while arming goes back to idle
#define COUNT_TO_ALARM    10     // 10 seconds of no current for alarm if armed
#define ALARM_MAX_SECONDS 120    // 2 minutes max alarm tone

#define START_TONE        143    // ~3500hz
#define START_DURATION    100

#define ALARM_TONE        167    // ~3000hz
#define TONE_ON_MS        500
#define TONE_OFF_MS       500


#define STATE_IDLE        0
#define STATE_ARM         1
#define STATE_DISARM      2
#define STATE_ARMED       3
#define STATE_ALARM       4

#if defined(__AVR_ATtinyX5__)
#define ANALOG_PIN        A3
#define TONE_PIN          4
//#define LED_PIN           1
#elif defined(ARDUINO_AVR_NANO)
#define ANALOG_PIN        A1
#define TONE_PIN          3
#define LED_PIN           LED_BUILTIN
#endif

#if defined(__AVR_ATtinyX5__)
#if F_CPU == 1000000L
#define PWM_PRESCALE      2
#define PWM_TOP           125 // 4khz
#define PWM_PRESCALE_BITS (_BV(CS11))
#elif F_CPU == 8000000L
#define PWM_PRESCALE      8
#define PWM_TOP           250 // 4khz
#define PWM_PRESCALE_BITS (_BV(CS12))
#endif
#else
#define PWM_TOP           250 // assuming 16mhz CPU its 8khz
#define PWM_PRESCALE      8
#define PWM_PRESCALE_BITS (_BV(CS11))
#endif

#ifdef __AVR_ATtinyX5__
#if F_CPU == 8000000L
#define PRESCALE        4096
#define PRESCALE_BITS   ((1 << CS13) | (1 << CS12) | (1 <<CS10)) // 4096 prescaler
#elif F_CPU == 4000000L
#define PRESCALE        2048
#define PRESCALE_BITS   ((1 << CS13) | (1 << CS12)) // 2048 prescaler
#elif F_CPU == 1000000L
#define PRESCALE        4096
#define PRESCALE_BITS   ((1 << CS13) | (1 << CS12) | (1 <<CS10)) // 4096 prescaler
#endif
#define ms2Timer(x) ((uint8_t)(F_CPU /(PRESCALE * (1/((double)x/1000)))))
#else
#define PRESCALE        256
#define PRESCALE_BITS   (1 << CS12) // 256 prescaler
#define ms2Timer(x) ((uint16_t)(F_CPU /(PRESCALE * (1/((double)x/1000)))))
#endif

#endif /* _DryerSignal_H_ */
