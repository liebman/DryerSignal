//
// I use a nano for debugging...
//
#if defined(ARDUINO_AVR_NANO)
#define DEBUG
#endif

#include "DryerSignal.h"

void (*timer_cb)();
volatile bool timer_running;
volatile unsigned int start_time;
volatile unsigned int pwm_duration;     // PWM cycle count down.
volatile unsigned int state_count;
unsigned int saved_count;

int state;

ISR(WDT_vect)
{
#if defined(LED_PIN)
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif

    state_count += 1; // time in current state

    // ReEnable the watchdog interrupt, as this gets reset when entering this ISR and
    // automatically enables the WDE signal that resets the MCU the next time the
    // timer overflows
#if defined(__AVR_ATtinyX5__)
    WDTCR |= bit (WDIE);
#else
    WDTCSR |= bit (WDIE);
#endif
}

void clearTimer()
{
#if defined(__AVR_ATtinyX5__)
    TCCR1 = 0;
    GTCCR = 0;
    TIMSK &= ~(_BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1A));
#else
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
#endif
    OCR1A  = 0;
    OCR1B  = 0;
}

ISR(TIMER1_OVF_vect)
{
    if (pwm_duration == 0)
    {
        return;
    }

    pwm_duration -= 1;
    if (pwm_duration == 0)
    {
        clearTimer();
        timer_running = false;
        if (timer_cb != NULL)
        {
            timer_cb();
        }
    }
}

ISR(TIMER1_COMPA_vect)
{
    unsigned int int_time = millis();

    // why to we get an immediate interrupt? (ignore it)
    if (int_time == start_time || int_time == start_time+1)
    {
        return;
    }

#if defined(__AVR_ATtinyX5__)
    TIMSK &= ~(1 << OCIE1A); // disable timer1 interrupts as we only want this one.
#else
    TIMSK1 &= ~(1 << OCIE1A); // disable timer1 interrupts as we only want this one.
#endif
    timer_running = false;
    if (timer_cb != NULL)
    {
        timer_cb();
    }
}

void startTimer(unsigned int ms, void (*func)())
{
    start_time = millis();
    uint16_t timer = ms2Timer(ms);
    // initialize timer1
    noInterrupts();
    // disable all interrupts
    timer_cb = func;
#if defined(__AVR_ATtinyX5__)
    TCCR1 = 0;
    TCNT1 = 0;

    OCR1A = timer;   // compare match register
    TCCR1 |= (1 << CTC1);// CTC mode
    TCCR1 |= PRESCALE_BITS;
    TIMSK |= (1 << OCIE1A);// enable timer compare interrupt
    // clear any already pending interrupt?  does not work :-(
    TIFR &= ~(1 << OCIE1A);
#else
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = timer;   // compare match register
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= PRESCALE_BITS;
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    // clear any already pending interrupt?  does not work :-(
    TIFR1 &= ~(1 << OCIE1A);
#endif
    timer_running = true;
    interrupts();
    // enable all interrupts
}

void startPWM(unsigned int duration, unsigned int top, void (*func)())
{
    noInterrupts();
    timer_cb = func;

    clearTimer();

#if defined(__AVR_ATtinyX5__)
    OCR1C = top-1;
#else
    ICR1  = top-1;
#endif

    OCR1B = 50*top/100;
    TCNT1 = 0; // needed???
#if defined(__AVR_ATtinyX5__)
    TCCR1 = PWM_PRESCALE_BITS;
    GTCCR = _BV(COM1B1) | _BV(PWM1B);
#else
    TCCR1A =  _BV(COM1B1) | _BV(WGM10);
#endif

#if defined(__AVR_ATtinyX5__)
    TIMSK |= _BV(TOIE1);
#else
    TCCR1B = _BV(WGM12) | PWM_PRESCALE_BITS;
    TIMSK1 = _BV(TOIE1);
#endif

    pwm_duration = F_CPU / PWM_PRESCALE / top / (1000.0 / (double)duration);
    timer_running = true;
    interrupts();
}

void alertTone();

volatile bool         alert_active;
volatile unsigned int alert_top;
volatile unsigned int alert_duration;
volatile unsigned int alert_delay;
volatile          int alert_count;

void alertDelay()
{
    alert_count -= 1;
    if (alert_count < 1)
    {
        alert_count  = 0;
        alert_active = false;
        return;
    }

    startTimer(alert_delay, &alertTone);
}

void alertTone()
{
    startPWM(alert_duration, alert_top, &alertDelay);
}

void startAlert(unsigned int top, unsigned int duration, unsigned int delay, unsigned int count)
{
    dbprintf("startAlert top:%d duration:%d delay:%d count:%d\n", top, duration, delay, count);
    alert_active   = true;
    alert_top      = top;
    alert_duration = duration;
    alert_delay    = delay;
    alert_count    = count;
    alertTone();
}

void power_down()
{
    // disable ADC
    ADCSRA &= ~bit(ADEN);

    if (timer_running || alert_active)
    {
        set_sleep_mode(SLEEP_MODE_IDLE);
    }
    else
    {
        dbprintf("power_down using PWR_DOWN!\n");
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }

    dbflush();
    // sleep!
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    // enable ADC
    ADCSRA |= bit(ADEN);
}

void setState(int new_state, unsigned int new_count)
{
    dbprintf("setState: %d count: %u\n", new_state, new_count);
    state       = new_state;
    state_count = new_count;
}

void setup()
{
    dbbegin(115200);
    dbprintln("Setup!");

#if defined(LED_PIN)
    digitalWrite(LED_PIN, HIGH);
    pinMode(LED_PIN, OUTPUT);
#endif

    digitalWrite(TONE_PIN, 0);
    pinMode(TONE_PIN, OUTPUT);

    setState(STATE_IDLE, 0);

    // clear various "reset" flags
    MCUSR = 0;
#if defined(__AVR_ATtinyX5__)
    // allow changes, disable reset
    WDTCR = bit (WDCE) | bit (WDE);
    // set interrupt mode and an interval
    WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 second delay
#else
    // allow changes, disable reset
    WDTCSR = bit (WDCE) | bit (WDE);
    // set interrupt mode and an interval
    WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 second delay
#endif

    startAlert(START_TONE, START_DURATION, 25, 3);
}

int last_count;
void loop()
{
    power_down();

    //
    // don't run the logic if alert or timer running!
    //
    if (timer_running || alert_active)
    {
        if (last_count != alert_count)
        {
            dbprintf("alert_count: %d\n", alert_count);
            last_count = alert_count;
        }
        return;
    }

    int sample = analogRead(ANALOG_PIN);
    dbprintf("state: %d state_count: %d sample: %d\n", state, state_count, sample);

    switch(state)
    {
        case STATE_IDLE:
            if (sample >= SAMPLE_THRESHOLD)
            {
                setState(STATE_ARM, 0);
            }
            state_count = 0;
            break;
        case STATE_ARM:
            if (state_count >= COUNT_TO_ARMED)
            {
                setState(STATE_ARMED, 0);
            }
            else if (sample < SAMPLE_THRESHOLD)
            {
                saved_count = state_count;
                setState(STATE_DISARM, 0);
            }
            break;
        case STATE_DISARM:
            if (state_count >= MISS_TO_IDLE)
            {
                setState(STATE_IDLE, 0);
            }
            else if (sample >= SAMPLE_THRESHOLD)
            {
                setState(STATE_ARM, saved_count);
            }
            break;
        case STATE_ARMED:
            if (sample >= SAMPLE_THRESHOLD)
            {
                state_count = 0;
            }
            else if (state_count >= COUNT_TO_ALARM)
            {
                setState(STATE_ALARM, 0);
            }
            break;
        case STATE_ALARM:
            startAlert(ALARM_TONE, TONE_ON_MS, TONE_OFF_MS, ALARM_MAX_SECONDS);
            setState(STATE_IDLE, 0);
            break;
        default:
            dbprintf("unknown state: %d", state);
            state = STATE_IDLE;
    }
}
