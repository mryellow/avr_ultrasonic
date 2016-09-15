#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

//#define F_CPU			8000000UL

#define SENSOR_NUM      2

#define TRIG_LENGTH     12 // Trigger pulse length (uS)
#define US_PER_CM       58 // 58uS / cm
#define MEASURE_TIME_MS 50 // Measurement interval

// settings for I2C
uint8_t I2C_buffer[SENSOR_NUM];
#define I2C_SLAVE_ADDRESS 0x10
//void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status);

static volatile uint8_t pulse_length[SENSOR_NUM];
static volatile uint8_t x;

struct Pins {
   volatile uint8_t p;
   volatile uint8_t *port;
   volatile uint8_t *pin;
   //uint8_t isr;
};

struct Pins trig_pins[SENSOR_NUM];
struct Pins echo_pins[SENSOR_NUM];

// Measurement Timer Interrupt
// Counts in centimetres
ISR(TIMER2_COMPA_vect) {
  for (x=0;x<SENSOR_NUM;x++) {
    pulse_length[x]++;
  }
}

// Echo Input Interrupt
// Measures echo length
// TODO: Macro to define one for each sensor, given interrupt, pins and ports.
// TODO: Detect each set of echo pins within it's own interrupt
ISR(PCINT0_vect) {
  // TODO: Everything within this interrupt vector is PORTB.
  uint8_t changedbits;
  changedbits = PINB ^ portbhistory;
  portbhistory = PINB;

  for (x=0; x<SENSOR_NUM; x++) {
     if(changedbits & _BV(echo_pins[x].p)) {
       // high
       if (*echo_pins[x].pin & _BV(echo_pins[x].p)) {  // Pulse start
         // Leaving timer running for other sensors
         //TCNT1 = 0;
         pulse_length[x] = 0;
       // low
       } else { // Pulse end
         I2C_buffer[x] = pulse_length[x];
       }
     }
  }
}

void sensor_setup(void) {
  // TODO: Configure array of pins
  for (x=0; x<SENSOR_NUM; x++) {
    DDRB |= _BV(trig_pins[x].p);    // Trig pin is output
    *trig_pins[x].port &= ~_BV(trig_pins[x].p);  // Set Trig to 0
  }
  for (x=0; x<SENSOR_NUM; x++) {
    DDRB &= ~_BV(echo_pins[x].p);   // Sensor pin is input
    *echo_pins[x].port |= _BV(echo_pins[x].p);   // Enable pull-up resistor
  }

  // Timer 1 configuration
  //TCCR1 = _BV(CTC1) | _BV(CS12); // CTC Mode, Clock = ClkI/O / 8
  TCCR2A = _BV(COM2A1) | _BV(WGM21); // Reset OC2A, CTC Mode
  TCCR2B = _BV(CS21); // Clock = ClkI/O / 8
  //OCR1C = US_PER_CM - 1;
  // TODO: Is `US_PER_CM` correct?
  OCR2A = US_PER_CM - 1;
  //TIMSK |= _BV(OCIE1A); // Enable Interrupt TimerCounter1 Compare Match A
  TIMSK2 |= _BV(OCIE2A); // Enable Interrupt TimerCounter2 Compare Match A

  // Setup INT0
  //MCUCR |= _BV(ISC00); // Interrupt on any logical change on INT0
  //GIMSK |= _BV(INT0);  // Enable external pin interrupt

  // PCINT0
  // TODO: Enable PCINTs as needed by pins
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2);
}

void trigger(uint8_t idx) {
  // Trig pulse
  *trig_pins[idx].port |= _BV(trig_pins[idx].p);
  _delay_us(TRIG_LENGTH);
  *trig_pins[idx].port &= ~_BV(trig_pins[idx].p);
}

int main(void) {
  // TODO: Put somewhere cleaner
  trig_pins[0].p     = PB1;
  trig_pins[0].port  = &PORTB;
  trig_pins[0].pin   = &PINB;
  //trig_pins[0].isr = &PCINT0_vect;

  echo_pins[0].p     = PB2;
  echo_pins[0].port  = &PORTB;
  echo_pins[0].pin   = &PINB;
  //echo_pins[0].isr = &PCINT0_vect;

  trig_pins[1].p    = PB1;
  trig_pins[1].port  = &PORTB;
  trig_pins[1].pin   = &PINB;
  //trig_pins[1].isr = &PCINT0_vect;

  echo_pins[1].p    = PB3;
  echo_pins[1].port  = &PORTB;
  echo_pins[1].pin   = &PINB;
  //echo_pins[0].isr = &PCINT0_vect;

  // Initialize I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            0
            //&handle_I2C_interrupt       // pointer to callback function
            );

  sensor_setup();

  sei();

  // give our slave address and enable I2C
  TWI_enable_slave_mode(  I2C_SLAVE_ADDRESS,      // device address of slave
                          0,                      // slave address mask
                          0                       // enable general call
                          );

  // TODO: Timer which triggers sensors in-turn, times them out too.. Use a timer?
  while(1) {
    // TODO: Trigger each pin in array
    for (x=0; x<SENSOR_NUM; x++) {
      trigger(x);
    }

    // Wait for echo
    _delay_ms(MEASURE_TIME_MS);

  }
}

/*
// Write then read.
void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status){
    if (TWI_match_addr == I2C_SLAVE_ADDRESS && status == TWI_success) {
      // Set buffer to be returned on next read cycle
      I2C_buffer[0] = result;
    }
}
*/
