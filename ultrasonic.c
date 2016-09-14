#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

//#define F_CPU			8000000UL

#define SENSOR_NUM 1

#define TRIG_PIN  PB1
#define ECHO_PIN  PB2

#define TRIG_LENGTH     12 // Trigger pulse length (uS)
#define US_PER_CM       58 // 58uS / cm
#define MEASURE_TIME_MS 50 // Measurement interval

// settings for I2C
uint8_t I2C_buffer[SENSOR_NUM];
#define I2C_SLAVE_ADDRESS 0x10
void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status);

static volatile uint8_t pulse_length[SENSOR_NUM];

// Measurement Timer Interrupt
// Counts in centimetres
ISR(TIMER2_COMPA_vect) {
  for (x=0;x<SENSOR_NUM;x++) {
    pulse_length[x]++;
  }
}

// Echo Input Interrupt
// Measures echo length
// TODO: Macro to define one for each sensor
ISR(PCINT0_vect) {
  // TODO: Detect `changedbits` in each port
  //uint8_t changedbits;
  //changedbits = PINB ^ portbhistory;
  //portbhistory = PINB;
  //if(changedbits & (1 << PINB0))

  // TODO: Detect array of pins from `changedbits`
  // high
  if (PINB & _BV(ECHO_PIN)) {  // Pulse start
    // Leaving timer running for other sensors
    //TCNT1 = 0;
    pulse_length[0] = 0;
  // low
  } else { // Pulse end
    I2C_buffer[0] = pulse_length[0];
  }
}

void sensor_setup(void) {
  // TODO: Configure array of pins
  DDRB |= _BV(TRIG_PIN);    // Trig pin is output
  PORTB &= ~_BV(TRIG_PIN);  // Set Trig to 0
  DDRB &= ~_BV(ECHO_PIN);   // Sensor pin is input
  PORTB |= _BV(ECHO_PIN);   // Enable pull-up resistor

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
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2);
}

// TODO: `trigger(port)`
void trigger(void) {
  // Trig pulse
  PORTB |= _BV(TRIG_PIN);
  _delay_us(TRIG_LENGTH);
  PORTB &= ~_BV(TRIG_PIN);
}

int main(void) {

  // Initialize I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            &handle_I2C_interrupt       // pointer to callback function
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
    trigger();

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
