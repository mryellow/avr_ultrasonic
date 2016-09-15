#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

//#define F_CPU			8000000UL

#define SENSOR_NUM      8

#define TRIG_DDR   DDRD
#define TRIG_PORT  PORTD

#define ECHO_DDR   DDRB
#define ECHO_PORT  PORTB
#define ECHO_PIN   PINB
#define ECHO_PCIE  PCIE0
#define ECHO_PCMSK PCMSK0

#define TRIG_LENGTH     12 // Trigger pulse length (uS)
#define US_PER_CM       58 // 58uS / cm
#define MEASURE_TIME_MS 50 // Measurement interval

// settings for I2C
uint8_t I2C_buffer[SENSOR_NUM];
#define I2C_SLAVE_ADDRESS 0x10
//void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status);

static volatile uint8_t pulse_length[SENSOR_NUM];

// Measurement Timer Interrupt
// Counts in centimetres
ISR(TIMER2_COMPA_vect) {
  uint8_t x;
  for (x=0; x<SENSOR_NUM; x++) {
    pulse_length[x]++;
  }
}

// Echo Input Interrupt
// Measures echo length
// https://github.com/borischernov/avr_hcsr04/blob/master/main.c
volatile uint8_t echoporthistory = 0xFF;
// TODO: Define which vector matches `ECHO_PORT`.
ISR(PCINT0_vect) {
  uint8_t changedbits;
  changedbits = ECHO_PIN ^ echoporthistory;
  echoporthistory = ECHO_PIN;

  uint8_t x;
  for (x=0; x<SENSOR_NUM; x++) {
    if(changedbits & _BV(x)) {
      // high
      if (ECHO_PIN & _BV(x)) {  // Pulse start
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
  // Configure ports
  TRIG_DDR  = 0xFF; // Trig pin is output
  TRIG_PORT = 0x00; // Set Trig to 0
  ECHO_DDR  = 0x00; // Sensor pin is input
  ECHO_PORT = 0xFF; // Enable pull-up resistor
  /*
  for (x=0; x<SENSOR_NUM; x++) {
    TRIG_DDR  |= _BV(x);    // Trig pin is output
    TRIG_PORT &= ~_BV(x);   // Set Trig to 0
    ECHO_DDR  &= ~_BV(x);   // Sensor pin is input
    ECHO_PORT |= _BV(x);    // Enable pull-up resistor
  }
  */

  // Timer 1 configuration
  //TCCR1 = _BV(CTC1) | _BV(CS12); // CTC Mode, Clock = ClkI/O / 8
  TCCR2A = _BV(WGM21); // CTC Mode
  TCCR2B = _BV(CS21); // Clock = ClkI/O / 8
  //OCR1C = US_PER_CM - 1;
  // TODO: Is `US_PER_CM` correct?
  OCR2A = US_PER_CM - 1;
  //TIMSK |= _BV(OCIE1A); // Enable Interrupt TimerCounter1 Compare Match A
  TIMSK2 |= _BV(OCIE2A); // Enable Interrupt TimerCounter2 Compare Match A

  // Setup INT0
  //MCUCR |= _BV(ISC00); // Interrupt on any logical change on INT0
  //GIMSK |= _BV(INT0);  // Enable external pin interrupt

  // PCINT
  PCICR |= _BV(ECHO_PCIE);
  ECHO_PCMSK = 0xFF; // Enable interrupts on PORTB
  /*
  uint8_t x;
  for (x=0; x<SENSOR_NUM; x++) {
    ECHO_PCMSK |= _BV(x);
  }
  */
}

int main(void) {
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

  // TODO: Triggers sensors in-turn, time them out too.. Use a timer instead?
  while(1) {
    // Trigger each pin
    uint8_t x;
    for (x=0; x<SENSOR_NUM; x++) {
      TRIG_PORT |= _BV(x);
      _delay_us(TRIG_LENGTH);
      TRIG_PORT &= ~_BV(x);
      //_delay_us(TRIG_LENGTH);

      // Wait for echo before next trigger
      _delay_ms(MEASURE_TIME_MS/SENSOR_NUM);
    }

  }

  return(0);
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
