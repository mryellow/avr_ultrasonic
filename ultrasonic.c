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

//#define ECHO_DDR   DDRB
//#define ECHO_PORT  PORTB
//#define ECHO_PIN   PINB
//#define ECHO_PCIE  PCIE0
//#define ECHO_PCMSK PCMSK0

#define TRIG_LENGTH     12 // Trigger pulse length (uS)
// Reduce resolution to fit full range within `uint8_t`.
// 255 * 1.0cm = 2.55m
// 255 * 1.5cm = 3.825m = 87uS/cm
// 255 * 1.56862745098cm = 4m
// 255 * 2.0cm = 5.10m
#define CM_PER_IT       1.5
#define US_PER_CM       58*CM_PER_IT // 58uS / cm
#define MEASURE_TIME_MS 20 // Measurement interval

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

void check_echo(volatile uint8_t *port, uint8_t pin, uint8_t sensor) {
  // high
  if (*port & _BV(pin)) {  // Pulse start
    // Leaving timer running for other sensors
    //TCNT1 = 0;
    pulse_length[sensor] = 0;
  // low
  } else { // Pulse end
    I2C_buffer[sensor] = pulse_length[sensor];
  }
}

volatile uint8_t echoportbhistory = 0xFF;
ISR(PCINT0_vect) {
  uint8_t changedbits;
  changedbits = PINB ^ echoportbhistory;
  echoportbhistory = PINB;

  uint8_t x;
  for (x=0; x<5; x++) {
    if(changedbits & _BV(x)) {
      check_echo(&PINB, x, x);
    }
  }

}

volatile uint8_t echoportchistory = 0xFF;
ISR(PCINT1_vect) {
  uint8_t changedbits;
  changedbits = PINC ^ echoportchistory;
  echoportchistory = PINC;

  if(changedbits & _BV(0)) {
    check_echo(&PINC, 0, 6);
  }
  if(changedbits & _BV(1)) {
    check_echo(&PINC, 1, 7);
  }
}

/*
volatile uint8_t echoporthistory = 0xFF;
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
*/

void sensor_setup(void) {
  // Configure ports
  TRIG_DDR  = 0xFF; // Trig pin is output
  TRIG_PORT = 0x00; // Set Trig to 0

  // Sensor pin is input
  DDRB &= ~_BV(0) & ~_BV(1) & ~_BV(2) & ~_BV(3) & ~_BV(4) & ~_BV(5);
  DDRC &= ~_BV(0) & ~_BV(1);

  // Enable pull-up resistor
  PORTB |= _BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5);
  PORTC |= _BV(0) | _BV(1);

  // Interrupt on any logical change
  PCICR |= _BV(PCIE0) | _BV(PCIE1);

  // Enable external pin interrupt
  PCMSK0 |= _BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5);
  PCMSK1 |= _BV(0) | _BV(1);

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
  OCR2A = US_PER_CM - 1;
  //TIMSK |= _BV(OCIE1A); // Enable Interrupt TimerCounter1 Compare Match A
  TIMSK2 |= _BV(OCIE2A); // Enable Interrupt TimerCounter2 Compare Match A

  // Setup INT0
  //MCUCR |= _BV(ISC00); // Interrupt on any logical change on INT0
  //GIMSK |= _BV(INT0);  // Enable external pin interrupt

  // PCINT
  //PCICR |= _BV(ECHO_PCIE);
  //ECHO_PCMSK = 0xFF; // Enable interrupts on PORTB
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
      //_delay_ms(MEASURE_TIME_MS/SENSOR_NUM);
      _delay_ms(MEASURE_TIME_MS);
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
