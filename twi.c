#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>
#include "twi.h"

// initialize the component
void TWI_init(long cpu_freq, long bit_rate, uint8_t* buffer, uint16_t max, void (*callback)(volatile uint8_t TWI_match_addr, volatile uint8_t TWI_return_code)){
    TWI_return_result = callback;
    p_TWI_buffer = buffer;
    TWI_buffer_max = max;
    TWBR = ((cpu_freq/bit_rate)-16)/2; // bit rate register
    TWSR = 0; // prescaler
    TWI_busy=0;
}

// master write to slave
void TWI_master_start_write(uint8_t slave_addr, uint16_t write_bytes){
    TWI_busy=1;
    if(write_bytes>TWI_buffer_max){
        TWI_write_bytes=TWI_buffer_max;
    }else{
        TWI_write_bytes=write_bytes;
    }
    TWI_operation=TWI_OP_WRITE_ONLY;
    TWI_master_state = TW_WRITE;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode
}

// master read from slave
void TWI_master_start_read(uint8_t slave_addr, uint16_t read_bytes){
    TWI_busy=1;
    if(read_bytes>TWI_buffer_max){
        TWI_read_bytes=TWI_buffer_max;
    }else{
        TWI_read_bytes=read_bytes;
    }
    TWI_operation=TWI_OP_READ_ONLY;
    TWI_master_state = TW_READ;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode
}

// master write then read without releasing buss between
void TWI_master_start_write_then_read(uint8_t slave_addr, uint16_t write_bytes, uint16_t read_bytes){
    TWI_busy=1;
    if(write_bytes>TWI_buffer_max){
        TWI_write_bytes=TWI_buffer_max;
    }else{
        TWI_write_bytes=write_bytes;
    }
    if(read_bytes>TWI_buffer_max){
        TWI_read_bytes=TWI_buffer_max;
    }else{
        TWI_read_bytes=read_bytes;
    }
    TWI_operation=TWI_OP_WRITE_THEN_READ;
    TWI_master_state = TW_WRITE;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode
}

// enable slave and start receiving messages
void TWI_enable_slave_mode(uint8_t my_slave_addr, uint8_t my_slave_addr_mask, uint8_t enable_general_call){
    TWAR = (my_slave_addr<<1);      // set my slave addr
    TWAMR = (my_slave_addr_mask<<1);    // set the addr mask for multi-device mode
    if(enable_general_call>0){
        TWAR |= _BV(TWGCE);             // enable general call receipts
    }
    TWCR = TWI_ACK;                     // enable ACK on SLA_W/SLA_R
}

// Routine to service interrupts from the TWI hardware.
// The most important thing is that this routine runs fast and returns control
// to the hardware asap. So, when results are ready and the callback is made to
// your application, be sure you return as quickly as possible. Remove significant
// work from the callback and instead perform that work in your main execution loop.
//
// See pages 229, 232, 235, and 238 of the ATmega328 datasheed for detailed
// explaination of the logic below.
ISR(TWI_vect){
    switch(TW_STATUS){

        case TW_REP_START:
        case TW_START:
            switch(TWI_master_state){
                case TW_WRITE:
                    TWI_buffer_pos=0; // point to 1st byte
                    TWDR = (TWI_target_slave_addr<<1) | 0x00; // set SLA_W
                    break;
                case TW_READ:
                    TWI_buffer_pos=0; // point to first byte
                    TWDR = (TWI_target_slave_addr<<1) | 0x01; // set SLA_R
                    break;
            }
            TWCR = TWI_ACK; // transmit
            break;

        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK:
            if(TWI_buffer_pos==TWI_write_bytes){
                if(TWI_operation==TWI_OP_WRITE_THEN_READ){
                    TWI_master_state=TW_READ; // now read from slave
                    TWCR = TWI_START; // transmit repeated start
                }else{
                    if(TWI_return_result){
                        (*TWI_return_result)(0, TWI_success);// callback with results
                    }
                    TWCR = TWI_STOP; // release the buss
                    while(TWCR & (1<<TWSTO)); // wait for it
                    TWI_busy=0;
                }
            }else{
                TWDR = p_TWI_buffer[TWI_buffer_pos++]; // load data
                TWCR = TWI_ENABLE; // transmit
            }
            break;

        case TW_MR_DATA_ACK:
            p_TWI_buffer[TWI_buffer_pos++]=TWDR; // save byte
        case TW_MR_SLA_ACK:
            if(TWI_buffer_pos==(TWI_read_bytes-1)){
                TWCR = TWI_NACK; // get last byte then nack
            }else{
                TWCR = TWI_ACK; // get next byte then ack
            }
            break;

        case TW_MR_DATA_NACK:
            p_TWI_buffer[TWI_buffer_pos++]=TWDR; // save byte
            if(TWI_return_result){
                (*TWI_return_result)(0, TWI_success);// callback with results
            }
            TWCR = TWI_STOP; // release the buss
            while(TWCR & (1<<TWSTO)); // wait for it
            TWI_busy=0;
            break;

        case TW_SR_SLA_ACK:
        case TW_SR_ARB_LOST_SLA_ACK:
        case TW_SR_GCALL_ACK:
        case TW_SR_ARB_LOST_GCALL_ACK:
            TWI_match_addr=TWDR>>1; // save the match address
            TWI_buffer_pos=0; // point to start of input buffer
            TWCR = TWI_ACK;
            break;

        case TW_SR_DATA_ACK:
        case TW_SR_GCALL_DATA_ACK:
            if(TWI_buffer_pos<TWI_buffer_max){
                p_TWI_buffer[TWI_buffer_pos++]=TWDR; // store data
            }
            TWCR = TWI_ACK;
            break;

        case TW_SR_STOP:
            TWI_buffer_len=TWI_buffer_pos; // bytes returned
            if(TWI_return_result){
                (*TWI_return_result)(TWI_match_addr, TWI_success); // callback with results
            }
            TWCR = TWI_ACK;
            break;

        case TW_ST_ARB_LOST_SLA_ACK:
        case TW_ST_SLA_ACK:
            TWI_match_addr=TWDR>>1; // save the match address
            TWI_buffer_pos=0; // point to start of input buffer
        case TW_ST_DATA_ACK:
            if(TWI_buffer_pos<TWI_buffer_max){
                TWDR = p_TWI_buffer[TWI_buffer_pos++]; // load data
            }
        case TW_SR_DATA_NACK:
        case TW_SR_GCALL_DATA_NACK:
        case TW_ST_DATA_NACK:
        case TW_ST_LAST_DATA:
            TWCR = TWI_ACK;
            break;

        case TW_MT_SLA_NACK:
        case TW_MT_DATA_NACK:
        case TW_MR_SLA_NACK:
            if(TWI_return_result){
                (*TWI_return_result)(TWI_match_addr, TW_STATUS);// callback with status
            }
        case TW_MT_ARB_LOST:
        //case TW_MR_ARB_LOST:
        default:
            TWCR=TWI_STOP;
            while(TWCR & (1<<TWSTO)); // wait for it
            TWCR=TWI_START; // try again
            break;
    }

}
