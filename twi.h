// successful return code
#define TWI_success 0x00

// buffers and variables
volatile uint16_t TWI_buffer_max;
volatile uint8_t*  p_TWI_buffer;
volatile uint16_t TWI_buffer_pos;
volatile uint16_t TWI_buffer_len;
volatile uint16_t TWI_read_bytes;
volatile uint16_t TWI_write_bytes;
volatile uint16_t TWI_bytes_returned;

volatile uint8_t TWI_target_slave_addr;
volatile uint8_t TWI_match_addr;

// keep track of current state
volatile uint8_t TWI_status;

// call types
volatile uint8_t TWI_master_state;
#define TWI_OP_WRITE_ONLY 0x01
#define TWI_OP_READ_ONLY 0x02
#define TWI_OP_WRITE_THEN_READ 0x03

// control variables
volatile uint8_t TWI_operation;
volatile uint8_t TWI_busy;
volatile uint8_t TWI_error;

// various states of hardware that will be set in response to interrupts
#define TWI_ENABLE  _BV(TWEN) | _BV(TWINT) | _BV(TWIE)
//
#define TWI_ACK     _BV(TWEA)  |    TWI_ENABLE
#define TWI_NACK                    TWI_ENABLE
#define TWI_START   _BV(TWSTA) |    TWI_ENABLE
#define TWI_STOP    _BV(TWSTO) |    TWI_ENABLE

// define callback function
void (*TWI_return_result)(volatile uint8_t TWI_match_addr, volatile uint8_t TWI_return_code);

// define supported funcitons
void TWI_init(long cpu_freq, long bit_rate, uint8_t* buffer, uint16_t max, void (*callback)(volatile uint8_t TWI_match_addr, volatile uint8_t TWI_return_code));
void TWI_master_start_write(uint8_t slave_addr, uint16_t write_bytes);
void TWI_master_start_read(uint8_t slave_addr, uint16_t read_bytes);
void TWI_master_start_write_then_read(uint8_t slave_addr, uint16_t write_bytes, uint16_t read_bytes);
void TWI_enable_slave_mode(uint8_t my_slave_addr, uint8_t my_slave_addr_mask, uint8_t enable_general_call);
