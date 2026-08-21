#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2CSlave.h"

#define DEBUG_PORT PORTD
#define DEBUG_DDR  DDRD
#define ISR_BIT    3
#define MASTER_BIT 4
#define WAIT_BIT   5

#if DEBUG
    #define dbg_ISR_ON()      DEBUG_PORT |= _BV(ISR_BIT)
    #define dbg_ISR_OFF()     DEBUG_PORT &= ~_BV(ISR_BIT)
    #define dbg_master_ON()   DEBUG_PORT |= _BV(MASTER_BIT)
    #define dbg_master_OFF()  DEBUG_PORT &= ~_BV(MASTER_BIT)
    #define dbg_toggle_wait() DEBUG_PORT ^= _BV(WAIT_BIT)
    #define dbg_wait_ON()     DEBUG_PORT |= _BV(WAIT_BIT)
    #define dbg_wait_OFF()    DEBUG_PORT &= ~_BV(WAIT_BIT)
#else
    #define dbg_ISR_ON()
    #define dbg_ISR_OFF()
    #define dbg_master_ON()
    #define dbg_master_OFF()
    #define dbg_toggle_wait()
    #define dbg_wait_ON()
    #define dbg_wait_OFF()
#endif

bool slave_operating = false; // true while slave is currently operating (transmitting to or receiving from a master)
bool i2c_configured = false;
bool i2c_master_inited = false; // true while operating in master mode

#define check_i2c_conf() {\
   if (!i2c_configured) {\
     return I2C_NOT_CONFIGURED;\
   }\
}

i2c_baud_config_t i2c_baud_getparams(uint32_t f_cpu,
                                     uint32_t target_f_scl)
{
    static const uint8_t prescalers[] = { 1, 4, 16, 64 };

    i2c_baud_config_t best = {
        .twbr = 0,
        .ps = 1,
        .frequency = f_cpu / 16
    };

    best.error =
        (best.frequency > target_f_scl)
            ? best.frequency - target_f_scl
            : target_f_scl - best.frequency;

    for (unsigned int i = 0; i < 4; ++i) {
        uint8_t ps = prescalers[i];

        /*
         * F = F_CPU / (16 + 2 * TWBR * PS)
         *   ==>
         * TWBR = (F_CPU / F - 16) / (2 * PS)
         */
        uint32_t denominator = 2UL * ps;

        if (f_cpu < 16UL * target_f_scl)
            continue;

        uint32_t numerator = f_cpu - 16UL * target_f_scl;

        uint32_t twbr = (numerator + target_f_scl * denominator / 2)
                      / (target_f_scl * denominator);

        if (twbr > 255)
            twbr = 255;

        uint32_t frequency =
            f_cpu / (16UL + 2UL * twbr * ps);

        uint32_t error =
            (frequency > target_f_scl)
                ? frequency - target_f_scl
                : target_f_scl - frequency;

        if (error < best.error) {
            best.twbr = (uint8_t)twbr;
            best.ps = ps;
            best.frequency = frequency;
            best.error = error;
        }
    }

    return best;
}

void i2c_init(i2c_baud_config_t params)
{
#if DEBUG
    DEBUG_DDR |= _BV(ISR_BIT) | _BV(WAIT_BIT) | _BV(MASTER_BIT);
    DEBUG_PORT |= _BV(ISR_BIT) | _BV(WAIT_BIT) | _BV(MASTER_BIT);
#endif

    TWBR = params.twbr;
    switch (params.ps) {
        case 1:  TWSR &= ~(_BV(TWPS1) | _BV(TWPS0)); break;
        case 4:  TWSR = (TWSR & ~(_BV(TWPS1) | _BV(TWPS0))) | _BV(TWPS0); break;
        case 16: TWSR = (TWSR & ~(_BV(TWPS1) | _BV(TWPS0))) | _BV(TWPS1); break;
        case 64: TWSR |= _BV(TWPS1) | _BV(TWPS0); break;
    }
    i2c_configured = true;
}

static bool (*i2c_slave_recv)(uint8_t);
static bool (*i2c_slave_req)(i2c_request_t);
static void (*i2c_slave_receive_start)(void);
//static void (*i2c_slave_receive_stop)(void);


void i2c_slave_setCallbacks(void (*start)(void),
                            bool (*recv)(uint8_t),
                            //void (*stop)(void),
                            bool (*req)(i2c_request_t))
{
  i2c_slave_recv = recv;
  i2c_slave_req = req;
  i2c_slave_receive_start = start;
  //i2c_slave_receive_stop = stop;
}

uint8_t i2c_slave_init(uint8_t address)
{
  check_i2c_conf();
  if (!i2c_slave_recv && !i2c_slave_req && !i2c_slave_receive_start)
      return SLAVE_CALLBACKS_NOT_SET;

  cli();
  // load address into TWI address register
  TWAR = address << 1;
  // set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
  TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
  sei();

  return OK;
}

void i2c_slave_stop(void)
{
  // clear acknowledge and enable bits
  cli();
  TWCR = 0;
  TWAR = 0;
  sei();
}

// --------- MASTER ------------

uint8_t i2c_master_init(uint8_t slave_address, i2c_master_mode_t mode) {
    check_i2c_conf();

    //wait for pending RX operation to complete
    while(slave_operating) dbg_toggle_wait();
    dbg_wait_ON();
    dbg_master_OFF();

    // transmit START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    dbg_master_ON();

    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    dbg_master_OFF();

    // check if the (repeated) start condition was successfully transmitted
    if(    ((TW_STATUS) != TW_START)
        && ((TW_STATUS) != TW_REP_START) ) {
        return 1;
    }
    dbg_master_ON();

    // load slave address into data register
    TWDR = (slave_address<<1)|mode;

    // start transmission of address
    dbg_master_OFF();
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) ) dbg_toggle_wait();
    dbg_master_ON();
    dbg_wait_ON();

    // check if the device has acknowledged the READ / WRITE mode
    if ( (mode == MasterTransmit) && (TW_STATUS != TW_MT_SLA_ACK) )
        return MASTER_TRM_NACKED_BY_SLAVE;
    if ( (mode == MasterReceive) && (TW_STATUS != TW_MR_SLA_ACK) )
        return MASTER_RCV_NACKED_BY_SLAVE;


    i2c_master_inited = true;
    return 0;
}

uint8_t i2c_master_write(uint8_t data)
{
    check_i2c_conf();
    if (slave_operating)  {return(42);} // TODO PROPER ERROR CODE
    if (!i2c_master_inited)
        return MASTER_NOT_INITED;

    // load data into data register
    TWDR = data;
    // start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );

    if( (TW_STATUS) != TW_MT_DATA_ACK )
        return MASTER_TRM_NACKED_BY_SLAVE;

    return 0;
}

uint8_t i2c_master_read(uint8_t *data) {
    check_i2c_conf();

    if (slave_operating)  {return(2);}
    if (!i2c_master_inited)
        return MASTER_NOT_INITED;

    TWCR = (1<<TWINT) | (1<<TWEN); //| (1<<TWEA) for multiple bytes read.

    while( !(TWCR & (1<<TWINT)) );
    if ((TW_STATUS) != TW_MR_DATA_NACK)
        return MASTER_RCV_NACKED_BY_SLAVE;

    *data = TWDR;

    return 0;
}

void i2c_master_done() {
    // transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
    // enable interrupt if slave address inited
    if (TWAR) TWCR |= (1<<TWIE);

    i2c_master_inited = false;
}


// ------------ ISR is used in SLAVE mode only (as for now) ----------


// TODO : manage following state codes
// TW_SR_GCALL_ACK          0x70
// TW_SR_ARB_LOST_GCALL_ACK 0x78
// TW_SR_GCALL_DATA_ACK     0x90
// TW_SR_GCALL_DATA_NACK    0x98



ISR(TWI_vect)
{
#if DEBUG
  DEBUG_PORT &= ~_BV(ISR_BIT);
#endif
  bool last_data = false;

  switch(TW_STATUS)
  {
    // ------ Slave Receiver ------
    case TW_SR_ARB_LOST_SLA_ACK:       // 0x68
      i2c_master_inited = false;
    case TW_SR_SLA_ACK:                // 0xA8
      slave_operating = true;
      if (i2c_slave_receive_start)
        i2c_slave_receive_start();
      break;
    case TW_SR_STOP:                   // 0xA0
      /*DDRC |= 1<<5;  // pull SCL down (streching)
      if (i2c_slave_receive_stop) i2c_slave_receive_stop();
      DDRC &= ~(1<<5);  // free the bus*/
      slave_operating = false;
      break;
    case TW_SR_DATA_NACK:              // 0x88
      // data received, NACK returned
    case TW_SR_DATA_ACK:               // 0xB8
      // data received, ACK returned
      if (i2c_slave_recv)
        last_data = i2c_slave_recv(TWDR);
      break;

    // ------ Slave Transmitter ------
    case TW_ST_ARB_LOST_SLA_ACK:       //0xB0
        i2c_master_inited = false;
    case TW_ST_SLA_ACK:                // 0xA8 SLA+R
      // Master is requesting data, call the request callback
      slave_operating = true;
      if (i2c_slave_req)
        last_data = i2c_slave_req(INITIAL);
      break;
    case TW_ST_DATA_ACK: // 0xB8 data transmitted, ACK received
      // master is requesting more data, call the request callback
      if (i2c_slave_req)
        last_data = i2c_slave_req(CONTINUATION);
      break;
    case TW_ST_DATA_NACK: // 0xC0
      // master closes the transmition
    case TW_ST_LAST_DATA: // 0xC8
      // slave closes the transmition
      if (i2c_slave_req)
        last_data = i2c_slave_req(DONE);
      slave_operating = false;
      break;

    // ------ Bus Error ------
    case TW_BUS_ERROR:
      // some sort of erroneous state, not much to do as for now...
      slave_operating = false;
      break;
    default:
      break;
  }

  // Set TWI status for next step.
  TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEN) | (last_data) ? 0 : (1<<TWEA);

#if DEBUG
  DEBUG_PORT |= _BV(ISR_BIT);
#endif
} 
