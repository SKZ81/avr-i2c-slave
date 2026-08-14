#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "I2CSlave.h"

bool receiving = false;
bool i2c_configured =false;

// f_twi = 1 / 16 + 2*TWBR*prescale
#if F_CPU == 16000000
    #define TWBR_value 72
#elif F_CPU == 8000000
    #define TWBR_value 32
#else
    #error F_CPU must be either 16000000 or 8000000 for i²c clock to be correct
#endif

#define check_i2c_conf() {\
   if (!i2c_configured) {\
        DDRD |= (1<<PD4) | (1<<PD3);\
        PORTD |= (1<<PD4) | (1<<PD3);\
        TWBR = TWBR_value;\
        i2c_configured = true;\
   }\
}


static void (*i2c_slave_recv)(uint8_t);
static void (*i2c_slave_req)(i2c_request_t);
static void (*i2c_slave_receive_start)(void);
//static void (*i2c_slave_receive_stop)(void);


void i2c_slave_setCallbacks(void (*start)(void),
                            void (*recv)(uint8_t),
                            //void (*stop)(void),
                            void (*req)(i2c_request_t))
{
  i2c_slave_recv = recv;
  i2c_slave_req = req;
  i2c_slave_receive_start = start;
  //i2c_slave_receive_stop = stop;
}

void i2c_slave_init(uint8_t address)
{
  cli();
  check_i2c_conf();

  // load address into TWI address register
  TWAR = address << 1;
  // set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
  TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
  sei();
}

uint8_t i2c_master_init(uint8_t slave_addressed, i2c_master_mode_t mode) {
    check_i2c_conf();

    //wait for pending RX operation to complete
    //TODO : add a delay ?
    while(receiving) PORTD ^= PD5;

    PORTD |= 1<<PD5;

    PORTD &= ~(1<<PD4);
//     DDRC |= 1<<4 | 1<<5;
    // transmit START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    PORTD |= 1<<PD4;
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );

    PORTD &= ~(1<<PD4);

    // check if the (repeated) start condition was successfully transmitted
    if(    ((TWSR & 0xF8) != TW_START)
        && ((TWSR & 0xF8) != TW_REP_START) ) {
        return 1;
    }
    PORTD |= 1<<PD4;

    // load slave address into data register
    TWDR = (slave_addressed<<1)|mode;
    // start transmission of address
    PORTD &= ~(1<<PD4);
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    PORTD |= 1<<PD4;

    PORTD &= ~(1<<PD5);

    PORTD &= ~(1<<PD4);
    // check if the device has acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8;
    if ( (mode == MasterTransmit) && (twst != TW_MT_SLA_ACK) ) return 1;
    if ( (mode == MasterReceive) && (twst != TW_MR_SLA_ACK) ) return 1;
    PORTD |= 1<<PD4;
    return 0;
}

uint8_t i2c_master_write(uint8_t data)
{
    if (receiving)  {return(2);} // should not happend, unless app coded by a crazy monkey

    // load data into data register
    TWDR = data;
    // start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );

    if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

    return 0;
}

uint8_t i2c_master_read(uint8_t *data) {
    TWCR = (1<<TWINT) | (1<<TWEN); //| (1<<TWEA) for multiple bytes read.

    while( !(TWCR & (1<<TWINT)) );
    if ((TWSR & 0xF8) != TW_MR_DATA_NACK){ return 1; }

    *data = TWDR;

    return 0;
}


void i2c_master_done() {
    // transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
    // enable interrupt if slave address inited
    if (TWAR) TWCR |= (1<<TWIE);
}

void i2c_slave_stop(void)
{
  // clear acknowledge and enable bits
  cli();
  TWCR = 0;
  TWAR = 0;
  sei();
}

ISR(TWI_vect)
{
  PORTD &= ~(1<<PD3);
  switch(TW_STATUS)
  {
    case TW_SR_SLA_ACK:
      receiving = true;
      if (i2c_slave_receive_start) i2c_slave_receive_start();
      break;
    case TW_SR_STOP:
      /*DDRC |= 1<<5;  // pull SCL down (streching)
      if (i2c_slave_receive_stop) i2c_slave_receive_stop();
      DDRC &= ~(1<<5);  // free the bus*/
      receiving = false;
      break;
    case TW_SR_DATA_ACK: // data received, ACK returned
      // received data from master, call the receive callback
      if (i2c_slave_recv) i2c_slave_recv(TWDR);
      break;
    case TW_ST_SLA_ACK: // SLA+R
      // master is requesting data, call the request callback
      receiving = true;
      if (i2c_slave_req) i2c_slave_req(INITIAL);
      break;
    case TW_ST_DATA_ACK: // data transmitted, ACK received
      // master is requesting data, call the request callback
      if (i2c_slave_req) i2c_slave_req(CONTINUATION);
      break;
    case TW_ST_DATA_NACK: // master closed the connexion (last byte)
      if (i2c_slave_req) i2c_slave_req(DONE);
      receiving = false;
      break;
    case TW_BUS_ERROR:
      // some sort of erroneous state, not much to do as for now...
      receiving = false;
      break;
    default:
      break;
  }
  // mainly for clearing TWINT
  TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
  PORTD |= 1<<PD3;
} 
