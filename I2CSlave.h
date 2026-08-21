#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Error code returned by following functions (when returning uint8_t)
#define OK                         0
#define I2C_NOT_CONFIGURED         1
#define SLAVE_CALLBACKS_NOT_SET    2 // At least one callback must be set
#define MASTER_NOT_INITED          3 // can't enable Master mode while slave is transmitting or reveiving
#define MASTER_TRM_NACKED_BY_SLAVE 4
#define MASTER_RCV_NACKED_BY_SLAVE 5
// #define

// used as argument to call request callback :
typedef enum {
    INITIAL = 1,      // when master open communication for reading
    CONTINUATION = 2, // when master ACKs transmitted data (it expects continuation)
    DONE = 3          // when masters (n)ACKs last transmitted data (NB : should be a nACK to indicate no data longer expected)
} i2c_request_t;

typedef struct {
    uint8_t twbr;
    uint8_t ps;       // 1, 4, 16 ou 64
    uint32_t frequency;
    uint32_t error;
} i2c_baud_config_t;

// Calculate best fit for TWBR and PreScaler from wanted SCL frequency
i2c_baud_config_t i2c_baud_getparams(uint32_t f_cpu,
                                     uint32_t target_f_scl);
// Initiate I²C from parameters
void i2c_init(i2c_baud_config_t params);

// Set callbacks (must be called before initing slave listening mode)
// NOTE: recv() and req() return a boolean indicating "last data", i.e.:
// for recv, the slave will NAck the next byte received, then expects a STOP
// for req, the slace is expecting a NAck, and will send 0xFF if further data is requested by master
void i2c_slave_setCallbacks(void (*start)(void),      // callback for Slave Receive START Condition
                            bool (*recv)(uint8_t),    // callback for ACKed Slave Receive Data
                            //void (*stop)(void),       // callback for Slave Receive STOP Condition
                            bool (*req)(i2c_request_t)    // callback for Slave Transmit (init, cont, done)
                           );

// Start slave listening mode
uint8_t i2c_slave_init(uint8_t address);
// Stop slave listening mode
void i2c_slave_stop(void);

// Declare the slave as busy. It will not accept further incoming request, answering NAck on the bus
static inline void i2c_slave_busy() {
    TWCR &= ~(1<<TWEA);
}
// Reactivate slave after a busy state
static inline void i2c_slave_ready() {
    TWCR |= (1<<TWEA);
}


static inline void __attribute__((always_inline)) i2c_slave_transmitByte(uint8_t data)
{
    TWDR = data;
}


typedef enum {
    MasterTransmit = 0,
    MasterReceive = 1
} i2c_master_mode_t;

// NB : initiating a master transaction will "pause" the listening slave mode.
// slave listening will be restored after the master transaction is done.
// No other master on the bus can address our slave address (if enabled) while we hold the bus
// So, PLEASE ENSURE you call i2c_master_done() in all case (even on error branches) to free the bus

// All functions are returning 0 if succesful (unless returning nothing)

uint8_t i2c_master_init(uint8_t slave_addressed, i2c_master_mode_t mode);

uint8_t i2c_master_write(uint8_t data);

uint8_t i2c_master_read(uint8_t *data);

void i2c_master_done();

// NB there is no REPEATED START dedicated method. Just call again i2c_master_init() without calling i2c_master_done() : this ensure we keep the bus by emiting a RS condition, instead of a P+S (STOP, then START) sequence (which may allow a concurrent master, if nany on the bus, to take its ownership).

#ifdef __cplusplus
};
#endif

#endif
