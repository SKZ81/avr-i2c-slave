#ifndef _I2C_SLAVE_STATE_MACHINE_H_
#define _I2C_SLAVE_STATE_MACHINE_H_

// === I²C slave State Machine command ===
typedef struct {
    uint8_t code;
    uint8_t arg_len;
    uint8_t (*callback)(uint8_t*, uint8_t);
    //uint8_t reply_len;
} i2c_slaveSM_command_t;

// === I²C slave State Machine initialization ===
void i2c_slaveSM_init(uint8_t i2c_address,
                      i2c_slaveSM_command_t *commands,
                      unsigned int nb_commands,
                      uint8_t *buffer,
                      unsigned int buffer_size);

#endif
