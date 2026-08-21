#include "I2CSlave.h"
#include "I2CSlave_state_machine.h"
#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#ifndef DEBUG
// DEBUG is enabled by defaut, use 0 here or use -DDEBUG=0 on compilation line to deactivate
#define DEBUG 1
#endif

#if DEBUG
    #define dbg(x, ...) printf_P(PSTR(x) ,##__VA_ARGS__)
#else
    #define dbg(x, ...)
#endif

typedef enum {
    WAITING = 0,
    READING_ARGS,
    EXECUTE_COMMAND,
    READY,
    TALKING
} i2c_slaveSM_status_t;

static i2c_slaveSM_command_t *commands = NULL;
static unsigned int nb_commands = -1;
static i2c_slaveSM_command_t *current_command = NULL;

static i2c_slaveSM_status_t status;

static uint8_t *buffer = NULL;
static unsigned int buffer_size;

static uint8_t buffer_index = 0;
static uint8_t reply_len = 0;

// =====================================================================

static inline void reset_state() {
    status = WAITING;
    reply_len = 0;
    current_command = NULL;
    buffer_index = 0;
    //memset(buffer, 0, buffer_size);
}

i2c_slaveSM_command_t *get_command(uint8_t command_code) {
    i2c_slaveSM_command_t *command_ptr = NULL;
    for(uint8_t i = 0; i < nb_commands; i++) {
        if (commands[i].code == command_code) {
            command_ptr = &(commands[i]);
        }
    }
    if (command_ptr == NULL) {
        dbg("get_command() : Could not not find command code %d\n", command_code);
    } else {
        dbg("get_command() : command code %d found, OK\n", command_code);
    }
    return command_ptr;
}


bool i2c_slaveSM_requested(i2c_request_t req_type) {
    bool last_data = false;

    switch(req_type) {
         case INITIAL:
            if (status == READY) {
                status = TALKING;
            } else {
                // we were not READY to reply, then forget all previous crap said by master
                dbg("Initial query but state is not READY (status == %d)\n", status);
                goto protocol_error;
            }
        // NOTE : if no error => no break !! execute following statement
        case CONTINUATION:
            if (status != TALKING) {
                dbg("Requested to tranmit data, while not in TALKING state (status == %d)\n", status);
                goto protocol_error;
            }
            if (buffer_index >= reply_len) {
                // NB : thanks to last_data management, this should not happend.
                // abnormal situation, give a trace here
                dbg("ERR : buffer_index >= reply_len\n");
                goto protocol_error;
            }
            last_data = ((buffer_index+1)==reply_len);
            dbg("send I2C data : %x %s\n", buffer[buffer_index], last_data?"(last)":"");
            i2c_slave_transmitByte(buffer[buffer_index]);
            buffer_index++;
            break;
        case DONE:
            if (buffer_index == reply_len) { // We're done !!
                dbg("Succesfully transmitted %d bytes\n", reply_len);
            } else {
                dbg("Trnasmition aborted early (%d / %d bytes sended)", buffer_index, reply_len);
            }
            reset_state();
            break;

        default:
            dbg("i2c_scale_requested : unknown req_type %d\n", req_type);
            reset_state();
            break;
    }

    return last_data;


protocol_error:
    i2c_slave_transmitByte(0xFF);
    reset_state();
    return true; // expect master to end transmission
}



bool i2c_slaveSM_receive(uint8_t data) {
    bool nack = false;

    dbg("I2C received byte : %x\n", data);
    switch(status) {
        case WAITING:
            current_command = get_command(data);
            if (current_command != NULL) {
                buffer_index = 0;
                if (current_command->arg_len == 0) {
                    // no arg to get, just execute callback
                    status = EXECUTE_COMMAND;
                } else {
                    if (current_command->arg_len > buffer_size) {
                        dbg("Error : buffer is not large enough to store %d bytes for command %d\n", current_command->arg_len, current_command->code);
                        reset_state();
                    } else {
                        // We have data arg to read
                        status = READING_ARGS;
                    }
                }
            }
            break;

        case READING_ARGS:
#if DEBUG
            if(!current_command) {
                dbg("READING_ARGS but current_command is NULL !\n");
                reset_state();
                break;
            }
#endif
            buffer[buffer_index++] = data;
            if (buffer_index == current_command->arg_len) {
                // all args received, execute callback
                status = EXECUTE_COMMAND;
            }
            break;

        default:
            dbg("Got data while state not WAITING nor READING_ARGS (status == %d)\n", status);
            reset_state();
    }

    if (status == EXECUTE_COMMAND) {
        i2c_slave_busy();
        reply_len = current_command->callback(buffer, buffer_size);
        i2c_slave_ready();
        if (reply_len > 0) {
            status = READY; // to send data
            buffer_index = 0;
        } else {
            // nothing to send, we're done
            reset_state();
        }
    }

    return nack;
}



void i2c_slaveSM_init(uint8_t address, uint32_t frequency,
                      i2c_slaveSM_command_t *i2c_commands,
                      unsigned int i2c_commands_size,
                      uint8_t *params_buffer,
                      unsigned int params_buffer_size) {
    commands = i2c_commands;
    nb_commands = i2c_commands_size;;
    buffer = params_buffer;
    buffer_size = params_buffer_size;

    i2c_slave_setCallbacks(NULL,
                           i2c_slaveSM_receive,
                           i2c_slaveSM_requested);
    i2c_baud_config_t params = i2c_baud_getparams(F_CPU,
                                                  frequency);
    dbg("I2C Baud parameters:\n");
    dbg("Requested freq = %lu bd, real freq = %lu bd (error = %lu Bd)\n", frequency, params.frequency, params.error);
    uint8_t prescaler[] = {1, 4, 16, 64};
    dbg("TWBR = 0x%x (%u), PreScaler = %u (index=%u)\n", params.twbr, params.twbr, prescaler[params.ps], params.ps);

    i2c_init(params);
    i2c_slave_init(address);

    reset_state();
}
