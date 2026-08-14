#include "I2CSlave.h"
#include "I2CSlave_state_machine.h"
#include <avr/pgmspace.h>
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
    READY,
    TALKING
} i2c_slaveSM_status_t;

static i2c_slaveSM_command_t *commands = NULL;
static unsigned int nb_commands = -1;
static i2c_slaveSM_command_t *current_command = NULL;

static i2c_slaveSM_status_t status;

static uint8_t *buffer = NULL;
unsigned int buffer_size;

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


void i2c_slaveSM_requested(i2c_request_t req_type) {
    switch(req_type) {
         case INITIAL:
            if (status == READY) {
                status = TALKING;
            } else {
                // we were not READY to reply, then forget all previous crap said by master
                dbg("Initial query but state is not READY (status == %d)\n", status);
                reset_state();
                break;
            }
        // NOTE : if no error => no break !! execute following statement
        case CONTINUATION:
            if (status != TALKING) {
                dbg("Requested to tranmit data, while not in TALKING state (status == %d)\n", status);
                reset_state();
                break;
            }
            if (buffer_index >= reply_len) {
                // abnormal situation, give a trace here
                dbg("ERR : buffer_index >= reply_len\n");
                reset_state();
                break;
            }
            dbg("send I2C data : %x %s\n", buffer[buffer_index], (buffer_index+1)==reply_len?"(last)":"");
            i2c_slave_transmitByte(buffer[buffer_index]);
            buffer_index++;
            if (buffer_index == reply_len) { // We're done !!
                dbg("Succesfully transmitted %d bytes\n", reply_len);
                reset_state();
            }
            break;

//         case DONE:
//             if (status != WAIT_DONE) {
//                 dbg("'warning !! got i2c request 'DONE', while status not DONE (status == %d)\n", status);
//             } else {
//                 dbg("Communication end.\n");
//             }
//             reset_state();
//             break;

        default:
            dbg("i2c_scale_requested : unknown req_type %d\n", req_type);
            reset_state();
            break;
    }
}



void i2c_slaveSM_receive(uint8_t data) {
    dbg("I2C received byte : %x\n", data);
    switch(status) {
        case WAITING:
            current_command = get_command(data);
            if (current_command != NULL) {
                buffer_index = 0;
                if (current_command->arg_len == 0) {
                    // no arg to get, just execute callback
                    reply_len = current_command->callback(buffer, buffer_size);
                    if (reply_len > 0) {
                        status = READY; // to send data
                        buffer_index = 0;
                    } else {
                        // nothing to send, we're done
                        reset_state();
                    }
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
                reply_len = current_command->callback(buffer, buffer_size);
                if (reply_len > 0) {
                    status = READY; // to send data
                    buffer_index = 0;
                } else {
                    reset_state();
                }
            }
            break;

        default:
            dbg("Got data while state not WAITING nor READING_ARGS (status == %d)\n", status);
            reset_state();
    }
}



void i2c_slaveSM_init(uint8_t address,
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
    i2c_slave_init(address);

    reset_state();
}
