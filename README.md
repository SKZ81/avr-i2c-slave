# AVR I2C Slave Library

## Work in progess

I'm bringing this lib to full TWI peripheral control, meaning full either :
- Master
- Slave
- Master/Slave (meaning multi-master bus configuration)

## State of the art (i²c slave doc)

This library provides interrupt-based I2C slave functionality for Atmel 8-bit microcontrollers equipped with a TWI peripheral. 

It is somewhat based on an existing library found here: https://github.com/devthrash/I2C-slave-lib

Files
-----
* `I2CSlave.c` -- Implements init/stop, and interrupt-based receive and request logic
* `I2CSlave.h` -- Function prototypes and a convience method for transmitting data

Usage
-----
**Provide callbacks for receiving and handling requests in your application code.**
eg:
```c
void I2C_received(uint8_t data);
void I2C_requested();

i2c_slave_setCallbacks(I2C_received, I2C_requested);
```

The library calls the received callback *for each byte* the master transmits to the slave.
The library calls the requested callback *for each byte* the master attempts to read from the slave.

**Init the I2C slave with the slave address**
```c
i2c_slave_init(I2C_ADDRESS);
```

**Transmitting data to the master when requested**
```c
i2c_slave_transmitByte(data);
```

Example
-------

The example in `main.c` implements a sample application in which the slave can receive a byte, 
and then will echo the byte when requested.

To compile:
`make`


## I²C Slave State Machine

It provides a higher-level API, with "standardized" implementation of I2C_received() and I2C_requested() as well as a state machine to receive master commands and arguments, and transmit answers.

The client application has to:

### Include state machine header
```c
#include "I2CSlave_state_machine.h"
```

### define a command array
The command array is declaring:
* the command code
* number of bytes to read (variable length not managed)
* the callback to process the command

eg :
```c
i2c_slaveSM_command_t commands = {
    {COMMAND_ID1, 0, do_command1}, // a command with no argument, returning 1 byte as a response
    {COMMAND_ID2, 2, do_command2}, // a command with 2 bytes as argument, no response
    {...}
};
```

The callback (like do_command2) is passed the buffer with stored arguments from master, process it, and store answer in the same buffer.
It return the response length, or -1 in case of failure.

*NOTE*: The slave is put in busy state while processing the command.

### Declare a buffer

large enough to store arguments AND response for ANY command.

### Initialize the SM
```c
i2c_slaveSM_init(I2C_address,
                 commands, nb_commands,
                 buffer, buffer_size);
```

NOTE: in the call from client app nb_commands can be substituted by `sizeof(commands)/sizeof(i2c_slaveSM_command_t)`

### And that's all done
Then, the app can completely "forgets" about i²c "low-level" management.
