/* I2C Echo Example */
#include "I2CSlave.h"
#include <stdlib.h>

#define I2C_ADDR 0x10
#define I2C_FREQ 100000
#define MAX_DATA 16

uint8_t data[MAX_DATA];
uint8_t data_index = 0;

bool I2C_received(uint8_t received_data)
{
  data[data_index++] = received_data;
  return data_index == MAX_DATA - 1; // always up to MAX_DATA bytes
}

bool I2C_requested(i2c_request_t request)
{
  bool last_data = false;
  switch(request) {
      case INITIAL:
          data_index = 0;
          // no break, start transmiting
      case CONTINUATION:
          i2c_slave_transmitByte(data[data_index++]);
          last_data = (data_index == MAX_DATA - 1);
          break;
      case DONE:
          data_index = 0;
          break;
  }
  return last_data;
}

void setup()
{
  // set received/requested callbacks
  i2c_slave_setCallbacks(NULL, I2C_received, /*NULL,*/ I2C_requested);

  i2c_init(i2c_baud_getparams(F_CPU, I2C_FREQ));
  // init I2C
  i2c_slave_init(I2C_ADDR);
}

int main()
{
  setup();

  // Main program loop
  while(1) {
      //ping 0x20
      i2c_master_init(0x20, MasterTransmit);
      i2c_master_done();
  }
}
