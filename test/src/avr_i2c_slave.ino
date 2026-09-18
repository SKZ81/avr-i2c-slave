extern "C" {
#include "I2CSlave.h"
}

#define I2C_ADDRESS   0x20
#define I2C_FREQUENCY 100000

#define RX_BUFFER_SIZE 8
#define TX_BUFFER_SIZE 8


// ============================================================================
// Common
// ============================================================================

static void printHex(uint8_t value)
{
    if (value < 0x10)
        Serial.print('0');

    Serial.print(value, HEX);
}


static void printResult(uint8_t result)
{
    Serial.print(F(" -> "));

    switch (result) {
    case OK:
        Serial.println(F("OK"));
        break;

    case I2C_NOT_CONFIGURED:
        Serial.println(F("I2C_NOT_CONFIGURED"));
        break;

    case SLAVE_CALLBACKS_NOT_SET:
        Serial.println(F("SLAVE_CALLBACKS_NOT_SET"));
        break;

    case MASTER_NOT_INITED:
        Serial.println(F("MASTER_NOT_INITED"));
        break;

    case MASTER_TRM_SLA_NACKED:
        Serial.println(F("MASTER_TRM_SLA_NACKED"));
        break;

    case MASTER_TRM_STOPPED_BY_SLAVE:
        Serial.println(F("MASTER_TRM_STOPPED_BY_SLAVE"));
        break;

    case MASTER_RCV_SLA_NACKED:
        Serial.println(F("MASTER_RCV_SLA_NACKED"));
        break;

    case MASTER_RCV_ERROR:
        Serial.println(F("MASTER_RCV_ERROR"));
        break;

    default:
        Serial.print(F("ERROR "));
        Serial.println(result);
        break;
    }
}


static void printRoleMenu()
{
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F(" I2CSlave test"));
    Serial.println(F("=============================="));
    Serial.println(F("Select role:"));
    Serial.println(F("  m - master"));
    Serial.println(F("  s - slave"));
    Serial.println();
}


// ============================================================================
// MASTER
// ============================================================================

static void masterWrite(uint8_t count)
{
    Serial.print(F("WRITE "));
    Serial.print(count);
    Serial.println(F(" bytes"));

    uint8_t result;

    result = i2c_master_init(I2C_ADDRESS, MasterTransmit);

    if (result != OK) {
        printResult(result);
        i2c_master_done();
        return;
    }

    for (uint8_t i = 0; i < count; ++i) {

        uint8_t data = 0xA0 + i;
        bool last = (i == count - 1);

        Serial.print(F("  TX "));
        printHex(data);
        Serial.print(F(" last="));
        Serial.println(last ? F("true") : F("false"));

        result = i2c_master_write(data, last);

        if (result != OK) {
            printResult(result);
            break;
        }
    }

    i2c_master_done();

    if (result == OK)
        Serial.println(F("WRITE completed"));
}


static void masterRead(uint8_t count)
{
    Serial.print(F("READ "));
    Serial.print(count);
    Serial.println(F(" bytes"));

    uint8_t result;

    result = i2c_master_init(I2C_ADDRESS, MasterReceive);

    if (result != OK) {
        printResult(result);
        i2c_master_done();
        return;
    }

    for (uint8_t i = 0; i < count; ++i) {

        uint8_t data = 0;
        bool last = (i == count - 1);

        result = i2c_master_read(&data, last);

        Serial.print(F("  RX "));
        printHex(data);
        Serial.print(F(" last="));
        Serial.println(last ? F("true") : F("false"));

        if (result != OK) {
            printResult(result);
            break;
        }
    }

    i2c_master_done();

    if (result == OK)
        Serial.println(F("READ completed"));
}


static void printMasterMenu()
{
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F(" MASTER"));
    Serial.println(F("=============================="));
    Serial.println(F("WRITE"));
    Serial.println(F("  1 - 0 bytes"));
    Serial.println(F("  2 - 2 bytes"));
    Serial.println(F("  3 - 4 bytes"));
    Serial.println(F("  4 - 5 bytes"));
    Serial.println();
    Serial.println(F("READ"));
    Serial.println(F("  5 - 2 bytes"));
    Serial.println(F("  6 - 4 bytes"));
    Serial.println(F("  7 - 6 bytes"));
    Serial.println();
    Serial.println(F("  m - menu"));
    Serial.println();
}


static void masterLoop()
{
    if (!Serial.available())
        return;

    char command = Serial.read();

    switch (command) {

    case '1':
        masterWrite(0);
        break;

    case '2':
        masterWrite(2);
        break;

    case '3':
        masterWrite(4);
        break;

    case '4':
        masterWrite(5);
        break;

    case '5':
        masterRead(2);
        break;

    case '6':
        masterRead(4);
        break;

    case '7':
        masterRead(6);
        break;

    case 'm':
    case 'M':
        printMasterMenu();
        break;

    default:
        break;
    }
}


// ============================================================================
// SLAVE
// ============================================================================

static volatile uint8_t rx_count = 0;
static volatile uint8_t tx_count = 0;

static uint8_t rx_max = 4;
static uint8_t tx_length = 4;


/*
 * Master -> Slave
 */

static void slaveReceiveStart()
{
    rx_count = 0;
}


static bool slaveReceive(uint8_t data)
{
    /*
     * Store received data.
     *
     * Returning true means:
     *   "this was the last data I want to receive"
     *
     * Therefore the next byte sent by the master will be NACKed.
     */
    if (rx_count < RX_BUFFER_SIZE)
        rx_count++;

    return (rx_count >= rx_max);
}


/*
 * Slave -> Master
 */

static bool slaveRequest(i2c_request_t request)
{
    switch (request) {

    case INITIAL:
        tx_count = 0;

    case CONTINUATION:

        if (tx_count < tx_length) {
            i2c_slave_transmitByte(0x10 + tx_count);
            tx_count++;
        }

        return (tx_count >= tx_length);


    case DONE:
        /*
         * The master has sent NACK (or the slave has otherwise
         * reached the end of the transaction).
         */
        return false;
    }

    return false;
}


static void printSlaveStatus()
{
    /*
     * These values are volatile because they are modified from ISR context.
     */
    uint8_t rx = rx_count;
    uint8_t tx = tx_count;

    Serial.print(F("RX count = "));
    Serial.println(rx);

    Serial.print(F("TX count = "));
    Serial.println(tx);
}


static void printSlaveMenu()
{
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F(" SLAVE"));
    Serial.println(F("=============================="));
    Serial.print(F("RX currently accepts "));
    Serial.print(rx_max);
    Serial.println(F(" bytes"));
    Serial.print(F("TX currently provides "));
    Serial.print(tx_length);
    Serial.println(F(" bytes"));
    Serial.println();
    Serial.println(F("RX tests (master -> slave)"));
    Serial.println(F("  1 - accept 2 bytes"));
    Serial.println(F("  2 - accept 4 bytes"));
    Serial.println(F("  3 - accept 6 bytes"));
    Serial.println();
    Serial.println(F("TX tests (slave -> master)"));
    Serial.println(F("  4 - provide 2 bytes"));
    Serial.println(F("  5 - provide 4 bytes"));
    Serial.println(F("  6 - provide 6 bytes"));
    Serial.println();
    Serial.println(F("  7 - show counters"));
    Serial.println(F("  m - menu"));
    Serial.println();
}


static void slaveLoop()
{
    if (!Serial.available())
        return;

    char command = Serial.read();

    switch (command) {

    case '1':
        rx_max = 2;
        Serial.println(F("Slave RX: 2 bytes"));
        break;

    case '2':
        rx_max = 4;
        Serial.println(F("Slave RX: 4 bytes"));
        break;

    case '3':
        rx_max = 6;
        Serial.println(F("Slave RX: 6 bytes"));
        break;

    case '4':
        tx_length = 2;
        Serial.println(F("Slave TX: 2 bytes"));
        break;

    case '5':
        tx_length = 4;
        Serial.println(F("Slave TX: 4 bytes"));
        break;

    case '6':
        tx_length = 6;
        Serial.println(F("Slave TX: 6 bytes"));
        break;

    case '7':
        printSlaveStatus();
        break;

    case 'm':
    case 'M':
        printSlaveMenu();
        break;

    default:
        break;
    }
}


// ============================================================================
// Setup / loop
// ============================================================================

enum Role {
    ROLE_NONE,
    ROLE_MASTER,
    ROLE_SLAVE
};

static Role role = ROLE_NONE;


void setup()
{
    Serial.begin(115200);

    delay(100);

    Serial.println();
    Serial.println(F("I2CSlave test firmware"));

    /*
     * Both master and slave use the same I2C initialization.
     */
    i2c_init(
        i2c_baud_getparams(F_CPU, I2C_FREQUENCY)
    );

    printRoleMenu();

    /*
     * Wait for role selection.
     *
     * This is deliberately blocking: we don't want the I2C peripheral
     * configured as slave before the user has selected the role.
     */
    while (role == ROLE_NONE) {

        if (!Serial.available())
            continue;

        char c = Serial.read();

        if (c == 'm' || c == 'M') {
            role = ROLE_MASTER;

            Serial.println(F("Selected MASTER"));
            printMasterMenu();
        }

        else if (c == 's' || c == 'S') {
            role = ROLE_SLAVE;

            Serial.println(F("Selected SLAVE"));

            i2c_slave_setCallbacks(
                slaveReceiveStart,
                slaveReceive,
                slaveRequest
            );

            uint8_t result = i2c_slave_init(I2C_ADDRESS);

            Serial.print(F("i2c_slave_init"));
            printResult(result);

            printSlaveMenu();
        }
    }
}


void loop()
{
    if (role == ROLE_MASTER)
        masterLoop();

    else if (role == ROLE_SLAVE)
        slaveLoop();
}

