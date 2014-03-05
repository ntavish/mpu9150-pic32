#ifndef PIC32_I2C_H
#define PIC32_I2C_H

#include <plib.h>
#include <peripheral/i2c.h>

#define I2CDEV I2C1

typedef enum
{
    MASTER,
    SLAVE
}i2cmode;

void i2c_init(I2C_MODULE i2cnum, i2cmode mode, BYTE address);
BOOL i2c_begin_transmission(I2C_MODULE i2cnum, BOOL restart);
void i2c_end_transmission(I2C_MODULE i2cnum);
I2C_RESULT i2c_restart_transmission(I2C_MODULE i2cnum);
BOOL i2c_write_byte(I2C_MODULE i2cnum, BYTE data);
I2C_RESULT i2c_request_data(I2C_MODULE i2cnum);
inline BOOL i2c_is_available(I2C_MODULE i2cnum);
BYTE i2c_read_byte(I2C_MODULE i2cnum);
void i2c_nack(I2C_MODULE i2cnum);
void i2c_ack(I2C_MODULE i2cnum);
/*
 *	Utility macro for master to send address and rw bit
 *  address - 7bit address
 *  rw is boolean value for read/notwrite.
 *  read = 1, write = 0
 */
#define i2c_send_address(i2cnum, address, rw) i2c_write_byte(i2cnum, (unsigned char)((address<<1)|(rw*0x1)))

/*
 * functions using i2c ^ defined in main.c
 */
BOOL mpu_i2c_init();
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif