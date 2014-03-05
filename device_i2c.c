#include "device_i2c.h"
#include <peripheral/i2c.h>
#include "hardwareprofile.h"

/*
 * Initialize i2c module:
 * I2C clock is 100KHz
 * If mode parameter is SLAVE, uses address to set slave address for the module
 * Enable module
 */
void i2c_init(I2C_MODULE i2cnum, i2cmode mode, BYTE address)
{
	//enabling i2c module doesnt need changing port
	//direction/value etc, and is not a pin muxed peripheral

	I2CConfigure ( i2cnum, I2C_ENABLE_SLAVE_CLOCK_STRETCHING);
	I2CSetFrequency ( i2cnum, 48000000, 300000);

	if(mode == SLAVE)
	{
		//address mask is set to 0
		I2CSetSlaveAddress ( i2cnum, address&0x7f, 0, I2C_USE_7BIT_ADDRESS );
	}

	I2CEnable(i2cnum, TRUE);
}

/*
 * Assert start condition
 * if restart, then assert repeat transmission
 * (Change direction of data without ending transmission)
 * Used by:master
 */
BOOL i2c_begin_transmission(I2C_MODULE i2cnum, BOOL restart)
{
	I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(i2cnum);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(i2cnum) );

        if(I2CStart(i2cnum) != I2C_SUCCESS)
        {
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(i2cnum);

    } while ( !(status & I2C_START) );

    return TRUE;

}

/*
 * Assert stop condition
 * Used by: master
 */
void i2c_end_transmission(I2C_MODULE i2cnum)
{
	I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(i2cnum);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(i2cnum);

    } while ( !(status & I2C_STOP) );
}

/*
 *
 * Used by:master
 */
I2C_RESULT i2c_restart_transmission(I2C_MODULE i2cnum)
{
	BYTE status;
	I2CRepeatStart(i2cnum);
	// Wait for the signal to complete
    do
    {
        status = I2CGetStatus(i2cnum);

    } while ( !(status & I2C_START) );

    return TRUE;
}

/*
 * Send byte of data, wait for ack. Transfer must have been started.
 * Used by: master,slave
 */
BOOL i2c_write_byte(I2C_MODULE i2cnum, BYTE data)
{
	while(!I2CTransmitterIsReady(i2cnum));
	if(I2CSendByte(i2cnum, data)!=I2C_SUCCESS)
		return FALSE;

	while(!I2CTransmissionHasCompleted(i2cnum));
    if (I2CByteWasAcknowledged(i2cnum))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*
 * A transfer must have been initiated prior to this
 * Used by: master, slave
 */
I2C_RESULT i2c_request_data(I2C_MODULE i2cnum)
{
	while( I2CReceiverEnable ( i2cnum, TRUE) != I2C_SUCCESS )
	{
		I2CClearStatus ( i2cnum, I2C_RECEIVER_OVERFLOW);

	}

	return I2C_SUCCESS;
}

/*
 * A transfer must have been initiated
 * Used by:master,slave
 */
inline BOOL i2c_is_available(I2C_MODULE i2cnum)
{
	return I2CReceivedDataIsAvailable(i2cnum);
}


/*
 * Get byte of data from i2c bus
 * Used by:master,slave
 */
BYTE i2c_read_byte(I2C_MODULE i2cnum)
{
	if(I2CReceiverEnable(i2cnum, TRUE) == I2C_RECEIVE_OVERFLOW)
	{
		//todo
		return 0;
	}
	else
	{
		while(!I2CReceivedDataIsAvailable(i2cnum));
		return I2CGetByte(i2cnum);
	}

}

void i2c_ack(I2C_MODULE i2cnum)
{
	I2CAcknowledgeByte(i2cnum, TRUE);
	while ( !I2CAcknowledgeHasCompleted(i2cnum) );
}

void i2c_nack(I2C_MODULE i2cnum)
{
	I2CAcknowledgeByte(i2cnum, FALSE);
	while ( !I2CAcknowledgeHasCompleted(i2cnum) );
}

/* slave clock stretching handled automatically */
/*
	Sending Data to slave
	---------------------
	master sends start sequence
	send address of slave with r/w low
	send internal register number you master wants to write to
	send data
	...can send more data here
	send stop sequence
*/


/*
	Reading data from slave
	---------------------
	Send start sequence
	send address of slave with r/w low
	send internal register number you master wants to read from
	Send start sequence again OR restart transmission
	send address of slave with r/w high
	read data byte
	send stop sequence
*/


BOOL mpu_i2c_init()
{
	//open i2cin master mode
	i2c_init(MPU_I2C, MASTER, 0);
}

// TODO timeouts in waiting for bus free

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	BYTE i;

	i2c_begin_transmission(MPU_I2C, FALSE);

	i2c_send_address(MPU_I2C, slave_addr, 0);

	i2c_write_byte(MPU_I2C, reg_addr);

	i2c_restart_transmission(MPU_I2C);

	i2c_send_address(MPU_I2C, slave_addr, 1);

	I2CReceiverEnable ( I2C1, TRUE);
	for(i=0;i<length;i++)
	{
		data[i] = i2c_read_byte(MPU_I2C);
		if(i<(length-1)) // nack on last byte
		{
			i2c_ack(MPU_I2C);
		}
		else
		{
			i2c_nack(MPU_I2C);
		}
	}

	i2c_end_transmission(MPU_I2C);

	return 0;
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
	BYTE i;
	i2c_begin_transmission(MPU_I2C, FALSE);

	i2c_send_address(MPU_I2C, slave_addr, 0);

	i2c_write_byte(MPU_I2C, reg_addr);

	// DO NOT restart transmission, or start condition.

	for(i=0;i<length;i++)
	{
		i2c_write_byte(MPU_I2C, data[i]);
	}

	i2c_end_transmission(MPU_I2C);

	return 0;
}
