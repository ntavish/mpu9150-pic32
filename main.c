/* 
 * File:   main.c
 * Author: tavish
 *
 * Created on February 26, 2014, 11:57 AM
 */

#ifdef __XC32
    #include <xc.h>          /* Defines special funciton registers, CP0 regs  */
#endif

#include <plib.h>           /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */

#include "GenericTypeDefs.h"
#include "device_i2c.h"
#include "console.h"
#include "time.h"
#include "hardwareprofile.h"

//empl
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (50)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC,
    PACKET_TYPE_COMPASS,
};

// CONFIG BITS
// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_4         // PLL Input Divider (4x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC4/PGED4)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#define MPU_ADDRESS (0x68)
#define MPUREG_WHO_AM_I 0x75

void (*mpu_cb)(void)=NULL;


void send_packet(char packet_type, void *data);
static void handle_input(char c);
int mpu_interrupt_setup(void (*cb)(void));
static void setup_gyro(void);

void imu_test1();
void imu_raw_readings_test();

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
	Printf("Gyro reading\r\n");
    hal.new_gyro = 1;
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    send_packet(PACKET_TYPE_TAP, data);
	Printf("Tap detected\r\n");
}

static void android_orient_cb(unsigned char orientation)
{
    send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
	Printf("Android orient");
}


int32_t main()
{
	uint8_t data;
	
	//if bootloader is used, needed for interrupts to work.
//	INTSetEBASE(0x9D006000);

	SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL);

	ConsoleInit();

	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	INTEnableInterrupts();

	mInitAllLEDs();

	mpu_i2c_init();
	
	i2c_read(MPU_ADDRESS, MPUREG_WHO_AM_I, 1, (BYTE *)&data);

//	Printf("WHO_AM_I: ");
//	PrintChar(data);
//	Printf("\r\n");

	{
	////////////IMU TEST CODE ////////////////////////

		imu_test1();

	//////////////////////////////////////////////////
    }
	
	while(1);
}

//no compass
void imu_test1()
{
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

	mpu_interrupt_setup(NULL);
	INTEnable(INT_INT1, INT_DISABLED);
	
	/* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    res(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS));
    /* Push both gyro and accel data into the FIFO. */
    res(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    res(mpu_set_sample_rate(DEFAULT_MPU_HZ));
	res(mpu_set_gyro_fsr(2000));
	res(mpu_set_accel_fsr(2));
    /* Read back configuration in case it was set improperly. */
    res(mpu_get_sample_rate(&gyro_rate));
    res(mpu_get_gyro_fsr(&gyro_fsr));
//	printf("gyro fsr  %u\r\n", gyro_fsr);
    res(mpu_get_accel_fsr(&accel_fsr));
//	printf("accel fsr  %u\r\n", accel_fsr);
	res(mpu_set_lpf(0));

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON ;
    hal.report = PRINT_QUAT;

	res(dmp_load_motion_driver_firmware());
    res(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));
    res(dmp_register_tap_cb(tap_cb));
    res(dmp_register_android_orient_cb(android_orient_cb));
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
    res(dmp_enable_feature(hal.dmp_features));
    res(dmp_set_fifo_rate(DEFAULT_MPU_HZ));
    res(mpu_set_dmp_state(1));
    hal.dmp_on = 1;
	
	INTEnable(INT_INT1, INT_ENABLED);

	while (1) {
        unsigned long sensor_timestamp;
//        if (count_out != 0)
//            /* A byte has been received via USB. See handle_input for a list of
//             * valid commands.
//             */
//        {
//            handle_input();
//            count_out = 0; // received data has been processed
//				}

        if (hal.motion_int_mode) {
            /* Enable motion interrupt. */
            mpu_lp_motion_interrupt(500, 1, 5);
            hal.new_gyro = 0;
            /* Wait for the MPU interrupt. */
            while (!hal.new_gyro){
						//Write code for Low Power Mode (LPM)
						}
            /* Restore the previous sensor configuration. */
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro) {
            /* Put the stm32 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */
            //continue;
        }

        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel[3], sensors;
            unsigned char more;
            long quat[4];
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);
            if (!more)
                hal.new_gyro = 0;
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
                send_packet(PACKET_TYPE_QUAT, quat);
        } else if (hal.new_gyro) {
            short gyro[3], accel[3];
            unsigned char sensors, more;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO.
             */
            mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
        }
    }

}


/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
void send_packet(char packet_type, void *data)
{
#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH], length;

    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO || packet_type == PACKET_TYPE_COMPASS) {
        short *sdata = (short*)data;
        buf[2] = (char)(sdata[0] >> 8);
        buf[3] = (char)sdata[0];
        buf[4] = (char)(sdata[1] >> 8);
        buf[5] = (char)sdata[1];
        buf[6] = (char)(sdata[2] >> 8);
        buf[7] = (char)sdata[2];
        length = 8;
    } else if (packet_type == PACKET_TYPE_QUAT) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        buf[10] = (char)(ldata[2] >> 24);
        buf[11] = (char)(ldata[2] >> 16);
        buf[12] = (char)(ldata[2] >> 8);
        buf[13] = (char)ldata[2];
        buf[14] = (char)(ldata[3] >> 24);
        buf[15] = (char)(ldata[3] >> 16);
        buf[16] = (char)(ldata[3] >> 8);
        buf[17] = (char)ldata[3];
		
		printf("q: %02x%02x%02x%02x ", (uint8_t)buf[2], (uint8_t)buf[3], (uint8_t)buf[4], (uint8_t)buf[5]);
		printf("%02x%02x%02x%02x ", (uint8_t)buf[6], (uint8_t)buf[7], (uint8_t)buf[8], (uint8_t)buf[9]);
		printf("%02x%02x%02x%02x ", (uint8_t)buf[10], (uint8_t)buf[11], (uint8_t)buf[12], (uint8_t)buf[13]);
		printf("%02x%02x%02x%02x\r\n", (uint8_t)buf[14], (uint8_t)buf[15], (uint8_t)buf[16], (uint8_t)buf[17]);
//		{
//			int i;
//			for(i=0;i<16;i++) WriteUART2(buf[2+i]);
//		}
        length = 18;
    } else if (packet_type == PACKET_TYPE_TAP) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        length = 4;
    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
        buf[2] = ((char*)data)[0];
        length = 3;
    } else if (packet_type == PACKET_TYPE_PEDO) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        length = 10;
    } else if (packet_type == PACKET_TYPE_MISC) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        buf[4] = ((char*)data)[2];
        buf[5] = ((char*)data)[3];
        length = 6;
    }

	// not using send_packet
	{
		int i;
//		for(i=0;i<length;i++) WriteUART2(buf[i]);
	}

}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

// not using handle input right now
static void handle_input(char c)
{
    const unsigned char header[3] = "inv";
    unsigned long pedo_packet[2];
		
    switch (hal.rx.cmd) {
    /* These commands turn the hardware sensors on/off. */
    case '8':
        if (!hal.dmp_on) {
            /* Accel and gyro need to be on for the DMP features to work
             * properly.
             */
            hal.sensors ^= ACCEL_ON;
            setup_gyro();
        }
        break;
    case '9':
        if (!hal.dmp_on) {
            hal.sensors ^= GYRO_ON;
            setup_gyro();
        }
        break;
    /* The commands start/stop sending data to the client. */
    case 'a':
        hal.report ^= PRINT_ACCEL;
        break;
    case 'g':
        hal.report ^= PRINT_GYRO;
        break;
    case 'q':
        hal.report ^= PRINT_QUAT;
        break;
    case 'c':
        hal.report ^= PRINT_COMPASS;
        break;
    /* The hardware self test can be run without any interaction with the
     * MPL since it's completely localized in the gyro driver. Logging is
     * assumed to be enabled; otherwise, a couple LEDs could probably be used
     * here to display the test results.
     */
    case 't':
//        run_self_test();
        break;
    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These commands can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
    case '1':
        if (hal.dmp_on)
            dmp_set_fifo_rate(10);
        else
            mpu_set_sample_rate(10);
        break;
    case '2':
        if (hal.dmp_on)
            dmp_set_fifo_rate(20);
        else
            mpu_set_sample_rate(20);
        break;
    case '3':
        if (hal.dmp_on)
            dmp_set_fifo_rate(40);
        else
            mpu_set_sample_rate(40);
        break;
    case '4':
        if (hal.dmp_on)
            dmp_set_fifo_rate(50);
        else
            mpu_set_sample_rate(50);
        break;
    case '5':
        if (hal.dmp_on)
            dmp_set_fifo_rate(100);
        else
            mpu_set_sample_rate(100);
        break;
    case '6':
        if (hal.dmp_on)
            dmp_set_fifo_rate(200);
        else
            mpu_set_sample_rate(200);
        break;
	case ',':
        /* Set hardware to interrupt on gesture event only. This feature is
         * useful for keeping the MCU asleep until the DMP detects as a tap or
         * orientation event.
         */
        dmp_set_interrupt_mode(DMP_INT_GESTURE);
        break;
    case '.':
        /* Set hardware to interrupt periodically. */
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
        break;
    case '7':
        /* Reset pedometer. */
        dmp_set_pedometer_step_count(0);
        dmp_set_pedometer_walk_time(0);
        break;
    case 'f':
        /* Toggle DMP. */
        if (hal.dmp_on) {
            unsigned short dmp_rate;
            hal.dmp_on = 0;
            mpu_set_dmp_state(0);
            /* Restore FIFO settings. */
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* When the DMP is used, the hardware sampling rate is fixed at
             * 200Hz, and the DMP is configured to downsample the FIFO output
             * using the function dmp_set_fifo_rate. However, when the DMP is
             * turned off, the sampling rate remains at 200Hz. This could be
             * handled in inv_mpu.c, but it would need to know that
             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
             * put the extra logic in the application layer.
             */
            dmp_get_fifo_rate(&dmp_rate);
            mpu_set_sample_rate(dmp_rate);
        } else {
            unsigned short sample_rate;
            hal.dmp_on = 1;
            /* Both gyro and accel must be on. */
            hal.sensors |= ACCEL_ON | GYRO_ON;
            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* Preserve current FIFO rate. */
            mpu_get_sample_rate(&sample_rate);
            dmp_set_fifo_rate(sample_rate);
            mpu_set_dmp_state(1);
        }
        break;
    case 'm':
        /* Test the motion interrupt hardware feature. */
        hal.motion_int_mode = 1;
        break;
    case 'p':
        /* Read current pedometer count. */
        dmp_get_pedometer_step_count(pedo_packet);
        dmp_get_pedometer_walk_time(pedo_packet + 1);
        send_packet(PACKET_TYPE_PEDO, pedo_packet);
        break;
    case 'x':
//        stm32_reset();
        break;
    case 'v':
        /* Toggle LP quaternion.
         * The DMP features can be enabled/disabled at runtime. Use this same
         * approach for other features.
         */
        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
        dmp_enable_feature(hal.dmp_features);
        break;
    default:
        break;
    }
		
}

// sets up mpu interrupt pin
int mpu_interrupt_setup(void (*cb)(void))
{
	//attach INT1 to RC4
	TRISCbits.TRISC4 = 1;
	INT1Rbits.INT1R  = 0b0111;
	CNPDCbits.CNPDC4 = 1;
	INTSetVectorPriority(INT_EXTERNAL_1_VECTOR, INT_PRIORITY_LEVEL_1);
	INTSetVectorSubPriority(INT_EXTERNAL_1_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
	mINT1SetEdgeMode(1); //rising edge
	INTClearFlag(INT_INT1);
	if(cb)
	{
		mpu_cb = cb;
		INTEnable(INT_INT1, INT_ENABLED);
	}
	else
	{
		mpu_cb = NULL;
		INTEnable(INT_INT1, INT_DISABLED);
	}
	return 0;
}

void __ISR(_EXTERNAL_1_VECTOR, ipl1) _INT1Interrupt(void)
{
	//not using this callback
	if(mpu_cb)
	{
		mpu_cb();
	}
	hal.new_gyro = 1;
	mLED_1_Toggle();
	INTClearFlag(INT_INT1);
}