/* 
 * File:   hardwareprofile.h
 * Author: tavish
 *
 * Created on February 27, 2014, 2:44 PM
 */

#ifndef HARDWAREPROFILE_H
#define	HARDWAREPROFILE_H

#ifdef	__cplusplus
extern "C" {
#endif


/* Microcontroller MIPs (FCY) */
#define SYS_FREQ     48000000L
#define FCY          SYS_FREQ
// bits per second (UART2: 8 bits, no parity, 1 stop bit)
#define BAUD_RATE	 115200

#define mInitAllLEDs()      LATAbits.LATA10=0; TRISAbits.TRISA10=0;
#define mLED_1              LATAbits.LATA10
#define mGetLED_1()         mLED_1
#define mLED_1_On()         mLED_1 = 1;
#define mLED_1_Off()        mLED_1 = 0;
#define mLED_1_Toggle()     mLED_1 = !mLED_1;

#define MPU_I2C I2C1

// for empl; use global define
#ifndef FOOTSENSE_TARGET_PIC32
#define FOOTSENSE_TARGET_PIC32
#endif
#ifndef MPU9150
#define MPU9150
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* HARDWAREPROFILE_H */

