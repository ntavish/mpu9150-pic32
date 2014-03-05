/* 
 * File:   time.h
 * Author: tavish
 *
 * Created on February 27, 2014, 2:43 PM
 */

#ifndef TIME_H
#define	TIME_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hardwareprofile.h"

#define TOGGLES_PER_SEC		1
#define CORE_TICK_RATE	    (SYS_FREQ/2/TOGGLES_PER_SEC)

#define ONE_SECOND (SYS_FREQ/2)						// 1s of PIC32 core timer ticks (== Hz)
#define MS_TO_CORE_TICKS(x) ((UINT64)(x)*ONE_SECOND/1000)
#define CT_TICKS_SINCE(tick) (ReadCoreTimer() - (tick))								// number of core timer ticks since "tick"

extern uint32_t delay_var;
extern uint32_t enhanced_ct_since(uint32_t tick);

// useful ones
int delay(unsigned long num_ms);
int millis(unsigned long *count);

#ifdef	__cplusplus
}
#endif

#endif	/* TIME_H */

