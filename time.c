#include <plib.h>
#include "time.h"

/********************************************************************
 * Function:        void enhanced_ct_since(uint32_t tick)
 *
 *
 * Note:            None
 *******************************************************************/
uint32_t enhanced_ct_since(uint32_t tick)
{
	uint32_t cur=ReadCoreTimer();
	if(tick < cur)
		return cur-tick;
	else
		return 0xffffffff-tick+cur;
}

int delay(unsigned long num_ms)
{
	uint32_t delay_var=ReadCoreTimer();
	while(enhanced_ct_since(delay_var)<MS_TO_CORE_TICKS(num_ms));
	return 0;
}

int get_ms(unsigned long *count)
{
	count[0] = ReadCoreTimer()/MS_TO_CORE_TICKS(1);
	return 0;
}