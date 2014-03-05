#include <plib.h>
#include "hardwareprofile.h"

const unsigned char CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void ConsoleInit(void)
{
	ANSELB=0x00;
	//UART2 tx RPB0
	TRISBbits.TRISB0=0;
	RPB0Rbits.RPB0R = 0b0010;
	
	OpenUART2(UART_EN, (1 << 12)|UART_TX_ENABLE, (SYS_FREQ/(1<<mOSCGetPBDIV())/16)/BAUD_RATE-1);
}

void ConsolePut(BYTE c)
{
    while(U2STAbits.TRMT == 0);
    U2TXREG = c;
}

void PrintChar(BYTE toPrint)
{
    BYTE PRINT_VAR;
    PRINT_VAR = toPrint;
    toPrint = (toPrint>>4)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    toPrint = (PRINT_VAR)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    return;
}

void PrintDec(BYTE toPrint)
{
    ConsolePut(CharacterArray[toPrint/10]);
    ConsolePut(CharacterArray[toPrint%10]);
}

void ConsolePutROMString(const char* str)
{
    BYTE c;

    while( (c = *str++) )
        ConsolePut(c);
}

void ConsoleWriteBuffer(BYTE *buf, unsigned int length)
{
	unsigned int i = 0;
	while(i<length)
	{
		ConsolePut(buf[i++]);
	}
}