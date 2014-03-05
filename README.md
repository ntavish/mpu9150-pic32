# MPU9150 DMP code for pic32

## Hardware

[Sparkfun pmu9150 breakout](https://www.sparkfun.com/products/11486)

[Fubarino mini pic32mx250f128d](http://fubarino.org/mini/)

|Breakout pin	|Pic32mxpin	|Fubarino silk name|
|---------------|---------------|------------------|
|INT  		|RC4  		|Pin 20/A12	   |
|SDA 		|RB9		|Pin 26		   |
|SCL 		|RB8		|Pin 25		   |

### Other pins:
Serial Tx - Uart2 pins - RB0 - Connect to serial converter for now.

## Overview
Initial code to get quaternion output from DMP. Outputs quaternion values 
as hexadecimal strings like `q: 1d7b8d3f ff686626 ff8e7e00 38ccc5a4`.
The format for each part is big-endian long which python struct can read as
```python
struct.unpack('>l', "ff686626".split[j].decode('hex'))[0]
```
Can divide all parts by 1073741824.0(1<<30), or normalize quaternion before use.

The function send_packet is not used, but it sends raw gyro, accel readings 
fine when configured.

There is a `test.py` application in `desktop/`, which draws a cube rotating 
cube according to quaternion output it recieved from serial port.

# Notes

Compiler is `gcc version 4.5.2 MPLAB XC32 Compiler v1.20 (Microchip Technology)`. Have 
`FOOTSENSE_TARGET_PIC32` and `MPU9150` in global preprocessor defines.