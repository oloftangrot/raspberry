/*
 *    LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    This permission notice shall be included in all copies or
 *    substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>


#include "LedControl.h"

//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

/* Binary constant generator macro
   By Tom Torfs - donated to the public domain
*/
/* All macro's evaluate to compile-time constants */
/* *** helper macros ***/
/* turn a numeric literal into a hex constant
   (avoids problems with leading zeroes)
   8-bit constants max value 0x11111111, always fits in unsigned long
*/
#define HEX__(n) 0x##n##LU
/* 8-bit conversion function */
#define B8__(x) ((x&0x0000000FLU)?1:0)        \
               +((x&0x000000F0LU)?2:0)        \
               +((x&0x00000F00LU)?4:0)        \
               +((x&0x0000F000LU)?8:0)        \
               +((x&0x000F0000LU)?16:0)        \
               +((x&0x00F00000LU)?32:0)        \
               +((x&0x0F000000LU)?64:0)        \
               +((x&0xF0000000LU)?128:0)
/* *** user macros ***/
/* for upto 8-bit binary constants */
#define B8(d) ((unsigned char)B8__(HEX__(d)))
/* for upto 16-bit binary constants, MSB first */
#define B16(dmsb,dlsb) (((unsigned short)B8(dmsb)<<8)        \
                        + B8(dlsb))
/* for upto 32-bit binary constants, MSB first */
#define B32(dmsb,db2,db3,dlsb) (((unsigned long)B8(dmsb)<<24)         \
                                  + ((unsigned long)B8(db2)<<16) \
                                  + ((unsigned long)B8(db3)<<8)         \
                                  + B8(dlsb))

const static uint8_t charTable[128] = {
    B8(01111110),B8(00110000),B8(01101101),B8(01111001),B8(00110011),B8(01011011),B8(01011111),B8(01110000),
    B8(01111111),B8(01111011),B8(01110111),B8(00011111),B8(00001101),B8(00111101),B8(01001111),B8(01000111),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(10000000),B8(00000001),B8(10000000),B8(00000000),
    B8(01111110),B8(00110000),B8(01101101),B8(01111001),B8(00110011),B8(01011011),B8(01011111),B8(01110000),
    B8(01111111),B8(01111011),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(01110111),B8(00011111),B8(00001101),B8(00111101),B8(01001111),B8(01000111),B8(00000000),
    B8(00110111),B8(00000000),B8(00000000),B8(00000000),B8(00001110),B8(00000000),B8(00000000),B8(00000000),
    B8(01100111),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00001000),
    B8(00000000),B8(01110111),B8(00011111),B8(00001101),B8(00111101),B8(01001111),B8(01000111),B8(00000000),
    B8(00110111),B8(00000000),B8(00000000),B8(00000000),B8(00001110),B8(00000000),B8(00000000),B8(00000000),
    B8(01100111),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),
    B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000),B8(00000000)
};

static    uint8_t status[64];     /* We keep track of the led-status for all 8 devices in this array */
static    int maxDevices;    /* The maximum number of devices we use */
static    int fd; // File deveice number.

static    void ledControlSpiTransfer( unsigned int addr, uint8_t opcode, uint8_t data);     /* Send out a single command to the device */
static    const char *device = "/dev/spidev0.0";
static    uint8_t mode;
static    uint8_t bits = 8;
static    uint32_t speed = 500000;
static    uint8_t delay = 0;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

void ledControlInit( unsigned int numDevices )
{
  int ret, i;

  if ( numDevices > 8 ) numDevices = 8;
  maxDevices = numDevices;

  fd = open(device, O_RDWR);
  if ( fd < 0 )
    pabort("can't open device");

  mode = 0; // SPI_CPHA; // | SPI_CPOL;
  ret = ioctl( fd, SPI_IOC_WR_MODE, &mode ); // Set spi mode.
  if (ret == -1) {
    pabort("can't set spi mode");
  }
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits); // Set bits per word.
  if (ret == -1) {
    pabort("can't set bits per word");
  }
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); // Set maximum speed.
  if (ret == -1) {
    pabort("can't set max speed hz");
  }
  memset( status, 0, 64 );
  for( i = 0; i < maxDevices; i++ ) {
    ledControlSpiTransfer(i,OP_DISPLAYTEST,0);
    ledControlSetScanLimit(i,7);       //scanlimit is set to max on startup
    ledControlSpiTransfer(i,OP_DECODEMODE,0);       //decode is done in source
    ledControlClearDisplay(i);
    ledControlShutdown( i, 1 );      //we go into shutdown-mode on startup
  }
}

void ledControlClose( void ) {
  close( fd );
}

int ledControlGetDeviceCount( void ) {
    return maxDevices;
}

void ledControlShutdown( unsigned int addr, int b) {
    if( addr >= maxDevices )
	return;
    if(b)
	ledControlSpiTransfer(addr, OP_SHUTDOWN,0);
    else
	ledControlSpiTransfer(addr, OP_SHUTDOWN,1);
}

void ledControlSetScanLimit( unsigned int addr, unsigned int limit ) {
    if( addr >= maxDevices )
	return;
    if( limit >= 0 || limit < 8)
    	ledControlSpiTransfer( addr, OP_SCANLIMIT, limit );
}

void ledControlSetIntensity( unsigned int addr, unsigned int intensity) {
    if( addr >= maxDevices )
	return;
    if( intensity < 16 )
	ledControlSpiTransfer(addr, OP_INTENSITY,intensity);
}

void ledControlClearDisplay( unsigned int addr ) {
    int offset, i;

    if( addr >= maxDevices )
	return;
    offset = addr * 8;
    for( i = 0; i < 8; i++ ) {
	status[offset+i]=0;
	ledControlSpiTransfer(addr, i+1,status[offset+i]);
    }
}

void ledControlSetLed(unsigned int addr, unsigned int row, unsigned int column, int state) {
    int offset;
    uint8_t val = 0x00;

    if( addr >= maxDevices)
	return;
    if( row > 7 || column > 7 )
	return;
    offset = addr * 8;
    val = B8(10000000) >> column;
    if(state)
	status[offset+row] = status[offset+row] | val;
    else {
	val = ~val;
	status[offset+row] = status[offset+row] & val;
    }
    ledControlSpiTransfer(addr, row+1,status[offset+row]);
}

void ledControlSetRow( unsigned int addr, unsigned int row, uint8_t value) {
    int offset;
    if( addr >= maxDevices )
	return;
    if( row > 7 )
	return;
    offset = addr * 8;
    status[offset+row] = value;
    ledControlSpiTransfer(addr, row + 1, status[offset+row] );
}

void ledControlSetColumn( unsigned int addr, unsigned int col, uint8_t value) {
    uint8_t val;
    int row;

    if( addr >= maxDevices)
	return;
    if( col > 7 )
	return;
    for( row = 0; row < 8; row++ ) {
	val=value >> (7-row);
	val=val & 0x01;
	ledControlSetLed( addr, row, col, val );
    }
}

void ledControlSetDigit( unsigned int addr, unsigned int digit, uint8_t value, int dp) {
    int offset;
    uint8_t v;

    if( addr >= maxDevices )
	return;
    if( digit > 7 || value > 15)
	return;
    offset = addr * 8;
    v = charTable[value];
    if( dp )
	v |= B8(10000000);
    status[offset+digit] = v;
    ledControlSpiTransfer( addr, digit+1, v );
}

void ledControlSetChar( unsigned int addr, unsigned int digit, char value, int dp) {
    int offset;
    uint8_t index,v;

    if( addr >= maxDevices )
	return;
    if( digit > 7 )
 	return;
    offset = addr * 8;
    index = (uint8_t) value;
    if( index >127 ) {
	//no defined beyond index 127, so we use the space char
	index = 32;
    }
    v = charTable[index];
    if( dp )
	v |= B8(10000000);
    status[offset+digit]=v;
    ledControlSpiTransfer(addr, digit+1,v);
}

static void ledControlSpiTransfer( unsigned int addr, uint8_t opcode, uint8_t data )
{
  uint8_t spidata[16] = { 0 };    /* The array for shifting the data to the devices */
  uint8_t rxBuf[16];
  int offset;
  int maxbytes = maxDevices * 2;
  int ret;

  if ( addr >= maxDevices ) pabort ( "Device addres was too high!" );
  offset = ( maxDevices - addr - 1 ) * 2;
  //put our device data into the array
  spidata[ offset     ] = opcode;
  spidata[ offset + 1 ] = data;
  struct spi_ioc_transfer tr = { // Prepare the tranfer record for the transaction.
    .tx_buf = (unsigned long) spidata,
    .rx_buf = (unsigned long) rxBuf,
    .len = maxbytes,
    .delay_usecs = delay,
    .speed_hz = speed,
    .bits_per_word = bits,
  };
  ret = ioctl( fd, SPI_IOC_MESSAGE( 1 ), &tr);
  if (ret < 1) {
    pabort( "Can't send spi message" );
  }
}

