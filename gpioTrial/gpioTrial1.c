//
// This demo is targeted for the Raspberry PI, Broadcom BCM2708 processor.
// The goal is to demonstrate how an general pupose IO-pin is acessed from a user space program without
// any need for a special kernel driver using the /dev/mem device.
//
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdint.h>

static volatile uint32_t *gpio;

int main(int argc, char **argv)
{
	int fd ;

	//Obtain handle to physical memory
	if ( ( fd = open ("/dev/mem", O_RDWR | O_SYNC) ) < 0 ) {
    printf( "Unable to open /dev/mem: %s\n", strerror(errno) );
    return -1;
	}
	//map a page of memory to gpio at offset 0x20200000 which is where GPIO goodnessstarts
	gpio = (uint32_t *) mmap( 0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x20200000 );

	if ( (int32_t) gpio < 0 ) {
		printf( "Mmap failed: %s\n", strerror(errno) );
		return -1;
	}
	//set gpio17 as an output
	//increment the pointer to 0x20200004
	//set the value through a little bit twiddling where we only modify the bits 21-23 in the register
	*(gpio + 1) = (*(gpio + 1) & ~(7 << 21)) | (1 << 21);

	for(;;) { 	//toggle gpio17 every second
    //set the pin high
    //increment the pointer to 0x2020001C
    *(gpio + 7) = 1 << 17;

    sleep(1);  //sleep

    //set the pin to low
    //increment the pointer to 0x20200028
    *(gpio + 10) = 1 << 17;

    sleep(1);
	}
}

