/********************************************************************************
 * Main function that demos the use of the i2c8Bit class for use with
 * the MCP23017 chip. The code requires that the i2c kernel modules are loaded.

   It is possible to  use the configuration tool raspi-config to enable i2c or 
   follow the following instruction.Open the raspi-black-list.conf file using 
   the following command:
	sudo nano /etc/modprobe.d/raspi-blacklist.conf”

   Comment out the “blacklist i2c-bcm2708” entry by putting a hash # sign in 
   front of it. So it looks like “#blacklist i2c-bcm2708” You then need to save 
   your changes (Ctrl-x in Nano) and reboot using the “sudo reboot” command. 
   After reboot check kernel modules:
	sudo modprobe i2c-dev
	sudo chmod o+rw /dev/i2c*

   To change the speed of the I2C bus you can type in the command line:
	sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=400000

   Install i2c-tools
	sudo apt-get install i2c-tools

   Check both i2c busses
	i2cdetect -y 0
	i2cdetect -y 1

   If a device is found on address 20h the MAX23017 chip was probed successfully.
 *	
 ********************************************************************************/

#include "i2c8Bit.h"
#include <iostream>
using namespace std;

int main(void)
{
	int i = 0;
	unsigned char pinStatus = 0;
	unsigned char outToggleState = 0;

	// Instantiate i2c8Bit object called mcp23017 at
	// device address 0x20 using i2c device "/dev/i2c-1" 
	i2c8Bit mcp23017( 0b00100000, string("/dev/i2c-1") );
        // Write data value 0b11111110 into register 00 (IODIRA) to
        // make GPA0 output, rest of the pins (GPA1-7) are inputs.
	mcp23017.writeReg( 0b00, 0b11111110 ); 
	
	while(i < 20) // repeat the following 20 times and then exit;
	{
		mcp23017.readReg( 0x12, pinStatus ); 	// Read the GPIOA register.
             
		if( ( pinStatus & 0b10000000 ) == 0){ 	// Test to see if pin GPA7 is equal to zero
							// i.e. if pushbutton is pressed.
			cout << "PushButton pressed..toggling LED" << endl;
			outToggleState ^= 1;    // Update vatiable to cause LED to toggle.
		}
		else
			cout << "Pushbutton not pressed...LED static" << endl;
		sleep(1);
		mcp23017.writeReg( 0x14, outToggleState ); // Wright the value of outToggleState to the OLATA register.
                i++;
                cout << i << " LED state is: "<< (int)outToggleState << endl;
	}
	cout << "exiting" << endl;
	return 0; // The destructor id called just before program exit
}
				
		
		
	
	
	
	
	

