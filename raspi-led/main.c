#include "LedControl.h"
#include <unistd.h>
// - See more at: http://embedded-lab.com/blog/?p=6862#sthash.ePNXcHHe.dpuf

const int numDev = 2;

int main( void ) {
  int i, j, count = 0;

  ledControlInit( numDev ); // set up
  for ( j = 0; j < numDev; j++ ) {   // Initialize the 3 MAX7219 devices
    ledControlShutdown( j, 0 ); // Enable display
    ledControlSetIntensity( j, 8 ); // Set brightness level (0 is min, 15 is max)
    ledControlClearDisplay( j ); // Clear display register
  }
  for ( ;; ) { //  loop()
    for ( j = 0; j < numDev; j++ ) { // Iterate over all devices
      for ( i = 0; i < 8; i++ ) { // Iterate over all digits
        ledControlSetDigit( j, 7 - i, count, 1 ); // Decimal point enabled
      }
      count++;
      if ( count == 16 ) count = 0;
      sleep( 1 ); // Sleep time given in seconds
    }
  }
  return 0;
}



