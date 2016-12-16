#include "display.h"
/*!
 * \file display.cpp
 * \brief display Implementation
 *
 * The implementation file for the display
 */

/*!
 * This is the display module, that will display when a hole is detected. 
 * \param hole_detected Input to indicate if the hole was detected.
 * \param clk The clock.
 */

  using namespace std;
void display::display_process() { 
  /*! If a hole is detected, it will display 1, else it will display 0 */
int  tmp1;     
    while ( true )  { 
      tmp1 = hole_detected.read(); 
      cout << "\n\t Hole: " << tmp1 <<"\n\n"; 
      wait(); 
    } 
}
  
