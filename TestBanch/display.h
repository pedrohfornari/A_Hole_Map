#ifndef DISPLAY_H
#define DISPLAY_H

#include <systemc.h>

/*!
 * \file display.h
 * \brief display Implementation
 *
 * The implementation file for the display
 */

/*!
 * This is the display module. 
 * \param hole_detector Input to indicate if the hole was detected.
 * \param clk The clock.
 */

  
SC_MODULE (display) {

  /*! Port sc_in<bool> hole_detected; */
  sc_in<bool> hole_detected;
  
  /*! Port sc_in<clk> clk; */
  sc_in_clk clk;
  
  /*! Functionality
   *\name display_process
   * When there is a hole, it will print out 1, else 0.
   */
  void display_process();
  
  /*! Constructor
   * SC_CTOR using SC_THREAD, based on the positive clock transition.
   */
  SC_CTOR(display){
    SC_CTHREAD(display_process, clk.pos());
  }
};
#endif
