#ifndef DETECTOR_H 
#define DETECTOR_H 

#include <systemc.h>

/*!
 * \file detector.h
 * \brief hole detection algorithm implementation
 *
 * The implementation file for the detector module
 */

/*!
 * This is the hole detector module. 
 * We have 5 inputs, 1 output and a clock.
 * The hole detection algorithm file receives 4 sensor data and
 * a timebase, than it runs the data through some filters and
 * tries to detect if there is a hole in that time sample or not.
 * \param sensor1 input of the 1st of the four sensors;
 * \param sensor2 input of the 2nd of the four sensors;
 * \param sensor3 input of the 3rd of the four sensors;
 * \param sensor4 input of  the 4th of the four sensors;
 * \param timebase input of the microcontroller timebase;
 * \param clk the clock;
 * \param hole_detected output to indicate if the hole was detected.
 */

SC_MODULE (detector)  {

  /*! Port sc_in<int> sensor1; */
  sc_in<int> sensor1;
  
  /*! Port sc_in<int> sensor2; */
  sc_in<int> sensor2;
  
  /*! Port sc_in<int> sensor3; */
  sc_in<int> sensor3;

  /*! Port sc_in<int> sensor4; */
  sc_in<int> sensor4;
 
  /*! Port sc_in<int> timebase; */
  sc_in<int> timebase;
  
  /*! Port sc_out<bool> hole_detected; */
  sc_out<bool> hole_detected;
  
  /*! Port sc_in<clk> clk; */
  sc_in_clk clk;
    
  /*! Functionality
   *\name detector
   * Using the inputs from the sensors, the detector
   * will use the selected algorithm to try and find
   * the moments the holes are detected, and then
   * output a bool hole_detected to indicate the
   * detection.
   */
  
  void detector_algorithm(void);
  void first_algorithm(void);
  void p1_algorithm(void);
  void l1_algorithm(void);
  void l2_algorithm(void);
  void p2_algorithm(void);
  void kalman_filter(void);
  
  /*! Constructor
   * SC_CTOR using SC_CTHREAD, based on the positive clock transition.
   */
  
  SC_CTOR(detector) { 
    SC_CTHREAD(detector_algorithm ,clk.pos()); 
  }  
}; 
#endif