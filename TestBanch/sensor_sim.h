#ifndef SENSOR_SIM_H 
#define SENSOR_SIM_H 

#include <systemc.h>

/*!
 * \file sensor_sim.h
 * \brief sensor data simulation implementation
 *
 * The implementation file for the sensor simulation module
 */

/*!
 * This is the sensor simulation module. 
 * The sensor data simulation program reads a .txt file and outputs
 * five vectors: 4 with real sensor data aquired and one that shows
 * a timebase for each sample.
 * \param sensor1 emulates the 1st of the four sensors;
 * \param sensor2 emulates the 2nd of the four sensors;
 * \param sensor3 emulates the 3rd of the four sensors;
 * \param sensor4 emulates the 4th of the four sensors;
 * \param timebase emulates the microcontroller timebase;
 * \param clk the clock;
 */

SC_MODULE (sensor_sim)  {
  /*! Port sc_out<int> sensor1; */
  sc_out<int> sensor1; 
  
  /*! Port sc_out<int> sensor2; */
  sc_out<int> sensor2;
  
  /*! Port sc_out<int> sensor3; */
  sc_out<int> sensor3; 
  
  /*! Port sc_out<int> sensor4; */
  sc_out<int> sensor4;
  
  /*! Port sc_out<int> timebase; */
  sc_out<int> timebase;
  
  /*! Port sc_in<clk> clk;; */
  sc_in_clk clk;
    
  /*! Functionality
   *\name sensor_simulation
   * Outputs the sensors' data and the timebase for each sample.
   */
  void sensor_simulation(); 

  /*! Constructor
   * SC_CTOR using SC_CTHREAD, based on the positive clock transition.
   */ 
  SC_CTOR(sensor_sim) { 
    SC_CTHREAD(sensor_simulation,clk.pos()); 
  }  
}; 
#endif