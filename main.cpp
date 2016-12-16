#include <systemc.h>
#include "sensor_sim.h"
#include "detector.h"
#include "display.h"
/*!
 * \file main.cpp
 * \brief  Hole detector project main file implementation
 *
 * The main file implementation for the hole detector project
 */

/*!
 * This is the main. 
 * There are have instanced 3 modules:
 * sensor_sim (s), detector (det), and display (d).
 */
/*!
 * The modules are connected by the
 * signals: sensor1, sensor2, sensor3, sensor4,
 * timebase and hole_detected.
 */
/*! 
 * The clock is set as 1ns. And the duration is set as
 * the number of samples of each *.txt file in order to
 * read the data aquired.
 *
 */

  using namespace std;
int sc_main( int argc, char *argv[] )  {
  // Signals
  sc_signal<int> sensor1;
  sc_signal<int> sensor2;
  sc_signal<int> sensor3;
  sc_signal<int> sensor4;
  sc_signal<int> timebase;
  sc_signal<bool> hole_detected;
  sc_clock clk("clk",1,SC_NS); // Create a 10ns period clock signal 
  
  sensor_sim *s;
  detector *det;
  display *d; 
  
  s = new sensor_sim("SENSORS");
  /*!
   * The sensor_sim 's' has the signal outputs 'sensor1',
   * 'sensor2', 'sensor3', 'sensor4', and 'timebase'. It also
   * has a clock 'clk'.
   */
  (*s) (sensor1, sensor2, sensor3, sensor4, timebase, clk);
  
  det = new detector("DETECTOR");
  /*!
   * The detector 'det' has the signal inputs 'sensor1',
   * 'sensor2', 'sensor3', 'sensor4', and 'timebase'. It has
   * the signal output 'hole_detected'. It also has a clock 
   * 'clk'.
   */
  (*det) (sensor1, sensor2, sensor3, sensor4, timebase, hole_detected, clk);
  
  d = new display("DISPLAY");
  /*!
   * The display 'd' has the signal input 'hole_detected'.
   * It also has a clock 'clk'.
   */
  (*d) (hole_detected, clk);
  
  
 /*!
  * All the signals are traced in a 'wave.vcd' file.
  */
  sc_trace_file *fp;
  fp = sc_create_vcd_trace_file("wave");
  
  sc_trace(fp, clk, "clk");
  
  sc_trace(fp, sensor1, "sensor1");
  sc_trace(fp, sensor2, "sensor2");
  sc_trace(fp, sensor3, "sensor3");
  sc_trace(fp, sensor4, "sensor4");
  sc_trace(fp, timebase, "timebase");
  sc_trace(fp, hole_detected, "hole_detected");
  
  sc_start(40000, SC_NS);
  
 /*! 
  * The simulation duration for each file are:
  */
 /*! Teste1.txt  : 27610;   */
 /*! 10kmph.txt  : 28590;   */
 /*! 20kmph.txt  : 17360;   */
 /*! 30kmph.txt  : 14700;   */
 /*! 40kmph.txt  : 14650;   */
 /*! 10kmphb.txt : 15920;   */

  sc_close_vcd_trace_file(fp);
  cout << "\n\n ..........End of Execution.........\n\n\n";
  return 0; 
}                                                            
