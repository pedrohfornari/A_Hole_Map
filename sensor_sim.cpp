#include "sensor_sim.h"                                                                    
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/*!
 * \file sensor_sim.cpp
 * \brief sensor data simulation implementation
 *
 * The implementation file for the sensor data simulation
 */

/*!
 * This is the sensor simulation module. 
 * The sensor data simulation program reads a .txt file and outputs
 * five vectors, 4 with real sensor data aquired and one that shows
 * a timebase for each sample.
 * \param sensor1 emulates the 1st of the four sensors;
 * \param sensor2 emulates the 2nd of the four sensors;
 * \param sensor3 emulates the 3rd of the four sensors;
 * \param sensor4 emulates the 4th of the four sensors;
 * \param timebase emulates the microcontroller timebase;
 * \param clk the clock;
 */
 using namespace std;
void sensor_sim::sensor_simulation() { 

  std::ifstream doc;
  
  /*! 
   * The file to be read and used as for output of this
   * module is set in the line "ifstream infile("./30kmph.txt");". 
   * The file is read line by line and deconstructed, in order to
   * get each of the vector's data. The data inside the files are
   * standardly organized as ":sensor1,sensor2,sensor3,sensor4,timebase:"
   * (without the quotes). 
   */
  string arry[5];
  vector <int> array;
  array.reserve(5);
  int i;
  ifstream infile("./30kmph.txt"); // for example
  string line = "";

  while (getline(infile, line))
  {
    stringstream strstr(line);
    string word = "";
    while (getline(strstr,word, ':'))
    {
      stringstream str(word);
      for(i = 0; i < 5; i++)
      {
	getline(str, arry[i], ',');
	array.push_back(atoi(arry[i].c_str()));
	//cout<<array[i]<<".";
      }
      sensor1.write(array[0]);
      sensor2.write(array[1]);
      sensor3.write(array[2]);
      sensor4.write(array[3]);
      timebase.write(array[4]);
      array.clear(); //dar output antes de dar clear
      wait();
    }
  }
}