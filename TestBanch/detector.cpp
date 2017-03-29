#include "detector.h"                                                                    
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/*!
 * \file detector.cpp
 * \brief hole detection algorithm implementation
 *
 * The implementation file for the hole detection algorithm
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
 * \param sensor4 input of the 4th of the four sensors;
 * \param timebase input of the microcontroller timebase;
 * \param clk the clock;
 * \param hole_detected output to indicate if the hole was detected.
 */


#define ARRAY_SIZE 23 /*! Size of the array used to calculate the mean value. 30 for kalamn filter, 15 for p1 and p2_algorithm*/
#define UPPER_LIMIT 1100 /*! Upper limit for saturation. 1100 for p1_algorithm and 3.25 for p2_algorithm*/
#define LOWER_LIMIT 0 /*! Lower limit (to remove negative values). 0 for p1_algorithm and p2_algorithm*/
#define REJECTED_HOLES 250 /*! Number of possible holes nearby to reject. */
#define THRESHOLD 178 /*! If the difference of the current data and the mean exceeds it, it is considered a possible hole. */
#define PROCESS_NOISE 0.15 /*! Process noise for the Kalman Filter. */
#define SENSOR_NOISE 3.5 /*! Sensor noise for the Kalman Fitler. */
#define NUMBER_OF_DETECTIONS 4 /*! Minimum number of close variations to indicate a possible hole. */
#define STOP_TIME 350000 /*! Time in the timebase when the car stopped. */
#define HOLENUM 12
/*! Note: Not all the defines are used in all functions */

  using namespace std;
void detector::detector_algorithm()
{
  
 /*! 
  * Here is chosen which algorithm will be used.
  * There were made more than one algorithm in order
  * to study different implementation behaviours.
  *
  * The developed algorithms are:
  * 1 - kalman_filter();
  * 2 - first_algorithm();
  * 3 - p1_algorithm();
  * 4 - l1_algorithm();
  * 5 - l2_algorithm();
  * 6 - p2_algorithm();
  */
 
  p1_algorithm();
}


void detector::kalman_filter(void)
{
 /*!
  * In the variables' declaration, all vectors are
  * initialized with zeros by default.
  */
 
  vector<int>  sensor1_data(ARRAY_SIZE);
  vector<int>  sensor2_data(ARRAY_SIZE);
  vector<int>  sensor3_data(ARRAY_SIZE);
  vector<int>  timebase_data(ARRAY_SIZE);
  vector<char> counter(4);
  
  int  sensor1_mean=0; //mean value of the sensor1 data
  int  sensor2_mean=0; //mean value of the sensor2 data
  int  sensor3_mean=0; //mean value of the sensor3 data
  
  char isAhole;
  bool hole_detected_flag=0;
  
  int  Iter_num = 0; //number of current iteration
  int  NOH = 0; //number of detected holes
  double last_time_detected = 0;
  int milliseconds = 0;
  int seconds = 0;
  int minutes = 0;
  
  /*!
   * Then the Kalman filter coeficients are set:
   * q = process noise;
   * r = sensor noise;
   * p = estimated error;
   * k = kalman gain.
   */
    float q = PROCESS_NOISE; //process noise
    float r = SENSOR_NOISE; //sensor noise
    float p = 0; //estimated error
    float k; //kalman gain

  while(true)
  {

    for (int i = (ARRAY_SIZE-1); i>0; i--)
    {
     /*!
      * It is executed a loop to shift the samples' position
      * within the vectors in order to receive a new data.
      */
      sensor1_data.at(i) = sensor1_data.at(i-1);
      sensor2_data.at(i) = sensor2_data.at(i-1);
      sensor3_data.at(i) = sensor3_data.at(i-1);
      timebase_data.at(i) = timebase_data.at(i-1);
     /*!
      * The sensors values are added in order to calculate
      * a mean value. Thus, this is a moving average filter
      * to be used in a future in order to compare the data.
      */ 
      sensor1_mean = sensor1_data.at(i)+sensor1_mean;
      sensor2_mean = sensor2_data.at(i)+sensor2_mean;
      sensor3_mean = sensor3_data.at(i)+sensor3_mean;
    }

  
  /*!
   * The Kalman filter coeficients are recalculated in each iteration
   * and used to all sensors. Also, the current sensor value passes through
   * this filter.
   */
    p = p+q;
    k = p/(p+r);
    
    sensor1_data.at(0) = (int)(sensor1_data.at(0) + k*(sensor1.read()-sensor1_data.at(0)));
    sensor2_data.at(0) = (int)(sensor2_data.at(0) + k*(sensor2.read()-sensor2_data.at(0)));
    sensor3_data.at(0) = (int)(sensor3_data.at(0) + k*(sensor3.read()-sensor3_data.at(0)));
    timebase_data.at(0) = timebase.read();
    p = (1-k)*p;

  
  /*!
   * A upper to the sensors' data is set in order to prevent failure in the filters
   */
    if(sensor1_data.at(0) > 3500) sensor1_data.at(0) = sensor1_mean;
    if(sensor2_data.at(0) > 3500) sensor2_data.at(0) = sensor2_mean;
    if(sensor3_data.at(0) > 3500) sensor3_data.at(0) = sensor3_mean;   
  
  /*!
   * The mean values of the sensors are calculated and then compared to
   * the current sample in order to check for possible holes.
   * A variable adds up at each possible hole.
   */
   
    sensor1_mean = (sensor1_data.at(0)+sensor1_mean)/ARRAY_SIZE; 
    sensor2_mean = (sensor2_data.at(0)+sensor2_mean)/ARRAY_SIZE;
    sensor3_mean = (sensor3_data.at(0)+sensor3_mean)/ARRAY_SIZE;
    
    // detection flag. 
    if((sensor1_data.at(0) >= (sensor1_mean+THRESHOLD))||(sensor1_data.at(0) <= (sensor1_mean-THRESHOLD))) isAhole++;
    if((sensor2_data.at(0) >= (sensor2_mean+THRESHOLD))||(sensor2_data.at(0) <= (sensor2_mean-THRESHOLD))) isAhole++;
    if((sensor3_data.at(0) >= (sensor3_mean+THRESHOLD))||(sensor3_data.at(0) <= (sensor3_mean-THRESHOLD))) isAhole++;

  
  if(isAhole > NUMBER_OF_DETECTIONS)
  {
   /*!
    * If the number of possible holes exceeds the NUMBER_OF_DETECTIONS we set,
    * a hole detected flag will be set, and the counter of holes will be reset to 0.
    */
    hole_detected_flag = 1;
    isAhole = 0;
  }

   /*! 
    * If any variation in the sensors bigger than the threshold is detected, and if there is't a hole near
    * the hole will be detected (High Pass Filter). 350000 because the car stopped there. 1000 because 
    * it's the GPS resolution (assuming max car speed = 20m/s = 72km/h).
    */
    if(((hole_detected_flag==1)&&(Iter_num>ARRAY_SIZE))&&((timebase_data.at(0)<STOP_TIME)&&((timebase_data.at(0)-last_time_detected)>3500))) 
    {			      
      hole_detected.write(1);
      hole_detected_flag = 0;
      NOH = NOH++; //number of holes
      last_time_detected = timebase_data.at(0);

     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data.at(0)%1000;
      seconds = (timebase_data.at(0) / 1000) % 60 ;
      minutes = ((timebase_data.at(0)/ (1000*60)) % 60);
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */

    }
    else
    {
      hole_detected.write(0);
    } 
    
    wait();
    Iter_num = Iter_num++;
    sensor1_mean = 0;
    sensor2_mean = 0;
    sensor3_mean = 0;
  
  }
}


void detector::first_algorithm()
{ 
int  sensor1_data[ARRAY_SIZE];
int  sensor2_data[ARRAY_SIZE];
int  sensor3_data[ARRAY_SIZE];
int  sensor4_data[ARRAY_SIZE];
int  timebase_data[ARRAY_SIZE];

int  sensor1_mean=0; //mean value of the sensor1 data
int  sensor2_mean=0; //mean value of the sensor2 data
int  sensor3_mean=0; //mean value of the sensor3 data
int  sensor4_mean=0; //mean value of the sensor4 data

int  hole_rejection[REJECTED_HOLES]; //variable to reject close holes
int  hole_rejection_flag=0;
bool hole_detected_flag=0;
int  Iter_num = 0; //number of current iteration
int  NOH = 0; //number of detected holes

int milliseconds = 0;
int seconds = 0;
int minutes = 0;


  for (int i = (REJECTED_HOLES-1);i>=0;i--) //hole_rejection inicialization
  {
    hole_rejection[i]=0;
  }
  
  for (int i = (ARRAY_SIZE-1); i>=0; i--) //sensors and timebase inicialization
  {
    sensor1_data[i] =0;
    sensor2_data[i] =0;
    sensor3_data[i] =0;
    sensor4_data[i] =0;
    timebase_data[i]=0;
  }

 /*!
  * In the variables' declaration, all vectors are
  * initialized with zeros by default.
  */


  while ( true )  { 

    for (int i = (ARRAY_SIZE-1); i>0; i--)
    {
     /*!
      * It is executed a loop to shift the samples' position
      * within the vectors in order to receive a new data.
      */
      sensor1_data[i] = sensor1_data[i-1];
      sensor2_data[i] = sensor2_data[i-1];
      sensor3_data[i] = sensor3_data[i-1];
      sensor4_data[i] = sensor4_data[i-1];
      timebase_data[i] = timebase_data[i-1];

     /*!
      * The sensors values are added in order to calculate
      * a mean value. Thus, this is a moving average filter
      * to be used in a future in order to compare the data.
      */
     
      sensor1_mean = sensor1_data[i]+sensor1_mean;
      sensor2_mean = sensor2_data[i]+sensor2_mean;
      sensor3_mean = sensor3_data[i]+sensor3_mean;
      sensor4_mean = sensor4_data[i]+sensor4_mean;
    }
    
   /*!
    * The current data passes through a moving average filter to reduce noise.
    */
    sensor1_data[0] = (sensor1.read()+sensor1_data[1]+sensor1_data[2]+sensor1_data[3]+sensor1_data[4])/5; //LPF to reduce noise
    sensor2_data[0] = (sensor2.read()+sensor2_data[1]+sensor2_data[2]+sensor2_data[3]+sensor2_data[4])/5;
    sensor3_data[0] = (sensor3.read()+sensor3_data[1]+sensor3_data[2]+sensor3_data[3]+sensor3_data[4])/5;
    sensor4_data[0] = (sensor4.read()+sensor4_data[1]+sensor4_data[2]+sensor4_data[3]+sensor4_data[4])/5;
    timebase_data[0] = timebase.read();
    
    sensor1_mean = (sensor1_data[0]+sensor1_mean)/ARRAY_SIZE; //mean values to compare
    sensor2_mean = (sensor2_data[0]+sensor2_mean)/ARRAY_SIZE;
    sensor3_mean = (sensor3_data[0]+sensor3_mean)/ARRAY_SIZE;
    sensor4_mean = (sensor4_data[0]+sensor4_mean)/ARRAY_SIZE;
    
    //cout << "\nSensor mean: "<< sensor_mean <<"\n";
    
    for (int i = (REJECTED_HOLES-1); i>0;i--) 
    {
     /*!
      * If a hole is detected to any other hole near 80 samples, it is ignored for it's too close (LPF).
      */
      hole_rejection[i] = hole_rejection[i-1];
      hole_rejection_flag = hole_rejection[i]+hole_rejection_flag;
    }
   /*!
    * The hole_detected_flag is set to 1 if the difference between any of the sensors' current data
    * and the mean exceeds the THRESHOLD and the rejection flag is not active.
    */
    hole_detected_flag = ((sensor1_data[0] >= (sensor1_mean+THRESHOLD))||(sensor1_data[0] <= (sensor1_mean-THRESHOLD))); // detection flag
    hole_detected_flag = ((sensor2_data[0] >= (sensor2_mean+THRESHOLD))||(sensor2_data[0] <= (sensor2_mean-THRESHOLD)))||(hole_detected_flag);
    hole_detected_flag = ((sensor3_data[0] >= (sensor3_mean+THRESHOLD))||(sensor3_data[0] <= (sensor3_mean-THRESHOLD)))||(hole_detected_flag);
    hole_detected_flag = ((sensor4_data[0] >= (sensor4_mean+THRESHOLD))||(sensor4_data[0] <= (sensor4_mean-THRESHOLD)))||(hole_detected_flag);
    hole_detected_flag = (hole_rejection_flag==0)&&(hole_detected_flag);
    
    if(hole_detected_flag==1) 
    {
     /*!
      * Some debugging values will be calculated, as the number of holes detected (NOH) until now
      * and the time of the occurance to compare to the real data aquisition.
      */
      hole_detected.write(1);
      NOH = NOH++;
      hole_rejection[0] = 1;
      hole_rejection_flag =0;
      
      
     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data[0]%1000;
      seconds = (timebase_data[0] / 1000) % 60 ;
      minutes = ((timebase_data[0]/ (1000*60)) % 60);
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */
    }
    else
    {
      hole_detected.write(0);
      hole_rejection[0] = 0;
      hole_rejection_flag =0;
    } 
    wait();
    Iter_num = Iter_num++;
    //cout << "\t\t\n HRF: " << hole_rejection_flag<<"\n";
    //cout << "\n Iteration number: " << Iter_num<<"\n";
    //cout << "\n Number of Holes detected: "<< NOH<<"\n";
    
    
  }
}


void detector::p1_algorithm()
{
  /*!
  * In the variables' declaration, all vectors are
  * initialized with zeros by default.
  */
  vector<int>  sensor1_data(ARRAY_SIZE);
  vector<int>  sensor2_data(ARRAY_SIZE);
  vector<int>  sensor3_data(ARRAY_SIZE);
  vector<int>  sensor4_data(ARRAY_SIZE);
  vector<int>  timebase_data(ARRAY_SIZE);
  vector<char> counter(4);
  char isAhole;
  
  vector<int>  hole_rejection(REJECTED_HOLES); //variable to reject close holes
  int  hole_rejection_flag=0;
  bool hole_detected_flag=0;
  int  Iter_num = 0; //number of current iteration
  int  NOH = 0; //number of detected holes
  
  int milliseconds = 0;
  int seconds = 0;
  int minutes = 0;    
  
  /*!
   * Then the Kalman filter coeficients are set:
   * q = process noise;
   * r = sensor noise;
   * p = estimated error;
   * k = kalman gain.
   */
    float q = PROCESS_NOISE; //process noise
    float r = SENSOR_NOISE; //sensor noise
    float p = 0; //estimated error
    float k; //kalman gain
  
  
  /*algorithm loop*/
  
    while(1)
    {
      /*Move values to its next position in the array*/
      for (int i = (ARRAY_SIZE-1); i>0; i--)
      {
    
     /*!
      * It is executed a loop to shift the samples' position
      * within the vectors in order to receive a new data.
      */
	sensor1_data.at(i) = sensor1_data.at(i-1);
	sensor2_data.at(i) = sensor2_data.at(i-1);
	sensor3_data.at(i) = sensor3_data.at(i-1);
	
	timebase_data.at(i) = timebase_data.at(i-1);
	
	/*!
	* If the sensors' value are out of the limits the counter is incremented.
	*/
	if(i < (ARRAY_SIZE-1)){
	  if((sensor1_data.at(i) > (sensor1_data.at(i+1) + THRESHOLD))||(sensor1_data.at(0) < (sensor1_data.at(i+1) - THRESHOLD))) counter.at(0) = counter.at(0) + 1;
	  if((sensor2_data.at(i) > (sensor2_data.at(i+1) + THRESHOLD))||(sensor2_data.at(0) < (sensor2_data.at(i+1) - THRESHOLD))) counter.at(1) = counter.at(1) + 1;
	  if((sensor3_data.at(i) > (sensor3_data.at(i+1) + THRESHOLD))||(sensor3_data.at(0) < (sensor3_data.at(i+1) - THRESHOLD))) counter.at(2) = counter.at(2) + 1;
	
	}
      }

     /*!
      * New values are read and if they're out of the limits, the counter
      * is incremented.      
      */
     /*!
    * The Kalman filter coeficients are recalculated in each iteration
    * and used to all sensors. Also, the current sensor value passes through
    * this filter.
    */
      p = p+q;
      k = p/(p+r);
      
      sensor1_data.at(0) = (int)(sensor1_data.at(0) + k*(sensor1.read()-sensor1_data.at(0)));
      sensor2_data.at(0) = (int)(sensor2_data.at(0) + k*(sensor2.read()-sensor2_data.at(0)));
      sensor3_data.at(0) = (int)(sensor3_data.at(0) + k*(sensor3.read()-sensor3_data.at(0)));
      timebase_data.at(0) = timebase.read();
      p = (1-k)*p;

      
      if (sensor1_data.at(0) > 3500) sensor1_data.at(0) = 900;
      
      if (sensor2_data.at(0) > 3500) sensor2_data.at(0) = 900;
    
      if (sensor3_data.at(0) > 3500) sensor3_data.at(0) = 900;
      
      timebase_data.at(0) = timebase.read();
      
      if((sensor1_data.at(0) > (sensor1_data.at(1) + THRESHOLD))||(sensor1_data.at(0) < (sensor1_data.at(1) - THRESHOLD))) counter.at(0) = counter.at(0) + 1;
      if((sensor2_data.at(0) > (sensor2_data.at(1) + THRESHOLD))||(sensor2_data.at(0) < (sensor2_data.at(1) - THRESHOLD))) counter.at(1) = counter.at(1) + 1;
      if((sensor3_data.at(0) > (sensor3_data.at(1) + THRESHOLD))||(sensor3_data.at(0) < (sensor3_data.at(1) - THRESHOLD))) counter.at(2) = counter.at(2) + 1;
      
    
    /*!
    * If a possible hole is detected near another hole by REJECTED_HOLES
    * samples, it will be ignored.
    */
    for (int i = (REJECTED_HOLES-1); i>0; i--) //If a hole is detected to any other hole near 80 samples, it is ignored for it's too close (LPF)
    {
      hole_rejection.at(i) = hole_rejection.at(i-1);
      hole_rejection_flag = hole_rejection[i]+hole_rejection_flag;
    }
    
    /*!
    * If the possibility of a hole is seen a lot in a close sample interval,
    * it will be considered a hole.
    */
    for(int i = 0; i<3; i++)
    {
      isAhole = isAhole + counter.at(i);
      if(isAhole > HOLENUM) hole_detected_flag = 1;
      hole_detected_flag = (hole_rejection_flag==0)&&(hole_detected_flag);
    }
    
    
    if(hole_detected_flag==1)
    {
      
     /*!
      * Some debugging values will be calculated, as the number of holes detected (NOH) until now
      * and the time of the occurance to compare to the real data aquisition.
      */
      hole_detected.write(1);
      NOH = NOH++;
      hole_rejection.at(0) = 1;
      hole_rejection_flag =0;
      
     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data.at(0)%1000;
      seconds = (timebase_data.at(0) / 1000) % 60 ;
      minutes = ((timebase_data.at(0)/ (1000*60)) % 60);
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */
    }
    else
    {
      hole_detected.write(0);
      hole_rejection.at(0) = 0;
      hole_rejection_flag =0;
    } 
    
    wait(); //waits for a new clock pulse
    Iter_num = Iter_num++;
   
    /*!
     * In the end the counter is reset.
     */
      for (int i = 0; i<counter.size(); i++)
      {
	counter.at(i) = 0;
      }
    isAhole = 0;
   
  }
}


void detector::l1_algorithm()
{ 
int  sensor1_data[ARRAY_SIZE];
int  sensor2_data[ARRAY_SIZE];
int  sensor3_data[ARRAY_SIZE];
int  sensor4_data[ARRAY_SIZE];
int  timebase_data[ARRAY_SIZE];

int  sensor1_mean=0; //mean value of the sensor1 data
int  sensor2_mean=0; //mean value of the sensor2 data
int  sensor3_mean=0; //mean value of the sensor3 data
int  sensor4_mean=0; //mean value of the sensor4 data
int  last_time_detected =0;

int  threshold =THRESHOLD; //maximum allowed variation to ignore

int  hole_samples[REJECTED_HOLES]; // if the number of samples per hole is not achieved the hole won't be detected (LPF)
int  minimum_samples = 3;
int  hole_samples_sum = 0;

bool hole_detected_flag=0;
int  Iter_num = 0; //number of current iteration
int  NOH = 0; //number of detected holes

int milliseconds = 0;
int seconds = 0;
int minutes = 0;


 
  for (int i =(REJECTED_HOLES-1);i>=0;i--) //hole_samples inicialization
  {
    hole_samples[i]=0;
  }
  
  for (int i = (ARRAY_SIZE-1); i>=0; i--) //sensors and timebase inicialization
  {
    sensor1_data[i] =0;
    sensor2_data[i] =0;
    sensor3_data[i] =0;
    sensor4_data[i] =0;
    timebase_data[i]=0;
  }
 /*!
  * In the variables' declaration, all values are
  * initialized with zeros by default.
  */
 
  while ( true )  { 


    for (int i = (ARRAY_SIZE-1); i>0; i--)
    {
     /*!
      * The arrays are realocated to receive the new data.
      * Also the mean value is recalculated.
      */
      sensor1_data[i] = sensor1_data[i-1];
      sensor2_data[i] = sensor2_data[i-1];
      sensor3_data[i] = sensor3_data[i-1];
      sensor4_data[i] = sensor4_data[i-1];
      timebase_data[i] = timebase_data[i-1];
      
      sensor1_mean = sensor1_data[i]+sensor1_mean;
      sensor2_mean = sensor2_data[i]+sensor2_mean;
      sensor3_mean = sensor3_data[i]+sensor3_mean;
      sensor4_mean = sensor4_data[i]+sensor4_mean;
    }
    
    
   /*!
    * The negative values of the data are set as positive.
    */
    sensor1_data[0] =sensor1.read();
    if(sensor1_data[0]<0)
    {
      sensor1_data[0] = -sensor1_data[0];
    }
    
    
    sensor2_data[0] =sensor2.read();
    if(sensor2_data[0]<0)
    {
      sensor2_data[0] = -sensor2_data[0];
    }
    
    
    sensor3_data[0] =sensor3.read();
    if(sensor3_data[0]<0)
    {
      sensor3_data[0] = -sensor3_data[0];
    }
    
    
    sensor4_data[0] =sensor4.read();
    if(sensor4_data[0]<0)
    {
      sensor4_data[0] = -sensor4_data[0];
    }
    
    
   /*!
    * The current data passes through a moving average filter to reduce noise.
    */
    sensor1_data[0] = (sensor1_data[0]+sensor1_data[1])/2;//+sensor1_data[2])/3;//+sensor1_data[3]+sensor1_data[4])/5; 
    sensor2_data[0] = (sensor2_data[0]+sensor2_data[1])/2;//+sensor2_data[2])/3;//+sensor2_data[3]+sensor2_data[4])/5;
    sensor3_data[0] = (sensor3_data[0]+sensor3_data[1])/2;//+sensor3_data[2])/3;//+sensor3_data[3]+sensor3_data[4])/5;
    sensor4_data[0] = (sensor4_data[0]+sensor4_data[1])/2;//+sensor4_data[2])/3;//+sensor4_data[3]+sensor4_data[4])/5;

    
    timebase_data[0] = timebase.read();
    
    sensor1_mean = (sensor1_data[0]+sensor1_mean)/ARRAY_SIZE; 
    sensor2_mean = (sensor2_data[0]+sensor2_mean)/ARRAY_SIZE;
    sensor3_mean = (sensor3_data[0]+sensor3_mean)/ARRAY_SIZE;
    sensor4_mean = (sensor4_data[0]+sensor4_mean)/ARRAY_SIZE;
    
    
    //cout << "\nSensor mean: "<< sensor_mean <<"\n";
    
   /*!
    * If a variation is detected (hole_samples), but no other variation is 
    * detected close by, it is ignored (LPF).
    */
    for (int i = (REJECTED_HOLES-1); i>0;i--) 
    {
      hole_samples[i] = hole_samples[i-1];
    }
    hole_samples[0] = 0; 
    
   /*!
    * The hole_detected_flag is set to 1 if the difference between any of the sensors' current data
    * and the mean exceeds the THRESHOLD..
    */ 
    hole_detected_flag = ((sensor1_data[0] >= (sensor1_mean+THRESHOLD))||(sensor1_data[0] <= (sensor1_mean-THRESHOLD)));
    hole_detected_flag = ((sensor2_data[0] >= (sensor2_mean+THRESHOLD))||(sensor2_data[0] <= (sensor2_mean-THRESHOLD)))||(hole_detected_flag);
    hole_detected_flag = ((sensor3_data[0] >= (sensor3_mean+THRESHOLD))||(sensor3_data[0] <= (sensor3_mean-THRESHOLD)))||(hole_detected_flag);
    hole_detected_flag = ((sensor4_data[0] >= (sensor4_mean+THRESHOLD))||(sensor4_data[0] <= (sensor4_mean-THRESHOLD)))||(hole_detected_flag);
    
   /*!
    * If the hole_detected_flag is set to 1, a possible hole is marked.
    */
    
    if (hole_detected_flag ==1)
    {
      hole_samples[0] = 1;
      //cout<<"\n\thole detected flag: "<< hole_detected_flag<<"\n";
    }
    
    hole_samples_sum = 0;
    
   /*!
    * If a number of possible holes are detected closeby it will be considered a hole.
    */
    for (int i= (5-1); i>=0; i--)
    {
      hole_samples_sum = hole_samples[i] + hole_samples_sum;
      //cout<<"\n\thole samples sum: "<< hole_samples_sum<<"\n";
    }
    
    hole_detected_flag = (hole_detected_flag)&&(hole_samples_sum >= minimum_samples);

    
    
   /*!
    * If any variation in the sensors bigger than the threshold is detected, and if there is't a hole nearby,
    * the hole will be detected (HPF). 350000 because the car stopped there. 500 for it's the GPS resolution
    * (assuming max car speed = 20m/s = 72km/h).*/
    if(((hole_detected_flag==1)&&(Iter_num>ARRAY_SIZE))&&((timebase_data[0]<STOP_TIME)&&((timebase_data[0]-last_time_detected)>500))) 
    {
     /*!
      * Some debugging values will be calculated, as the number of holes detected (NOH) until now
      * and the time of the occurance to compare to the real data aquisition.
      */
      hole_detected.write(1);
      NOH = NOH++; //number of holes
      last_time_detected = timebase_data[0];
      
     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data[0]%1000;
      seconds = (timebase_data[0] / 1000) % 60 ;
      minutes = ((timebase_data[0]/ (1000*60)) % 60);
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */
    }
    else
    {
      hole_detected.write(0);
    } 
    //..................................................
    
    wait();
    Iter_num = Iter_num++;
    //cout << "\t\t\n HRF: " << hole_rejection_flag<<"\n";
    //cout << "\n Iteration number: " << Iter_num<<"\n";
    //cout << "\n Number of Holes detected: "<< NOH<<"\n";
    
    
  }
}


void detector::l2_algorithm()
{ 
int  sensor1_data[ARRAY_SIZE];
int  sensor2_data[ARRAY_SIZE];
int  sensor3_data[ARRAY_SIZE];
int  sensor4_data[ARRAY_SIZE];
int  timebase_data[ARRAY_SIZE];

int  sensor1_mean=0; //mean value of the sensor1 data
int  sensor2_mean=0; //mean value of the sensor2 data
int  sensor3_mean=0; //mean value of the sensor3 data
int  sensor4_mean=0; //mean value of the sensor4 data
int  last_time_detected =0;

int  threshold = THRESHOLD; //maximum allowed variation to ignore
int  compare_temp =0;

int  hole_samples[80]; // if the number of minimum samples per hole is not achieved the hole won't be detected (LPF)
int  minimum_samples = 3;
int  hole_samples_sum = 0;

bool hole_detected_flag=0;
int  Iter_num = 0; //number of current iteration
int  NOH = 0; //number of detected holes

int milliseconds = 0;
int seconds = 0;
int minutes = 0;

    
  for (int i =79;i>=0;i--) //hole_samples inicialization
  {
    hole_samples[i]=0;
  }
  
  for (int i = (ARRAY_SIZE-1); i>=0; i--) //sensors and timebase inicialization
  {
    sensor1_data[i] =0;
    sensor2_data[i] =0;
    sensor3_data[i] =0;
    sensor4_data[i] =0;
    timebase_data[i]=0;
  }
  
 /*!
  * In the variables' declaration, all values are
  * initialized with zeros by default.
  */
  
  while ( true )
  { 

    for (int i = (ARRAY_SIZE-1); i>0; i--)
    {
      
     /*!
      * The arrays are realocated to receive the new data.
      * Also the mean value is recalculated.
      */
     
      sensor1_data[i] = sensor1_data[i-1];
      sensor2_data[i] = sensor2_data[i-1];
      sensor3_data[i] = sensor3_data[i-1];
      sensor4_data[i] = sensor4_data[i-1];
      timebase_data[i] = timebase_data[i-1];
      
      sensor1_mean = sensor1_data[i]+sensor1_mean;
      sensor2_mean = sensor2_data[i]+sensor2_mean;
      sensor3_mean = sensor3_data[i]+sensor3_mean;
      sensor4_mean = sensor4_data[i]+sensor4_mean;
     }
    
    
   /*!
    * The negative values of the data are set as positive.
    */
   
    sensor1_data[0] =sensor1.read();
    
    if(sensor1_data[0]<0)
    {
      sensor1_data[0] = -sensor1_data[0];
    }
        
    sensor2_data[0] =sensor2.read();
    
    if(sensor2_data[0]<0)
    {
      sensor2_data[0] = -sensor2_data[0];
    }
        
    sensor3_data[0] =sensor3.read();
    
    if(sensor3_data[0]<0)
    {
      sensor3_data[0] = -sensor3_data[0];
    }
        
    sensor4_data[0] =sensor4.read();
    
    if(sensor4_data[0]<0)
    {
      sensor4_data[0] = -sensor4_data[0];
    }

    
    
   /*!
    * The current data passes through a moving average filter to reduce noise.
    */
    sensor1_data[0] = (sensor1_data[0]+sensor1_data[1]+sensor1_data[2])/3;//+sensor1_data[3]+sensor1_data[4])/5; 
    sensor2_data[0] = (sensor2_data[0]+sensor2_data[1]+sensor2_data[2])/3;//+sensor2_data[3]+sensor2_data[4])/5;
    sensor3_data[0] = (sensor3_data[0]+sensor3_data[1]+sensor3_data[2])/3;//+sensor3_data[3]+sensor3_data[4])/5;
    sensor4_data[0] = (sensor4_data[0]+sensor4_data[1]+sensor4_data[2])/3;//+sensor4_data[3]+sensor4_data[4])/5;

    
    timebase_data[0] = timebase.read();
    
    
  /*!
   * The mean value is calculated using a moving average filter as well.
   */
  
    sensor1_mean = (sensor1_data[0]+sensor1_mean)/ARRAY_SIZE; 
    sensor2_mean = (sensor2_data[0]+sensor2_mean)/ARRAY_SIZE;
    sensor3_mean = (sensor3_data[0]+sensor3_mean)/ARRAY_SIZE;
    sensor4_mean = (sensor4_data[0]+sensor4_mean)/ARRAY_SIZE;

    
    
    //cout << "\nSensor mean: "<< sensor_mean <<"\n";
    
   /*!
    * If a variation is detected (hole_samples), but no other variation is 
    * detected close by, it is ignored (LPF).
    */
   
    for (int i = (REJECTED_HOLES); i>0;i--)
    {
      hole_samples[i] = hole_samples[i-1];
    }
    hole_samples[0] = 0;
    
    
   /*!
    * The compare_temp flag checks if the sum of the difference of each
    * sensor's data with the mean exceeds 4 times the threshold.
    */
    compare_temp = abs(sensor1_mean-sensor1_data[0]);
    compare_temp = compare_temp+abs(sensor2_mean-sensor2_data[0]);
    compare_temp = compare_temp+abs(sensor3_mean-sensor3_data[0]);
    compare_temp = compare_temp+abs(sensor4_mean-sensor4_data[0]);
        
    hole_detected_flag = (compare_temp>4*threshold);
    
    /*
    hole_detected_flag = ((sensor1_data[0] >= (sensor1_mean+threshold))||(sensor1_data[0] <= (sensor1_mean-threshold)));
    hole_detected_flag = ((sensor2_data[0] >= (sensor2_mean+threshold))||(sensor2_data[0] <= (sensor2_mean-threshold)))||(hole_detected_flag);
    hole_detected_flag = ((sensor3_data[0] >= (sensor3_mean+threshold))||(sensor3_data[0] <= (sensor3_mean-threshold)))||(hole_detected_flag);
    hole_detected_flag = ((sensor4_data[0] >= (sensor4_mean+threshold))||(sensor4_data[0] <= (sensor4_mean-threshold)))||(hole_detected_flag);
    */
    
   /*!
    * If the hole_detected_flag is set to 1, a possible hole is marked.
    */
    
    if (hole_detected_flag ==1)
    {
      hole_samples[0] = 1;
      //cout<<"\n\thole detected flag: "<< hole_detected_flag<<"\n";
    }

   /*!
    * If a number of possible holes are detected closeby it will be considered a hole.
    */
    hole_samples_sum = 0;
    for (int i= (5-1); i>=0; i--)
    {
      hole_samples_sum = hole_samples[i] + hole_samples_sum;
      //cout<<"\n\thole samples sum: "<< hole_samples_sum<<"\n";
    }
    
    hole_detected_flag = (hole_detected_flag)&&(hole_samples_sum >= minimum_samples);

    
   /*!
    * If any variation in the sensors bigger than the threshold is detected, and if there
    * is't a hole nearby, the hole will be detected (HPF). 500 because it's the GPS resolution
    * (assuming max car speed = 20m/s = 72km/h).
    */
   
    if(((hole_detected_flag==1)&&(Iter_num>ARRAY_SIZE))&&((timebase_data[0]<STOP_TIME)&&((timebase_data[0]-last_time_detected)>500))) 
    {
     /*!
      * Some debugging values will be calculated, as the number of holes detected (NOH) until now
      * and the time of the occurance to compare to the real data aquisition.
      */
     
      hole_detected.write(1);
      NOH = NOH++; //number of holes
      last_time_detected = timebase_data[0];
      
     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data[0]%1000;
      seconds = (timebase_data[0] / 1000) % 60 ;
      minutes = ((timebase_data[0]/ (1000*60)) % 60);
      
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */
     
    }
    else
    {
      hole_detected.write(0);
    } 
    
    wait();
    Iter_num = Iter_num++;
    //cout << "\t\t\n HRF: " << hole_rejection_flag<<"\n";
    //cout << "\n Iteration number: " << Iter_num<<"\n";
    //cout << "\n Number of Holes detected: "<< NOH<<"\n";
  }
    
}


void detector::p2_algorithm()
{


  vector<float>  sensor1_data(ARRAY_SIZE);
  vector<float>  sensor2_data(ARRAY_SIZE);
  vector<float>  sensor3_data(ARRAY_SIZE);
  vector<float>  sensor4_data(ARRAY_SIZE);
  vector<int>  timebase_data(ARRAY_SIZE);
  float coeficients[ARRAY_SIZE] = {0.0005, 0.001, 0.0025, 0.005, 0.0075, 0.01, 0.0125, 0.025, 0.0125, 0.01, 0.0075, 0.005, 0.0025, 0.001, 0.0005};
  vector<char> counter(4);
  char isAhole;
  
  vector<int>  hole_rejection(REJECTED_HOLES); //variable to reject close holes
  int  hole_rejection_flag=0;
  bool hole_detected_flag=0;
  float  filter1 = 0, filter2 = 0, filter3 = 0, filter4 = 0;
  int  Iter_num = 0; //number of current iteration
  int  NOH = 0; //number of detected holes

  int milliseconds = 0;
  int seconds = 0;
  int minutes = 0;    
  
  /*!
  * In the variables' declaration, all vectors are
  * initialized with zeros by default.
  */  
  
    while(1)
    {
     /*!
      * The arrays are realocated to receive a new data.
      */
      for (int i = (ARRAY_SIZE-1); i>0; i--)
      {
	sensor1_data.at(i) = sensor1_data.at(i-1);
	sensor2_data.at(i) = sensor2_data.at(i-1);
	sensor3_data.at(i) = sensor3_data.at(i-1);
	sensor4_data.at(i) = sensor4_data.at(i-1);
	timebase_data.at(i) = timebase_data.at(i-1);
      }

     /*!
      * The new values are converted to distance
      */
      sensor1_data.at(0) = sensor1.read() / 27.6233 / 2;
      if (sensor1_data.at(0) > 3500) sensor1_data.at(0) = 900;
      sensor2_data.at(0) = sensor2.read() / 27.6233 / 2;
      if (sensor2_data.at(0) > 3500) sensor2_data.at(0) = 900;
      sensor3_data.at(0) = sensor3.read() / 27.6233 / 2;
      if (sensor3_data.at(0) > 3500) sensor3_data.at(0) = 900;
      sensor4_data.at(0) = 900 / 27.6233 / 2;
      timebase_data.at(0) = timebase.read();
      
      for(int i = 0; i < (ARRAY_SIZE); i++)
      {
      /*!
       * The data is filtered.
       */
	filter1 = filter1 + coeficients[i]*sensor1_data.at(i);
	filter2 = filter2 + coeficients[i]*sensor2_data.at(i);
	filter3 = filter3 + coeficients[i]*sensor3_data.at(i);
	filter4 = filter4 + coeficients[i]*sensor4_data.at(i);
	//cout << filter1 << endl << filter2<<endl<<filter3<<endl<<filter4<<endl;
      }
      /*!
       * It's checed if the values are out of the limits.
       */
      if((filter1 > UPPER_LIMIT)||(filter1 < LOWER_LIMIT)) hole_detected_flag = 1;
      if((filter2 > UPPER_LIMIT)||(filter2 < LOWER_LIMIT)) hole_detected_flag = 1;
      if((filter3 > UPPER_LIMIT)||(filter3 < LOWER_LIMIT)) hole_detected_flag = 1;
      if((filter4 > UPPER_LIMIT)||(filter4 < LOWER_LIMIT)) hole_detected_flag = 1;
	
    /*!
    * If a possible hole is detected near another hole by REJECTED_HOLES
    * samples, it will be ignored.
    */    
    for (int i = (REJECTED_HOLES-1); i>0; i--)
    {
      hole_rejection.at(i) = hole_rejection.at(i-1);
      hole_rejection_flag = hole_rejection[i]+hole_rejection_flag;
    }
    
    /*!
     * It's then decided if it's a hole or not
     */
    
    hole_detected_flag = (hole_rejection_flag==0)&&(hole_detected_flag);
    
    
  
    if(hole_detected_flag==1)
    {
     /*!
      * Some debugging values will be calculated, as the number of holes detected (NOH) until now
      * and the time of the occurance to compare to the real data aquisition.
      */
      hole_detected.write(1);
      NOH = NOH++;
      hole_rejection.at(0) = 1;
      hole_rejection_flag =0;
      
     /*!
      *
      * Prints for debug:
      */
     /** cout << "\n\n\n Iteration number: " << Iter_num;*/
     /** cout << "\n Current data:    "<< sensor1_data.at(0)<<" "<<sensor2_data.at(0)<<" "<<sensor3_data.at(0); //Sensor current data filtered*/
     /** cout << "\n Mean data:       "<< sensor1_mean<<" "<<sensor2_mean<<" "<<sensor3_mean; //mean values being compared*/
     /** cout << "\n Data Difference: "<< abs(sensor1_mean-sensor1_data.at(0))<<" "<<abs(sensor2_mean-sensor2_data.at(0))<<" "<<abs(sensor3_mean-sensor3_data.at(0));*/
     /*! cout << "\n Number of Holes detected: "<< NOH; // sum of holes detected
      *
      */

     /*!
      * The time of detection is calculated from the timebase.
      */
      milliseconds = timebase_data.at(0)%1000;
      seconds = (timebase_data.at(0) / 1000) % 60 ;
      minutes = ((timebase_data.at(0)/ (1000*60)) % 60);
     /*!
      * Time print for debug:
      * cout << "\n Time of the detection: "<<minutes<<":"<<seconds<<":"<<milliseconds<<"  Timebase: "<<timebase_data[0];
      *
      */
    }
    else
    {
      hole_detected.write(0);
      hole_rejection.at(0) = 0;
      hole_rejection_flag =0;
    } 
    
    wait();
    Iter_num = Iter_num++;
    /*!
     * In the end the counter and the filtered values are reset.
     */
      for (int i = 0; i<counter.size(); i++)
      {
	counter.at(i) = 0;
      }
    isAhole = 0;
    filter1 = 0;
    filter2 = 0;
    filter3 = 0;
    filter4 = 0;
  }
}