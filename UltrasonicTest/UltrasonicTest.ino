/*
 * HCSR04Ultrasonic/examples/UltrasonicDemo/UltrasonicDemo.pde
 *
 * SVN Keywords
 * ----------------------------------
 * $Author: cnobile $
 * $Date: 2011-09-17 02:43:12 -0400 (Sat, 17 Sep 2011) $
 * $Revision: 29 $
 * ----------------------------------
 */

#include <Ultrasonic.h>

#define TRIGGER_PIN  5
#define ECHO_PIN     4

#define TRIGGER_PIN2  8
#define ECHO_PIN2     9

#define TRIGGER_PIN3  6
#define ECHO_PIN3     7
Ultrasonic U1(TRIGGER_PIN, ECHO_PIN);

Ultrasonic U2(TRIGGER_PIN2, ECHO_PIN2);

Ultrasonic U3(TRIGGER_PIN3, ECHO_PIN3);

void setup()
  {
  Serial.begin(9600);
  }

void loop()
  {
  static float cm, cm2, cm3;
  long microsec = U1.timing();
  long microsec2 = U2.timing();
  long microsec3 = U3.timing();
  
  cm = U1.convert(microsec, Ultrasonic::CM);
  cm3 = U2.convert(microsec2, Ultrasonic::CM);
  cm2 = U3.convert(microsec3, Ultrasonic::CM);
  //inMsec = ultrasonic.convert(microsec, Ultrasonic::IN);
  Serial.print("CM: ");
  Serial.print(cm);
  Serial.print(", CM: ");
  Serial.print(cm2);
  Serial.print(", CM: ");
  Serial.println(cm3);
  delay(1);
  }
