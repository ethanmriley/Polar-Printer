/* 
This is a test sketch for the Adafruit assembled two Motor Shields for Arduino
v2
It runs two stepper motors on separate boards simultaneously.
Requires input by user on Serial Monitor to begin.
*/

#include <AccelStepper.h>           // For running motors simultaneously
#include <Wire.h>
#include <Adafruit_MotorShield.h>   // For interfacing with Motor Shields
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "math.h"                   // For asin
#include <avr/pgmspace.h>

//PINS
const int pingPin = 7;          // Range-Finder Pin

// GLOBAL VARIABLES
float y_Offset = 0;             // 1.25 inch offset in y-direction of radial arm.
float calibrationDistance=8.92; // This is the distance from the ultrasonic range-finder for the 
                                // radial arm to be positioned correctly
float x_Offset=0;               // Starting position for radial arm is 5" so we don't have to deal with neg. numbers
int inputVal;                   // used to store serial input
int numPos = 10;                 // Number of positions to plot
int currPos = 0;                // Current index of position arrays
const float PROGMEM radii[774] = {6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,6.48,7.56,8.64,9.72,9.72,8.64,7.56,6.48,4.32,3.24,2.16,1.08,1.08,2.16,3.24,4.32,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,11.88,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,10.08,11.16,11.16,10.08,7.92,6.84,5.76,4.68,3.60,4.68,5.76,6.84,7.92,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.52,10.44,9.36,8.28,7.20,8.28,9.36,10.44,11.52,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88,11.88,10.80,11.88};       // inches
const float PROGMEM angles[774] = {2.00,4.00,6.00,8.00,12.00,14.00,16.00,18.00,2.00,4.00,6.00,8.00,12.00,14.00,16.00,18.00,22.00,24.00,26.00,28.00,32.00,34.00,36.00,38.00,22.00,24.00,26.00,28.00,32.00,34.00,36.00,38.00,42.00,44.00,46.00,48.00,52.00,54.00,56.00,58.00,42.00,44.00,46.00,48.00,52.00,54.00,56.00,58.00,62.00,64.00,66.00,68.00,72.00,74.00,76.00,78.00,62.00,64.00,66.00,68.00,72.00,74.00,76.00,78.00,82.00,84.00,86.00,88.00,92.00,94.00,96.00,98.00,82.00,84.00,86.00,88.00,92.00,94.00,96.00,98.00,102.00,104.00,106.00,108.00,112.00,114.00,116.00,118.00,102.00,104.00,106.00,108.00,112.00,114.00,116.00,118.00,122.00,124.00,126.00,128.00,132.00,134.00,136.00,138.00,122.00,124.00,126.00,128.00,132.00,134.00,136.00,138.00,142.00,144.00,146.00,148.00,152.00,154.00,156.00,158.00,142.00,144.00,146.00,148.00,152.00,154.00,156.00,158.00,162.00,164.00,166.00,168.00,172.00,174.00,176.00,178.00,162.00,164.00,166.00,168.00,172.00,174.00,176.00,178.00,182.00,184.00,186.00,188.00,192.00,194.00,196.00,198.00,182.00,184.00,186.00,188.00,192.00,194.00,196.00,198.00,202.00,204.00,206.00,208.00,212.00,214.00,216.00,218.00,202.00,204.00,206.00,208.00,212.00,214.00,216.00,218.00,222.00,224.00,226.00,228.00,232.00,234.00,236.00,238.00,222.00,224.00,226.00,228.00,232.00,234.00,236.00,238.00,242.00,244.00,246.00,248.00,252.00,254.00,256.00,258.00,242.00,244.00,246.00,248.00,252.00,254.00,256.00,258.00,262.00,264.00,266.00,268.00,272.00,274.00,276.00,278.00,262.00,264.00,266.00,268.00,272.00,274.00,276.00,278.00,282.00,284.00,286.00,288.00,292.00,294.00,296.00,298.00,282.00,284.00,286.00,288.00,292.00,294.00,296.00,298.00,302.00,304.00,306.00,308.00,312.00,314.00,316.00,318.00,302.00,304.00,306.00,308.00,312.00,314.00,316.00,318.00,322.00,324.00,326.00,328.00,332.00,334.00,336.00,338.00,322.00,324.00,326.00,328.00,332.00,334.00,336.00,338.00,342.00,344.00,346.00,348.00,352.00,354.00,356.00,358.00,342.00,344.00,346.00,348.00,352.00,354.00,356.00,358.00,12.00,28.00,32.00,48.00,52.00,68.00,72.00,88.00,92.00,108.00,112.00,128.00,132.00,148.00,152.00,168.00,172.00,188.00,192.00,208.00,212.00,228.00,232.00,248.00,252.00,268.00,272.00,288.00,292.00,308.00,312.00,328.00,332.00,348.00,352.00,8.00,2.10,4.10,16.10,18.10,2.10,4.10,6.10,8.10,10.10,12.10,14.10,16.10,18.10,22.10,24.10,36.10,38.10,22.10,24.10,26.10,28.10,30.10,32.10,34.10,36.10,38.10,42.10,44.10,56.10,58.10,42.10,44.10,46.10,48.10,50.10,52.10,54.10,56.10,58.10,62.10,64.10,76.10,78.10,62.10,64.10,66.10,68.10,70.10,72.10,74.10,76.10,78.10,82.10,84.10,96.10,98.10,82.10,84.10,86.10,88.10,90.10,92.10,94.10,96.10,98.10,102.10,104.10,116.10,118.10,102.10,104.10,106.10,108.10,110.10,112.10,114.10,116.10,118.10,122.10,124.10,136.10,138.10,122.10,124.10,126.10,128.10,130.10,132.10,134.10,136.10,138.10,142.10,144.10,156.10,158.10,142.10,144.10,146.10,148.10,150.10,152.10,154.10,156.10,158.10,162.10,164.10,176.10,178.10,162.10,164.10,166.10,168.10,170.10,172.10,174.10,176.10,178.10,182.10,184.10,196.10,198.10,182.10,184.10,186.10,188.10,190.10,192.10,194.10,196.10,198.10,202.10,204.10,216.10,218.10,202.10,204.10,206.10,208.10,210.10,212.10,214.10,216.10,218.10,222.10,224.10,236.10,238.10,222.10,224.10,226.10,228.10,230.10,232.10,234.10,236.10,238.10,242.10,244.10,256.10,258.10,242.10,244.10,246.10,248.10,250.10,252.10,254.10,256.10,258.10,262.10,264.10,276.10,278.10,262.10,264.10,266.10,268.10,270.10,272.10,274.10,276.10,278.10,282.10,284.10,296.10,298.10,282.10,284.10,286.10,288.10,290.10,292.10,294.10,296.10,298.10,302.10,304.10,316.10,318.10,302.10,304.10,306.10,308.10,310.10,312.10,314.10,316.10,318.10,322.10,324.10,336.10,338.10,322.10,324.10,326.10,328.10,330.10,332.10,334.10,336.10,338.10,342.10,344.10,356.10,358.10,342.10,344.10,346.10,348.10,350.10,352.10,354.10,356.10,358.10,2.20,4.20,6.20,8.20,10.20,12.20,14.20,16.20,18.20,22.20,24.20,26.20,28.20,30.20,32.20,34.20,36.20,38.20,42.20,44.20,46.20,48.20,50.20,52.20,54.20,56.20,58.20,62.20,64.20,66.20,68.20,70.20,72.20,74.20,76.20,78.20,82.20,84.20,86.20,88.20,90.20,92.20,94.20,96.20,98.20,102.20,104.20,106.20,108.20,110.20,112.20,114.20,116.20,118.20,122.20,124.20,126.20,128.20,130.20,132.20,134.20,136.20,138.20,142.20,144.20,146.20,148.20,150.20,152.20,154.20,156.20,158.20,162.20,164.20,166.20,168.20,170.20,172.20,174.20,176.20,178.20,182.20,184.20,186.20,188.20,190.20,192.20,194.20,196.20,198.20,202.20,204.20,206.20,208.20,210.20,212.20,214.20,216.20,218.20,222.20,224.20,226.20,228.20,230.20,232.20,234.20,236.20,238.20,242.20,244.20,246.20,248.20,250.20,252.20,254.20,256.20,258.20,262.20,264.20,266.20,268.20,270.20,272.20,274.20,276.20,278.20,282.20,284.20,286.20,288.20,290.20,292.20,294.20,296.20,298.20,302.20,304.20,306.20,308.20,310.20,312.20,314.20,316.20,318.20,322.20,324.20,326.20,328.20,330.20,332.20,334.20,336.20,338.20,342.20,344.20,346.20,348.20,350.20,352.20,354.20,356.20,358.20,8.30,10.30,12.30,28.30,30.30,32.30,48.30,50.30,52.30,68.30,70.30,72.30,88.30,90.30,92.30,108.30,110.30,112.30,128.30,130.30,132.30,148.30,150.30,152.30,168.30,170.30,172.30,188.30,190.30,192.30,208.30,210.30,212.30,228.30,230.30,232.30,248.30,250.30,252.30,268.30,270.30,272.30,288.30,290.30,292.30,308.30,310.30,312.30,328.30,330.30,332.30,348.30,350.30,352.30};      // degrees

// Create a motor shield object with the default I2C address, 0x60
Adafruit_MotorShield AFMSbot = Adafruit_MotorShield();
// Create a motor shield object with the I2C address 0x61 
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *Angle  = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *Radius = AFMStop.getStepper(200, 1);

// Create wrapper functions for use with the AccelStepper Library
// Set so that forwardstepA corresponds to CW and backwardstepA
// corresponds to CCW
void forwardstepA() {  
  Angle->onestep(FORWARD, SINGLE);
}
void backwardstepA() {  
  Angle->onestep(BACKWARD, SINGLE);
}
// Set so forwardstepR corresponds to 'OUT' and backwardstepA
// corrseponds to 'IN'
void forwardstepR() {  
  Radius->onestep(FORWARD, SINGLE);
}
void backwardstepR() {  
  Radius->onestep(BACKWARD, SINGLE);
}
// Create Astepper and Rstepper objects from AccelStepper library
AccelStepper Astepper(forwardstepA, backwardstepA); // use functions to step
AccelStepper Rstepper(forwardstepR, backwardstepR); // use functions to step

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Plotting Program Start!");
  delay(5000);                  // Delay to allow rangers time to steup otherwise initial reading will be erroneous
  float distance_=distance();   // distance for calibration needs to be grabbed before motors are initialized
  Serial.print("Initial Distance: ");
  Serial.println(distance_);

  AFMStop.begin();             // create with the default frequency 1.6KHz
  AFMSbot.begin();
  
  // Set speed in rpm
  Astepper.setSpeed(200);
  Rstepper.setSpeed(300);
  
  // Wait for user input to start
  Serial.println("Waiting for user input to start... Please make sure motor power is ON.");
  inputVal = 0;
  while (inputVal == 0){
    if(Serial.available()== 0){
      inputVal = Serial.parseInt(); //read int or parselong for ..long...
    }
  }

  Serial.println("Beginning system calibration...");
  calibrate(distance_);
  moveToPolarCoord(pgm_read_float_near(&radii[currPos]), pgm_read_float_near(&angles[currPos]));
}

float distance(){
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);
  float distance_=(float) duration;
  distance_ = distance_ / 74.0 / 2.0; // convert microseconds to inches
  return distance_;
}

void calibrate(float distance_){
  float dist = distance_-calibrationDistance;
  long steps = (long) (dist/0.0015);
  if(dist<0){
    dist=abs(dist);
    steps = (long) (dist/0.0015);
    Rstepper.move(steps);
    Rstepper.setSpeed(300);
  }
  else{
    Rstepper.setCurrentPosition(steps);
    Rstepper.moveTo(0);
    Rstepper.setSpeed(-300);
  }
  Serial.print("Distance to move: ");
  Serial.println(dist);
  Rstepper.runToPosition();  
  // Set current positions so we only have to deal with positive positions
  Astepper.setCurrentPosition(968);  // 90 deg
  long initialDist = (long) (x_Offset/0.0015);
  Rstepper.setCurrentPosition(initialDist); // Disc Diameter is 15.5".  
}

// There are 0.186 deg/step for the angle motor set to 200 steps per rotation
// converAngleToSteps() takes angle in degrees and returns number of steps
long convertAngleToSteps(float degrees_){
  long steps=(long) (degrees_/0.186);
  return steps;
}

// There are 0.0015"/step for the radius motor set to 200 steps per rotation
long convertRadiusToSteps(float inches){
  float x=sqrt(inches*inches-y_Offset*y_Offset); // If there is an offset of the radial arm
                                       // then lateral movement corresponds to the
                                       // projection of the desired radius with y=a
                                       // onto the x-axis
                                       // if the radial arm is perfectly aligned with
                                       // the x-axis, offset=0
 long steps= (long) (x/0.0015);
 return steps;
}

float convertStepsToRadius(long steps){
  float inches = (float) (steps)*0.0015;
  inches = inches-x_Offset;
  inches = abs(inches);
  inches = sqrt(inches*inches+y_Offset*y_Offset);
  return inches;
}

float convertStepsToAngle(long steps){
  float degree = (float) (0.186*steps);
  return degree;
}

// move to Polar Coordinate position (r, theta) given in degrees and inches
void moveToPolarCoord(float r, float theta){
  // if there is a y-axis offset, then the change in angle due to a change
  // in radius must be accounted for. See documentation for derivation.
  /*if(y_Offset!=0 && convertStepsToRadius(Rstepper.currentPosition())!=r){
    float dtheta = asin(y_Offset/convertStepsToRadius(Astepper.currentPosition()))-asin(y_Offset/r); // returns value in radians
    dtheta = 57.3*dtheta; // Convert radians to degrees
    float thetaNew = convertStepsToAngle(Astepper.currentPosition())-dtheta;
    Serial.print("New angle is: ");
    Serial.println(thetaNew);
    Astepper.setCurrentPosition(convertAngleToSteps(thetaNew));
  }*/
  
  Serial.println("Moving to r/step; deg/step: ");
  Serial.print(r); Serial.print(", ");
  Serial.print(convertRadiusToSteps(r));
  Serial.print("; ");
  Serial.print(theta); Serial.print(", ");
  Serial.println(convertAngleToSteps(theta));
  Serial.print("From (r,theta): "); Serial.print(Rstepper.currentPosition()); 
  Serial.print(", "); Serial.println(Astepper.currentPosition());
  if(convertRadiusToSteps(r)<Rstepper.currentPosition()){
    Rstepper.setSpeed(-300);  
  }
  else{
    Rstepper.setSpeed(300);  
  }
  if(convertAngleToSteps(theta)<Astepper.currentPosition()){
    Astepper.setSpeed(-200);  
  }
  else{
    Astepper.setSpeed(200);  
  }
  Astepper.moveTo(convertAngleToSteps(theta));
  Rstepper.moveTo(convertRadiusToSteps(r));
  // Wait for user input to start
  /*Serial.println("Waiting for user input to move to next position.");
  inputVal = 0;
  while (inputVal == 0){
    if(Serial.available()== 0){
      inputVal = Serial.parseInt(); //read int or parselong for ..long...
    }
  }*/
}
void(* resetFunc) (void) = 0;//declare reset function at address 0

void loop() { 
  if(Rstepper.currentPosition()==Rstepper.targetPosition() && Astepper.currentPosition()==Astepper.targetPosition() && currPos<numPos){
    Serial.println("Setting next Position");
    currPos++;
    moveToPolarCoord(pgm_read_float_near(&radii[currPos]), pgm_read_float_near(angles[currPos]));  
  }

  if(currPos==numPos){
    Serial.println("Plotting Complete!");
    Serial.println("Disabling Motors...");
    Rstepper.disableOutputs();
    Astepper.disableOutputs();
    Serial.println("Resetting Program... Please check for overheating on motors, shields, and supplies.");
    delay(1000);
    resetFunc(); //call reset 
  }
  /*
  Serial.print("R Position: ");
  Serial.println(Rstepper.currentPosition());
  Serial.print("A Position: ");
  Serial.println(Astepper.currentPosition());*/
  Rstepper.runSpeedToPosition();
  Astepper.runSpeedToPosition();
}


