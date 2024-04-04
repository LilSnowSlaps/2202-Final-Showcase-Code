//Final Showcase Code


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>
#include "ESP32Servo.h"

// Function declarations
void doHeartbeat();

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725  
const int cServoPin          = 41;                 // GPIO pin for servo motor  
const int cServoPin2          = 42;                    // GPIO pin for servo motor
const int cServoChannel      = 5;                     // PWM channel used for the RC servo motor
const int cServoChannel2      = 6;                     // PWM channel used for the RC servo motor

//set up my own servo channel 
Servo servo3;
const int servoPin3 = 9; // Define the pin connected to the servo
const int minAngle = 10; // Minimum angle for the servo
const int maxAngle = 170; // Maximum angle for the servo
const int interval = 3000; // Interval in milliseconds for changing angle
static int currentAngle = minAngle;
unsigned long previousMillis2 = 0; // Store the previous time
int counter = 0; //count for loops in phase 2

// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
unsigned long currentMillis2 = 0;

//HeartBEat Stuff
unsigned long colorReadStart = 0;
unsigned long colorReadInterval = 2000;
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

//time
long timecount3000ms = 0;
bool timeup3000ms = false;
long timecount500ms = 0;
bool timeup500ms = false;
long timecount80s = 0;
bool timeup80s = false;
long currenttime =0;

//Switch Parameter
int part = 0;
int b2b = 0;

//IR Sensor 
#define IR_DETECTOR         14   
IR Scan = IR();                                                                // instance of IR for detecting IR signals

//ultrasonic Definitions
const int trigPin = 2;
const int echoPin = 1;
int walldistance = 0;

//Color Sensor Print values 
#define PRINT_COLOUR 999
bool greenFlag = 0; // 0 means green is not detected, 1 means green is detected

// Define motor control pins
#define LEFT_MOTOR_A 35
#define LEFT_MOTOR_B 36
#define RIGHT_MOTOR_A 37
#define RIGHT_MOTOR_B 38

// Define constants for movement
#define DRIVE_SPEED 150 // Adjust as needed
#define TURN_SPEED 150  // Adjust as needed
#define TURN_DURATION 1000 // Time in milliseconds to turn
#define DRIVE_DURATION 3000 // Time in milliseconds to drive forward
#define SQUARE_DURATION 500 // Time in milliseconds to drive in a square

//variable declarations
unsigned int mode = 0;
unsigned int modePBDebounce = 0;

// check ultrasonic distance function
double getDistance() {
  double distance;
  long duration;

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance in metres
  distance = duration * 0.034 / 2;

  return distance;
}

//Prototypez of functions
void moveForward(int a);
void moveBackward( int b);
void turnLeft(int c);
void turnRight(int d );
void donothing();
void echoInterrupt();
void IRinterrupt();

// update heartbeat LED
// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}
void setServoPosition(int degrees) {  //writes the servo to a duty cycle   
  int dutyCycle = degreesToDutyCycle(degrees);
  ledcWrite(cServoChannel, dutyCycle);
}

int degreesToDutyCycle(int degrees) { //maps servo degrees to duty cycle
  // Map degrees (0-180) to duty cycle (400-2100). changed from 500-2500 before
  return map(degrees, 0, 180, 400, 2100);
}


void setup() {

   // Attach the servo3 to the pin (custom servo)
  servo3.attach(servoPin3);

  //IR
   Scan.Begin(IR_DETECTOR, 1200);                                              //set up IR Detection @ 1200 baud


  //UltraSonic 
   pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication


  //Color Sensor
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED
  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 
  pinMode(cServoPin, OUTPUT);                      // configure servo GPIO for output
  pinMode(cServoPin2, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcSetup(cServoChannel2, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);         // assign servo pin to servo channel
  ledcAttachPin(cServoPin2, cServoChannel2);         // assign servo pin to servo channel

  //interrupt
   attachInterrupt(digitalPinToInterrupt(echoPin), echoInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14),IRinterrupt,HIGH);


  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

}

void loop() {
 
  interrupts();
   currentMillis2 = millis(); // Get the current time
  //Timing Logic 
  currenttime = millis();
  timecount3000ms = timecount3000ms +1;
  if (timecount3000ms>3000){
    timeup3000ms = true;
    timecount3000ms = 0;
  }

  timecount500ms = timecount500ms +1;
  if (timecount500ms>500){
    timeup500ms = true;
    timecount500ms = 0;
  }

  timecount80s = timecount80s +1;
if(timecount80s>80000){
  timeup80s = true;
  timecount80s = 0;
}

  switch(part){

    case 0: //Drive Algorithm & sorting algorythm
      noInterrupts(); //stops the interrupts
    for(int i = 0; i<4; i++){

 //color sensor code 
  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
  }

 // Check if it's time to read the color
  if (millis() - colorReadStart >= colorReadInterval || colorReadStart == 0 && timeup80s == false) {
    // Read color values from the sensor after a 2-second interval for accurate reading
    if (millis() - colorReadStart >= colorReadInterval) {
      tcs.getRawData(&r, &g, &b, &c);          
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      // Check if the green value is above 15
      if (g > 40 && g < 2000 && c > 160 && c < 200) {        
        Serial.println("Green Detected.");
        greenFlag = 1;
      } else {
        Serial.println("Green not detected");
        greenFlag = 0;
      }
      // Reset the color read start time
      colorReadStart = millis();
    }
  }
  if (greenFlag == 1) {
    Serial.println("Writing 180 degrees to servo.");
     ledcWrite(cServoChannel, degreesToDutyCycle(180));
    ledcWrite(cServoChannel2, degreesToDutyCycle(0));
    moveForward(4000);
    turnLeft(2000);
    moveForward(4000);
    turnLeft(6000);
    moveForward(4000);
    turnLeft(6000);
    moveForward(8000);
    turnLeft(2000);
    }
  else {
      // 149 Degrees was the highest we could go before, it was because of the degreesToDutyCycle function being uncalibrated
      // this degreesToDutyCycle function maps 0-180 degrees to 500-2500 pwm.
      // to 400-2100 pwm.
      Serial.println("Writing 0 degrees to servo.");
      ledcWrite(cServoChannel, degreesToDutyCycle(90));
      ledcWrite(cServoChannel2, degreesToDutyCycle(90));
    moveForward(4000);
    turnLeft(2000);
    moveForward(4000);
    turnLeft(6000);
    moveForward(4000);
    turnLeft(6000);
    moveForward(8000);
    turnLeft(2000);
    }
  } 

    if(timeup80s){
      part = 1;
    }
     break;

  case 1: //Back to base algorithm
    switch (b2b){
      case 0:
       //IR & Ultrasonic Loop Code 
      if (Scan.Available()) {                                            // if IR data is received, stop and start moving in that direction
                  donothing();
                  Serial.println(Scan.Get_IR_Data());                              // output received data to serial
                  double ultraDistance = getDistance();
                  Serial.print("Distance (metres): ");
                    Serial.println(ultraDistance);
                  if(ultraDistance > 50){
                     interrupts(); // allows 
                    moveForward(1000);
                  }
                  else{
                    turnLeft(4000);
                    b2b = 1;
                  }
                }
      break;

      case 1:
      servo3.write(10); //raise the gate 
        part = 2;
      break;

    default:
      Serial.println("SOMETHING IS VERY WRONG");
      break;
    }
  break;
    
  case 2: //end of process
  donothing();
  break;

  default: 
  Serial.println("Switch Statement is not working");
  break;
  }
}
  


//Drive Methods 
void moveForward(int duration) { //move forwards
  for(int i = 0; i<duration, i++;){
   digitalWrite(LEFT_MOTOR_A, HIGH); //forward channel high
  digitalWrite(LEFT_MOTOR_B, LOW);
  digitalWrite(RIGHT_MOTOR_A, HIGH); //forward channel high
  digitalWrite(RIGHT_MOTOR_B, LOW);
  }
}

void moveBackward(int duration) {
    for(int i = 0; i<duration, i++;){
  digitalWrite(LEFT_MOTOR_A, LOW);
  digitalWrite(LEFT_MOTOR_B, HIGH); //backward channel high
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, HIGH); //backward channel high
  }
}

void turnLeft(int duration) {
    for(int i = 0; i<duration, i++;){
  digitalWrite(LEFT_MOTOR_A, LOW);
  digitalWrite(LEFT_MOTOR_B, HIGH); //backwards channel high
  digitalWrite(RIGHT_MOTOR_A, HIGH); //forwards channel high
  digitalWrite(RIGHT_MOTOR_B, LOW);
  }
}

void turnRight(int duration) {
    for(int i = 0; i<duration, i++;){
  digitalWrite(LEFT_MOTOR_A, HIGH); //forwrads channel high
  digitalWrite(LEFT_MOTOR_B, LOW);
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, HIGH); //backwards channel high
  }
}

void donothing(){
   digitalWrite(LEFT_MOTOR_A, LOW); //forwrads channel high
  digitalWrite(LEFT_MOTOR_B, LOW);
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, LOW); //backwards channel high
}

void echoInterrupt() { //ultrasonic interrupt
  noInterrupts();
  walldistance  = getDistance(); //gets current distance from an object
  if (walldistance < 50) { //if its less than 50 cm turn around 
    timeup3000ms = false;
    for ( int i = 0; 3000 > i, i ++;){
      turnLeft(1);
      donothing();
    }
  }
}

void IRInterrupt() { //ir interrupt
   if (Scan.Available()) {                                            // if data is received
               interrupts(); //allows for the ultrasonic interrupt 
              donothing(); //stop
              moveForward(10000); //move forwards a long time 
              Serial.println(Scan.Get_IR_Data());                              // output received data to serial
            }
}

