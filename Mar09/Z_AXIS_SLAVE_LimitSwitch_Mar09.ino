//Z-Axis Movement Code

#include "hardware/i2c.h"
#include <AccelStepper.h>
#include <Wire.h> 

#define motorInterfaceType 1 //required by AccelStepper type
#define Z_MOTION 56       // z-motion i2c address
#define dirPin 16         // motor direction
#define stepPin 17        // step enable
#define stepMs1Pin 21     // M1 A4988 step enable
#define stepMs2Pin 20     // M2 A4988 step enable
#define stepMs3Pin 19     // M3 A4988 step enable
#define upperLimit_pin 0
#define LED_OutofRange 9

const int stepsPerRevolution = 3200;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
int initial_homing=-1;
int prev_z1;

void setup()
 {
    stepper.setMaxSpeed(100);              // stepper max speed of steps per second
    stepper.setAcceleration(30);
    pinMode(LED_OutofRange,OUTPUT);
    pinMode(upperLimit_pin, OUTPUT);            // homing switch pin
    pinMode(stepMs1Pin, OUTPUT);            // set Ms1Pin as an Outpu, Pin (21)
    pinMode(stepMs2Pin, OUTPUT);            // set Ms2Pin as an Outpu, Pin (20)         
    pinMode(stepMs3Pin, OUTPUT);            // set Ms3Pin as an Outpu, Pin (19)
    pinMode(LED_BUILTIN, OUTPUT);           // Built in LED goes high to indicate opperation function
    digitalWrite(stepMs1Pin,HIGH);          // microstepping A4988 pins MS1: 0 |full step, | 1 | 1/2 step, | 0 | 1/4 step, | 1 | 1/8 step, | 1 | 1/16 step,
    digitalWrite(stepMs2Pin,HIGH);          // microstepping A4988 pins MS2: 0 |full step, | 0 | 1/2 step, | 1 | 1/4 step, | 1 | 1/8 step, | 1 | 1/16 step,
    digitalWrite(stepMs3Pin,HIGH);          // microstepping A4988 pins MS3: 0 |full step, | 0 | 1/2 step, | 0 | 1/4 step, | 0 | 1/8 step, | 1 | 1/16 step,
    gpio_set_function(4, GPIO_FUNC_I2C);    // initializes GPIO pin 4 as the I2C as the SDA line
    gpio_set_function(5, GPIO_FUNC_I2C);    // initializes GPIO pin 4 as the I2C as the SCL line
    gpio_pull_down(0);                        // initializes internal pull-up resistor to GPIO pin 0
    gpio_pull_up(4);                        // initializes internal pull-up resistor to GPIO pin 4
    gpio_pull_up(5);                        // initializes internal pull-up resistor to GPIO pin 5
    Wire.begin(Z_MOTION);                   // join i2c bus with address #56
    Wire.onReceive(receiveEvent);           // register event
    Wire.onRequest(requestEvent);
    Serial.begin(9600);                     // start serial for output
    Serial.printf("Slave Sketch here...");  // Serial Skave function check 
    HomingPoint();
  }


void HomingPoint()
  {
    while(!digitalRead(upperLimit_pin))  //pull-up resistor keeps it high, press makes it low
      {         
        stepper.moveTo(initial_homing);
        stepper.run();
        initial_homing--;
        delay(5);
      }
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(200);              // stepper max speed of steps per second
    stepper.setAcceleration(30);
    initial_homing=1;
    while(digitalRead(upperLimit_pin))  //pull-up resistor keeps it high, press makes it low
      {     
        stepper.moveTo(initial_homing);
        stepper.run();
        initial_homing++;
        delay(5);
      }
    Serial.print("We're home");
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(1200);              // stepper max speed of steps per second
    stepper.setSpeed(1000); 
    stepper.setAcceleration(1000); 
  }


int Displacement_Step_Converter_Z(int displacement_um) //function returns the step value after converting um input displacement
  {
    float pinion_circumference = 100531; //this is in um //this is been calculated by hand (C = pi*D)
    int step_resolution = round(pinion_circumference/3200);
    int StepValue = round(displacement_um/step_resolution);
    return StepValue;
  }


void requestEvent()
  {
    if (stepper.currentPosition() == prev_z1) 
      {
        Wire.write(1); //sending number 1 once stepper.currentPosition 
      }
    else
      {
      } 
  }

void receiveEvent(int howMany) // howMANY is always equal to no. of bytes received
  {                             //an int is 4 bytes, therefore taking up 4 spaces in the array                                    
    Serial.println("Transmission from master device detected!");
    Serial.print("Reading message...\t");
    char buf[howMany];              //creating a character storage buffer
    Serial.print(howMany);
    Serial.println(" bytes");       //no. of received bytes along i2c bus
    for (int i=0; i<=howMany; i++)    // loop through all bytes received
      {
        buf[i] = Wire.read();           // append byte to buffer
      }
    Serial.print("Message received:\n");

    char header = buf[0];           //only element 0 as a char is 1 byte
    Serial.print("header:\t");
    Serial.println(header);

    int z1;                          
    memcpy(&z1, &buf[1], sizeof(z1));    //decoding the character to an int, from elements 1-4 of the buffer, since int=4 bytes
    Serial.print("z-coord:\t");
    Serial.println(z1);
    z1 = Displacement_Step_Converter_Z(z1);
    Serial.print("Motor Steps:\t"); 
    Serial.println(z1);
    //This set of if/else if statement is for typical movement
    Serial.println("Wait and watch me move dad!!");
    delay(1000);
    if ((z1 >= 0)&&(z1 < 2080)) //z needs to be within range
      {
        if((stepper.currentPosition() != z1))    //move down from origin when not at position and not triggered failsafe
          {
              // Set the target position:
              stepper.moveTo(z1);
              // Run to target position with set speed and acceleration/deceleration:
              stepper.runToPosition();
          }
        prev_z1 = z1;  
      }
    else
      {
        Serial.println("Out of range");
        digitalWrite(LED_OutofRange,HIGH); 
        delay(1000);
        digitalWrite(LED_OutofRange,LOW); 
      }
    Serial.println("Current Coordinate finished"); 
    delay(100); 
  }

void loop() 
  {                
   delay(100);    // main loop with one second delay  
  }


