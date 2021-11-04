// Define/ Include libraries

#include <BasicLinearAlgebra.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> // backend support for gyro accel library
#include <Adafruit_LSM9DS1.h> // accelerometer gyro library
#include <PID_v1.h>
#include <math.h>
#include <list>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//Define our input/output pins:

// Teensy 3.5 Pin definitions:

// #define A_XPIN A3 // the analog input (0-5V) for controlling roll direction // the joystick will read 0-1024 analog -- NOT NEEDED FOR RADIO
// #define A_YPIN A4 // analog input pin (0-5V) for controlling pitch direction // the joystick will read 0-1024 analog -- NOT NEEDED FOR RADIO
// #define A_SPIN_PIN A2 // analog input pin (0-5V) for controlling rotating on top of ball -- NOT NEEDED FOR RADIO
// #define STARTERPIN 2 // for toggling the balancing on/off -- NOT NEEDED FOR RADIO

#define SDA_PIN 18 // for writing data to the IMU
#define SCL_PIN 19 // for controlling clock for IMU

#define MOTOR1SPEEDPIN 2 // for writing forward PWM signal to motor controller
#define MOTOR1DIRPIN 3 // for writing backward PWM signal to motor controller
#define ENCODER1A_PIN 4 // motor 1 encoder pin phase A
#define ENCODER1B_PIN 5 // motor 1 encoder pin phase B

#define MOTOR2SPEEDPIN 29 // for writing forward PWM signal to motor controller
#define MOTOR2DIRPIN 30 // for writing backward PWM signal to motor controller  
#define ENCODER2A_PIN 31 // motor 2 encoder pin phase A
#define ENCODER2B_PIN 32 // motor 2 encoder pin phase B

#define MOTOR3SPEEDPIN 20 // for writing forward PWM signal to motor controller
#define MOTOR3DIRPIN 21 // for writing backward PWM signal to motor controller
#define ENCODER3A_PIN 22 // motor 3 encoder pin phase A
#define ENCODER3B_PIN 23 // motor 3 encoder pin phase B

#define MOSI_PIN 11 // Radio communications
#define MISO_PIN 12 // Radio communications
#define SCK0_PIN 13 // Radio communications
#define CE_PIN 14 // Radio transmit enable pin
#define CSN_PIN 15 // Radio chip select pin
#define RADIO_IRQ 16 // Radio interupt pin signalling something has happened on radio

// Define dimensions of robot: set all of these before getting robot working
 
#define MIN_MOTOR_CONTROL_SIGNAL 0
#define MAX_MOTOR_CONTROL_SIGNAL 255
#define WHEEL_RADIUS 30 //  Wheel radius measured in millimeters This is only used for the spin vector velocity, we can set to 1 for this case because all wheels at same radius
#define ROBOT_RADIUS 30
enum directions{CLOCKWISE = HIGH, COUNTERCLOCKWISE = LOW};




//Define Timers:
unsigned long timer = millis();
int velocityInterval = 50;
unsigned long velocityPrev = millis();
unsigned long prevDebugTime = millis();
unsigned long timer2 = millis();
volatile unsigned long radioTimer = 0; // checks for timeout between sending signals
const int shortRadioTimeoutInterval = 100; // Wait 50 milliseconds to declare radio signal lost
unsigned int dt = 0;


// Test timer
unsigned long testTimer = 0;
const int testInterval = 500;
bool state = LOW;

//Define our variables: ========================================================================================================================================================================

// Calculated Variables:


// Accelerometer Variables

double xAccel = 0, yAccel = 0, zAccel = 0; // accelerometer readings
double AxTheta = 0, AyTheta = 0; // angles IMU is tilted in according to accelerometer
double GxTheta = 0, GyTheta = 0, GzTheta = 0; // gyro readings
double xFuseAngle = 0, yFuseAngle = 0; // sensor fusion angles - complementary filter high pass low pass
Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1(); // IMU class instance

int xJoystick = 0, yJoystick = 0, spinControl = 0; // values between 0 and 255 from PWM signal reading to control ro


bool haltBalancing = false;
bool prevStarterState = false;
float calibrationY = -0.2, calibrationX = -1.65;

//Class Definitions: ========================================================================================================================================================================

class Motor{


  public:
  const int PULSES_PER_REVOLUTION = 700; // Is this A & B pulses, or just A or just B?
  const int MAX_MOTOR_RPM = 251;
  int speedPin, directionPin, encoderPinA, encoderPinB;
  volatile long pulseCount = 0;
  volatile bool wheelDirection = CLOCKWISE;
  volatile bool lastStateOfA = LOW;
  
  Motor(const int speedPin, const int directionPin, const int encoderPinA, const int encoderPinB){
     this->speedPin = speedPin;
     this->directionPin = directionPin;
     this->encoderPinA = encoderPinA;
     this->encoderPinB = encoderPinB;

     for (int i = 0; i < 3; i++) velocities.push_back(0.0);

  }

  long double GetTotalDispl(){
    UpdateTotalDispl();
    return totalDispl;
  }

  double GetVelocity(){
    UpdateVelocity();
    return velocity;
  }

  private:
  
  long prevPulseCount = 0;
  long long prevUpdateTime = micros();
  std::list<double> velocities; //
  double velocity = 0;
  long double totalDispl = 0; // Total distance wheel has moved measured in mm
  enum directions{CLOCKWISE = 1, COUNTERCLOCKWISE = 0};

  double Average(const std::list<double> &v, int len){
  double sum = 0;
  for (auto it = v.begin(); it!= v.end(); ++it){
    sum += *it;
  }
  return sum/len;
}

void UpdateTotalDispl(){
  totalDispl = pulseCount*WHEEL_RADIUS*2*3.14159265/PULSES_PER_REVOLUTION; // Displacement in millimeters
  return;
}


void UpdateVelocity(){ // Could build the velocity update polling interval into the function, but it's inefficient to do it like this for now, although convenient and cleaner code
  
    long dp = pulseCount-prevPulseCount;  // set the change in pulses since last check
    prevPulseCount = pulseCount;    // update pulse count value
    velocity = dp*PI*60*1000/50/PULSES_PER_REVOLUTION; // unit: mm/s
    return;
  }
  
};


// Encoder Variables
Motor motor1(MOTOR1SPEEDPIN, MOTOR1DIRPIN, ENCODER1A_PIN, ENCODER1B_PIN);
Motor motor2(MOTOR2SPEEDPIN, MOTOR2DIRPIN, ENCODER2A_PIN, ENCODER2B_PIN);
Motor motor3(MOTOR3SPEEDPIN, MOTOR3DIRPIN, ENCODER3A_PIN, ENCODER3B_PIN);

// Define parameters of the robot:


// Define PID loops and their vars ========================================================================================================================================================================

double pitchSetpoint, pitchInput, pitchOutput; // PIDy variables
double rollSetpoint, rollInput, rollOutput; // PIDx variables
float Kp = 7, Ki = 0, Kd = 0; // set PID loop control constants // I could tune these with time constants

//First two PID loops control tilt based on how fast the speed is.  Kp Ki and Kd will be small to allow soft reaction in system and not oscillate
//PID xSpeedControl(&rollSpeedOutput, &rollSetpoint, &desiredPWMxSetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
//PID ySpeedControl(&pitchSpeedOuutput, &pitchSetpoint, &desiredPWMySetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
// Second Two PID loops control the PWM motors based on input tilt and tilt setpoint controlled by above PID loops
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

double wheel1PID_PWMOutput = 0;
double wheel2PID_PWMOutput = 0;
double wheel3PID_PWMOutput = 0;

float kpWheel = 1.5, kiWheel = 0, kdWheel = 0;

double motor1Velocity = 0;
double motor2Velocity = 0;
double motor3Velocity = 0;
double setpointV1 = 0;
double setpointV2 = 0;
double setpointV3 = 0;
    
PID wheel1PID(&motor1Velocity, &wheel1PID_PWMOutput, &setpointV1, kpWheel, kiWheel, kdWheel, DIRECT);
PID wheel2PID(&motor2Velocity, &wheel2PID_PWMOutput, &setpointV2, kpWheel, kiWheel, kdWheel, DIRECT);
PID wheel3PID(&motor3Velocity, &wheel3PID_PWMOutput, &setpointV3, kpWheel, kiWheel, kdWheel, DIRECT);
    


// Radio Variables Setup

// Hardware configuration
RF24 radio(CE_PIN, CSN_PIN);                         // Set up nRF24L01 radio on SPI bus, pins 14 (CE) & 15 (CSN)

// Demonstrates another method of setting up the addresses
byte address[][5] = { 0xCC, 0xCE, 0xCC, 0xCE, 0xCC , 0xCE, 0xCC, 0xCE, 0xCC, 0xCE};

// Role management
typedef enum { role_sender = 1, role_receiver } role_e;                 // The various roles supported by this sketch
const char* role_friendly_name[] = { "invalid", "Sender", "Receiver"};  // The debug-friendly names of those roles
role_e role;                                                            // The role of the current running sketch

volatile uint32_t message_count = 0;

uint16_t volatile joystick_signal = 0x00;
uint8_t xJoystickSignal = 0x00; // receives the x portion of the joystick signal 16 bit number
uint8_t yJoystickSignal = 0x00; // receives the y portion of the joystick signal 16 bit number
const unsigned int joystickRadioMaxVal = 255; // max analog value for teensy and arduino


void setupEncoders(){

  pinMode(ENCODER1A_PIN, INPUT);
  pinMode(ENCODER1B_PIN, INPUT);
  pinMode(ENCODER2A_PIN, INPUT);
  pinMode(ENCODER2B_PIN, INPUT);
  pinMode(ENCODER3A_PIN, INPUT);
  pinMode(ENCODER3B_PIN, INPUT);
  
  attachInterrupt(ENCODER1A_PIN, pulseInterruptA1, CHANGE);
  attachInterrupt(ENCODER2A_PIN, pulseInterruptA2, CHANGE);
  attachInterrupt(ENCODER3A_PIN, pulseInterruptA3, CHANGE);
  
}

void setupSensor() {
  // 1.) Set the accelerometer range
  IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  IMU.setupMag(IMU.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  IMU.setupGyro(IMU.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setupRadio() {

  //pinMode(role_pin, INPUT);    // set up the role pin
  //digitalWrite(role_pin,HIGH);                     // Change this to LOW/HIGH instead of using an external pin
  delay(20);                                       // Just to get a solid reading on the role pin

  printf_begin();

  // Setup and configure rf radio
  radio.begin();
  //radio.setPALevel(RF24_PA_LOW);
  radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
  radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads

  // Open pipes to other node for communication
  // This simple sketch opens a pipe on a single address for these two nodes to
  // communicate back and forth.  One listens on it, the other talks to it.

  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
  radio.startListening();
  radio.writeAckPayload( 1, &message_count, sizeof(message_count) );  // Add an ack packet for the next time around.  This is a simple
  ++message_count;

  radio.printDetails();                             // Dump the configuration of the rf unit for debugging
  delay(50);
  attachInterrupt(RADIO_IRQ, check_radio, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver

}


void setup() {

  // Set up the pins for input output
  //pinMode(A_XPIN, INPUT);  // Only used with analog joystick connection
  //pinMode(A_YPIN, INPUT);  // Only used with analog joystick connection
  //pinMode(A_SPIN_PIN, INPUT);  // Only used with analog joystick connection
  pinMode(MOTOR1SPEEDPIN, OUTPUT);
  pinMode(MOTOR2SPEEDPIN, OUTPUT);
  pinMode(MOTOR3SPEEDPIN, OUTPUT);
  pinMode(MOTOR1DIRPIN, OUTPUT);
  pinMode(MOTOR2DIRPIN, OUTPUT);
  pinMode(MOTOR3DIRPIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  analogWriteFrequency(MOTOR1SPEEDPIN, 10000);
  analogWriteFrequency(MOTOR2SPEEDPIN, 10000);
  analogWriteFrequency(MOTOR3SPEEDPIN, 10000);

  // the adafruit library takes care of the Serial pin setup and communication

  Serial.begin(115200); // want a fast communication rate

  while (!Serial) {
    delay(1); // wait for serial console to open - might not need for arduino
  }

  if (!IMU.begin()) {
    Serial.println("IMU unable to connect, check wiring and restart");
     while (1); // don't do anything
  }
  Serial.println("IMU found");
  setupSensor();
  //setupRadio();
  setupEncoders();

  // Turn the PID loops on and make them quick
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);
  rollPID.SetOutputLimits(-255, 255);
  pitchPID.SetSampleTime(10);
  rollPID.SetSampleTime(10);

      // Turn the PID loops on and make them quick
    wheel1PID.SetMode(AUTOMATIC);
    wheel2PID.SetMode(AUTOMATIC);
    wheel3PID.SetMode(AUTOMATIC);
    wheel1PID.SetOutputLimits(-255, 255);
    wheel2PID.SetOutputLimits(-255, 255);
    wheel3PID.SetOutputLimits(-255, 255);
    wheel1PID.SetSampleTime(50);
    wheel2PID.SetSampleTime(50);
    wheel3PID.SetSampleTime(50);

}

void loop() {
  //Serial.print(digitalRead(ENCODER1A_PIN));
  //Serial.print(digitalRead(ENCODER2A_PIN));
  //Serial.println(digitalRead(ENCODER3A_PIN));

  //testLight(); // Blink the light at interval to indicate looping program
  if(millis() - testTimer >= 2000){
      if(state == LOW){
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      state = HIGH;
      }
      else {
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      state = LOW;
      }
      testTimer = millis();
    }

  if (prevStarterState == HIGH) {
    //if (digitalRead(STARTERPIN) == LOW){
    //  haltBalancing = !haltBalancing;
    //}
  }
  // bool prevStarterState = digitalRead(STARTERPIN); // forgotten purpose
  haltBalancing = false;
  if (haltBalancing == false) {

    // receive data for x and y angles from IMU here and input into variables
    IMU.read();

    sensors_event_t a, m, g, temp; // get a new sensor event

    IMU.getEvent(&a, &m, &g, &temp); // read the data into the sensor event variables

    // the accelerometers are swapped on the IMU, x swaps with y in this case
    xAccel = -a.acceleration.y; // Need the negative of Y to agree with the X gyro
    yAccel = a.acceleration.x;
    zAccel = a.acceleration.z;

    // now calculating the angle the accelerometer says we are pointing
    AxTheta = atan2(xAccel , sqrt(zAccel * zAccel +  yAccel * yAccel)) * 180 / PI; // angle IMU tilts in x direction according to accelerometer using cos(theta) = A/H where A is dot product of two vectors
    AyTheta = atan2(yAccel , sqrt(zAccel * zAccel + xAccel * xAccel)) * 180 / PI; // angle IMU tilts in y direction according to accelerometer

    // Gyro Rate of rotationca
    GxTheta = g.gyro.x * 180 / PI; // acquire degrees/s angular velocity
    GyTheta = g.gyro.y * 180 / PI;
    //GzTheta = g.gyro.z; // Unused for project


    dt = millis() - timer; // Time change since last loop (Used for integration)
    timer = millis(); // Reset current time to prepare for next loop
        
    
    //Sensor fusion of gyro and accelerometer with complementary filter - make sure the sign of AxTheta and AyTheta matches our desired!!
    xFuseAngle = 0.97 * (xFuseAngle + (GxTheta * dt) / 1000) + 0.03 * AxTheta; // Integrate the angular velocity and combine with AxTheta
    yFuseAngle = 0.97 * (yFuseAngle + (GyTheta * dt) / 1000) + 0.03 * AyTheta;
    
    // Put results into the PID variable inputs
    pitchInput =  yFuseAngle + calibrationY; // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees
    rollInput = xFuseAngle + calibrationX; // // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees

    rollInput = -rollInput;
    //==============================

    // Read in the desired additional motion of joystick // THIS IS WHERE WE PUT IN THE RADIO CONTROL MESSAGE RECEIVED, perhaps use an interupt to get the data

    // Need a timer that resets joystick orientation to neutral if no signal is received for a certain interval

    noInterrupts();
    xJoystickSignal = joystick_signal & 0x00FF;
    yJoystickSignal = (joystick_signal >> 8) & 0xFF; // Bit shift right 8 bits to reach the y portion of the 16 bit number

    if (millis() - radioTimer > shortRadioTimeoutInterval) {
    unsigned long  joystickSignal = 0x7F7F; // set the radio signal to neutral to prevent ball from rolling away indefinitely
    }
    interrupts();

    // Put our desired tilt into PID setpoint variable
    pitchSetpoint = map(yJoystickSignal, 0, joystickRadioMaxVal, -5, 5); // we map joystick to tell the robot to pitch 0-5 degrees for movement control // in future setpoint will be controlled by PID loop
    rollSetpoint = map(xJoystickSignal, 0, joystickRadioMaxVal, -5, 5); // we map joystick to tell the robot to roll 0-5 degrees for movement control

    pitchSetpoint = 0; // overriding above calcs for testing balance without radio
    rollSetpoint = 0; // overriding above calcs for testing balance without radio 

    // Compute Outputs for the PID loop

    pitchPID.Compute(); // PID library does calculations to do proper motor outputs
    rollPID.Compute();


    // Dynamically deciding spin control calculates the current fastest possible speed of a wheel, and subtracts that from the theoretical fastest, and converts to a PWM signal
    float outputGain = 1;
    float spin = spinControl / outputGain; // Calculate and scale this value appropriately before putting into below vector
    float horizontalSpeed = rollOutput / outputGain;
    float forwardSpeed = pitchOutput / outputGain;
/*
    Serial.print("Pitch Input: ");
    Serial.print(pitchInput);
    Serial.print(" | Roll Input: ");
    Serial.print(rollInput);
    Serial.print(" | PitchOutput: ");
    Serial.print(pitchOutput);
    Serial.print(" | RollOutput: ");
    Serial.print(rollOutput);
    //Serial.print(" | PitchSetpoint: ");
    //Serial.print(pitchSetpoint);
    //Serial.print(" | RollSetpoint: ");
    //Serial.print(rollSetpoint);
    //Serial.print(" | xJoystickSignal: ");
    //Serial.print(xJoystickSignal);
    //Serial.print(" | yJoystickSignal: ");
    //Serial.print(yJoystickSignal);
*/
    // Do the math to move robot and send commands

    
    // Desired movement inputs
    const double MAX_ROBOT_SPEED = motor1.MAX_MOTOR_RPM*2*PI*WHEEL_RADIUS/60*sqrt(3)/2;  // unit mm/s about 700 mm/s for one wheel, but whole robot is less because wheels at angle (PI/6 used here, NEED BETTER GEOMETRY UNDERSTANDING))
    BLA:: Matrix<3, 1> PIDOutputVector = {spin, horizontalSpeed, forwardSpeed}; // omega (spin), x, y.  PWM values are actually put into this variable (The PID loop will compensate for discrepancy)
    //BLA:: Matrix<3, 1> PIDOutputVector = {0, (int)600*sin(((millis()/50)%360)*PI/180), 0};

    // Matrix for conversion of robot speed to individual wheel speed // only proportions matter currently,  WHEEL_RADIUS == 30
    BLA::Matrix<3, 3> H = { WHEEL_RADIUS, 1, 0, WHEEL_RADIUS, -0.5, -sin(PI / 3), WHEEL_RADIUS, -0.5, sin(PI / 3)};// /(WHEEL_RADIUS); // if we were doing actual velocity inputs w/out PID loop

    //Output wheel velocities
    BLA:: Matrix<3, 1> WheelAngularSpeeds = H * PIDOutputVector; // convert velocity outputs to wheel outputs

  if(millis() - velocityPrev > velocityInterval){
    motor1Velocity = motor1.GetVelocity();
    motor2Velocity = motor2.GetVelocity();
    motor3Velocity = motor3.GetVelocity();
    velocityPrev = millis();
  }
    
    setpointV1 = WheelAngularSpeeds(0, 0);
    setpointV2 = WheelAngularSpeeds(1, 0);
    setpointV3 = WheelAngularSpeeds(2, 0);

    // Model a PWM to SPEED function to help approximate the PID controller

    double approxV1PWM = 0.388786*setpointV1-21;
    double approxV2PWM = 0.388786*setpointV2-21;
    double approxV3PWM = 0.388786*setpointV3-21;



    wheel1PID.Compute(); // PID library does calculations to do proper motor outputs
    wheel2PID.Compute();
    wheel3PID.Compute();

    double combineFuncPID1 = approxV1PWM + wheel1PID_PWMOutput; // Use the approxPWM to speed conversion function, then add a PID loop to help adjust for error
    double combineFuncPID2 = approxV2PWM + wheel2PID_PWMOutput; // Use the approxPWM to speed conversion function, then add a PID loop to help adjust for error
    double combineFuncPID3 = approxV3PWM + wheel3PID_PWMOutput; // Use the approxPWM to speed conversion function, then add a PID loop to help adjust for error

    // Control algorithm for MD10C 10A Motor controllers
    ControlMD10C(MOTOR1SPEEDPIN, MOTOR1DIRPIN, (int)wheel1PID_PWMOutput ); // PWM pin, direction pin and PWM speed controlled by MD10C motor controller  (int)wheel1PID_PWMOutput  (int)255*sin(((millis()/50)%360)*PI/180)
    ControlMD10C(MOTOR2SPEEDPIN, MOTOR2DIRPIN, (int)wheel2PID_PWMOutput);
    ControlMD10C(MOTOR3SPEEDPIN, MOTOR3DIRPIN, (int)wheel3PID_PWMOutput);

    //Console feedback 

  if (millis()-prevDebugTime >= 100){

    prevDebugTime = millis();
    
    Serial.print(" | Desired Actual V1: ");
    Serial.print(setpointV1);
    /*
    Serial.print(" | Desired Actual V2: ");
    Serial.print(setpointV2);
    Serial.print(" | Desired Actual V3: ");
    Serial.print(setpointV3);
*/
    Serial.print(" | TotalDisp1: ");
    Serial.print((int)motor1.GetTotalDispl());
    //Serial.print(" | TotalDisp2: ");
    //Serial.print((int)motor2.GetTotalDispl());
    //Serial.print(" | TotalDisp3: ");
    //Serial.print((int)motor3.GetTotalDispl());
    Serial.print(" | Motor1Speed: ");
    Serial.print(motor1Velocity); //Serial.print(motor1.GetVelocity()%10);
    Serial.print(" | Motor1error: ");
    Serial.print(setpointV1-motor1Velocity); 
    /*
    Serial.print(" | Motor2Speed: ");
    Serial.print(motor2Velocity);
    Serial.print(" | Motor3Speed: ");
    Serial.print(motor3Velocity);
    */
    Serial.print(" | Motor1PWMOutput: ");
    Serial.println(wheel1PID_PWMOutput);  // wheel1PID_PWMOutput  255*sin(((millis()/50)%360)*PI/180)
    /*
    Serial.print(" | Motor2PWMOutput: ");
    Serial.print(wheel2PID_PWMOutput);
    Serial.print(" | Motor3PWMOutput: ");
    Serial.println(wheel3PID_PWMOutput);
*/
    //Serial.print(" | Roll Input: ");
    //Serial.print(rollInput);

  }


  }
  else {}

}

// DISCLAIMER:: This program is PID feedback controlled and needs to be modified for dimensional accuracy should real life speed/distance control be desired

// Radio Interupt

void check_radio(void)                               // Receiver role: Does nothing!  All the work is in IRQ
{

  bool tx, fail, rx;
  radio.whatHappened(tx, fail, rx);                   // What happened?

  if ( tx ) {                                         // Have we successfully transmitted?
    Serial.println(F("Ack Payload:Sent"));
  }

  if ( fail ) {                                       // Have we failed to transmit?
    Serial.println(F("Ack Payload:Failed"));
  }

  if ( rx || radio.available()) {                     // Did we receive a message?

    // static unsigned long got_time;                  // Get this payload and dump it
    radio.read( &joystick_signal, sizeof(uint16_t) );
    Serial.print(F("Got payload "));
    radio.writeAckPayload( 1, &message_count, sizeof(message_count) );  // Add an ack packet for the next time around.  This is a simple packet counter
    ++message_count;
    radioTimer = millis();
  }
}

void testLight(){
  if(millis() - testTimer >= 2000){
    if(state == LOW){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    state = HIGH;
    }
    else {
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    state = LOW;
    }
    testTimer = millis();
  }
}

  // Control algorithm for MD10C 10A Motor controllers

void ControlMD10C(int PWMPin, int DIRPin, signed int PWMInput){
    
    if (PWMInput >= 0) { // set direction pin to be HIGH or LOW (CLOCKWISE or COUNTERCLOCKWISE)
      digitalWrite(DIRPin, COUNTERCLOCKWISE);
    }
    else {
      digitalWrite(DIRPin, CLOCKWISE);
    }
    
    // Write to the motors // commented out because testing IMU first
      analogWrite(PWMPin, abs(PWMInput));
}


// Encoder Interrupt Functions

  void pulseInterruptA1()
{
  int Lstate = digitalRead(ENCODER1A_PIN);
  if((motor1.lastStateOfA == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(ENCODER1B_PIN);
    if(val == LOW && motor1.wheelDirection)
    {
      motor1.wheelDirection = COUNTERCLOCKWISE; //Reverse (Counterclockwise)
    }
    else if(val == HIGH && !motor1.wheelDirection)
    {
      motor1.wheelDirection = CLOCKWISE;  //Forward (Clockwise)
    }
    
    if (motor1.wheelDirection == CLOCKWISE){motor1.pulseCount++;}
    else if (motor1.wheelDirection == COUNTERCLOCKWISE){motor1.pulseCount--;}
  }
  motor1.lastStateOfA = Lstate;
// Serial.print("Interrupt A1 "); Serial.println(motor1.wheelDirection);
}
  
    void pulseInterruptA2()
{
  int Lstate = digitalRead(ENCODER2A_PIN);
  if((motor2.lastStateOfA == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(ENCODER2B_PIN);
    if(val == LOW && motor2.wheelDirection)
    {
      motor2.wheelDirection = COUNTERCLOCKWISE; //Reverse (Counterclockwise)
    }
    else if(val == HIGH && !motor2.wheelDirection)
    {
      motor2.wheelDirection = CLOCKWISE;  //Forward (Clockwise)
    }
      if (motor2.wheelDirection == CLOCKWISE){motor2.pulseCount++;}
    else if (motor2.wheelDirection == COUNTERCLOCKWISE){motor2.pulseCount--;}
  }
  motor2.lastStateOfA = Lstate;
// Serial.print("Interrupt A2 "); Serial.println(motor2.wheelDirection);
}

    void pulseInterruptA3()
{
  int Lstate = digitalRead(ENCODER3A_PIN);
  if((motor3.lastStateOfA == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(ENCODER3B_PIN);
    if(val == LOW && motor3.wheelDirection)
    {
      motor3.wheelDirection = COUNTERCLOCKWISE; //Reverse (Counterclockwise)
    }
    else if(val == HIGH && !motor3.wheelDirection)
    {
      motor3.wheelDirection = CLOCKWISE;  //Forward (Clockwise)
    }
    if (motor3.wheelDirection == CLOCKWISE){motor3.pulseCount++;}
    else if (motor3.wheelDirection == COUNTERCLOCKWISE){motor3.pulseCount--;}
  }
  motor3.lastStateOfA = Lstate;
  //Serial.print("Interrupt A3 "); Serial.println(motor3.wheelDirection);
}
