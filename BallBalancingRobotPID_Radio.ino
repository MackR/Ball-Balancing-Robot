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

// Radio nRF24L01 module Libraries

#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//Define our input/output pins:

  #define A_XPIN A0 // the analog input (0-5V) for controlling roll direction // the joystick will read 0-1024 analog
  #define A_YPIN A1 // analog input pin (0-5V) for controlling pitch direction // the joystick will read 0-1024 analog
  #define A_SPIN_PIN A2 // analog input pin (0-5V) for controlling rotating on top of ball
  #define SDA_PIN A4 // for writing data to the IMU
  #define SCL_PIN A5 // for controlling clock for IMU
  #define STARTERPIN 2 // for toggling the balancing on/off
  #define MOTOR1FWDPIN 3 // for writing forward PWM signal to motor controller
  #define MOTOR1BWDPIN 9 // for writing backward PWM signal to motor controller
  #define MOTOR2FWDPIN 5 // for writing forward PWM signal to motor controller
  #define MOTOR2BWDPIN 10 // for writing backward PWM signal to motor controller
  #define MOTOR3FWDPIN 6 // for writing forward PWM signal to motor controller
  #define MOTOR3BWDPIN 11 // for writing backward PWM signal to motor controller


// Define dimensions of robot: set all of these before getting robot working

#define ROBOT_RADIUS 1 // This is only used for the spin vector velocity, we can set to 1 for this case because all wheels at same radius
#define WHEEL_RADIUS 1
#define MAX_MOTOR_SPEED 1
#define MAX_ROBOT_SPEED MAX_MOTOR_SPEED*WHEEL_RADIUS  // should add a safety factor in here
#define MIN_MOTOR_CONTROL_SIGNAL 0
#define MAX_MOTOR_CONTROL_SIGNAL 255


  //Define Timers:
  unsigned long timer = millis();
  unsigned int dt = 0;


  //Define our variables:
  double pitchSetpoint, pitchInput, pitchOutput; // PIDy variables
  double rollSetpoint, rollInput, rollOutput; // PIDx variables
  double xAccel = 0, yAccel = 0, zAccel = 0; // accelerometer readings
  double AxTheta = 0, AyTheta = 0; // angles IMU is tilted in according to accelerometer
  double GxTheta = 0, GyTheta = 0, GzTheta = 0; // gyro readings
  double xFuseAngle = 0, yFuseAngle = 0; // sensor fusion angles - complementary filter high pass low pass
  int xJoystick = 0, yJoystick = 0, spinControl = 0; // values between 0 and 255 from PWM signal reading to control ro
  float Kp = 7, Ki = 0, Kd = 2; // set PID loop control constants // I could tune these with time constants
  bool haltBalancing = false;
  bool prevStarterState = false;
  float calibrationY = 1.2, calibrationX = -1;
  Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1(); // IMU class instance

  // Define parameters of the robot:

  

// Define PID loops



 //First two PID loops control tilt based on how fast the speed is.  Kp Ki and Kd will be small to allow soft reaction in system and not oscillate
  //PID xSpeedControl(&rollSpeedOutput, &rollSetpoint, &desiredPWMxSetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
  //PID ySpeedControl(&pitchSpeedOutput, &pitchSetpoint, &desiredPWMySetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
// Second Two PID loops control the PWM motors based on input tilt and tilt setpoint controlled by above PID loops
  PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
  PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

// Radio Variables Setup 

  // Hardware configuration
RF24 radio(7,8);                          // Set up nRF24L01 radio on SPI bus plus pins 7 (CE) & 8 (CSN)

// Demonstrates another method of setting up the addresses
byte address[][5] = { 0xCC,0xCE,0xCC,0xCE,0xCC , 0xCE,0xCC,0xCE,0xCC,0xCE};

// Role management
typedef enum { role_sender = 1, role_receiver } role_e;                 // The various roles supported by this sketch
const char* role_friendly_name[] = { "invalid", "Sender", "Receiver"};  // The debug-friendly names of those roles
role_e role;                                                            // The role of the current running sketch

static uint32_t message_count = 0;
uint16_t volatile joystick_signal = 0x00;
uint8_t xJoystickSignal = 0x00; // receives the x portion of the joystick signal 16 bit number
uint8_t yJoystickSignal = 0x00; // receives the y portion of the joystick signal 16 bit number



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
  Serial.println(role_friendly_name[role]);

  // Setup and configure rf radio
  radio.begin();  
  //radio.setPALevel(RF24_PA_LOW);
  radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
  radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads
  
  // Open pipes to other node for communication
  // This simple sketch opens a pipe on a single address for these two nodes to 
  // communicate back and forth.  One listens on it, the other talks to it.
  
    radio.openWritingPipe(address[1]);
    radio.openReadingPipe(1,address[0]);
    radio.startListening();
    radio.writeAckPayload( 1, &message_count, sizeof(message_count) );  // Add an ack packet for the next time around.  This is a simple
    ++message_count;        
  
  radio.printDetails();                             // Dump the configuration of the rf unit for debugging
  delay(50);
  attachInterrupt(0, check_radio, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver
  
}

void setup() {

  // Set up the pins for input output
  pinMode(A_XPIN, INPUT);
  pinMode(A_YPIN, INPUT);
  pinMode(A_SPIN_PIN, INPUT);
  pinMode(MOTOR1FWDPIN, OUTPUT);
  pinMode(MOTOR2FWDPIN, OUTPUT);
  pinMode(MOTOR3FWDPIN, OUTPUT);
  pinMode(MOTOR1BWDPIN, OUTPUT);
  pinMode(MOTOR2BWDPIN, OUTPUT);
  pinMode(MOTOR3BWDPIN, OUTPUT);
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
  setupRadio();


  // Turn the PID loops on and make them quick
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255,255);
  rollPID.SetOutputLimits(-255,255);
  pitchPID.SetSampleTime(10);
  rollPID.SetSampleTime(10);

}

void loop() {

  if (prevStarterState == HIGH){
  //if (digitalRead(STARTERPIN) == LOW){
  //  haltBalancing = !haltBalancing;
  //}
  }
  bool prevStarterState = digitalRead(STARTERPIN);
  haltBalancing = false;
if (haltBalancing == false){

  // receive data for x and y angles from IMU here and input into variables
  IMU.read();

  sensors_event_t a, m, g, temp; // get a new sensor event

  IMU.getEvent(&a, &m, &g, &temp); // read the data into the sensor event variables

// the accelerometers are swapped on the IMU, x swaps with y in this case
  xAccel = -a.acceleration.y; // Need the negative of Y to agree with the X gyro
  yAccel = a.acceleration.x;
  zAccel = a.acceleration.z;

  // now calculating the angle the accelerometer says we are pointing
  AxTheta = atan2(xAccel , sqrt(zAccel*zAccel +  yAccel*yAccel)) * 180 / PI; // angle IMU tilts in x direction according to accelerometer using cos(theta) = A/H where A is dot product of two vectors
  AyTheta = atan2(yAccel , sqrt(zAccel*zAccel + xAccel*xAccel)) * 180 / PI; // angle IMU tilts in y direction according to accelerometer

  // Gyro Rate of rotation
  GxTheta = g.gyro.x* 180/PI; // acquire degrees/s angular velocity
  GyTheta = g.gyro.y*180/PI; 
  //GzTheta = g.gyro.z; // Unused for project


  dt = millis()-timer; // Time change since last loop (Used for integration)
  timer = millis(); // Reset current time to prepare for next loop
  
  
  //Sensor fusion of gyro and accelerometer with complementary filter - make sure the sign of AxTheta and AyTheta matches our desired!!
  xFuseAngle = 0.97 * (xFuseAngle + (GxTheta * dt)/1000) + 0.03 * AxTheta; // Integrate the angular velocity and combine with AxTheta
  yFuseAngle = 0.97 * (yFuseAngle + (GyTheta * dt)/1000) + 0.03 * AyTheta;

  // Put results into the PID variable inputs
  pitchInput =  yFuseAngle + calibrationY; // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees
  rollInput = xFuseAngle + calibrationX; // // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees

//============================== SWAP THE PITCH & ROLL DIRECTIONS BECAUSE BOARD SETUP IN REAL LIFE
double holder = rollInput;
rollInput = pitchInput;
pitchInput = holder;
//==============================

  // Read in the desired additional motion of joystick // THIS IS WHERE WE PUT IN THE RADIO CONTROL MESSAGE RECEIVED, perhaps use an interupt to get the data

  // Need a timer that resets joystick orientation to neutral if no signal is received for a certain interval
  
  xJoystickSignal = joystick_signal & 0x00FF;
  yJoystickSignal = (joystick_signal >> 8) & 0xFF; // Bit shift right 8 bits to reach the y portion of the 16 bit number
  joystickSignal = 0x00; // reset the value for the next loop, incase signal is lost
  const unsigned int joystickMaxVal = 255;
  
  xJoystick = 512; // analogRead(A_XPIN);
  yJoystick = 512; // analogRead(A_YPIN);
  spinControl = 0; // analogRead(A_SPIN_PIN);

  // Put our desired tilt into PID setpoint variable
  pitchSetpoint = map(yJoystickSignal, 0, joystickMaxVal, -5, 5); // we map joystick to tell the robot to pitch 0-5 degrees for movement control // in future setpoint will be controlled by PID loop
  rollSetpoint = map(xJoystickSignal, 0, joystickMaxVal, -5, 5); // we map joystick to tell the robot to roll 0-5 degrees for movement control


  // Compute Outputs for the PID loop

  pitchPID.Compute(); // PID library does calculations to do proper motor outputs
  rollPID.Compute();


// Dynamically deciding spin control calculates the current fastest possible speed of a wheel, and subtracts that from the theoretical fastest, and converts to a PWM signal
  float outputGain = 1;
  float spin = spinControl/outputGain; // Calculate and scale this value appropriately before putting into below vector
  float horizontalSpeed = rollOutput/outputGain; 
  float forwardSpeed = pitchOutput/outputGain;

//Serial.print("Pitch Input: ");
//Serial.print(pitchInput);
//Serial.print(" | Roll Input: ");
//Serial.print(rollInput);
Serial.print(" | PitchOutput: ");
Serial.print(pitchOutput);
Serial.print(" | RollOutput: ");
Serial.print(rollOutput); 
//Serial.print(" | PitchSetpoint: ");
//Serial.print(pitchSetpoint);
//Serial.print(" | RollSetpoint: ");
//Serial.print(rollSetpoint);
Serial.print(" | xJoystickSignal: ");
Serial.print(xJoystickSignal);
Serial.print(" | yJoystickSignal: ");
Serial.print(yJoystickSignal); 

// Do the math to move robot and send commands
// Desired movement inputs
BLA:: Matrix<3, 1> PIDOutputVector = {spin, horizontalSpeed, forwardSpeed}; // omega (spin), x, y.  PWM values are actually put into this variable (The PID loop will compensate for discrepancy)


// Matrix for conversion of robot speed to individual wheel speed // only proportions matter currently,  ROBOT_RADIUS == 1
BLA::Matrix<3, 3> H = { ROBOT_RADIUS, 1, 0, ROBOT_RADIUS, -0.5, -sin(PI / 3), ROBOT_RADIUS, -0.5, sin(PI / 3)}; // * 1/ROBOT_RADIUS if we were doing actual velocity inputs w/out PID loop

//Output wheel velocities
BLA:: Matrix<3, 1> WheelAngularSpeeds = H * PIDOutputVector; // convert velocity outputs to wheel outputs


  double motor1Output = WheelAngularSpeeds(0, 0); 
  double motor2Output = WheelAngularSpeeds(1, 0);
  double motor3Output = WheelAngularSpeeds(2, 0);



int controlPin1, controlPin2, controlPin3;
  if (motor1Output >= 0) {
    controlPin1 = MOTOR1FWDPIN;
    digitalWrite(MOTOR1BWDPIN, LOW);

  }
  else{
    controlPin1 = MOTOR1BWDPIN;
    digitalWrite(MOTOR1FWDPIN, LOW); 
  }
  
  if (motor2Output >= 0) {
    controlPin2 = MOTOR2FWDPIN;
    digitalWrite(MOTOR2BWDPIN, LOW);
  }
  else{
    controlPin2 = MOTOR2BWDPIN;
    digitalWrite(MOTOR2FWDPIN, LOW);
    
  }
  if (motor3Output >= 0) {
    controlPin3 = MOTOR3FWDPIN;
    digitalWrite(MOTOR3BWDPIN, LOW);
  }
  else{
    controlPin3 = MOTOR3BWDPIN;
    digitalWrite(MOTOR3FWDPIN, LOW);
    
  }


  //Serial.print(" | Roll Input: ");
//Serial.print(rollInput);
  
// Write to the motors
//  analogWrite(controlPin1, motor1Output);
//  analogWrite(controlPin2, motor2Output);
//  analogWrite(controlPin3, motor3Output);


Serial.print(" | Motor1Output: ");
Serial.print(motor1Output);
Serial.print(" | Motor2Output: ");
Serial.print(motor2Output);
Serial.print(" | Motor3Output: ");
Serial.println(motor3Output);

} 
else{}

}

// DISCLAIMER:: This program is PID feedback controlled and needs to be modified for dimensional accuracy should real life speed/distance control be desired

// Radio Interupt

void check_radio(void)                               // Receiver role: Does nothing!  All the work is in IRQ
{
  
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);                     // What happened?
  
  if ( tx ) {                                         // Have we successfully transmitted?
       Serial.println(F("Ack Payload:Sent")); 
  }
  
  if ( fail ) {                                       // Have we failed to transmit?
       Serial.println(F("Ack Payload:Failed"));  
  }
  
  if ( rx || radio.available()){                      // Did we receive a message?

      // static unsigned long got_time;                  // Get this payload and dump it
      radio.read( &joystick_signal, sizeof(uint16_t) );
      Serial.print(F("Got payload "));
      radio.writeAckPayload( 1, &message_count, sizeof(message_count) );  // Add an ack packet for the next time around.  This is a simple packet counter
      ++message_count;                                
    
  }
}
