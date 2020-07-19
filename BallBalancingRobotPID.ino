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
  float Kp = 8, Ki = 2, Kd = 2; // set PID loop control constants // I could tune these with time constants
  bool haltBalancing = false;
  bool prevStarterState = false;
  Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1(); // IMU class instance

  // Define parameters of the robot:

  

// Define PID loops



 //First two PID loops control tilt based on how fast the speed is.  Kp Ki and Kd will be small to allow soft reaction in system and not oscillate
  //PID xSpeedControl(&rollSpeedOutput, &rollSetpoint, &desiredPWMxSetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
  //PID ySpeedControl(&pitchSpeedOutput, &pitchSetpoint, &desiredPWMySetpoint, Kp2, Ki2, Kd2, DIRECT); // controls overall motor speed by controlling setpoint of tilt angle
// Second Two PID loops control the PWM motors based on input tilt and tilt setpoint controlled by above PID loops
  PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
  PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

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
  if (digitalRead(STARTERPIN) == LOW){
    haltBalancing = !haltBalancing;
  }
  }
  bool prevStarterState = digitalRead(STARTERPIN);
  
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

  // Read in the desired additional motion of joystick
  xJoystick = analogRead(A_XPIN);
  yJoystick = analogRead(A_YPIN);
  spinControl = analogRead(A_SPIN_PIN);

  // Put results into the PID variable inputs
  pitchInput =  yFuseAngle; // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees
  rollInput = xFuseAngle; // // need to make sure they are all on the same scale so we map joystick input to 0-5 degrees

  // Put our desired tilt into PID setpoint variable
  pitchSetpoint = map(xJoystick, 0, 1024, -5, 5); // we map joystick to tell the robot to pitch 0-5 degrees for movement control
  rollSetpoint = map(xJoystick, 0, 1024, -5, 5); // we map joystick to tell the robot to roll 0-5 degrees for movement control

  // Compute Outputs for the PID loop
  pitchPID.Compute(); // PID library does calculations to do proper motor outputs
  rollPID.Compute();


// Dynamically deciding spin control calculates the current fastest possible speed of a wheel, and subtracts that from the theoretical fastest, and converts to a PWM signal

  float* spin;
  float* horizontalSpeed;
  float* forwardSpeed;

  *spin = spinControl;  // Calculate and scale this value appropriately before putting into below vector
  *horizontalSpeed = rollOutput;
  *forwardSpeed = pitchOutput;


// Do the math to move robot and send commands
// Desired movement inputs
BLA:: Matrix<3, 1> PIDOutputVector = {*spin, *horizontalSpeed, *forwardSpeed}; // omega (spin), x, y.  PWM values are actually put into this variable (The PID loop will compensate for discrepancy)

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
    digitalWrite(MOTOR2BWDPIN,HIGH);
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
  
// Write to the motors
  analogWrite(controlPin1, motor1Output);
  analogWrite(controlPin2, motor2Output);
  analogWrite(controlPin3, motor3Output);

} 

}

}


// DISCLAIMER:: This program is PID feedback controlled and needs to be modified for dimensional accuracy should real life speed/distance control be desired
