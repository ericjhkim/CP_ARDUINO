/*
  Nano-Bot for Arduino Nano 33 IOT (for upload via Arduino IDE)
*/

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <PID_v1.h>

float RAD2DEG = 180/PI;

float ax, ay, az, gx, gy, gz;
float theta = 0.0;
float dtheta = 0.0;

byte cmd[4]; // serial command buffer

long first_time, now_time;

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the Tuning Parameters
double Kp=1.0, Ki=0.0, Kd=0.0;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//###################################################################
void setup() {

  // Start hardware serial
  // Reset the qik
  digitalWrite(4, LOW);
  pinMode(4, OUTPUT); // drive low
  delay(1);
  pinMode(4, INPUT);  // return to high-impedance input (reset is internally pulled up on qik)
  delay(10);

  // Initialize motor controller serial
  Serial1.begin(9600);
  Serial1.write(0xAA);

  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Setpoint = 0.0;
  myPID.SetOutputLimits(-127, 127);       //[-127, 127] tell the PID to range between motor PWM response
  myPID.SetSampleTime(10);                // Ensure that the PID is called at regular interval  
  myPID.SetMode(AUTOMATIC);               //turn the PID on 
  
  Serial.println("Initialization complete");
}


//###################################################################
void loop() {

  first_time = millis();
  now_time = millis();

  while ((now_time - first_time)/1000.0 < 15.0){  

    getAngle();
    myPID.Compute();
    setMotors(Output,Output);
    
    Serial.print(Input);
    Serial.print("__:__");
    Serial.print(Output);
    Serial.print("__:__");
    Serial.println((now_time - first_time)/1000.0);
    
    now_time = millis();
  }

  Serial.println("Test ended.");
  setMotors(0,0);
  while(1);
}

//###################################################################
void getAngle() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    theta = atan(ax/sqrt(sq(ay)+sq(az)))*RAD2DEG;
    Input = theta;
  }
}

//###################################################################
void setMotors(int MLeft, int MRight) {
  if (MLeft>=0){ 
    cmd[0] = 0x88;
  }else{
    cmd[0] = 0x8A;
  }
  cmd[1] = abs(MLeft);
  
  if (MRight>=0){ 
    cmd[2] = 0x8C;
  }else{
    cmd[2] = 0x8E;
  }
  cmd[3] = abs(MRight);
  Serial1.write(cmd, 4);
}
