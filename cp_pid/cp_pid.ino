#include <Arduino_LSM6DS3.h>
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <Average.h>
#include <PID_v1.h>

PololuQik2s9v1 qik(10, 11, 4);                  // Motor encoder arduino connection pins
float xlx, xly, xlz;                            // Accelerometer values
float gx, gy, gz;                               // Gyroscope values

float RAD2DEG = 180/PI;
float TH_OFFSET = -7.0;                         // Offset for pole angle calibration
float MAX_THETA = 80;                           // Maximum theta value

long first_time, now_time, prev_time=0, loop_time;

float state_th;                                 // Variable for pole angle estimation
float theta_xl;                                 // Pole angle measurement from accelerometer

bool init_flag = true;                          // Second initialization semaphore

Average<float> th_ave(10);                       // Smoothing filter for pole angle measurement (from external library)

// MOTOR CONTROLS
int maxSpeed = 127;

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the Tuning Parameters
double consKp=8.5, consKi=0.0, consKd=0.4;
  
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//==============SETUP======================

void setup() {

    Serial.begin(19200);

    Setpoint = 0.0;
    
    qik.init();
    
    myPID.SetOutputLimits(-maxSpeed, maxSpeed);       //[-127, 127]//tell the PID to range between motor PWM response
    myPID.SetSampleTime(10);                          // Ensure that the PID is called at regular interval  
    myPID.SetMode(AUTOMATIC);                         //turn the PID on
    
    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    Serial.println("<Arduino is ready>");    
}

//===============LOOP=======================

void loop() {

    float command;

    if (init_flag){
        // Initial encoder time measurement
        state_tpos_prev = millis();

        // Initial pole angle measurement
        IMU.readAcceleration(xlx, xly, xlz);
        state_th = atan2(-xlz,xly)*RAD2DEG+TH_OFFSET;
        init_flag = false;
        Serial.println("<Second initialization complete>");
    }

    first_time = millis();
    now_time = millis();

    // SENSOR TEST
    while (((now_time - first_time)/1000.0 < 15) && (abs(Input) < MAX_THETA)) {
        now_time = millis();
        loop_time = now_time - prev_time;
        prev_time = now_time;
        
        getAngle();
        
        myPID.Compute();
        qik.setSpeeds(Output, Output);        

        Serial.print(Input);
        Serial.print("__:__");
        Serial.print(Output);
        Serial.print("__:__");
        Serial.println(now_time);
    }

    qik.setSpeeds(0, 0);
    Serial.println("<EXPERIMENT ENDED>");
    while (1) {
    }
}

//==========================================
// Takes raw IMU and encoder data and calculates pole angle for PID
void getAngle() {

    IMU.readGyroscope(gx, gy, gz);                                            // Take gyroscope measurement
    IMU.readAcceleration(xlx, xly, xlz);                                      // Take accelerometer measurement

    gx = gx - 1.22;                                                           // Stationary offset adjustment

    if (!isnan(gx) && !isnan(xly) && !isnan(xlz)){
        // Set pole angle =================================================================================
        theta_xl = atan2(-xlz,xly)*RAD2DEG+TH_OFFSET;
        state_th = 0.9*(state_th+gx*(loop_time/1000.0)) + 0.1*theta_xl;       // Kalman filter operation to combine gyro and xl measurements (gyro outputs in deg/s)
           
        th_ave.push(state_th);
        Input = th_ave.mean();                                                // Theta with averaging filter
    }
}
