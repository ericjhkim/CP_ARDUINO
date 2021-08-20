#include <Arduino_LSM6DS3.h>
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <Average.h>

PololuQik2s9v1 qik(10, 11, 4);                  // Motor encoder arduino connection pins
float xlx, xly, xlz;                            // Accelerometer values
float gx, gy, gz;                               // Gyroscope values

// Rotary Encoder Inputs
#define CLK 2
#define DT 3
#define ENCVCC 12
#define ENCGND 9
#define ENCPPR 150 // Counts/revln

float WHEEL_DIA = 0.08;                         // Wheel diameter in metres
int counter = -1;                               // Wheel encoder counter
int state_clk;                                  // Flag to check whether encoder value should be updated
int state_clk_prev;                                 // - Related flag
float state_x = 0;                              // Most recent position measurement
float state_x_prev = 0;                         // Previous position measurement

long state_tpos = 0;                            // Absolute most recent time for position/velocity calculations
long state_tpos_prev = 0;                       // Absolute previous time for position/velocity calculations
float state_dx = 0;                             // Velocity state measurement
float dt_v = 5;                                 // Velocity state update interval (ms)

float state[4] = {0, 0, 0, 0};                  // Structure for storing states {x, dx, theta, dtheta}
float x_max = 1;                                // Maximum position value
float theta_max = 60 * PI / 180;                // Maximum theta value

long state_t_init;                              // Time when arduino completed initialization (to have a time buffer between initialization and running functions)
long state_t;                                   // Current time
long state_t_prev;                              // Time at last measurement
float dt;                                       // Time elapsed since last pole angle measurement
float theta_xl;                                 // Pole angle measurement from accelerometer
float state_th;                                 // Variable for pole angle estimation
float state_dth;                                // Variable for pole angle rate of change
float state_th_prev;                            // Variable for previous pole angle estimation (used only when performing integration for dth)

int varyindx = 0;                               // Index for velocity smoothing filter
float varray[5] = {0,0,0,0,0};                  // Structure for velocity smoothing filter
int varysize = sizeof(varray)/4;                // Size of array in bytes - 4 bytes in float

bool init_flag = true;                          // Second initialization semaphore

Average<float> th_ave(3);                       // Smoothing filter for pole angle measurement (from external library)
Average<float> dth_ave(3);                      // Smoothing filter for d(pole angle) measurement (from external library)

// NEURAL NETWORK STUFF
// Node weights (x, dx, theta, dtheta)
float w1 = 0;
float w2 = 2.434;
float w3 = 2.095;
float w4 = 1.228;
float wB = 0.01461;

float endNode;

// MOTOR CONTROLS AND LIMITS
int speedVal = 0;
int maxSpeed = 32;

//==============SETUP======================

void setup() {

    Serial.begin(9600);
    qik.init();
    encoderInit();

    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");

        while (1);
    }

    Serial.println("<Arduino is ready>");
    state_t_init = millis();
    
}

//===============LOOP=======================

void loop() {

    float command;

    if (init_flag){
        // Initial encoder time measurement
        state_tpos_prev = millis();

        // Initial pole angle measurement
        IMU.readAcceleration(xlx, xly, xlz);
        state_th = atan(xlz / xly + 0.18);
        state_th_prev = state_th;
        state_t_prev = millis();
        init_flag = false;
        Serial.println("<Second initialization complete>");
    }

    // SENSOR TEST
    while ((state_t - state_t_init)/1000 < 15) {
        getStates();
        command = feedForward(state);
        
        if (abs(state[2]) < 5*PI/180){
            maxSpeed = 32;
        }
        else{
            maxSpeed = 127;
        }

        if (command > 0.5) { //move forwards
            // speedVal += 10;
            // speedVal = min(speedVal, maxSpeed);
            // qik.setSpeeds(speedVal, speedVal);
            qik.setSpeeds(maxSpeed, maxSpeed);
        }
        else { //move backwards
            // speedVal -= 10;
            // speedVal = max(speedVal, -maxSpeed);
            // qik.setSpeeds(speedVal, speedVal);
            qik.setSpeeds(-maxSpeed, -maxSpeed);
        }

        if (abs(state[2]) > theta_max){
            Serial.println("<FAILED: Agent crashed>");
            break;
        }

        Serial.print(state[0]);                 // Position
        Serial.print("__:__");
        Serial.print(state[1]);                 // Velocity
        Serial.print("__:__");
        Serial.print(state[2]*180/PI);          // Pole angle
        Serial.print("__:__");
        Serial.print(state[3]*180/PI);          // Angular velocity
        Serial.print("__:__");
        Serial.println(millis());               // Time
    }

    qik.setSpeeds(0, 0);
    Serial.println("<EXPERIMENT ENDED>");
    while (1) {
    }

}

//==========================================
// Takes raw IMU and encoder data and calculates position, velocity, pole angle, angular velocity
void getStates() {

    // Set distance ===================================================================================
    state[0] = state_x;

    // Set velocity ===================================================================================
    state[1] = state_dx;
    
    // if (state_x_prev == state_x) {                                          // Force velocity to be 0 if no change in distance from previous measurement
    // state[1] = 0;
    // }
    // state_x_prev = state_x;

    // Set pole angle =================================================================================
    // (Experimentally determined 0.18 bias for tuning)
    state_t = millis();                                                         // Convert time to seconds
    if (state_t - state_t_prev >= 5){                                           // Update angle measurements at most every 5 ms to avoid inf division errors (typically 50 ms)
        IMU.readGyroscope(gx, gy, gz);                                          // Take gyroscope measurement
        IMU.readAcceleration(xlx, xly, xlz);                                    // Take accelerometer measurement
        theta_xl = atan(xlz / xly) + 0.18;
        dt = (state_t - state_t_prev);                                          // Time since last data call (seconds)
        state_th = 0.9 * (state_th + (-gx * PI / 180) * (dt/1000)) + 0.1 * theta_xl;   // Kalman filter operation to combine gyro and xl measurements (gyro outputs in deg/s)

        // th_ave.push(state_th);
        // state[2] = th_ave.mean();                                               // Theta with averaging filter
        state[2] = state_th;                                                 // Theta without averaging filter

        // Set angular velocity ===========================================================================
        state_dth = (-gx+1) * PI / 180;                                         // 1 degree tuning
        // dth_ave.push(state_dth);
        // state[3] = dth_ave.mean();                                              // dTheta with averaging filter
        state[3] = state_dth;

        // state[3] = (state[2] - state_th_prev)/(dt/1000);                     // dTheta without averaging filter
        state_th_prev = state[2];                                            // Previous angle value is unused
        state_t_prev = state_t;                                                 // Update latest "previous time"
    }
}

//==========================================
// NEAT neural network - takes inputs and outputs the value at output node
float feedForward(float nnInputs[]) { 
  endNode = w1 * nnInputs[0] + w2 * nnInputs[1] + w3 * nnInputs[2] + w4 * nnInputs[3];
  endNode += wB;
  return sigmoid(endNode);
}

//==========================================
//Sigmoid Activation function
float sigmoid(float x) {
  float y = 1 / (1 + exp(-1 * x));
  return y;
}

//==========================================
// Updates encoder count via interrupt for position and velocity measurements
void updateEncoder() {
    // Read the current state of CLK
    float varysum = 0;
    float vmean = 0;

    state_clk = digitalRead(CLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (state_clk != state_clk_prev  && state_clk == 1) {

        // If the DT state is different than the CLK state, the encoder is rotating CCW so decrement
        if (digitalRead(DT) != state_clk) {
            counter --;
        } else {
        // Encoder is rotating CW so increment
            counter ++;
        }

        // Calculate cart position based on wheel encoder counter
        state_x = WHEEL_DIA * PI * counter / ENCPPR;

    }

    // Update velocity value on every 10 ms (if time interval dt is too small, errors may occur)
    state_tpos = millis();
    if (state_tpos - state_tpos_prev >= dt_v) {
        state_dx = 1000*(state_x - state_x_prev) / (state_tpos - state_tpos_prev);

        // Smoothing filter for velocity measurements
        varray[varyindx] = state_dx;
        varyindx ++;
        if (varyindx == varysize){
            varyindx = 0;
        }
        for (int i = 0; i < varysize; i++){
            varysum += varray[i];
        }
        vmean = varysum/varysize;
        state_dx = vmean;
        
        state_tpos_prev = state_tpos;
        state_x_prev = state_x;
    }

  // Remember last CLK state
  state_clk_prev = state_clk;
}

//==========================================
// Initializes encoder interrupt routine
void encoderInit() {
    pinMode(ENCVCC, OUTPUT);
    pinMode(ENCGND, OUTPUT);
    digitalWrite(ENCVCC, HIGH);
    digitalWrite(ENCGND, LOW);

    pinMode(CLK, INPUT);
    pinMode(DT, INPUT);

    // Read the initial state of CLK
    state_clk_prev = digitalRead(CLK);

    // Call updateEncoder() when any high/low changed seen on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);

    state_tpos_prev = millis();
}