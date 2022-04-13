#include <Arduino_LSM6DS3.h>
#include <Average.h>

float xlx, xly, xlz;                            // Accelerometer values
float gx, gy, gz;                               // Gyroscope values

float RAD2DEG = 180/PI;

bool init_flag = true;                          // Second initialization semaphore

float loop_time;                                // Timing variables for angle estimation
float t_start;
float t_now;
float theta;                                    // Angle estimation
float theta_xl;                                 // Angle estimation from accelerometer

Average<float> th_ave(10);                      // Smoothing filter for pole angle measurement (from external library)

//==============SETUP======================
void setup() {

    Serial.begin(9600);

    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");

        while (1);
    }

    Serial.println("<Arduino is ready>");

    t_start = millis();
}

//===============LOOP=======================
void loop() {
    getAngle();

    Serial.print(xlx);                  // Accelerometer readings
    Serial.print("  |  ");              // Separator
    Serial.print(xly);
    Serial.print("  |  ");
    Serial.print(xlz);
    Serial.print("  |  ");
    Serial.print(gx);                   // Gyroscope readings
    Serial.print("  |  ");
    Serial.print(gy);
    Serial.print("  |  ");
    Serial.print(gz);
    Serial.print("  |  ");
    Serial.println(theta);              // Angle estimation

    t_start = t_now;
}

//==========================================
// Takes raw IMU and calculates pole angle
void getAngle() {
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);                                      // Take gyroscope measurement (degrees/s)
    }

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(xlx, xly, xlz);                                // Take accelerometer measurement
    }

    if (!isnan(gx) && !isnan(xly) && !isnan(xlz)){                          // Prevent nan values in angle estimation
        t_now = millis();
        loop_time = t_now - t_start;
        theta_xl = atan2(-xlz,xly)*RAD2DEG;
        theta = 0.9*(theta+gx*(loop_time/1000.0)) + 0.1*theta_xl;           // Combine gyro and xl measurements (gyro outputs in deg/s)
        
    //    th_ave.push(theta);                                                  // Uncomment to filter noise
    //    theta = th_ave.mean();                                               // Uncomment to filter noise
    }
}