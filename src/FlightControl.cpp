/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
#include <Wire.h>
#include <MPU9250.h>
#include <PIDWrapper.h>
#include <Motor.h>
#include <Arduino.h>

MPU9250 imu;

const double consKp = 3, consKi = 0.02, consKd = 0.45;
const double aggKp = consKp * 2, aggKi = consKi * 2, aggKd = consKd * 2;
const double consYawKp = consKp, consYawKi = consKi, consYawKd = consKd;
const double aggYawKp = consYawKp * 2, aggYawKi = aggYawKi * 2, aggYawKd = consYawKi * 2;
const double rollMinValue = -180, rollMaxValue = 180;
PIDWrapper rollPID(consKp, consKi, consKd, aggKp, aggKi, aggKd, rollMinValue, rollMaxValue);
const double pitchMinValue = -90, pitchMaxValue = 90;
PIDWrapper pitchPID(consKp, consKi, consKd, aggKp, aggKi, aggKd, pitchMinValue, pitchMaxValue);
const double yawMinValue = 0, yawMaxValue = 360;
PIDWrapper yawPID(consYawKp, consYawKi, consYawKd, aggYawKp, aggYawKi, aggYawKd, yawMinValue, yawMaxValue);

const int ch1 = 2;
const int ch2 = 3;
const int ch3 = 4;
const int ch4 = 5;
Motor motor1, motor2, motor3, motor4;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  delay(3000);
  Serial.println("Setup IMU");
  imu.Setup();

  Serial.println("Setup Motors");
  motor1.Setup(ch1);
  motor2.Setup(ch2);
  motor3.Setup(ch3);
  motor4.Setup(ch4);
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 temp = (int) (4*temp + .5);
 return (double) temp/4;
}

void loop() {
    // receive current sensor data
    MPU9250::RollPitchYaw rpy;
    bool newData = imu.CalculateRollPitchYaw(&rpy);

    if(!newData)
    {
      return;
    }

    // receive remote control set points
    double setpointRoll, setpointPitch, setpointYaw, setpointThrottle;
    setpointRoll = 0.0;
    setpointPitch = 0.0;
    setpointYaw = 180.0;
    setpointThrottle = 20.0;

    // calculate pids
    double roll, pitch, yaw;
    roll = rollPID.Compute(rpy.roll, setpointRoll);
    pitch = pitchPID.Compute(rpy.pitch, setpointPitch);
    yaw = yawPID.Compute(rpy.yaw, setpointYaw);

    roll = modifiedMap(roll, rollMinValue, rollMaxValue, 1000.0, 2000.0);
    pitch = modifiedMap(pitch, pitchMinValue, pitchMaxValue, 1000.0, 2000.0);
    yaw = modifiedMap(yaw, yawMinValue, yawMaxValue, 1000.0, 2000.0);
    setpointThrottle = modifiedMap(setpointThrottle, 0.0, 100.0, 1000.0, 2000.0);

    Serial.print("roll: "); Serial.print(rpy.roll); Serial.print("\t");Serial.print(setpointRoll); Serial.print("\t"); Serial.print(roll);
    Serial.print("\tpitch: "); Serial.print(rpy.pitch); Serial.print("\t");Serial.print(setpointPitch); Serial.print("\t"); Serial.print(pitch);
    Serial.print("\tyaw: "); Serial.print(rpy.yaw); Serial.print("\t"); Serial.print(setpointYaw); Serial.print("\t");Serial.print(yaw);
    Serial.println();

    // limit max throttle
    double throttle;
    throttle = min(setpointThrottle, 1800);

    // calculate escs
    double esc1, esc2, esc3, esc4;
    esc1 = throttle - pitch + roll - yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc2 = throttle + pitch + roll + yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc3 = throttle + pitch - roll - yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc4 = throttle - pitch - roll + yaw; //Calculate the pulse for esc 4 (front-left - CW)

    /*if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    }*/

    // set motor speeds
    motor1.SetSpeed(esc1);
    motor2.SetSpeed(esc2);
    motor3.SetSpeed(esc3);
    motor4.SetSpeed(esc4);
}
