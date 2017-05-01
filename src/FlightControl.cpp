/**********************************************************************************************
 * Quadcopter control v0.1
 * by Jakob Murschall
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Wire.h>
#include <IMU_Mahony.h>
#include <PIDWrapper.h>
#include <Motor.h>

IMU_Mahony imu;

const double consKp = 0.75, consKi = 0.4, consKd = 0.08;
const double aggKp = 0.875, aggKi = 0.5, aggKd = 0.1;
const double consYawKp = 0, consYawKi = 0, consYawKd = 0.2;
const double aggYawKp = 0, aggYawKi = 0, aggYawKd = 0.5;
PIDWrapper rollPID(consKp, consKi, consKd, aggKp, aggKi, aggKd);
PIDWrapper pitchPID(consKp, consKi, consKd, aggKp, aggKi, aggKd);
PIDWrapper yawPID(consYawKp, consYawKi, consYawKd, aggYawKp, aggYawKi, aggYawKd);

const int ch1 = 2;
const int ch2 = 3;
const int ch3 = 4;
const int ch4 = 5;
Motor motor1, motor2, motor3, motor4;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.setup();

  Serial.print("Setting up motors ...");
  motor1.Setup(ch1);
  motor2.Setup(ch2);
  motor3.Setup(ch3);
  motor4.Setup(ch4);
  Serial.println("done");
}

void loop() {
    // receive current sensor data
    IMU_Mahony::RollPitchYaw rpy = imu.CalculateRollPitchYaw();

    // receive remote control set points
    double setpointRoll, setpointPitch, setpointYaw, setpointThrottle;
    setpointRoll = 50;
    setpointRoll = map(setpointRoll, 0, 100, 1000, 2000);
    setpointPitch = 50;
    setpointPitch = map(setpointPitch, 0, 100, 1000, 2000);
    setpointYaw = 50;
    setpointYaw = map(setpointYaw, 0, 100, 1000, 2000);
    setpointThrottle = 50;
    setpointThrottle = map(setpointThrottle, 0, 100, 1000, 2000);

    // calculate pids
    double roll, pitch, yaw;
    roll = rollPID.Compute(rpy.roll, setpointRoll);
    pitch = pitchPID.Compute(rpy.pitch, setpointPitch);
    yaw = yawPID.Compute(rpy.yaw, setpointPitch);

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
