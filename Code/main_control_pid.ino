#include <Wire.h>
#include <JY901.h>
#include <Servo.h>

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;

//customized parameters for servos
int pos_initial_1 = 74;
int pos_initial_2 = 100;
int pos_initial_3 = 92;
int pos_initial_4 = 95;

double angle_servo_max = 45.000;       // maximum servo rotation angle 
double angle_servo_increment;          // record the increment of angle based on initial servo position 
double v_angle_z;                      // record the angluar velocity in real time 

double error, previous_error = 0.000;  // initial error
double integral = 0.000;
double derivative;
double dt = 5.000;
double v_setpoint = 0.000;  // ideal static angular velocity
double kp = 0.1;  // proportion parameter
double ki = 0;    // integral parameter
double kd = 0.1;  // derivative parameter

void setup() 
{
  Serial.begin(9600);
  JY901.StartIIC();
  servo_1.attach(A0);   
  servo_2.attach(A1);   
  servo_3.attach(A2);  
  servo_4.attach(A3); 
} 

void loop() 
{
  JY901.GetGyro(); // get angular velocity 
  v_angle_z = (float)JY901.stcGyro.w[2]/32768*2000; // convert to units in degree / second
  error = -(v_setpoint - v_angle_z);
  integral = integral + error * dt;
  derivative = (error - previous_error)/dt;
  angle_servo_increment = kp * error + ki * integral + kd * derivative; // output
  if (abs(angle_servo_increment)>=angle_servo_max){angle_servo_increment=(angle_servo_increment/abs(angle_servo_increment))*angle_servo_max;}
  servo_1.write(pos_initial_1+angle_servo_increment);
  servo_2.write(pos_initial_2+angle_servo_increment);
  servo_3.write(pos_initial_3+angle_servo_increment);
  servo_4.write(pos_initial_4+angle_servo_increment);
  previous_error = error;
  Serial.print(angle_servo_increment);
  Serial.print("\n");
  delay(dt);
}
