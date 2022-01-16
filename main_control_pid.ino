/*-Rocket Fin Control System-*/
/*The side fin control system is developed based on PID algorithm*/
/*The system follows Fin_activation -> Fin_test -> close_loop_PID Process*/
/*Author: Bingcheng Suo*/
/*01/15/2022*/

#include <Wire.h>
#include <JY901.h>
#include <Servo.h>

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;

bool   system_start_flag = 1;
int    pos_initial_1 = 74;             //customized parameters for servos
int    pos_initial_2 = 100;
int    pos_initial_3 = 100;
int    pos_initial_4 = 105;
int    i;
int    t = 1;
double angle_servo_max = 30.000;       // maximum servo rotation angle 
double angle_servo_increment;          // record the increment of angle based on initial servo position 
double v_angle_z;                      // record the angluar velocity in real time 
double angle_z;
double angle_z_temp=0.000;
double angle_sum = 0.000;
double angle_z_offset = 0.000;
double error, previous_error = 0.000;  // initial error
double integral = 0.000;
double derivative;
double dt = 10.000;
double v_setpoint = 0.000;             // ideal static angular velocity
double kp = 0.1;                       // proportion parameter
double ki = 0;                         // integral parameter
double kd = 0.1;                       // derivative 

void setup() 
{
  Serial.begin(9600);
  JY901.StartIIC();
  pinMode(13, OUTPUT);       // LED pin, indicating system start status
  digitalWrite(13, HIGH);  
  while(system_start_flag){  //rotate to activate fins
    JY901.GetGyro();         // get angular velocity 
    v_angle_z = (float)JY901.stcGyro.w[2]/32768*2000; // convert to units in degree / second
    angle_sum += v_angle_z*t*0.01;
    delay(t);
    //Serial.print(angle_sum);
    //Serial.print("\n");
    if(abs(angle_sum)>=360.00){system_start_flag = 0;break;}
  }
  delay(2000);
  digitalWrite(13, LOW);
  servo_1.attach(A0);   // ativate all the servos
  servo_2.attach(A1);   
  servo_3.attach(A2);  
  servo_4.attach(A3);
  servo_1.write(pos_initial_1); // calibrate servo zero position 
  servo_2.write(pos_initial_2);
  servo_3.write(pos_initial_3);
  servo_4.write(pos_initial_4);
  for(i=0;i<=4;i++){ // LED blink for calibration complete
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  delay(1000);
  digitalWrite(13, HIGH);
  // test whether seros are working properly, letting them rotate to desired angles
  for(angle_servo_increment=0;angle_servo_increment<=30;angle_servo_increment+=1){
    servo_1.write(pos_initial_1+angle_servo_increment);
    servo_2.write(pos_initial_2+angle_servo_increment);
    servo_3.write(pos_initial_3+angle_servo_increment);
    servo_4.write(pos_initial_4+angle_servo_increment);
    delay(40);
  }
  for(angle_servo_increment=30;angle_servo_increment>=-30;angle_servo_increment-=1){
    servo_1.write(pos_initial_1+angle_servo_increment);
    servo_2.write(pos_initial_2+angle_servo_increment);
    servo_3.write(pos_initial_3+angle_servo_increment);
    servo_4.write(pos_initial_4+angle_servo_increment);
    delay(40);
  }
  for(angle_servo_increment=-30;angle_servo_increment<=0;angle_servo_increment+=1){
    servo_1.write(pos_initial_1+angle_servo_increment);
    servo_2.write(pos_initial_2+angle_servo_increment);
    servo_3.write(pos_initial_3+angle_servo_increment);
    servo_4.write(pos_initial_4+angle_servo_increment);
    delay(40);
  }
  digitalWrite(13, LOW);
  delay(1000);
  for(i=0;i<=4;i++){ // LED blink for servo test complete
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
  }
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
  //Serial.print(angle_servo_increment);
  Serial.print("\n");
  delay(dt);
}
