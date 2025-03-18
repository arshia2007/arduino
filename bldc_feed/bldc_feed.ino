/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

float current = 0; /** The current in amps */

Encoder encFeed(7,6);
// int feed = 0;

int feeder_pwm=18;
int feeder_dir=16;

int feeder_cpr=538;
float ap_count_feeder=0;
float sp_angle_feeder=0;
float sp_count_feeder=0;

float kP=150,kI=5,kD=1;


void setup() {
  Serial.begin(2000000);
  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial8.begin(115200);
  
  while (!Serial8) {;}
  analogWrite(13,255);
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial8);

  feed_pid_timer.begin(feed_pos_pid,10000);

  pinMode(feeder_pwm,OUTPUT);
  pinMode(feeder_dir,OUTPUT);
}
  float nrpm=0;
  float orpm = 0;

void setPosition(int pwm,int dir ,int speed)
{
  digitalWrite(dir,speed<0?LOW:HIGH);
  analogWrite(pwm,abs(speed));
}

void feed_pos_pid()
{

ap_count_feeder=encFeed.read();
sp_count_feeder=sp_angle_feeder*feeder_cpr/360.0;

err_feed=sp_count_feeder-ap_count_feeder;
der_feed=(err_feed-lastError_feed)/0.01;
integ_feed=integ_feed+(err_feed-lastError_feed)*0.01;
lastError_feed=err_feed;

speed_feed=kP*err_feed+kI*integ_feed+kD*der_feed;
constrain(speed_feed,-14361,1436);
// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.print("Sp_feed:");
// Serial.print(sp_angle_feed);
// Serial.print("     Ap_feed:");
// Serial.println(int((ap_angle_feed*360.0/shooter_rotation_cpr)));
// //       // Serial.println(enc1.read());
setPosition(feeder_pwm,feeder_dir,int(speed_feed));

}


void loop() {
  
  /** Call the function setCurrent() to set the motor current */

    if (Serial.available() > 0) {
                 String input = Serial.readString();//15001
                // Serial.println(input.length());
      
                  nrpm=input.toFloat()*7;
                  // feed=input.substring(5).toInt();
                 
               }
    if(nrpm != 0){
      sp_angle_feeder = -3550;
      while(orpm < nrpm){
        orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
        UART.setRPM(orpm);
        delay(50);
        Serial.println("1");
        Serial.println((orpm/7));
      }
      while(orpm > nrpm){
        orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
        UART.setRPM(orpm);
        delay(50);
        Serial.println("2");
        Serial.println((orpm/7));
      } 
    }else{
      orpm = 0;
      sp_angle_feeder = 0;

    }
//                Serial.print()
// Serial.print(current);
  //UART.setCurrent(current);
  // if (feed == 1){
  //   UART.setRPM(orpm);
  //   sp_angle_feeder=3550;
  // }else if (feed == 0){
  //   UART.setRPM(0);
  //   sp_angle_feeder = 0;

  // }


    

  // if ( UART.getVescValues() ) {
  //   Serial.println("Values: ");
  //   Serial.println(UART.data.rpm/7);
  //   Serial.println(UART.data.inpVoltage);
  //   Serial.println(UART.data.ampHours);
  //   Serial.println(UART.data.pidPos);

  //   Serial.println(UART.data.avgMotorCurrent);
  //   Serial.println(UART.data.avgInputCurrent);

  //   Serial.println(UART.data.tachometer);

  // }

  // UART.setBrakeCurrent(current);
  
}