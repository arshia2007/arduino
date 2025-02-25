#include<Servo.h>
#include <Wire.h>
IntervalTimer myTimer;

// #include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1
 
// TFLI2C tflI2C;
 
// int16_t  tfDist;    // distance in centimeters
// int16_t  tfAddr = TFL_DEF_ADR;  // Use this default I2C address

int roller1_pin=22;//left
int roller1_spd=20;

int roller2_pin=17;//right
int roller2_spd=19;

int piston1=0;
int piston2=1;

int ball_push=2;
int ball_pull=3;

Servo servo_left;
Servo servo_right;


void setup(){
  servo_left.attach(37);
  servo_right.attach(36);

  //myTimer.begin(tof,5000);
  //delay(1000);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(roller1_pin, OUTPUT);
  pinMode(roller1_spd, OUTPUT);

  pinMode(roller2_pin, OUTPUT);
  pinMode(roller2_spd, OUTPUT);

  pinMode(ball_push,OUTPUT);
  pinMode(ball_pull,OUTPUT);
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,LOW);

  pinMode(piston1,OUTPUT);
  pinMode(piston2,OUTPUT);
  digitalWrite(piston1,LOW);
  digitalWrite(piston2,LOW);

  for(int angle=30;angle>=0;angle--){
      if(angle<125)
          servo_right.write(180-angle);
        //Serial.println(180-angle);
        
        servo_left.write(angle);
        Serial.println(angle);
        delay(15);
      }


}
void servomotion(int start_angle, int end_angle){
  if(start_angle<end_angle){ 
       for(int angle=start_angle;angle<=end_angle;angle++){
        if(angle<125)
          servo_right.write(180-angle-5);
        
        servo_left.write(angle);
        delay(15);
      }
    }
    else if(start_angle>end_angle){
      for(int angle=start_angle;angle>=end_angle;angle--){
      if(angle<125)
        servo_right.write(180-angle);
        //Serial.println(180-angle);
        
        servo_left.write(angle);
        //Serial.println(angle);
        delay(15);
      }
    }
    
}

void rollers(){
 digitalWrite(roller1_pin,HIGH);
 analogWrite(roller1_spd,255);

//  digitalWrite(roller2_pin,HIGH);
//  analogWrite(roller2_spd,255); 

}

void stoprollers(){
 digitalWrite(roller1_pin,LOW);
 analogWrite(roller1_spd,0);

 digitalWrite(roller2_pin,LOW);
 analogWrite(roller2_spd,0); 


}

void dcv_control(){
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,HIGH);
  delay(1000);

  // digitalWrite(ball_push,LOW);
  // digitalWrite(ball_pull,LOW);
  // delay(1000);

  digitalWrite(ball_push,HIGH);
  digitalWrite(ball_pull,LOW);
  delay(500);
 digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,HIGH);
  delay(500);
    digitalWrite(ball_push,HIGH);
  digitalWrite(ball_pull,HIGH);  
}
// void tof(){
//   if(tflI2C.getData(tfDist, tfAddr)){
//        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
//     // }
//   //delay(50);
//   if(tfDist<25){
//   digitalWrite(piston1,HIGH);
//   digitalWrite(piston2,LOW);
//   delay(1000);
//   // digitalWrite(piston1,LOW);
//   // digitalWrite(piston2,LOW);
//   // delay(500);
//   digitalWrite(piston1,LOW);
//   digitalWrite(piston2,HIGH);
  

//   }}
//}
void function(){
  digitalWrite(piston1,LOW);
  digitalWrite(piston2,HIGH);
  delay(1000);
  // digitalWrite(piston1,LOW);
  // digitalWrite(piston2,LOW);
  // delay(500);
  digitalWrite(piston1,HIGH);
  digitalWrite(piston2,LOW);
  delay(500);
  digitalWrite(piston1,HIGH);
  digitalWrite(piston2,HIGH);
  
}
int value=0;
void loop()
{
//Serial.println("Okay");
if (Serial.available() > 0) {  // Check if data is available
        String input = Serial.readStringUntil('\n');  // Read input
        input.trim();  // Remove any whitespace
        value = input.toInt();  // Convert the entire string to an integer

        //Serial.print("Received value: ");
    }              Serial.println(value);


//Serial.println(value);


    if(value==1){
    rollers();
    // dcv_control();
    servomotion(0,118);
    delay(2000);
    dcv_control();
    // function();
      //tof();
      value=-1;
    }
    else if(value==2){
      servomotion(118,0);
      stoprollers();
      value=-2;
    }
    else if(value==3){
      function();
      //servomotion(135,0);
    //rollers();
      value=-3;
    }
    }