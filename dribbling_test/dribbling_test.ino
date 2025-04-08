#include<Servo.h>
#include <Wire.h>
IntervalTimer myTimer;


int roller1_pin=28;//left
// int roller1_spd=23;


int extend=32;
int retract=31;

int ball_push=26;  //27
int ball_pull=27;  //26

// int dribble_arm_pwm[2] = {19, 18};
// int dribble_arm_dir[2] = {17, 16};

// bool limitSwitchState[2] = {false};       // 1        3
// int limitSwitch[2]={14,15};               // 0        2

Servo servo_left;
Servo servo_right;


void setup(){
  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  servo_left.attach(2);
  servo_right.attach(1);
  servo_left.write(180);
  servo_right.write(0);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(roller1_pin, OUTPUT);
  digitalWrite(roller1_pin,HIGH);
  // pinMode(roller1_spd, OUTPUT);

  pinMode(ball_push,OUTPUT);
  pinMode(ball_pull,OUTPUT);

  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,HIGH);
  delay(1000);
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,LOW);

  pinMode(extend,OUTPUT);
  pinMode(retract,OUTPUT);

  digitalWrite(extend,HIGH);
  digitalWrite(retract,LOW);
  delay(1000);
  digitalWrite(extend,LOW);
  digitalWrite(retract,LOW);

  // for(int i=0;i<2;i++)
  // {
  //   pinMode(limitSwitch[i],INPUT_PULLUP);
  // }

}

void servomotion(int start_angle, int end_angle){
  if(start_angle<end_angle){ 
    for(int angle=start_angle;angle<=end_angle;angle++){
    // if(angle<125)
    servo_right.write(angle);

    servo_left.write(180-angle);
    delay(15);
    }
  }
  else if(start_angle>end_angle){
    for(int angle=start_angle;angle>=end_angle;angle--){
    // if(angle<125)
    servo_right.write(angle);
    //Serial.println(180-angle);
    servo_left.write(180-angle);
    //Serial.println(angle);
    delay(15);
    }
  }     
}

void rollers(){
  digitalWrite(roller1_pin,LOW);
  // analogWrite(roller1_spd,255*64);

} 

void stoprollers(){
  digitalWrite(roller1_pin,HIGH);
  // analogWrite(roller1_spd,0);

}

void piston1(){
  digitalWrite(ball_push,HIGH);
  digitalWrite(ball_pull,LOW);
  delay(500);
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,HIGH);
  delay(500);
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,LOW);  
  delay(500);
}


void piston2(){
  digitalWrite(extend,LOW);
  digitalWrite(retract,HIGH);
  delay(600);

  digitalWrite(extend,HIGH);
  digitalWrite(retract,LOW);
  delay(500);

  digitalWrite(extend,LOW);
  digitalWrite(retract,LOW);

}

// void dribble_arm_out(){
//   for (int i = 0; i < 2; i++){
//     limitSwitchState[i] = digitalRead(limitSwitch[i]);
//   }
//   while (limitSwitchState[1] != LOW){
//     for (int i = 0; i < 2; i++){
//       digitalWrite(dribble_arm_dir[i], HIGH);
//       analogWrite(dribble_arm_pwm[i], 255*32);
//     }

//     limitSwitchState[1] = digitalRead(limitSwitch[1]);
//     // limitSwitchState[3] = digitalRead(limitSwitch[3]);
//   }

//   for (int i = 0; i < 2; i++){
//     digitalWrite(dribble_arm_dir[i], HIGH);
//     analogWrite(dribble_arm_pwm[i], 255*0);
//   }

// }

// void dribble_arm_in(){
//   for (int i = 0; i < 2; i++){
//     limitSwitchState[i] = digitalRead(limitSwitch[i]);
//   }
//   while (limitSwitchState[0] != LOW){
//     for (int i = 0; i < 2; i++){
//       digitalWrite(dribble_arm_dir[i], LOW);
//       analogWrite(dribble_arm_pwm[i], 255*32);
//     }

//     limitSwitchState[0] = digitalRead(limitSwitch[0]);
//     // limitSwitchState[2] = digitalRead(limitSwitch[2]);
//   }

//   for (int i = 0; i < 2; i++){
//     digitalWrite(dribble_arm_dir[i], HIGH);
//     analogWrite(dribble_arm_pwm[i], 255*0);
//   }

// }

int value=0;
void loop()
{
  // for (int i = 0; i < 4; i++){
  //   limitSwitchState[i] = digitalRead(limitSwitch[i]);

  // }

  //Serial.println("Okay");
  if (Serial.available() > 0) {  // Check if data is available
        String input = Serial.readStringUntil('\n');  // Read input
        input.trim();  // Remove any whitespace
        value = input.toInt();  // Convert the entire string to an integer

        //Serial.print("Received value: ");
  }              
  Serial.println(value);


  if(value==1){
    // dribble_arm_out();
    servomotion(0,90);
    delay(1000);
    piston1();
    delay(100); //200
    rollers(); 
    // delay(2000);
    value=-1;
  }
  else if(value==2){
    servomotion(90,0);
    stoprollers();
    // delay(200);
    // dribble_arm_in();
    value=-2;
  }
  else if(value==3){
    piston2();
    value=-3;
  }
 
}