#include <Servo.h>
Servo right;
Servo left;
int last1 = 180, last2 = 0, i;
int dir = 21;
int pwm = 23;
void setup()
{
  pinMode(dir, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(9600);
  right.attach(36);
  left.attach(37);
  right.write(180);
  left.write(0);

}

void loop()
{
  // digitalWrite(dir, HIGH);
  // analogWrite(pwm,255);

  
  Serial.println("angle");
  if (Serial.available() > 0) {  
    String input = Serial.readString(); 

    if (input == "STOP"){
      right.write(50);
      left.write(50);
      Serial.println("Servo stopped");
    }else{ 
      int angle = input.toInt();
      Serial.println("Angle: ");
      if (angle>last2){
        for (i=last2; i <= angle; i++){
          right.write(180-i);
          left.write(i);
        }
      }else {
        for (i = last2; i>=angle; i--){
          right.write(180-i);
          left.write(i);
        }
      }
      last1 = 180-i;
      last2 = i;
    }
  }
}
