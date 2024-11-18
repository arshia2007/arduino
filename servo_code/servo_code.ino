#include <Servo.h>
Servo s1;
Servo s2;
int last1 = 180, last2 = 0, i;

void setup()
{
  Serial.begin(9600);
  s1.attach(3);
  s2.attach(5);
  s1.write(180);
  s2.write(0);

}

void loop()
{
  if (Serial.available() > 0) {  
    String input = Serial.readString(); 

    if (input == "STOP"){
      s1.write(50);
      s2.write(50);
      Serial.println("Servo stopped");
    }else{ 
      int angle = input.toInt();
      Serial.println("Angle: ");
      if (angle>last2){
        for (i=last2; i <= angle; i++){
          s1.write(180-i);
          s2.write(i);
        }
      }else {
        for (i = last2; i>=angle; i--){
          s1.write(180-i);
          s2.write(i);
        }
      }
      last1 = 180-i;
      last2 = i;
    }
  }
}
