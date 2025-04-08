#include <Pixy2I2C.h>
Pixy2I2C pixy;

int piston[4]={31,32,26,27};

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
analogWrite(13,255);
for(int i=0;i<4;i++)
  pinMode(piston[i],OUTPUT); 
pixy.init();
digitalWrite(piston[2],HIGH);
digitalWrite(piston[3],LOW);
delay(500);
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],LOW);

}
String input="";
void loop() {
  // put your main code here, to run repeatedly:
if(Serial.available())
{
  input=Serial.readStringUntil('\n');

  
}
Serial.println(input);
Serial.println("100");
if(input == "s")
{
  digitalWrite(piston[2],HIGH);
  digitalWrite(piston[3],LOW);
  delay(500);
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],LOW);
  delay(500);
  
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],HIGH);
  delay(600);
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],LOW);
 pixy.setLamp(1, 1);
    while (true) {
      Serial.println("pixy_while");
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        digitalWrite(piston[1],LOW);
        digitalWrite(piston[0],HIGH); 
        delay(500);
        digitalWrite(piston[1],HIGH);
        digitalWrite(piston[0],LOW);
        delay(500);
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }
}

        digitalWrite(piston[0],LOW);
        digitalWrite(piston[1],LOW);
        digitalWrite(piston[2],LOW);
        digitalWrite(piston[3],LOW);

            input="";

  }

}