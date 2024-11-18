
int enablePin = 5;
int in1 = 9;
int in2 = 8;
int EN2 = 6;


void setup()
{
  pinMode(enablePin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(EN2, OUTPUT);
  Serial.begin(9600);
  digitalWrite(in1, LOW);
  analogWrite(enablePin, 0);
  digitalWrite(in2, LOW);
  analogWrite(EN2, 0);
}
void loop()
{
  digitalWrite(in1, HIGH);
  analogWrite(enablePin, 0);
  digitalWrite(in2, HIGH);
  analogWrite(EN2, 100);
}













