// Encoder pin definitions
const int encoderPinA = 2; 
const int encoderPinB = 3; 

volatile int encoderPosition = 0; 
volatile unsigned long lastUpdateTime = 0; // To calculate time intervals
volatile int encoderCounts = 0; // Counts in the interval

const int ppr = 600; // Replace with your encoder's pulses per revolution (PPR)
//const int cpr = 2400; 
// Timing
const unsigned long interval = 1000; // Interval for RPM calculation (1 second)

void setup() {
  Serial.begin(9600);

  pinMode(encoderPinA, INPUT_PULLUP); //default HIGH to prevent fluctuating signals 
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  lastUpdateTime = millis(); // Initialize timer
}

void loop() {
  // Check if the time interval has elapsed
  if (millis() - lastUpdateTime >= interval) {    //miillis - keeps track of time (upto 50 days)without stopping the program (in milliseconds obv)
    // Calculate RPM
    double rpm = (encoderCounts / (double)ppr) * (60 * (1000.0 / interval));
    //Serial.print("RPM: ");
    //Serial.println(rpm);

    // Reset counts for the next interval
    encoderCounts = 0;
    lastUpdateTime = millis();
    
  }
}

void updateEncoder() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  static int lastStateA = stateA;
  static int lastStateB = stateB;

  if (lastStateA == stateA) {
    if (stateA == stateB) {
      encoderPosition++;
    } else {
      encoderPosition--;
    }
  } else {
    if (stateA == stateB) {
      encoderPosition--;
    } else {
      encoderPosition++;
    }
  }

  encoderCounts++; // Increment counts for RPM calculation

  lastStateA = stateA;
  lastStateB = stateB;


  // Debug: Display the encoder position
  Serial.print("Encoder Position: ");
  Serial.println(encoderPosition);
}
