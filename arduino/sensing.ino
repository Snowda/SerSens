 
int ledPin = 13;        // choose the pin for the LED
int sensor1 = 2;        // choose the input pin (for sensor)
int errorState = false; // we start, assuming no motion detected
int value = false;            // variable for reading the pin status
 
void setup() {
  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(sensor1, INPUT);     // declare sensor as input
 
  Serial.begin(9600);
}
 
void loop(){
  val = digitalRead(sensor1);  // read input value
  if (value) {            // check if the input is HIGH
    digitalWrite(ledPin, HIGH);  // turn LED ON
    if (!errorState) {
      // we have just turned on
      Serial.println("Sensor out of range.");
      // We only want to print on the output change, not state
      pirState = true;
    }
  } else {
    digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState){
      // we have just turned of
      Serial.println("Sensor ok");
      // We only want to print on the output change, not state
      pirState = false;
    }
  }
}
