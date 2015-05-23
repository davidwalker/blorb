 #include <application.h>


// these constants won't change:
int ledPin = D7;      // led connected to digital pin 13
int knockSensor = A0; // the piezo is connected to analog pin 0
int threshold = 2000;  // threshold value to decide when the detected sound is a knock or not


// these variables will change:
int sensorReading = 0;      // variable to store the value read from the sensor pin
int ledState = LOW;         // variable used to store the last LED status, to toggle the light


int redPin = D0;
int bluePin = D1;
int greenPin = A1;

int min = 3;
int max = 255;
int brightness = min;
int fadeAmount = 1;

int divider = 0;
int divLimit = 0;

int cycles = 2;

int setThreshold(String s);
int setMin(String s);
int setMax(String s);
int setDivLimit(String s);
void animate(void);

void setup() {
 Spark.variable("sensor", &sensorReading, INT);
 Spark.variable("threshold", &threshold, INT);
 Spark.variable("brightness", &brightness, INT);
 Spark.variable("fadeAmount", &fadeAmount, INT);
 Spark.variable("min", &min, INT);
 Spark.variable("max", &max, INT);
 Spark.variable("cycles", &cycles, INT);

 Spark.function("setThreshold", setThreshold);
 Spark.function("setDivLimit", setDivLimit);
 Spark.function("setMin", setMin);
 Spark.function("setMax", setMax);
 pinMode(ledPin, OUTPUT); // declare the ledPin as as OUTPUT

 pinMode(redPin, OUTPUT);
 pinMode(greenPin, OUTPUT);
 pinMode(bluePin, OUTPUT);

 Serial.begin(9600);       // use the serial port
 Serial.println("Starting...");
}

void loop() {

  Serial.println(greenPin);
  analogWrite(greenPin, 255);
  delay(1000);
  analogWrite(greenPin, 0);

  Serial.println(redPin);
  analogWrite(redPin, 255);
  delay(1000);
  analogWrite(redPin, 0);

  Serial.println(bluePin);
  analogWrite(bluePin, 255);
  delay(1000);
  analogWrite(bluePin, 0);


  if(false){
  sensorReading = analogRead(knockSensor);

  if(sensorReading < 2030 || sensorReading > 2050) {
    Serial.println(sensorReading);
  }

  // if the sensor reading is greater than the threshold:
  if (sensorReading <= threshold) {
    // toggle the status of the ledPin:
    ledState = !ledState;
    // update the LED pin itself:
    digitalWrite(ledPin, ledState);
    // send the string "Knock!" back to the computer, followed by newline
    //trigger = true;
    Serial.println("Knock!");
    if(cycles == 0) {
      cycles = 6;
    }
  }

  if(cycles > 0){
    animate();
  }

  delay(1);  // delay to avoid overloading the serial port buffer
  }
}


int setThreshold(String s)
{
  threshold = s.toInt();
  return threshold;
}
int setMin(String s)
{
  min = s.toInt();
  return min;
}
int setMax(String s)
{
  max = s.toInt();
  return max;
}
int setDivLimit(String s)
{
  divLimit = s.toInt();
  return divLimit;
}

void animate(){
  divider++;
  if (divider < divLimit)
    return;
  divider = 0;

  brightness += fadeAmount;

  if(brightness <= min || brightness >= max){ 
    fadeAmount = -fadeAmount;
    cycles--;
  }

  //analogWrite(redPin, brightness);
  //analogWrite(greenPin, brightness);
  //analogWrite(bluePin, brightness);
}



