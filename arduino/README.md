# Arduino
## Code Description
Arduino code to read data from the ultrasonic sensor and send out the result over a serial port.

Servo is contorled by the Arduino Servo library and is connected to pin 12
```C
#include <Servo.h>
Servo myServo;
myServo.attach(12);
```

Tirg and Echo pins of the Ultrasonic Sensors are the followings: 
```C
#define ECHOPIN1 11
#define TRIGPIN1 10
#define ECHOPIN2 9
#define TRIGPIN2 8
```

Function for calculating the distance measured by the Ultrasonic sensor is the following:
```C
int calculateDistance(int triger_pin, int echo_pin){ 
  
  digitalWrite(triger_pin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triger_pin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(triger_pin, LOW);
  
  long duration_temp = pulseIn(echo_pin, HIGH); // Reads the echoPin1, returns the sound wave travel time in microseconds
  
  long distance_temp = duration_temp*0.034/2;
  return distance_temp;
}
```

The data is sent over the serial port with baud rate of 9600
```C
void loop() {
  // rotates the servo motor from 15 to 165 degrees
  for(int i=15; i < 165 ;i++){  
  myServo.write(i);
  delay(30);
  distance1 = calculateDistance(TRIGPIN1, ECHOPIN1);// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
  distance2 = calculateDistance(TRIGPIN2, ECHOPIN2);
  Serial.print(i); // Sends the current degree into the Serial Port
  Serial.print(","); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing

  Serial.print(distance1); // Sends the distance value into the Serial Port
  Serial.print(",");
  
  Serial.print(i);
  Serial.print(","); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
  
  Serial.print(distance2); // Sends the distance value into the Serial Port
  Serial.print("~"); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
  }
```