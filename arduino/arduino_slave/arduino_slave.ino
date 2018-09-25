#include <Wire.h>
#include <SoftwareSerial.h>

byte distance1;
byte distance2;

SoftwareSerial zedboard_dist1(8, 9); // RX, TX
SoftwareSerial zedboard_dist2(10, 11); // RX, TX

void setup() {
  zedboard_dist2.begin(19200);
  zedboard_dist1.begin(19200);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  //Serial.begin(19200);
  distance1 = 0;
  distance2 = 0;
}

void loop() {
   
 zedboard_dist1.listen();
  if (zedboard_dist1.available()) {
    distance1 = zedboard_dist1.read();
  }
  delayMicroseconds(500);
  // Now listen on the second port
  zedboard_dist2.listen();
  if (zedboard_dist2.available()) {
    distance2 = zedboard_dist2.read();
  }
  delayMicroseconds(500);
  /*Serial.print(distance1, HEX);
  Serial.print(",");
  Serial.println(distance2, HEX);*/
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write(distance1); // respond with message of 4 bytes
  Wire.write(',');
  Wire.write(distance2);
  Wire.write('~');
  // as expected by master
}
