////////////////////////
// Inclide libraries
///////////////////////
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // Servo motor driver

////////////////////////////
// Setup constants
////////////////////////////
// Servo LD-3015MG
#define MIN_PULSE_WIDTH_1  500 // this is the 'minimum' pulse length count (out of 4096)
#define MAX_PULSE_WIDTH_1  2500 // this is the 'maximum' pulse length count (out of 4096)
#define MIN_ROT_1 0
#define MAX_ROT_1 270
#define CENTRAL_SERVO_PIN  0

// Servo SG90
#define MIN_PULSE_WIDTH_2  500 
#define MAX_PULSE_WIDTH_2  2400
#define MIN_ROT_2 0
#define MAX_ROT_2 180
#define LEFT_SERVO_PIN 1


#define MIN_PULSE_WIDTH_3  500 
#define MAX_PULSE_WIDTH_3  2400
#define MIN_ROT_3 -180
#define MAX_ROT_3 0
#define RIGHT_SERVO_PIN 2
//
#define FREQUENCY 50 // Analog servos run at ~50 Hz updates
// Slave arduino information
#define SLAVE_ARDUINO_ADDRESS 8
#define NUMBER_OF_BYTES_TO_BE_REQUESTED_FROM_SLAVE 4 // left_distance is 1 byte, distance 2 is 1 byte, they will be separated by "," which is 1 byte and will be terminated by "~" which is 1 byte


/////////////////////////
// Function Prototypes
/////////////////////////
int pulseWidth(int angle, int min_rotation, int max_rotation,int min_pulse_width,int max_pulse_width);
void write_to_serial_port();
void get_sensor_data();
void run_servo();
void stop_servo();
void position_servos_to_initial_position();
boolean isValidNumber(String str);
////////////////////////////
// Setup I2C communication
///////////////////////////
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);


/////////////////////////
// Global variables
//////////////////////////
int center_servo_current_angle = 90;
int run_center_servo = 0;
byte left_distance = 0;
byte right_distance = 0;
int center_servo_direction = 1;
int center_servo_end_angle = 165;
int center_servo_start_angle = 15;
int sensor_servo_angle = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  
  delay(10);
  position_servos_to_initial_position();
}

void loop() {
  if (Serial.available()){
        char command = (char)Serial.read();
        if (command == 'r' || command == 's' || command == 'a'){
          String command_str = Serial.readStringUntil('~'); 
          if(command == 'a'){
            set_sensor_servo(command_str);
            }else if (command == 'r'){
            run_servo();
          }else if (command == 's'){
            stop_servo();
            }
          }
    }

  if(run_center_servo == 0){
      write_to_serial_port(); 
      delay(30);
    }
  
    get_sensor_data();
    
    //
    if(run_center_servo == 1){
      write_to_serial_port(); 
      pwm.setPWM(CENTRAL_SERVO_PIN , 0, pulseWidth(center_servo_current_angle, MIN_ROT_1,MAX_ROT_1,MIN_PULSE_WIDTH_1,MAX_PULSE_WIDTH_1));
    if (center_servo_current_angle >= center_servo_end_angle) {
        center_servo_direction = -1;
        delay(30);
      }else if(center_servo_current_angle <= center_servo_start_angle) {
        center_servo_direction = 1;
        delay(30);
      }
     center_servo_current_angle = center_servo_current_angle + center_servo_direction;
    delay(30);
    }

  }



void get_sensor_data(){
    Wire.requestFrom(SLAVE_ARDUINO_ADDRESS, NUMBER_OF_BYTES_TO_BE_REQUESTED_FROM_SLAVE);    // request 4 bytes from slave device #8
    
    while (Wire.available()) { // slave may send less than requested
    left_distance = Wire.read();    // receive a byte
    char comma = Wire.read();
    // If the comma is not there the reading is not correct, so return.
    /*if (comma != ',') {
      left_distance = 0; 
      right_distance = 0;
      return;
    }*/
    right_distance = Wire.read();    // receive a byte
    char tilde = Wire.read();
    
    // If the tilde is not there the reading is not correct, so return.
    /*if (tilde != '~') {
      left_distance = 0; 
      right_distance = 0;
      return;      
      }*/
  }
}


//void write_to_serial_port(int angle, long left_distance, long right_distance, float angleX,float angleY, float angleZ, float headingFiltered, int run_center_servo_prm, int sensor_angle_prm){
void write_to_serial_port(){  
  //String temp = "{\"run_center_servo\":\""+(String)run_center_servo_prm+"\",\"sensor_servo_angle\":\""+(String)sensor_angle_prm+"\",\"left_distance\":\""+(String)left_distance+"\",\"right_distance\":\""+(String)right_distance+"\",\"center_servo_angle\":\""+(String)angle+"\",\"angleX\":\""+(String)angleX+"\",\"angleY\":\""+(String)angleY+"\",\"angleZ\":\""+(String)angleZ+"\",\"headingFiltered\":\""+(String)headingFiltered+"\"}~"; 
  //String temp = "{\"run_center_servo\":\""+(String)run_center_servo_prm+"\",\"left_distance\":\""+(String)left_distance+"\",\"right_distance\":\""+(String)right_distance+"\",\"center_servo_angle\":\""+(String)angle+"\",\"angleX\":\""+(String)angleX+"\",\"angleY\":\""+(String)angleY+"\",\"angleZ\":\""+(String)angleZ+"\",\"headingFiltered\":\""+(String)headingFiltered+"\"}~"; 
  //String temp = (String)angle;
  //String temp = "{\"a1\":\""+(String)run_center_servo_prm+"\",\"a2\":\""+(String)left_distance+"\",\"a3\":\""+(String)right_distance+"\",\"a4\":\""+(String)angle+"\",\"a5\":\""+(String)angleX+"\",\"a6\":\""+(String)angleY+"\",\"a7\":\""+(String)angleZ+"\",\"a8\":\""+(String)headingFiltered+"\"}~"; 
  //String temp = "{\"a1\":\""+(String)run_center_servo_prm+"\"}~"; 
  String temp = (String)run_center_servo+","+left_distance+","+right_distance+","+(String)center_servo_current_angle+","+(String)sensor_servo_angle+"~"; 
  //long randNumber1 = random(20, 32);
  //long randNumber2 = random(20, 32);
  //String temp = (String)run_center_servo+","+(String)randNumber1+","+(String)randNumber2+","+(String)center_servo_current_angle+","+(String)sensor_servo_angle+"~"; 
  Serial.print(temp);
  Serial.flush();
  }


int pulseWidth(int angle, int min_rotation, int max_rotation,int min_pulse_width,int max_pulse_width) {
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, min_rotation, max_rotation, min_pulse_width, max_pulse_width);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}


void run_servo(){
  run_center_servo = 1;
  }

void stop_servo(){
  run_center_servo = 0;
  }
  
void set_sensor_servo(String command_str){
   if (isValidNumber(command_str)){
    sensor_servo_angle = command_str.toInt();
     if (sensor_servo_angle >= 0 && sensor_servo_angle < 180) {
        pwm.setPWM(LEFT_SERVO_PIN , 0, pulseWidth(sensor_servo_angle, MIN_ROT_2,MAX_ROT_2,MIN_PULSE_WIDTH_2,MAX_PULSE_WIDTH_2));
        pwm.setPWM(RIGHT_SERVO_PIN , 0, pulseWidth(-sensor_servo_angle, MIN_ROT_3,MAX_ROT_3,MIN_PULSE_WIDTH_3,MAX_PULSE_WIDTH_3));
     }else {
      int sensor_servo_angle = 45;
         pwm.setPWM(LEFT_SERVO_PIN , 0, pulseWidth(sensor_servo_angle, MIN_ROT_2,MAX_ROT_2,MIN_PULSE_WIDTH_2,MAX_PULSE_WIDTH_2));
         pwm.setPWM(RIGHT_SERVO_PIN , 0, pulseWidth(-sensor_servo_angle, MIN_ROT_3,MAX_ROT_3,MIN_PULSE_WIDTH_3,MAX_PULSE_WIDTH_3));     }
  }
}

void position_servos_to_initial_position(){
      Serial.println("Positioning central servo to 90 degrees...");
      int i = 15;
      for(i = 45; i < 90; i++){
      pwm.setPWM(CENTRAL_SERVO_PIN , 0, pulseWidth(i, MIN_ROT_1,MAX_ROT_1,MIN_PULSE_WIDTH_1,MAX_PULSE_WIDTH_1));
      delay(50);
      }
      Serial.println("Central servo is at "+ String(i));
      center_servo_current_angle = 90;
      
     Serial.println("Positioning side servos to 90 degree...");
      for(i = 5; i < 175; i++){
         pwm.setPWM(LEFT_SERVO_PIN , 0, pulseWidth(i, MIN_ROT_2,MAX_ROT_2,MIN_PULSE_WIDTH_2,MAX_PULSE_WIDTH_2));
         pwm.setPWM(RIGHT_SERVO_PIN , 0, pulseWidth(-i, MIN_ROT_3,MAX_ROT_3,MIN_PULSE_WIDTH_3,MAX_PULSE_WIDTH_3));
      delay(30);
        }
     for(i = 175; i >= 90; i--){
         pwm.setPWM(LEFT_SERVO_PIN , 0, pulseWidth(i, MIN_ROT_2,MAX_ROT_2,MIN_PULSE_WIDTH_2,MAX_PULSE_WIDTH_2));
         pwm.setPWM(RIGHT_SERVO_PIN , 0, pulseWidth(-i, MIN_ROT_3,MAX_ROT_3,MIN_PULSE_WIDTH_3,MAX_PULSE_WIDTH_3));
      delay(30);
        }
      Serial.println("Side servo is at "+ String(i));
      sensor_servo_angle = 90;

  }

  boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(isDigit(str.charAt(i))) return true;
        }
   return false;
}
