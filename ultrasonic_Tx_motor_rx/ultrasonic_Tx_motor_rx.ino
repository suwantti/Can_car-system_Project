#include <SPI.h> //front
#include <mcp2515.h>
#include  "Nunchuk.h"
#include <Wire.h>
//----------------------------------------------------------------------------------
#include <Wire.h> // 모터 테스트
#include <Adafruit_PWMServoDriver.h>

#define PIN_MOTOR_M1_IN1 8      //Define the positive pole of M1 
#define PIN_MOTOR_M1_IN2 9      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 6      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 7      //Define the negative pole of M2
Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);
//----------------------------------------------------------------------------------
                                  int mappedValue = 0;
int trigPin = 7;                  // ultrasonic 
int echoPin = 8;                  // ultrasonic 
float maxDistance = 200;          // ultrasonic 
float soundVelocity = 340;        // ultrasonic
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // ultrasonic 

struct can_frame canMsg;
struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(4);

int distance; // ultrasonic
int isObstacle = false;

unsigned long previousCANMillis = 0;  // CAN 메시지 전송 시간 기록
unsigned long CANInterval = 10;       // 10ms 주기

void setup() {
  Serial.begin(115200);
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);
  pinMode(trigPin, OUTPUT); // ultrasonic
  pinMode(echoPin, INPUT);  // ultrasonic
  
  Wire.begin();   // nunchuk
  nunchuk_init(); // nunchuk
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousCANMillis >= CANInterval) {
    previousCANMillis = currentMillis;
    
  distance = getDistance();  // ultrasonic
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }
    Serial.println(); 
    //Serial.println(distance); 
  }

  //uint8_t highByte = (distance >> 8) & 0xFF; // 상위 바이트
  //uint8_t lowByte = distance & 0xFF;        // 하위 바이트
  uint8_t data = distance & 0xFF;
                      if (nunchuk_read()) {
                      int wiiValue = nunchuk_joystickY(); 
                      mappedValue = map(wiiValue, -126, 125, 0, 50);
                      mappedValue = constrain(mappedValue, 0, 50);

                      }
    
  canMsg1.can_id  = 0xF6;
  canMsg1.can_dlc = 2;
  canMsg1.data[0] = data;
  //mcp2515.sendMessage(&canMsg1);

                      //int highByte = (mappedValue >> 8) & 0xFF; 
                      int lowByte = mappedValue & 0xFF;
                      //canMsg2.can_id  = 0xF6;
                      //canMsg2.can_dlc = 2;
                      //canMsg1.data[1] = highByte;
                      canMsg1.data[1] = lowByte;
                      mcp2515.sendMessage(&canMsg1);
  }
  
  if(canMsg.can_id == 0x80){
  if(canMsg.data[0] == 0x0F){ // 초음파가 15이내가 아니고 wii 값이 30넘으면 모터 속도 15
      Motor(1,1,15);
      Motor(2,1,15);
  }
  else if(canMsg.data[0] == 0x00){
      Motor(1,0,0);
      Motor(2,0,0);
      }
   
  else if(canMsg.data[0] == 0x14){ //후진
      Motor(1,-1,15);
      Motor(2,-1,15);
    }
  else{
    Motor(1,0,0);
    Motor(2,0,0);
    
    }
  }
  Serial.println(mappedValue);
  //if(canMsg.can_id == 0x80){ //ID가 80이고 초음파가 15이내면 정지
    //if(canMsg.data[0] == 0x00){
      //Motor(1,0,0);
      //Motor(2,0,0);
      //}
  //}
}
//////////////////////////////////////////////////////////////////////////////////
float getDistance() {
  unsigned long pingTime; 
  float distance;        
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  
  
  pingTime = pulseIn(echoPin, HIGH, rangingTimeOut); 
  if (pingTime != 0) {  // ultrasonic 
    distance = pingTime * soundVelocity / 2 / 10000; 
    return distance;    // ultrasonic 
  }
  else                  
    return maxDistance; 
}
//////////////////////////////////////////////////////////////////////////////////

void motorPWM(int channel, int motor_speed){
  motor_speed = constrain(motor_speed, 0, 50);
  int motor_pwm = map(motor_speed, 0, 50, 0, 4095);
  if (motor_pwm == 4095){
    pwm_motor.setPWM(channel, 4096, 0);
  }
  else if (motor_pwm == 0){
    pwm_motor.setPWM(channel, 0, 4096);
  }
  else{
    pwm_motor.setPWM(channel, 0, motor_pwm);
    // pwm_motor.setPWM(channel, 0, 4095 - motor_pwm);
  }
}

// Control motor rotation.
void Motor(int Motor_ID, int dir, int Motor_speed){
  if(dir > 0){dir = 1;}
  else if (dir < 0) {dir = -1;}
  else {dir = 0;}

  if (Motor_ID == 1){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, Motor_speed);
      //Serial.println("Motor M1 1");
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M1_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      //Serial.println("Motor M1 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      //Serial.println("Motor M1 STOP");
      }
  }
  else if (Motor_ID == 2){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M2_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      //Serial.println("Motor M2 1");
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, Motor_speed);
      //Serial.println("Motor M2 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      //Serial.println("Motor M2 STOP");
      }
  }
}
