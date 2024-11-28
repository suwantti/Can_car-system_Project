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
//struct can_frame canMsg2;
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
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousCANMillis >= CANInterval) {
    previousCANMillis = currentMillis;
    
  distance = getDistance();  // ultrasonic
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

  }

  //uint8_t highByte = (distance >> 8) & 0xFF; // 상위 바이트
  //uint8_t lowByte = distance & 0xFF;        // 하위 바이트
  uint8_t data = distance & 0xFF; // ultrasonic data 
  canMsg1.can_id  = 0xF6;
  canMsg1.can_dlc = 2;
  canMsg1.data[0] = data;
  
                      if (nunchuk_read()) { // wii data
                      int wiiValue = nunchuk_joystickY(); 
                      mappedValue = map(wiiValue, -126, 125, 0, 50);
                      mappedValue = constrain(mappedValue, 0, 50);

                      }
                      int lowByte = mappedValue & 0xFF;
                      canMsg1.data[1] = lowByte;
                      mcp2515.sendMessage(&canMsg1);  // ultrasonic, wii data tx
  }
  
  if(canMsg.can_id == 0x80){
  if(canMsg.data[0] == 0x0F){ // 초음파가 15미만이 아니고 wii 값이 30초과이고 50이 아니면 모터 속도 15
      Motor(1,1,15);          // 50이 아닌 조건을 넣은 이유는 모터 가속도를 값을 추출하기위해 추가하였음
      Motor(2,1,15);
  }
  else if(canMsg.data[0] == 0x1E){  // 가속도를 위해 wii 값이 50이면 모터 속도 30 (50은 16진수로 1E임)
      Motor(1,1,30);
      Motor(2,1,30);
      }
      
  else if(canMsg.data[0] == 0x00){  // 초음파 값이 15미만이면 메시지 값으로 0을 받고 0이면 모터 정지
      Motor(1,0,0);
      Motor(2,0,0);
      }
   
  else if(canMsg.data[0] == 0x14){  // wii 값이 0이면 메시지 값으로 0x14로 보내고 모터 속도는 15(후진)
      Motor(1,-1,15);
      Motor(2,-1,15);
    }
  else{ // 초기 상태 값, 즉 아무 값이 들어오지 않은 상태 - > 모터 정지 상태임
    Motor(1,0,0);
    Motor(2,0,0);
    
    }
  }

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
