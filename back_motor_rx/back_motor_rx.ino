#include <SPI.h> //back
#include <mcp2515.h>

//----------------------------------------------------------------------------------
#include <Wire.h> // 모터 테스트
#include <Adafruit_PWMServoDriver.h>

#define PIN_MOTOR_M1_IN1 6      //Define the positive pole of M1 
#define PIN_MOTOR_M1_IN2 7      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 8      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 9      //Define the negative pole of M2
Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);
//----------------------------------------------------------------------------------

struct can_frame canMsg;
struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(4);

void setup() {
  Serial.begin(115200);
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop(){
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
}

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
