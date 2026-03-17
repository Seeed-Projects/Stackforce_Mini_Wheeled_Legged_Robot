#include <Arduino.h>
#include "SF_Servo.h"
#include "sbus.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"
#include "bipedal_data.h"

#define ROLL_OFFSET 0

SF_Servo servos = SF_Servo(Wire); //实例化舵机
bfs::SbusRx sbusRx(&Serial1);//实例化接收机

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

void getRCValue();
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear);
void inverseKinematics();

std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;
IKparam IKParam;
float height;
uint8_t lowest = 70;
uint8_t highest = 130;
float X,Y;
float Y_demand;
float Kp_Y=0.1; //横滚控制kp
float Kp_roll = 0.05;
float Phi;
float turn, forward;
float L = 100;//体长
float rollLimit=20;
PIDController PID_VEL{0.2,0,0,1000,50};

int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

float kp1 = 0.38;//该P值需要精调节
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;
float Kp_x = 1.1;

float pitch;
SF_BLDC_DATA  BLDCData;
SF_IMU mpu6050 = SF_IMU(Wire);
SF_BLDC motors = SF_BLDC(Serial2);
robotposeparam robotPose;
robotmotionparam robotMotion;
int M0Dir,M1Dir;
float targetSpeed;
float stab_roll = 0;

void getMPUValue(){
  mpu6050.update();
  //tockn
  robotPose.pitch = -mpu6050.angle[0];// 摆放原因导致调换
  robotPose.roll = mpu6050.angle[1];// 摆放原因导致调换
  robotPose.yaw = mpu6050.angle[2];
  robotPose.GyroX = mpu6050.gyro[1]; 
  robotPose.GyroY = -mpu6050.gyro[0];
  robotPose.GyroZ = -mpu6050.gyro[2];
}



void setup() {
  Serial.begin(921600);
  Wire.begin(1,2,400000UL);
  mpu6050.init();
  servos.init();
  servos.setAngleRange(0,300);
  servos.setPluseRange(500,2500);
  sbusRx.Begin(SBUSPIN,-1);
  motors.init();
  motors.setModes(4,4);
  M0Dir = -1;
  M1Dir = -1;
}

uint8_t loopCnt;
float turnTorque,turnTarget;
float turnKp;

void loop() {
  BLDCData = motors.getBLDCData();
  getMPUValue();
  getRCValue();


  robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX, -5, 5);
  targetSpeed = map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX, -20, 20);
  Y_demand = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowest, highest));//腿高期望
  Phi = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*rollLimit, rollLimit);//滚转角期望

  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2;
  float targetAngle = PID_VEL(targetSpeed - speedAvg);
  float turnTorque = turnKp * (robotMotion.turn-robotPose.GyroZ);
  float torque1 = kp1*(targetAngle - robotPose.pitch) + turnTorque;
  float torque2 = kp1*(targetAngle - robotPose.pitch) - turnTorque;

  motors.setTargets(M0Dir*torque1, M1Dir*torque2);

  loopCnt++;
  if(loopCnt>=100){
    Serial.printf("status: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",robotPose.pitch,robotPose.GyroZ,targetAngle,turnTorque,torque1,torque2);
    loopCnt=0;
  }
  
  X = -Kp_x * (targetSpeed - speedAvg); // 根据速度调整X坐标
  Y = Y + Kp_Y * (Y_demand - Y); // 横滚PID

  uint16_t Remoter_Input = Y;
  float E_H = (L/2) * sin(Phi*(PI/180));
  stab_roll = stab_roll + Kp_roll * (0 - robotPose.roll); // 复杂地形适应控制PID
  float L_Height = Remoter_Input + stab_roll;
  float R_Height = Remoter_Input - stab_roll;

  IKParam.XLeft = X;
  IKParam.XRight = X;
  IKParam.YLeft = L_Height;
  IKParam.YRight = R_Height;
  // Serial.printf("%f,%f,%f,%f,%f\n",X,Phi,E_H,L_Height,R_Height);

  inverseKinematics();
}


// 读取遥控器
void getRCValue(){
  if(sbusRx.Read()){
    sbusData = sbusRx.ch();
    RCValue[0] = sbusData[0];
    RCValue[1] = sbusData[1];
    RCValue[2] = sbusData[2];
    RCValue[3] = sbusData[3];
    RCValue[4] = sbusData[4];
    RCValue[5] = sbusData[5];

    RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
    RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
  }
}

void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
  servos.setAngle(3, servoLeftFront);//左前
  servos.setAngle(4, servoLeftRear);//左后
  servos.setAngle(5, servoRightFront);//右前
  servos.setAngle(6, servoRightRear);//右后
}



void inverseKinematics(){
  float alpha1,alpha2,beta1,beta2;
  uint16_t servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;

  float aLeft = 2 * IKParam.XLeft * L1;
  float bLeft = 2 * IKParam.YLeft * L1;
  float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
  float dLeft = 2 * L4 * (IKParam.XLeft - L5);
  float eLeft = 2 * L4 * IKParam.YLeft;
  float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;
  else IKParam.alphaLeft = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaLeft = beta1;
  else IKParam.betaLeft = beta2;
  
  float aRight = 2 * IKParam.XRight * L1;
  float bRight = 2 * IKParam.YRight * L1;
  float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (IKParam.XRight - L5);
  float eRight = 2 * L4 * IKParam.YRight;
  float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaRight = alpha1;
  else IKParam.alphaRight = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaRight = beta1;
  else IKParam.betaRight = beta2;

  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);//弧度转角度
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear = 90 + alphaLeftToAngle;
  servoRightFront = 270 - betaRightToAngle;
  servoRightRear = 270 - alphaRightToAngle;

  setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
}







