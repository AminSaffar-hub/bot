#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "Timer.h"

#define encoderBLA  A12   //int20
#define encoderBLB  A13    //int21
#define encoderFLA  A10   //int19
#define encoderFLB  A11   //int18
#define encoderFRA  A14   //int23
#define encoderFRB  A15   //int22
#define encoderBRA  A8   //pcint10
#define encoderBRB  A9   //pcint9


#define enableM1 6
#define enableM2 7
#define enableM3 9
#define enableM4 8

#define MLF1 10
#define MLF2 11
#define MRF1 13
#define MRF2 12

#define MLB1 A2
#define MLB2 A3
#define MRB1 A1
#define MRB2 A0

#define num_motors 4

float cmd[num_motors];
float* p;
int pwm_base[num_motors];
float Vc[num_motors];
float Vpos[num_motors];
int flag_cmd = 0;
float set_vel_zeros[num_motors]={0,0,0,0};

float R = 65;
float encoder_ticks = 390;
float precision = 2;
float distance_tick = (2 * PI*R) / (encoder_ticks*precision);


float distFR = 0;
int encoderFRPos = 0;
boolean encoderFRA_set = false;
boolean encoderFRB_set = false;

float distFL = 0;
int encoderFLPos = 0;
boolean encoderFLA_set = false;
boolean encoderFLB_set = false;

float distBL = 0;
int encoderBLPos = 0;;
boolean encoderBLA_set = false;
boolean encoderBLB_set = false;

float distBR = 0;
int encoderBRPos = 0;;
boolean encoderBRA_set = false;
boolean encoderBRB_set = false;

void set_pwm(float* pwm)
{
  analogWrite(enableM4,(int)abs(pwm[0]));
  if (pwm[0]>= 0) { digitalWrite(MLF1,HIGH);digitalWrite(MLF2,LOW);}
  else { digitalWrite(MLF1,LOW);digitalWrite(MLF2,HIGH);}
  analogWrite(enableM3,(int)abs(pwm[1]));
  if (pwm[1]>= 0) { digitalWrite(MRF1,HIGH);digitalWrite(MRF2,LOW);}
  else { digitalWrite(MRF1,LOW);digitalWrite(MRF2,HIGH);}
  analogWrite(enableM1,(int)abs(pwm[3]));
  if (pwm[3]>= 0) { digitalWrite(MLB1,HIGH);digitalWrite(MLB2,LOW);}
  else { digitalWrite(MLB1,LOW);digitalWrite(MLB2,HIGH);}
  analogWrite(enableM2,(int)abs(pwm[2]));//(int)pwm[1]);
  if (pwm[2]>= 0) { digitalWrite(MRB1,HIGH);digitalWrite(MRB2,LOW);}
  else { digitalWrite(MRB1,LOW);digitalWrite(MRB2,HIGH);}
//  delay(5);
}
void set_pwm_a(float* pwm)
{
  analogWrite(enableM4,(int)abs(pwm[0]));
  if (pwm[0]>= 0) { digitalWrite(MLF1,HIGH);digitalWrite(MLF2,LOW);}
  else { digitalWrite(MLF1,LOW);digitalWrite(MLF2,HIGH);}
  analogWrite(enableM1,(int)abs(pwm[1]));
  if (pwm[1]>= 0) { digitalWrite(MRB1,HIGH);digitalWrite(MRB2,LOW);}
  else  digitalWrite(MRB1,LOW);digitalWrite(MRB2,HIGH);
  analogWrite(enableM3,(int)abs(pwm[1]));
   if (pwm[1]>= 0) { digitalWrite(MRF1,HIGH);digitalWrite(MRF2,LOW);}
  else { digitalWrite(MRF1,LOW);digitalWrite(MRF2,HIGH);}
  analogWrite(enableM2,(int)abs(pwm[3]));//(int)pwm[1]);
  if (pwm[3]>= 0) { digitalWrite(MLB1,HIGH);digitalWrite(MLB2,LOW);}
  else { digitalWrite(MLB1,LOW);digitalWrite(MLB2,HIGH);}
//  delay(5);
}
void doencoderFRA() {
  encoderFRA_set = digitalRead(encoderFRA) == HIGH;
  encoderFRPos += (encoderFRA_set == encoderFRB_set) ? -1 : +1;
  distFR += (encoderFRA_set == encoderFRB_set) ? -distance_tick : +distance_tick;
}

void doencoderFRB() {
  encoderFRB_set = digitalRead(encoderFRB) == HIGH;
  encoderFRPos += (encoderFRA_set != encoderFRB_set) ? -1 : +1;
  distFR += (encoderFRA_set != encoderFRB_set) ? -distance_tick : +distance_tick;
}

void doencoderFLA() {
  encoderFLA_set = digitalRead(encoderFLA) == HIGH;
  encoderFLPos += (encoderFLA_set == encoderFLB_set) ? -1 : +1;
  distFL += (encoderFLA_set == encoderFLB_set) ? -distance_tick : +distance_tick;
}

void doencoderFLB() {
  encoderFLB_set = digitalRead(encoderFLB) == HIGH;
  encoderFLPos += (encoderFLA_set != encoderFLB_set) ? -1 : +1;
  distFL += (encoderFLA_set != encoderFLB_set) ? -distance_tick : +distance_tick;
}

void doencoderBRA() {
  encoderBRA_set = digitalRead(encoderBRA) == HIGH;
  encoderBRPos += (encoderBRA_set == encoderBRB_set) ? -1 : +1;
  distBR += (encoderBRA_set == encoderBRB_set) ? -distance_tick : +distance_tick;
}

void doencoderBRB() {
  encoderBRB_set = digitalRead(encoderBRB) == HIGH;
  encoderBRPos += (encoderBRA_set != encoderBRB_set) ? -1 : +1;
  distBR += (encoderBRA_set != encoderBRB_set) ? -distance_tick : +distance_tick;
}

void doencoderBLA() {
  encoderBLA_set = digitalRead(encoderBLA) == HIGH;
  encoderBLPos += (encoderBLA_set == encoderBLB_set) ? -1 : +1;
  distBL += (encoderBLA_set == encoderBLB_set) ? -distance_tick : +distance_tick;
}

void doencoderBLB() {
  encoderBLB_set = digitalRead(encoderBLB) == HIGH;
  encoderBLPos += (encoderBLA_set != encoderBLB_set) ? -1 : +1;
  distBL += (encoderBLA_set != encoderBLB_set) ? -distance_tick : +distance_tick;
}

void message_cmd (std_msgs::Float32MultiArray table ){  
  for (int i = 0 ;i<num_motors ;i++) Vc[i] = table.data[i]; 
  flag_cmd = 1; 
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> sub_cmd("pwm_cmd", &message_cmd );
std_msgs::Float32MultiArray vel_pose;
ros::Publisher chatter("vel_pose", &vel_pose);


void setup() {
  //Serial.begin (57600);
  
  nh.initNode();
  
  nh.subscribe(sub_cmd);
   vel_pose.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  vel_pose.layout.dim[0].label = "height";
  vel_pose.layout.dim[0].size = 4;
  vel_pose.layout.dim[0].stride = 1;
  vel_pose.layout.data_offset = 0;
  vel_pose.data = (float *)malloc(sizeof(float)*8);
  vel_pose.data_length = 4;
  nh.advertise(chatter);

  vel_pose.data[0] = 0.0;
  vel_pose.data[1] = 0.0;
  vel_pose.data[2] = 0.0;
  vel_pose.data[3] = 0.0;

  chatter.publish(&vel_pose);
  
  pinMode(encoderBLA, INPUT);
  pinMode(encoderBLB, INPUT);
  pinMode(encoderFRA, INPUT);
  pinMode(encoderFRB, INPUT);
  pinMode(encoderFLA, INPUT);
  pinMode(encoderFLB, INPUT);
  pinMode(encoderBRA, INPUT);
  pinMode(encoderBRB, INPUT);

  pinMode(MLF1,OUTPUT);pinMode(MLF2,OUTPUT);
  pinMode(MRF1,OUTPUT);pinMode(MRF2,OUTPUT);
  pinMode(MLB1,OUTPUT);pinMode(MLB2,OUTPUT);
  pinMode(MRB1,OUTPUT);pinMode(MRB2,OUTPUT);

  attachPinChangeInterrupt(20, doencoderBLA, CHANGE); //PINA12
  attachPinChangeInterrupt(21, doencoderBLB, CHANGE); //PINA13

  attachPinChangeInterrupt(23, doencoderFRA, CHANGE); //PIN21
  attachPinChangeInterrupt(22, doencoderFRB, CHANGE); //PIN20
  attachPinChangeInterrupt(16,doencoderBRA,CHANGE); //PINA8
  attachPinChangeInterrupt(17,doencoderBRB,CHANGE); //PINA9
  attachPinChangeInterrupt(19,doencoderFLA,CHANGE); //PINA11
  attachPinChangeInterrupt(18,doencoderFLB,CHANGE); //PINA10 

}
long lastTime = 0;
float diffFR = 0.0;
float diffFL = 0.0;
float diffBR = 0.0;
float diffBL = 0.0;

float lastFR = 0.0;
float lastFL = 0.0;
float lastBR = 0.0;
float lastBL = 0.0;

float set_vel_pwm[num_motors]={0.0,0.0,0.0,0.0};

void loop() {

  diffFR = ((encoderFRPos - lastFR) / (917*(millis()-lastTime)*0.001)) * 2 * PI  ;
  if (abs(diffFR) <  62.0) vel_pose.data[2] = diffFR;
  lastFR  = encoderFRPos;
  

  diffFL = ((encoderFLPos - lastFL) / (917*(millis()-lastTime)*0.001)) * 2 * PI  ;
  if (abs(diffFL) <  62.0) vel_pose.data[0] = diffFL;
  lastFL  = encoderFLPos;
  

  diffBR = ((encoderBRPos - lastBR) / (917*(millis()-lastTime)*0.001)) * 2 * PI  ;
  if (abs(diffBR) <  62.0) vel_pose.data[3] = diffBR;
  lastBR  = encoderBRPos;
  

  diffBL = ((encoderBLPos - lastBL) / (917*(millis()-lastTime)*0.001)) * 2 * PI  ;
  if (abs(diffBL) <  62.0) vel_pose.data[1] = diffBL;
  lastBL  = encoderBLPos;
  lastTime  = millis();
 
  
  chatter.publish(&vel_pose);
  
  Serial.print("diffFR = ");Serial.print (diffFR);
  Serial.print("     diffFL = ");Serial.print (diffFL);
  Serial.print("     diffBR = ");Serial.print (diffBR);
  Serial.print("     diffBL = ");Serial.println (diffBL);

  if (flag_cmd == 1) set_pwm(Vc);
  else set_pwm(set_vel_zeros);
  nh.spinOnce();
  
//set_pwm(set_vel_pwm);
  delay(10);
  //Do stuff here
}
