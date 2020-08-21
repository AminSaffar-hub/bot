#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include "Timer.h"

#define encoderBLA  A12   //int20
#define encoderBLB  A13    //int21
#define encoderFRA  A11   //int19
#define encoderFRB  A10   //int18
#define encoderFLA  A15   //int23
#define encoderFLB  A14   //int22
#define encoderBRA  A8   //pcint10
#define encoderBRB  A9   //pcint9


#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

float R = 65;
float encoder_ticks = 390;
float precision = 2;////
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
#define num_motors 4
float cmd[num_motors];
float* p;
int pwm_base[num_motors];
float Vc[num_motors];
float Vpos[num_motors];
int flag_cmd = 0;

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
ros::Publisher chatter("chatter", &vel_pose);
void setup() {
  Serial.begin (57600);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(chatter);
  
  pinMode(encoderBLA, INPUT);
  pinMode(encoderBLB, INPUT);
  pinMode(encoderFRA, INPUT);
  pinMode(encoderFRB, INPUT);
  pinMode(encoderFLA, INPUT);
  pinMode(encoderFLB, INPUT);
  pinMode(encoderBRA, INPUT);
  pinMode(encoderBRB, INPUT);

  attachPinChangeInterrupt(20, doencoderBLA, CHANGE); //PINA12
  attachPinChangeInterrupt(21, doencoderBLB, CHANGE); //PINA13

  attachPinChangeInterrupt(23, doencoderFLA, CHANGE); //PIN21
  attachPinChangeInterrupt(22, doencoderFLB, CHANGE); //PIN20
  attachPinChangeInterrupt(16,doencoderBRA,CHANGE); //PINA8
  attachPinChangeInterrupt(17,doencoderBRB,CHANGE); //PINA9
  attachPinChangeInterrupt(19,doencoderFRA,CHANGE); //PINA11
  attachPinChangeInterrupt(18,doencoderFRB,CHANGE); //PINA10 

}
long lastTime = 0;
float diffFR;
float diffFL;
float diffBR;
float diffBL;

float lastFR = 0.0;
float lastFL = 0.0;
float lastBR = 0.0;
float lastBL = 0.0;
void loop() {
   //t.update();

   
  diffFR = ((encoderFRPos - lastFR) / (917*(millis()-lastTime)*0.001)) ;//* 2 * PI  ;
  lastFR  = encoderFRPos;
  vel_pose.data[0] = diffFR;

  diffFL = ((encoderFLPos - lastFL) / (917*(millis()-lastTime)*0.001)) ;//* 2 * PI  ;
  lastFL  = encoderFLPos;
  vel_pose.data[1] = diffFL;

  diffBR = ((encoderBRPos - lastBR) / (917*(millis()-lastTime)*0.001)) ;//* 2 * PI  ;
  lastBR  = encoderBRPos;
  vel_pose.data[2] = diffBR;

  diffBL = ((encoderBLPos - lastBL) / (917*(millis()-lastTime)*0.001)) ;//* 2 * PI  ;
  lastBL  = encoderBLPos;
  lastTime  = millis();
  vel_pose.data[3] = diffBL;
  chatter.publish(&vel_pose);
  Serial.print("diffFR = ");Serial.print (diffFR);
  Serial.print("     diffFL = ");Serial.print (diffFL);
  Serial.print("     diffBR = ");Serial.print (diffBR);
  Serial.print("     diffBL = ");Serial.println (diffBL);
  nh.spinOnce();
  delay(1);
  //Do stuff here
}
