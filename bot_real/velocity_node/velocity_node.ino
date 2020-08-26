
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define enableM1 5
#define enableM2 6
#define enableM3 10
#define enableM4 11

#define MLF1 8
#define MLF2 7
#define MRF1 4
#define MRF2 2

#define MLB1 A0
#define MLB2 A1
#define MRB1 A3
#define MRB2 A2

#define num_motors 4


float cmd[num_motors];
float* p;
int pwm_base[num_motors];
float Vc[num_motors];
float Vpos[num_motors];
int flag_cmd = 0;


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
  delay(5);
}

ros::NodeHandle nh;
float set_vel_zeros[num_motors]={0,0,0,0};

void message_cmd (std_msgs::Float32MultiArray table ){  
  for (int i = 0 ;i<num_motors ;i++) Vc[i] = table.data[i]; 
  flag_cmd = 1; 
}
void message_Vpos (std_msgs::Float32MultiArray table_Vpos ){  
  for (int i = 0 ;i<num_motors ;i++) Vpos[i] = table_Vpos.data[i]; 
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_cmd("pwm_cmd", &message_cmd );

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(MLF1,OUTPUT);pinMode(MLF2,OUTPUT);
  pinMode(MRF1,OUTPUT);pinMode(MRF2,OUTPUT);
  pinMode(MLB1,OUTPUT);pinMode(MLB2,OUTPUT);
  pinMode(MRB1,OUTPUT);pinMode(MRB2,OUTPUT);
  nh.initNode();
  nh.subscribe(sub_cmd);

}


void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
if (flag_cmd == 1) set_pwm(Vc);
else set_pwm(set_vel_zeros);
  
  delay(1);
}
