
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define KP 1
#define KI 1
#define KD 0

#define enableM1 5
#define enableM2 6
#define enableM3 10
#define enableM4 11

#define MLF1 4
#define MLF2 2
#define MRF1 8
#define MRF2 7

#define MLB1 A0
#define MLB2 A1
#define MRB1 A3
#define MRB2 A2

#define num_motors 4

float prev_error[num_motors];
float error[num_motors];
float sum_error[num_motors];
float diff_error[num_motors];
float cmd[num_motors];
float* p;
int pwm_base[num_motors];
float Vc[num_motors];
float Vpos[num_motors];
int flag_cmd = 0;


float* set_in_interval(float* uin)
{
 for (int flag = 0; flag < num_motors ; flag++)
 {
    pwm_base[flag] += uin[flag];
    
    if (pwm_base[flag] > 255) cmd[flag] = 255;
    else if (pwm_base[flag] < 20) cmd[flag] = 20;
    else cmd[flag] = pwm_base[flag];
 }
  return(cmd);
}

void set_pwm(float* pwm)
{
  analogWrite(enableM3,(int)abs(pwm[0]));
  if (pwm[0])> 0 { digitalWrite(MLF1,HIGH);digitalWrite(MLF2,LOW);}
  else { digitalWrite(MLF1,LOW);digitalWrite(MLF2,HIGH);}
  analogWrite(enableM4,(int)pwm[1]);
  if (pwm[1])> 0 { digitalWrite(MRF1,HIGH);digitalWrite(MRF2,LOW);}
  else { digitalWrite(MRF1,LOW);digitalWrite(MRF2,HIGH);}
  analogWrite(enableM1,(int)pwm[2]);
  if (pwm[2])> 0 { digitalWrite(MLB1,HIGH);digitalWrite(MLB2,LOW);}
  else { digitalWrite(MLB1,LOW);digitalWrite(MLB2,HIGH);}
  analogWrite(enableM2,(int)pwm[3]);//(int)pwm[1]);
  if (pwm[3])> 0 { digitalWrite(MRB1,HIGH);digitalWrite(MRB2,LOW);}
  else { digitalWrite(MRB1,LOW);digitalWrite(MRB2,HIGH);}
  delay(5);
}

void set_pwm_manuel()
{
  analogWrite(enableM3,150);//(int)pwm[0]);
  analogWrite(enableM4,150);//(int)pwm[1]);
  digitalWrite(MLF1,HIGH);
  digitalWrite(MLF2,LOW);
  digitalWrite(MRF1,HIGH);
  digitalWrite(MRF2,LOW);

  analogWrite(enableM1,150);//(int)pwm[0]);
  analogWrite(enableM2,200);//(int)pwm[1]);
  digitalWrite(MLB1,HIGH);
  digitalWrite(MLB2,LOW);
  digitalWrite(MRB1,HIGH);
  digitalWrite(MRB2,LOW);
  delay(5);
}

void velocity_control()
{

  for (int flag = 0; flag < num_motors ; flag++)
  {
   error[flag] = Vc[flag]-Vpos[flag];
   sum_error[flag] += error[flag];
   diff_error[flag] = error[flag] - prev_error[flag];
   prev_error[flag] = error[flag];

   cmd[flag] = KP * error[flag] + KI * sum_error[flag] + KD * diff_error[flag] ;
  }
   
 p = set_in_interval(cmd);
 set_pwm(p);
 
}

ros::NodeHandle nh;
std_msgs::Float32MultiArray ss;
ros::Publisher chatter("chatter", &ss);
float set_vel_zeros[num_motors]={0,0,0,0};

void message_cmd (std_msgs::Float32MultiArray table ){  
  for (int i = 0 ;i<num_motors ;i++) Vc[i] = table.data[i]; 
  flag_cmd = 1; 
}
void message_Vpos (std_msgs::Float32MultiArray table_Vpos ){  
  for (int i = 0 ;i<num_motors ;i++) Vpos[i] = table_Vpos.data[i]; 
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_cmd("vel_cmd", &message_cmd );
ros::Subscriber<std_msgs::Float32MultiArray> sub_Vpos("vel_pos", &message_Vpos );

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(MLF1,OUTPUT);
  pinMode(MLF2,OUTPUT);
  pinMode(MRF1,OUTPUT);
  pinMode(MRF2,OUTPUT);
  pinMode(MLB1,OUTPUT);
  pinMode(MLB2,OUTPUT);
  pinMode(MRB1,OUTPUT);
  pinMode(MRB2,OUTPUT);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_Vpos);
  nh.advertise(chatter);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  //set_pwm_manuel();
  if (flag_cmd == 1) set_pwm(Vc);
  else set_pwm(set_vel_zeros);
  //set_pwm(set_vel_zeros);
  //chatter.publish( &ss );
  delay(1);
}
