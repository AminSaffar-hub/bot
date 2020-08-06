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






void set_pwm()
{ float pwm = 255.0;
  /*analogWrite(enableM3,(int)abs(pwm));
  if (pwm > 0) { digitalWrite(MRF1,HIGH);digitalWrite(MRF2,LOW);}
  else { digitalWrite(MRF1,LOW);digitalWrite(MRF2,HIGH);}
*/
  analogWrite(enableM1,(int)abs(pwm));
  if (pwm> 0) { digitalWrite(MRB1,HIGH);digitalWrite(MRB2,LOW);}
  else { digitalWrite(MRB1,LOW);digitalWrite(MRB2,HIGH);}
  /*analogWrite(enableM4,(int)abs(pwm[1]));
  if (pwm[1]> 0) { digitalWrite(MRF1,HIGH);digitalWrite(MRF2,LOW);}
  else { digitalWrite(MRF1,LOW);digitalWrite(MRF2,HIGH);}
  analogWrite(enableM1,(int)abs(pwm[2]));
  if (pwm[2]> 0) { digitalWrite(MLB1,HIGH);digitalWrite(MLB2,LOW);}
  else { digitalWrite(MLB1,LOW);digitalWrite(MLB2,HIGH);}
  analogWrite(enableM2,(int)abs(pwm[3]));//(int)pwm[1]);
  if (pwm[3]> 0) { digitalWrite(MRB1,HIGH);digitalWrite(MRB2,LOW);}
  else { digitalWrite(MRB1,LOW);digitalWrite(MRB2,HIGH);}*/
  delay(5);
}


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
 
}

void loop() {

  set_pwm();
  delay(10);
}
