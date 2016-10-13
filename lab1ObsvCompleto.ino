#include <Servo.h>
Servo myservo;
const int servo = 9;
const int echo = 22;
const int trig = 23;
const int potpin = A0;
//const int clk = 31;
unsigned long t;
double u,val,d0,d,r,x;

const double a12=0.05;
const double b1=0.7776;
const double b2=31.1;
const double k1=0.0131;
const double k2=0.0062;
const double N=0.0131;
const double g1=0.7909;
const double g2=4.8942;
const double tol=1;          //distancia de tolerancia

double x10=12.5;
double x20=0;
double u0=0;
double d00=12.5;
double x1,x2;
const double lim=0.23;
double mas=98;
double men=62;

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(servo,520,2400);
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
} 
 
void loop() 
{ 
  while((millis()-x)>=50)
  {
    x=millis();
    r = analogRead(potpin);
    r = map(r, 0, 1024, 4, 36);
    
    digitalWrite(trig,LOW);
    delayMicroseconds(10);
    digitalWrite(trig,HIGH);
    delayMicroseconds(15);
    t=pulseIn(echo,HIGH);
    d0=double(0.017*t);
    if((4<=d0)&&(d0<=36))
    d=d0;
    
    x1=x10+a12*x20+g1*d00+b1*u0;
    x2=x20+g2*d00+b2*u0;
    u=r*N-x10*k1-x20*k2;
    u0=u;
    x10=x1;
    x20=x2;
    d00=d-x10;
   
   // Anti windup
    if(u>lim)
    u0=lim;
    if(u<-lim)
    u0=-lim;
    val=1000*u0;
    val = map(val, -850*lim, 780*lim, 62, 88);
//  val=80;
    myservo.write(val);
 //   Serial.print(r);
 //   Serial.print("   ");
//    Serial.print(d);
      Serial.println(d);
//    Serial.print("   ");
//    Serial.print(u0);
//    Serial.print("   ");
//    Serial.println(val);
  }
}



