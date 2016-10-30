#include <Servo.h>
Servo myservo;
const int servo = 13;
const int echo = 23;
const int trig = 22;
const int potpin = A0;
unsigned long t;
double u,val,d0,d,r,x,uu;

const double a1=-6.5936;
const double a2=0.6703;
const double b=25.9728;
const double k1=0.0241;
const double k2=0.0114;
const double N=0.0241;
const double g=6.5936;

double x10=12.5;
double x20=0;
double u0=0;
double d00=12.5;
double x1,x2;
double lim=0.4;


void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(servo);
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
} 
 
void loop() 
{ 
  while((millis()-x)>=50)
  {
    x=millis();
    r = analogRead(potpin);
    r = map(r, 0, 1024, 5, 35);
 // r=12;  
    digitalWrite(trig,LOW);
    delayMicroseconds(10);
    digitalWrite(trig,HIGH);
    delayMicroseconds(15);
    t=pulseIn(echo,HIGH);
    d0=double(0.017*t);
    if((5<=d0)&&(d0<=35))
    d=d0;
    
    x1=d00;
    x2=a1*d00+a2*(x20+g*d00)+b*u0;
    u=r*N-x10*k1-(x20+g*d00)*k2;
    u0=u;
    x10=x1;
    x20=x2;
    d00=d;

//ERROR
if((x10>=(r-0.5))&&(x10<=(r+0.5)))
{u0=0;}

//if(x10<r)
//{u0=u0+0.01;}
//if(x10>r)
//{u0=u0-0.01;}
    
  //// Anti windup
u0 = constrain(u0,-lim,lim);
        
uu=1000*u0;
val = map(uu, -1000*lim, 1000*lim, 72, 106); // (73,106)
//val=87;
    myservo.write(val);  
    Serial.print(r);
    Serial.print("   ");
  //Serial.println(d); 
     Serial.print(d);
    Serial.print("   ");    
    Serial.print(u0);
    Serial.print("   ");
    Serial.println(val);
  }
}

