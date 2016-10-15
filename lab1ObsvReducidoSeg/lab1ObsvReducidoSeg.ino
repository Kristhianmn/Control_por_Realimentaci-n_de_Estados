#include <Servo.h>
Servo myservo;
const int servo = 9;
const int echo = 22;
const int trig = 23;
const int potpin = A0;
unsigned long t;
double u,val,d0,d,r,x;

const double a1=-6.5936;
const double a2=0.6703;
const double b=25.9728;
const double k1=0.0131;
const double k2=0.0062;
//const double k1=0.117;
//const double k2=0.0338;
//const double N=1;
const double N=0.0131;
const double g=6.5936;

double x10=12.5;
double x20=0;
double u0=0;
double d00=12.5;
double x1,x2;
double lim=0.23;
double e;
double e0;
double tol=3;
double tol0=1;
double ki=0.0127;
double asup=88;
double ainf=62;

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(servo,520,2500);
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
} 
 
void loop() 
{ 
  while((millis()-x)>=75)
  {
    x=millis();
    r = analogRead(potpin);
    r = map(r, 0, 1024, 4, 30);
    
    digitalWrite(trig,LOW);
    delayMicroseconds(10);
    digitalWrite(trig,HIGH);
    delayMicroseconds(15);
    t=pulseIn(echo,HIGH);
    d0=double(0.017*t);
    if((4<=d0)&&(d0<=30))
    d=d0;
    
    x1=d00;
    x2=a1*d00+a2*(x20+g*d00)+b*u0;
    u=r*N-x10*k1-(x20+g*d00)*k2;
    u0=u;
    x10=x1;
    x20=x2;
    d00=d;
    
  //// Anti windup
    if(u>lim)
    u0=lim;
    if(u<-lim)
    u0=-lim;

val=1000*u0;
val = map(val, -1000*lim, 1250*lim, 62, 88);   
//val=((asup-ainf)/(2*lim))*(u0+lim)+ainf;

//if((d>=(r-tol0)&&(d<=(r+tol0))))
//{val=75;}
//if((d>=(r-tol))&&(d<r))
//{val=77;}
//if((d<=(r+tol))&&(d>r))
//{val=73;}

    //81 grados es el medio
 //val=75;
    myservo.write(val);  
    Serial.print(r);
    Serial.print("   ");
 // Serial.print(d); 
     Serial.println(d);
   // Serial.print("   ");
   // Serial.print(u0);
   // Serial.print("   ");
   // Serial.println(val);
  }
}
