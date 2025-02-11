#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> // For atan2 and sqrt functions
#include <HardwareSerial.h>
#include<SoftwareSerial.h>
#include<ODriveUART.h>
SoftwareSerial odrive_serial(6,9);
ODriveUART odrive(odrive_serial);
unsigned long prevTime = 0;
long  baudrate=9600;
double prevAngle=0;
double omegab =0;
double prevomegab =0.0;
double delta_omegab=0;
float temp2=0;
int min_angle=-45;
int max_angle=45;

//The block below consists of parameters of the squareli.
double Ib=0.035;
double Iw=0.00018;
double mb=0.269;
double mw=0.081;
double rw=0.06;
double r=0.075*sqrt(2);
double L=0.15;

double wfw=0;
double wfb=0;
double wiw=0;
double wib=0;


//Calibrate below code
double biasj=11.0,biass=0;
double delj =125, dels=50;

void four_cases();

void setup(void) 
{
  odrive_serial.begin(baudrate);
  odrive.setParameter("axis1.controller.input_vel ","0");
  Serial.begin(115200);
  odrive.setParameter("axis1.controller.config.control_mode","CONTROL_MODE_VELOCITY_CONTROL");
  while (!Serial)
  delay(10); // Pause until serial console opens
}
void loop() 
{
  temp2=max(-0.181*analogRead(A0)+45.87,-50);
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Time in seconds
  prevTime = currentTime;
  prevomegab=omegab;
  omegab=((temp2-prevAngle)*M_PI/180)/deltaTime;
  prevAngle=temp2;

  if(omegab>=25)
    omegab=25;
  else if(omegab<=-25)
    omegab=-25;
  prevAngle=temp2;
  Serial.print("temp2: "); Serial.print(temp2); Serial.print(",omegab: "); Serial.print(omegab); // Serial.print(",min_angle:"); Serial.print(min_angle); Serial.print(",max_angle:"); Serial.print(max_angle);
  //Serial.println();
  double deltaomegab= omegab-prevomegab;

  if(abs(deltaomegab)>3.6)
  {
    omegab=prevomegab;
    four_cases();
    Serial.print("you have a bad reading for omgegab enjoy ahahahahhaahahahahaa");
  }
  else four_cases();
  Serial.println(); Serial.println();
}

void four_cases()
{
   if(temp2<-0.5 && temp2>-40.5 && omegab< 0) //right side falling or body rotates to left
    {
     Serial.println(".                                              rightside falling");
     wiw=odrive.getParameterAsFloat("axis1.controller.input_vel");
     wfw=-(sqrt(pow(wiw*2*3.1415,2)+((Ib/Iw)*pow(omegab,2)))/(2*3.1415)+biass);
     Serial.print(wfw);
     odrive.setParameter("axis1.controller.input_vel ",(String)wfw);
     delay(dels);
    // for(;wiw<=wfw;)
     //{
      //double w=wiw+0.01;
      // odrive.setParameter("axis1.controller.input_vel ",(String)w );
      // delay(200);
     //}
     // restab mecha
    // for(;wiw<=wfw;)
    // {
     // double w=wiw+0.01;
      //wiw=wiw+0.01;
      // odrive.setParameter("axis1.controller.input_vel ",(String)w );
      // delay(200);
    //restab mecha
   // }
    }
    
    else if(temp2>0.5 && temp2<40.5 && omegab>0)  //left side falling or body rotates to right 
    {
      Serial.println("                   left side falling");
      wiw=-odrive.getParameterAsFloat("axis1.controller.input_vel");
      wfw=sqrt(pow(wiw,2)+((Ib/Iw)*pow(omegab,2)))/(2*3.1415);
      Serial.print(wfw);
      odrive.setParameter("axis1.controller.input_vel ",(String)wfw);
      delay(dels);
   // for(;wiw>=wfw;)
    // {
    //  double w=wiw-0.01;
    //  Serial.println("hiloop");
     // wiw=wiw-0.01;
     //  odrive.setParameter("axis1.controller.input_vel ",(String)w );
     //  delay(200);
    //restab mecha
    }
    else if(temp2<-40.5)           //rightside varun jump
    {
      Serial.println("hi0");
      wfw=(sqrt(Ib*mb*9.81*L*(sqrt(2)-1))/(Iw+mw*pow(r,2)))/(2*3.1415)+biasj; //no 2pi
      wiw=-(wfw);
      odrive.setParameter("axis1.controller.input_vel ",(String)wfw);
 // Serial.println(wfw);
      delay(delj);
      odrive.setParameter("axis1.controller.input_vel",(String)wiw);
      delay(delj);                   //JUMP mecha
    }
    else if(temp2>40.5)             //lefttside varun jump
    {                               //we add 2nd condition so that centri acc causing temp2 to go to 90 dont cause it to enter this if case
      Serial.println("hi0");
      wfw=-((sqrt(Ib*mb*9.81*L*(sqrt(2)-1))/(Iw+mw*pow(r,2)))/(2*3.1415926)+biasj); //no 2pi
      wiw=-(wfw);
      odrive.setParameter("axis1.controller.input_vel ",(String)wfw);
 // Serial.println(wfw);
      delay(delj);
      odrive.setParameter("axis1.controller.input_vel",(String)wiw);
      delay(delj);                  //JUMP mecha
    }
}