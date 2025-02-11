#include <ODriveUART.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>  // For atan2 and sqrt functions
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
double cmd = 0;
const double lowerlim = -0.3;//torque
const double upperlim = 0.3;//torque
int goal = 0;
double intx = 0;
bool flag = 1;
double dx;
String inputString;
// float currtime,prevtime;
float yaw = 0.0;             // Initialize yaw to 0
float elapsedTime, currentTime, previousTime;
SoftwareSerial odrive_serial(8, 9);
unsigned long baudrate = 9600;  // Must match what you configure on the ODrive (see docs for details)
int jump = 15;
ODriveUART odrive(odrive_serial);
void setup() {
  odrive_serial.begin(baudrate);
  odrive.setParameter("axis1.controller.input_torque ","0");
  delay(1000);

  Serial.begin(baudrate);
  
  while (!Serial);  // Wait for Serial Monitor to open
  Serial.println("ODrive communication setup complete.");
  delay(100);
  // previousTime=millis()/1000;
  //Try to initialize
  // Wire.begin();
  // Wire.setClock(400000);  // Set I2C speed to 400kHz in `setup()`

  // // Initialize MPU6050
  // Wire.beginTransmission(MPU);
  // Wire.write(0x6B);  // Power management register
  // Wire.write(0);     // Wake up the sensor
  // Wire.endTransmission(true);
}
float getAngle()
{
  float x = analogRead(A0);
//  Serial.println(x);
//   delay(1000);  // Read the analog input on A0
  return max(0.186*x-49.1,-50);  // return angle
}
void loop() {
  // Print angles
  float angleY = getAngle();
  Serial.print(" , Pitch:"); Serial.print((String)angleY);
  Serial.print(", maxangle:");Serial.print(45);
  Serial.print(", minangle:");Serial.println(-45);
  //  Serial.print(odrive.getParameterAsString("vbus_voltage"));
  // delay(10);  
   
  if(angleY>30)
  {
    Serial.print("hi");
    odrive.setParameter("axis1.controller.input_torque ","0.2");
    delay(1000);
    odrive.setParameter("axis1.controller.input_torque ","-0.3");
    delay(1000);
  }
  if(angleY<-30)
  { odrive.setParameter("axis1.controller.input_torque ","-0.2");
    delay(1000);
    odrive.setParameter("axis1.controller.input_torque ","0.175");
    delay(1000);
  } 
  else
   PID(angleY);
   getdata();
  
}
// z 0.02085 0.001 -0.00140
// double Kp = 0.0205;//0.01785;//0.0205;//0.66//0.0155//
// double Kd =0.00295;//-0.00540;//0.00295;//0.016//0.069//0.00295
// double Ki = 0.0003;//0.00011;//0.00;//0.0095 //0.0003
double Kp = 0.0205;//0.00699;//0.005 ;
double Kd = 0.00295;//0.000195;//0.000195 ;
double Ki = 0.0003;//48; // Disable for now
// best so far- kp=0.0205, kd=0.00295, ki=0.0003  
double prevError;
void PID(double anglex) {

  double error = anglex - goal;
  // currentTime=millis()/1000;
  dx = (error - prevError);
  intx += error;
  prevError=error;
  if(intx>0)intx=min(intx,5);
  else intx=max(-5,intx);
  // Serial.print(", error: ");
  // Serial.print(error);
  // Serial.print(",cmd : ");
  // Serial.print(cmd);
   cmd = (Kp * error + Kd * dx + Ki * intx)/2; 
  // double diffterm = Kp * error + Kd * dx + Ki * intx;
  // if(cmd*diffterm>0)cmd-=diffterm;
  // else cmd+=diffterm;

  if (cmd < lowerlim) cmd = lowerlim;
  else if (cmd > upperlim) cmd = upperlim;
  // Serial.print("addterm: ");
  // Serial.print((Kp * error + Ki * intx + Kd * dx));
  // Serial.print(", cmd: ");
  // Serial.print(cmd);
  // // delay(max(15.0,abs(anglex)));
  odrive.setParameter("axis1.controller.input_torque ", (String)cmd);
  // previousTime=currentTime;
  // odrive.setTorque(-cmd);
  // Serial.print(", current_torque: ");
  // Serial.print((odrive.getParameterAsFloat("axis1.controller.input_torque")));
  // delay(10);
// 
}


void getdata()
{
  if (Serial.available()) {
  // String input = Serial.readStringUntil('\n'); // Read input till newline
  // input.trim(); // Remove any trailing spaces or newlines
  // double newA, newB, newC;
    char s = Serial.read();
    if(s=='z')
    {
      Serial.print("hi");
      Serial.print(", cmd: ");
      Serial.print(cmd);
      Serial.print(", dx: ");
      Serial.print(dx);
      delay(200);
      odrive.clearErrors();
      delay(500);
      // Kp = (double)Serial.parseFloat()/100;
      // Ki = (double)Serial.parseFloat()/100;
      // Kd = (double)Serial.parseFloat()/100;
      // Serial.print("Updated values: Ki = ");
      // Serial.print(Kp);
      // Serial.print(", Kp = ");
      // Serial.print(Ki);
      // Serial.print(", Kd = ");
      // Serial.println(Kd);
      // delay(500);

    }
    // int count = sscanf(input.c_str(), "%lf %lf %lf", &newA, &newB, &newC);
    // if (count == 3) {
  } 
}

