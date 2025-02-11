// Define system parameters
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <ODriveUART.h>
#include <SoftwareSerial.h>

SoftwareSerial odrive_serial(8, 9);
unsigned long baudrate = 9600; // Must match what you configure on the ODrive (see docs for details)
ODriveUART odrive(odrive_serial);
const float K[]={0.60733038, 0.27125464}; // LQR gain matrix (pre-computed){0.63608292, 0.06346459}
float tiltangle,prevangle;
float cmd;

// State variables
  // Tilt angle of the plate (radians)
  // Angular velocity of the plate (rad/s)

// Control variable
float torque = 0.0;
const double upperlim = 0.3;
const double lowerlim =-0.3;
// Timing variables
unsigned long last_time = 0;
const float dt = 1;   // Time step (10 ms)

//Adafruit_MPU6050 mpu;
unsigned long prevTime = 0;
float getAngle()
{
  float x = analogRead(A0);  // Read the analog input on A0
  return -0.181*x+45.87;  // return angle
}
// 

// LQR control algorithm
float computeTorque(float theta, float theta_dot) {
  // State vector
  float x[] = {theta, theta_dot};

  // Compute control input u = -Kx
  float u = 0.0;
  for (int i = 0; i < 3; i++) {
    u -= K[i] * x[i];
  }
  
  return u;
}

// Main setup function
void setup() 
{
  // Initialize Serial Monitor

  Serial.begin(9600); // Serial to PC
  
  while (!Serial)
    delay(10); // Pause until serial console opens
    odrive_serial.begin(baudrate);
  odrive.setParameter("axis1.controller.config.control_mode","1");
  odrive.setParameter("axis1.controller.input_torque ","0");

  delay(1000);

 // Serial.println("Waiting for ODrive...");
  //while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    //delay(100);

 // Serial.println("found ODrive");
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  odrive.setParameter("axis1.controller.config.control_mode ","1");
  Serial.println("Enabling closed loop control...");  

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize
  

  // Initialize motor driver pin

  // Initialize IMU
 // initIMU();
 prevTime = millis();
  prevangle=getAngle();
}



// Main loop function
void loop() 
{
  // Get the current time
  unsigned long current_time = millis();

  
  // Calculate yaw using gyroscope data
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Time in seconds
  prevTime = currentTime;

  Serial.println("");
  tiltangle = getAngle();
  


  
    float tiltvel = (tiltangle - prevangle)*M_PI/(180*deltaTime) ;
    prevangle=tiltangle;
     // Compute control torque
    torque = computeTorque(tiltangle*M_PI/180, tiltvel);
    cmd=torque;
    if (torque<-0.3)torque=-0.3;
    else if (torque>0.3)torque=0.3;
    // Control the flywheel
    ODriveFeedback feedback = odrive.getFeedback();
    // Serial.println(feedback.pos);
    // Serial.print("vel:");
        // Serial.println(tiltvel);

   
      if(tiltangle>30)
      {
       Serial.print("hi");
       odrive.setParameter("axis1.controller.input_torque ","-0.15");
       delay(1000);
       odrive.setParameter("axis1.controller.input_torque ","0.15");
       delay(1000);
      }
     if(tiltangle<-30)
     { odrive.setParameter("axis1.controller.input_torque ","0.15");
       delay(1000);
       odrive.setParameter("axis1.controller.input_torque ","-0.15");
       delay(1000);
     } 
      
      else
       {
        odrive.setParameter("axis1.controller.input_torque ",(String)torque);
        
       }
    

    // if (Serial.available() > 0) {  // Check if data is available to read
    //     char input = Serial.read();  // Read the incoming character
    //     if (input == 'q') {  // Check if the input character is 'q'
    //       odrive.setParameter("axis1.controller.input_torque ","0");
    //       while(1){
    //             if (Serial.available() > 0) { 
    //             input = Serial.read();  
    //             if (input == 'q') break;
            // }  }  } 
    // Debugging: Print state variables
    // Serial.print("Theta: ");
    // Serial.print(tiltangle);
    // Serial.print(" | Theta_dot: ");
    // Serial.print(tiltvel);
    // //Serial.print(" | Omega_f: ");
    // Serial.print(" | Torque: ");
     Serial.println(torque);
    // Serial.println(RPS);
    // // delay(500);
}