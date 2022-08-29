/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/
#include <Arduino.h>
#include <Wire.h>

//#include <BMx280I2C.h>
#include <PID_v2.h>
#include "BMP280_TempPres.h"


#define PIN_INPUT 0
#define RELAY_PIN 6


//Define Variables we'll be connecting to
float setpoint;
#include "command_buffer.h"
CCommandBuffer commandBuffer;
#include "commandProcessing.h"

//Specify the initial tuning parameters
double Kp=20, Ki=.01, Kd=0;
PID_v2 myPID( Kp, Ki, Kd, PID::Direct);
int WindowSize = 250;
unsigned long windowStartTime;

float plow =0, phigh = .60, lowo = 0, higho = 1080;

float pmap(float p)
{// Convert real world values (psi) to control variables
float remap = p*(higho-lowo)/(phigh-plow);
return remap;
}
float windowStartTime2 =0;
#define ArraySize 22
//
//  TAKE THE CAP OFF THE BOTTLE WHEN STARTING UP TO CALIBRATE THE PRESSURE SENSOR TO ZERO
//
float Duration[ArraySize] = {180,180,60,60,60,45,45,45,45,45,45,45,45,45,45,45,90,90,90,90,90,60};  // ramp duration in minutes
float setPoint[ArraySize] = {.05,.1,.2,.3,.35,.36,.37,.38,.39,.4,.41,.42,.43,.44,.45,.46,.465,.47,.45,.46,.43,0};   // pressures in psi
float Times[ArraySize];
int ip = 2;      // to pick up in the middle of the duration array, set ip to the starying index

float startPoint = 0.;
double setpt;

bool firstTime = true;
float setPointFunc()
{// Ramp pressure up slowly
  if (ip == ArraySize-1)  return pmap(setPoint[ArraySize-1]);

  if (firstTime == true)
  {
      float windowStartTime2 = (float)((millis()-windowStartTime2)/1000ul)/60.;
      firstTime = false;
  }
  float timeMinute = (float)((millis()-windowStartTime2)/1000ul)/60.;

  float setptm = setPoint[ip];
  float timeSwitch=0;
  if (timeMinute > Times[ip] )
  {

    timeSwitch = timeMinute;
    ip++;
  }
  else if(ip > 0 && timeMinute < Times[ip-1])  // Happens when restarting with ip > 0
  {
     timeMinute = Times[ip-1];
  }
  if( ip == ArraySize-1)
  {
    setptm = setPoint[ArraySize-1];
  }
  else if (ip == 0)
  {
     setptm = setPoint[0];
  }
  else
  {
    setptm = setPoint[ip-1] + (setPoint[ip] - setPoint[ip-1]) * (1.-(Times[ip]-timeMinute)/(Times[ip]-Times[ip-1]));
  }
  
    return setptm;
}


void setup()
{
  Serial.begin(9600);
  //Serial.println(" Setup start ");
  presSetup();
  float press = pressure();  // Read the pressure
  Serial.println(" Starting - Calibrating Open Valves ");
  delay(15000);
  
  pinMode(RELAY_PIN, OUTPUT);
   pinMode(LED_BUILTIN, OUTPUT);

  // set up times to switch
  Times[0] = Duration[ip];  
  for( int i=0;  i<ArraySize-1+ip; ++i)
  {  
    Times[i+1] = Times[i] + Duration[i+1+ip];

  }
  
  setpt = setPointFunc();


  //tell the PID to range between 0 and the full window size
  //Serial.println(" PID start ");
  myPID.SetOutputLimits(0, WindowSize);
  myPID.Start(0,            // current input
              0,            // current output
              setpt);       // setpoint

  windowStartTime = millis();

}

unsigned long previousMillis = 0;
unsigned long interval = 2000;
void loop()
{
 
  
  float press = pressure();  // Read the pressure

  float input = pmap(press);   // Convert the reading
  setpoint = setPointFunc();
  float stpt = pmap(setpoint);
  float output = myPID.Run(input);     // call the controller function
  myPID.Start(input,           // current input
              output,          // current output
              stpt); // setpoint
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    Serial.print("Setpoint ");
    Serial.print(setpoint );
    Serial.print(",");
    Serial.print(" Pump ");
    Serial.print(output/300. );
    Serial.print(",");
    Serial.print(" Pressure ");
    Serial.print(press);
    Serial.print(",");
    Serial.print(" Time ");
    Serial.print(currentMillis/(1000*3600));
    Serial.print(",");
    Serial.print(" Index ");
    Serial.println(ip);
  }
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  while (millis() - windowStartTime > WindowSize) {
    // time to shift the Relay Window

    windowStartTime += WindowSize;
  }
  float diff = (float) (millis() - windowStartTime);
  

  if (output > diff)
    {digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);}
  else
    {digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);}
}
