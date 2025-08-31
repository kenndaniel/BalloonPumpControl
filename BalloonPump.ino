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
#include <PID_v2.h>

#define RELAY_PIN 6
#define BALLOON_PRES 7
#define ATMOS_PRES 8

#include "BMP280_TempPres.h"
using namespace std;

// Gyro variables
const int MPU_addr = 0x68;
int16_t axis_X, axis_Y, axis_Z;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

// Define Variables we'll be connecting to
float setpoint;

// Specify the initial tuning parameters
double Kp = 20, Ki = .0, Kd = 0;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
int WindowSize = 250;
unsigned long windowStartTime;

float readAngle()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  axis_X = Wire.read() << 8 | Wire.read();
  axis_Y = Wire.read() << 8 | Wire.read();
  axis_Z = Wire.read() << 8 | Wire.read();
  int xAng = map(axis_X, minVal, maxVal, -90, 90);
  int yAng = map(axis_Y, minVal, maxVal, -90, 90);
  int zAng = map(axis_Z, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  return x;
}

float setPointAngle = 183.3; // smaller numbers make a larger balloon
float printAngle()
{
  // return percentage of setpoint
  return (100. * readAngle() / setPointAngle);
}

float plow = 0, phigh = .40, lowo = 0, higho = 1080;

float pmap(float p)
{ // Convert real world values (psi) to control variables
  float remap = p * (higho - lowo) / (phigh - plow);
  return remap;
}

float windowStartTime2 = 0;
#define ArraySize 18
//
//  TAKE THE CAP OFF THE BOTTLE WHEN STARTING UP TO CALIBRATE THE PRESSURE SENSOR TO ZERO
//
float Duration[ArraySize] = {60, 60, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
// ramp duration in minutes
float setPoint[ArraySize] = {.1, .12, .18, .21, .24, .26, .28, .30, .31, .32, .33, .34, .35, .36, .37, .38, .39, .40};
// pressures in psi
float Times[ArraySize + 1]; // start times measured from boot time in minutes
int ip = 0;                 // set ip to the starting index for restarts
// Upper limit for any setpoint -
float maxSetPoint = .40;

void stop()
{
  Serial.println(" Balloon is done stretching - stopping inflation");
  maxSetPoint = .25; // decrease the pressure setpoint
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("**** Setpoint is reduced to " + String(maxSetPoint) + " **** Current Pressure ");
  Serial.println(press);
  interval = 5 * 60000; // slow down the output
  stopping = true;
}

float startPoint = .0;
double setpt;

bool firstTime = true;
float setPointFunc()
{ // Ramp pressure up slowly
  if (ip == ArraySize - 1)
    return setPoint[ArraySize - 1];

  if (firstTime == true)
  {
    windowStartTime2 = (float)(millis() / 1000ul) / 60.;
    // set up times to switch
    Times[0] = windowStartTime2;
    for (int i = 0; i < ArraySize; ++i)
    {
      Times[i + 1] = Times[i] + Duration[i];
    }
    if (ip > 0)
    { // if restating with a partial balloon pressure
      windowStartTime2 = Times[ip - 1];
    }
    Serial.println(" ip " + String(ip));
    Serial.println(" windowStartTime2 " + String(windowStartTime2));
    Serial.println(" Times " + String(Times[ip]));
    firstTime = false;
  }
  float currentTime = (float)(millis() / 1000ul) / 60.;
  // Serial.println(" currentTime "+String(currentTime) );
  float timeMinute = currentTime + windowStartTime2;
  // Serial.println(" timeMinute "+String(timeMinute) );
  // Serial.println(" windowStartTime2 "+String(windowStartTime2) );

  float setptm = setPoint[ip];
  if (timeMinute > Times[ip])
  {

    ip++;
  }

  if (ip >= ArraySize - 1)
  {
    setptm = setPoint[ArraySize - 1];
    ip = ArraySize - 1; // Set the index to the last element
  }
  else if (ip == 0)
  {
    setptm = setPoint[0];
  }
  else
  {
    setptm = setPoint[ip - 1] + (setPoint[ip] - setPoint[ip - 1]) * (1. - (Times[ip] - timeMinute) / (Times[ip] - Times[ip - 1]));
  }

  if (setptm > maxSetPoint)
  {
    setptm = maxSetPoint;
    stop();
  }

  // Serial.println(" ip "+String(ip) );
  // Serial.print(" Setpoint = ");
  // Serial.println(setptm);
  return setptm;
}

// ********************************* Setup *****************************************************

bool pressureError = false;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  presBegin(); // Start the pressure sensors
  // Setup the gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  Serial.println(" Setup start ");

  // Serial.println("Set initial atmospheric pressure to be subtracted from pressure measurements");

  // Serial.println(F("\nType: 'new' to indicate starting a new balloon with zero pressure\n"
  //                "Type 'restart' to indicate restarting with a partial pressure balloon\n"
  //                "If you type the wrong value, remove power, release the pressure, start again and select new\n"
  //                " Then quickly close all valves."));

  bool validInput = false;
  // while (true)
  // {
  //   while (Serial.available() == 0)
  //   {
  //   }
  //   String cmdInput = Serial.readString();
  //   if (cmdInput.length() > 0)
  //   {
  //     if (cmdInput.startsWith(String('r')) )
  //     { // Restarting a balloon with partial pressure
  //       readAtmosphericPressure();
  //       break;
  //     }
  //     else if (cmdInput.startsWith(String('n')) )
  //     { // Stating from zero
  //       readAtmosphericPressure();
  //       break;
  //     }

  //   }

  //   Serial.println("Please choose a valid selection either new or restart ");
  // }
  readAtmosphericPressure();

  Serial.print(" The maximum pressure is set to  ");
  Serial.println(maxSetPoint);

  float press = pressure(); // Read the pressure
  Serial.print(" Current presure reading: press= ");
  Serial.println(press);
  int i;
  for (i = 0; i < ArraySize; ++i)
  { // set the initial setpoint
    // If the balloon is partially pressureized find the starting index
    ip = i;
    if (setPoint[i] > press)
    {
      ip = i;
      Serial.print(" Starting Index = ");
      Serial.println(ip);
      break;
    }
  }
  if (i == ArraySize)
  {
    Serial.println("DANGER DANGER current pressure is too high - Something is wrong.");
    pressureError = true;
  }
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  setpt = setPointFunc();
  Serial.print(" Initial Setpoint = ");
  Serial.println(setpt);

  // tell the PID to range between 0 and the full window size
  // Serial.println(" PID start ");
  myPID.SetOutputLimits(0, WindowSize);
  myPID.Start(0,      // current input
              0,      // current output
              setpt); // setpoint

  windowStartTime = millis();
}

//**************************** LOOP ****************************
float input = 0.;
unsigned long previousMillis = 0;
unsigned long interval = 60000; // interval to print in milliseconds
float currentOut = 0.;
float previousOut = 0.;
int iPnt = 0, iCnt = 0;
float pressAvg = 0, setPointAvg = 0, outputAvg = 0;
float angleAvg = 0;
bool stopping = false;
bool off = false;

void loop()
{
  if (pressureError == true)
    return;
  float press = pressure(); // Read the pressure
  float angle = readAngle();

  setpoint = setPointFunc();
  // Serial.print(" Loop Setpoint = ");
  // Serial.println(setpoint);

  input = pmap(press); // Convert the reading
  float stpt = pmap(setpoint);
  float output = myPID.Run(input); // call the controller function
  myPID.Start(input,               // current input
              output,              // current output
              stpt);               // new setpoint
  unsigned long currentMillis = millis();
  currentOut = output / 300.;

  // average the output over the interval
  outputAvg += currentOut;
  setPointAvg += setpoint;
  pressAvg += press;
  angleAvg += printAngle();
  iPnt++;

  if (currentMillis - previousMillis >= interval)
  {

    previousMillis = currentMillis;
    Serial.print(" Angle %  ");
    Serial.print(angleAvg / iPnt);
    Serial.print(",");
    Serial.print(" Setpoint ");
    Serial.print(setPointAvg * 100 / iPnt);
    Serial.print(",");
    Serial.print(" Pump ");
    Serial.print(outputAvg / iPnt);
    Serial.print(",");
    Serial.print(" Pressure ");
    Serial.print(pressAvg * 100 / iPnt);
    Serial.print(" Temperature ");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(" Time ");
    Serial.print(currentMillis / (1000. * 3600.));
    Serial.print(",");
    Serial.print(" Index ");
    Serial.println(ip);

    if ((angleAvg / iPnt) < 100. && stopping == false) // check the angle
    {                                                  // Balloon is done stretching
      stop()
    }

    pressAvg = 0;
    setPointAvg = 0;
    outputAvg = 0;
    angleAvg = 0;
    iPnt = 0;
    previousOut = currentOut;

    // if (stopping == true)
    // {
    //   if (pressAvg/iPnt < .35)
    //   { // the pressure is low enough to turn off the pump
    //     Serial.println(" Pressure is low enough to turn off the pump");
    //     digitalWrite(RELAY_PIN, LOW);
    //     digitalWrite(LED_BUILTIN, LOW);
    //     off = true;
    //   }
    // }
  }
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  // if (off == true) output = 0.; // turn off the pump

  while (millis() - windowStartTime > WindowSize)
  {
    // time to shift the Relay

    windowStartTime += WindowSize;
  }
  float diff = (float)(millis() - windowStartTime);

  if (output > diff)
  {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
