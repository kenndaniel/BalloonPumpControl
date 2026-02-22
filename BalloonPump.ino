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
 *
 * Strain Rate Formulas https://chatgpt.com/c/68da5972-59d0-8329-b226-97c9bb0c0c09
 ********************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <PID_v2.h>

enum mode
{
  FILLING,
  STRETCHING,
  HOLDING,
  EXHAUSTING,
  WAITING,
  MEASURING
};
mode MODE = FILLING;

#define MEASURE_SET_POINT .25 // pressure setpoint when in MEASURING mode
#define FILLED_ANGLE 183.3    // angle where filling will stop and exhaust begins
float fillingLimit = 192.;    // when the angel is less than this, transition to filling

// #define TESTING
#include "BMP280_TempPres.h"

#include "utilities.h"

void restart()
{
  float press = pressure(); // Read the pressure
  Serial.print(" Current presure reading: press= ");
  Serial.println(press);

  int i = 0;
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
  ip = i;
}

void stop()
{ // Balloon is done stretching
  if (MODE == WAITING)
    return;
  Serial.println(" Balloon is done stretching - reducing pressure to .25 ");
  digitalWrite(RELAY_PIN, LOW);  // turn off the pump
  digitalWrite(LED_BUILTIN, LOW);
  delay(19000); // let the pressure equilibrate
  float press = pressure(); // Read the pressure
  Serial.print(" Current presure reading: press= ");
  Serial.println(press);
  while (press >= measureSetPoint)
  {
    digitalWrite(EXHAUST_VALVE, HIGH); // Open exhaust valve
    digitalWrite(LED_BUILTIN, HIGH);
    delay(15000);
    digitalWrite(EXHAUST_VALVE, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delay(20000); // Let the pressure equilibrate
    press = pressure();
    Serial.print(" Current presure reading: press= ");
    Serial.println(press);
  }

  Serial.print(" pressure is less than measuring setpoint ");
  Serial.println(measureSetPoint);
  Serial.print("Setpoint is reduced to " + String(measureSetPoint) + "  Current Pressure ");
   Serial.println(press);
  Serial.println(" Output frequency 5 minutes ****");
  interval = 5 * 60000; // slow down the output
  MODE = WAITING;       // pressure is down and waiting to pull out the tube
  restart();
  Serial.print(" Changing to ");
  printMode();
}

// ********************************* Setup *****************************************************
bool off = false;

bool pressureError = false;
void setup()
{
  pinMode(EXHAUST_VALVE, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  presBegin(); // Start the pressure sensors
  // Setup the gyro (angle measurement)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  Serial.println(" Setup start Version 2/7/2026 ");

  bool validInput = false;
  readAtmosphericPressure();

  Serial.print(" The maximum pressure is set to  ");
  Serial.println(maxSetPoint);

  for (int i = 0; i < ArraySize; ++i)
  {
    Duration[i] *= speedUp;
  }

  if (ip == ArraySize)
  {
    Serial.println("DANGER DANGER current pressure is too high - Something is wrong.");
    stop();
  }
  Serial.println(" Waiting for input - 'm' measure, 's' stretch 'e' to exhaust 'f' initial fill  ");
  while (true)
  {
    while (Serial.available() == 0)
    {
    }
    String cmdInput = Serial.readString();
    if (cmdInput.length() > 0)
    {
      if (cmdInput.startsWith(String('s')))
      { // Restarting a balloon with partial pressure
        MODE = STRETCHING;
        break;
      }
      else if (cmdInput.startsWith(String('e')))
      { // exhaust until measuringSetPoint to check for leaks
        MODE = EXHAUSTING;
        stop(); //
        break;
      }
      else if (cmdInput.startsWith(String('m')))
      { // Measuring angle
        MODE = MEASURING;
        Serial.print(" Changing to ");
        printMode();
        off = true;           // turn off the pump while measuring angle
        interval = 5 * 60000; // slow down the output
        break;
      }
      else if (cmdInput.startsWith(String('f')))
      { // initial filling
        MODE = FILLING;
        Serial.print(" Changing to ");
        printMode();
        break;
      }
    }
    break;
    Serial.println("Please choose a valid selection 'm' measure, 's' stretch 'e' to exhaust 'f' initial fill  ");
  }
  printMode();
  float setpt = setPointFunc();
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
float currentOut = 0.;
float previousOut = 0.;
int iPnt = 0, iCnt = 0;
float pressAvg = 0, setPointAvg = 0, outputAvg = 0;
float angleAvgPct = 0;
float angleAvg = 0;
float angleAvg2 = 200.;

void loop()
{
  float output = 0.;
  unsigned long currentMillis = millis();
  float stpt = 0.;
  float press = pressure(); // Read the pressure
  float angle = readAngle();
  // Serial.print(" Current presure reading: press= ");
  // Serial.print(pressure());
  //  Serial.print(" Ang = ");
  //  Serial.println(angle);

  if ((angle < FILLED_ANGLE && MODE == STRETCHING)) // check the size
  {
    Serial.print(angle);
    Serial.print(" is less than filling limit ");
    Serial.println(FILLED_ANGLE);
    MODE = EXHAUSTING; // Balloon is done stretching
    Serial.print(" Changing to ");
    printMode();
    stop();
  }

  if (angleAvg2 < fillingLimit && MODE == FILLING)
  {
    Serial.print(angleAvg2);
    Serial.print(" is less than filling limit ");
    Serial.println(fillingLimit);
    MODE = STRETCHING;
    Serial.print(" Changing to ");
    printMode();
    delay(5000); // let the pressure equilibrate
    restart();
  }
  if (MODE == STRETCHING || MODE == WAITING )
  {
    setpoint = setPointFunc();
    // Serial.print(" Loop Setpoint = ");
    // Serial.println(setpoint);

    input = pmap(press); // Convert the reading
    stpt = pmap(setpoint);
    output = myPID.Run(input); // call the controller function
    myPID.Start(input,         // current input
                output,        // current output
                stpt);         // new setpoint
    currentMillis = millis();
  }
  if (MODE == MEASURING || MODE == EXHAUSTING)
    output = 0.; // turn off the pump

  currentOut = output / WindowSize;

  // average the output over the interval
  outputAvg += currentOut;
  setPointAvg += setpoint;
  pressAvg += press;
  angleAvgPct += printAngle();
  angleAvg += angle;
  iPnt++;

  if (currentMillis - previousMillis >= interval)
  {
    angleAvg2 = angleAvg / iPnt;
    int hrs = (currentMillis / (1000 * 3600));
    int mins = (currentMillis / (1000 * 60)) % 60;
    previousMillis = currentMillis;
    Serial.print(" Angle %  ");
    Serial.print(100.+angleAvg / iPnt);
    Serial.print(", ");
    Serial.print(angleAvg2);
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
    Serial.print(String(hrs) + ":" + String(mins));
    Serial.print(",");
    Serial.print(" Index ");
    Serial.print(ip);
    printMode();

    pressAvg = 0;
    setPointAvg = 0;
    outputAvg = 0;
    angleAvg = 0;
    iPnt = 0;
    previousOut = currentOut;
  }
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/

  while (millis() - windowStartTime > WindowSize)
  {
    // time to shift the Relay

    windowStartTime += WindowSize;
  }
  float diff = (float)(millis() - windowStartTime);

  if (MODE == FILLING)
    diff = -1.;

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
