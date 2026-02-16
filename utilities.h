
#define EXHAUST_VALVE 4
#define OPEN HIGH
#define CLOSED LOW
#define RELAY_PIN 6
#define BALLOON_PRES 7
#define ATMOS_PRES 8

void stop();
void restart();

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

float windowStartTime2 = 0;
#define ArraySize 39
//
//  TAKE THE CAP OFF THE BOTTLE WHEN STARTING UP TO CALIBRATE THE PRESSURE SENSOR TO ZERO
//
float Duration[ArraySize] = {60, 60, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
                             40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
// ramp duration in minutes
float setPoint[ArraySize] = {.1, .12, .18, .21, .24, .26, .28, .30, .31, .32, .33, .34, .35, .36,
                             .37, .38, .39, .40, .41, .42, .43, .44, .45, .46, .47, .48, .49, .50, .51, .52, .53, .54, .55, .56, .57, .58, .59, .60, .61}; // pressure setpoints in psi
// pressures in psi
float Times[ArraySize + 1]; // start times measured from boot time in minutes
int ip = 0;
float speedUp = .33; // factor to speed up filling by decreacing durations

// Upper limit for any setpoint -
float maxSetPoint = setPoint[ArraySize-1];
float measureSetPoint = MEASURE_SET_POINT; // pressure to measure the angle at - this is the point where the balloon is fully stretched and we can measure the angle to determine if the balloon is done stretching
float setPointAngle = FILLED_ANGLE; // smaller numbers make a larger balloon

#ifdef TESTING
unsigned long interval = 5000; // interval to print in milliseconds
#else
unsigned long interval = 60000; 
#endif

float plow = 0, phigh = .40, lowo = 0, higho = 1080;

float pmap(float p)
{ // Convert real world values (psi) to control variables
    float remap = p * (higho - lowo) / (phigh - plow);
    return remap;
}

float iSTA = 0;
float systemTestAngle()
{
    if (MODE == FILLING || MODE == STRETCHING)
    {
        float ang = 200. - (200. - FILLED_ANGLE) * iSTA / 100.;
        iSTA+=1.;
        return ang;
        Serial.print(ang);
        Serial.print(" ");
    }
    return FILLED_ANGLE;  // for WAITING AND MEASURING
}

float readAngle()
{
#ifdef TESTING
    return systemTestAngle();
#endif
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

float printAngle()
{
    // return percentage of setpoint
    float angle = (100. * (1 - readAngle() / (float)FILLED_ANGLE));
    //Serial.print( " Angle = ");
    //Serial.println(angle);
    return angle;
}

bool firstTime = true;
float setPointFunc()
{ // Ramp pressure up slowly
    #ifdef TESTING
    return .5;
    #endif

    if (ip == ArraySize - 1)
        return setPoint[ArraySize - 1];

    if (MODE == EXHAUSTING || MODE == WAITING)
    {
        return measureSetPoint;
    }

    if (firstTime == true)
    {
        windowStartTime2 = (float)(millis() / 1000ul) / 60.;
        // set up times to switch
        Times[0] = windowStartTime2;
        for (int i = 0; i < ArraySize; ++i)
        {
            Times[i + 1] = Times[i] + Duration[i];
        }
        restart(); // will increase ip if there is pressure already in the balloon
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

void printMode()
{  
    switch (MODE)
    {
    case FILLING:
      Serial.println("Fill");
      break;
    case STRETCHING:
      Serial.println("Stretch");
      break;
    case EXHAUSTING:
      Serial.println("Exhaust");
      break;
    case WAITING:
      Serial.println("Wait");
      break;
    case MEASURING:
      Serial.println("Measure");
    }
}