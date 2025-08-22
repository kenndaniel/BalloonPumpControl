

#include <BMP280_DEV.h> // Include the BMP280_DEV.h library

float temperature, press, altitude; // Create the temperature, pressure and altitude variables
BMP280_DEV bmp280;					// Instantiate (create) a BMP280_DEV object and set-up for I2C operation (address 0x77)
BMP280_DEV bmp280Atm;	

float zeroPres = 0; // Reset to the first pressure reading in presSetup() then used as bias error
int eeAddress = 0;	// EEPROM address to start reading from

float pressure()
{
	
	bool good = false;
	bmp280.startForcedConversion();
	for (int i = 0; i < 250; ++i)
	{
		// Start BMP280 forced conversion (if we're in SLEEP_MODE)

		if (bmp280.getMeasurements(temperature, press, altitude)) // Check if the measurement is complete
		{
			Serial.println("Balloon pressure reading");
			// Serial.print(temperature);                    // Display the results
			//Serial.print(F("*C   "));
			//Serial.print(press);
			//Serial.println(F("hPa   "));
			//Serial.print(altitude);
			//Serial.println(F("m"));
			delay(100);
			return (press) * 0.01450377 - zeroPres;
		}
	}
	Serial.println(" BMP 280 is humg *********************");
	//bmp280.startForcedConversion();
	return 0.;
}

void presBegin()
{
	// initialize the atmospheric pressure sensor
	
	bmp280Atm.begin(); // Default initialisation, place the BMP280 into SLEEP_MODE
	// bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
	// bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
	// bmp280.setIIRFilter(IIR_FILTER_4);
	// float press = pressure();             // Set the IIR filter to setting 4
	// Serial.print(" Init ");Serial.println(press);

	// Initialize the balloon pressure sensor

	bmp280.begin(BMP280_I2C_ALT_ADDR);
}

void readAtmosphericPressure()
{
     // This must be called only once at the start of the program

	 bool good = false;
	bmp280Atm.startForcedConversion();
	for (int i = 0; i < 250; ++i)
	{
		// Start BMP280 forced conversion (if we're in SLEEP_MODE)

		if (bmp280Atm.getMeasurements(temperature, press, altitude)) // Check if the measurement is complete
		{
			Serial.println("Atmospheric pressure reading");
			//Serial.print(temperature);                    // Display the results
			//Serial.print(F("*C   "));
			Serial.print(press);
			Serial.print(F("hPa   "));
			//Serial.print(altitude);
			//Serial.println(F("m"));
			delay(100);
			zeroPres = (press) * 0.01450377 ; // Set the zero pressure
			Serial.print(zeroPres*100.);
			Serial.println(F("psi  "));
			return (zeroPres)  ;
		}
	}
	Serial.println(" BMP 280Atm is humg *********************");
	return 0.;


	return; // set the zero pressure on setup
}
// void presSetup()
// {
//   Serial.begin(9600);                           // Initialise the serial port
//   bmp280.begin(BMP280_I2C_ALT_ADDR);              // Default initialisation, place the BMP280 into SLEEP_MODE
//   //bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
//   //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
//   //bmp280.setIIRFilter(IIR_FILTER_4);
//   float press = pressure();             // Set the IIR filter to setting 4
//   //Serial.print(" Init ");Serial.println(press);
//   //zeroPres = press;
//   return  ;  // set the zero pressure on setup

// }

/* #include <Arduino.h>
#include <Wire.h>

#include <BMx280I2C.h>

#define I2C_ADDRESS 0x76  // Sometimes this is 76 or 77

//create a BMx280I2C object using the I2C interface with I2C Address
BMx280I2C bmx280(I2C_ADDRESS);
float zeroPres = 0.;


float pressure() {
  // put your main code here, to run repeatedly:

	delay(1000);

	//start a measurement
	if (!bmx280.measure())
	{
		Serial.println("could not start measurement, is a measurement already running?");
		return -1;
	}

	//wait for the measurement to finish

	for (int i=0; i<250; i++)
	{ Serial.print("+ ");
	  delay(200);
	  if (bmx280.hasValue())
	  {
	  Serial.print("Pressure: "); Serial.println(bmx280.getPressure());
	  Serial.print("Pressure psi "); Serial.println((bmx280.getPressure64()-zeroPres)*0.0001450377);
	  //Serial.print("Temperature: "); Serial.println(bmx280.getTemperature());
	  return (bmx280.getPressure64()-zeroPres)*0.0001450377;
	  }

   }

	Serial.println(" BMP 280 is humg *********************");
	bmx280.resetToDefaults();
	return -1;
}


void presBegin() {
  // put your setup code here, to run once:
	//Serial.begin(9600);

	Wire.begin();

	//begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
	//and reads compensation parameters.
	if (!bmx280.begin())
	{
		Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
		while (1);
	}


	if (bmx280.isBME280())
		Serial.println("sensor is a BME280");
	else
		Serial.println("sensor is a BMP280");

	//reset sensor to default parameters.
	bmx280.resetToDefaults();

	//by default sensing is disabled and must be enabled by setting a non-zero
	//oversampling setting.
	//set an oversampling setting for pressure and temperature measurements.
	bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x01);   //OSRS_P_x01  OSRS_P_x16
	bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x01);   //OSRS_T_x01  OSRS_T_x16
	zeroPres = pressure()/0.0001450377;  // set the zero pressure on setup

}
 */