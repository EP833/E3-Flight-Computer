#include <SD.h>                 // SD Card
#include <Arduino_LPS22HB.h>    // Barometer
#include <SPI.h>                // SPI Communication 
#include <SimpleKalmanFilter.h> // Kalman Filter
#include <Arduino_LSM9DS1.h>    // IMU


boolean viewInSerialPlotter=true;      //true optimises for serial plotter, false for serial monitor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;

// Setup for Kalman Filters
SimpleKalmanFilter XgyroFilter(2, 2, 0.05);   // X gyroscope kalman filter
SimpleKalmanFilter YgyroFilter(.6, .6, 0.3);  // Y gyroscope kalman filter
SimpleKalmanFilter ZgyroFilter(.6, .6, 0.3);  // Z gyroscope kalman filter
SimpleKalmanFilter XacelFilter(.1, .1, 0.17); // X accelerometer kalman filter
SimpleKalmanFilter YacelFilter(.1, .1, 0.17); // Y accelerometer kalman filter
SimpleKalmanFilter ZacelFilter(.1, .1, 0.17); // Z accelerometer kalman filter
SimpleKalmanFilter PresFilter(2,2,1);    // Pressure kalman filter


// Setup variables
bool launched = false;      // Variable for launch detection 
unsigned long CurTime;      // Variable for launch detection timer
unsigned long TimeCheck;    // Variable for launch detection timer
const int chipSelect = 10;  // Variable for SD card chip select
float AltitudeGround;
float Pressure, filterPressure, Altitude, AltitudeAGL;  // Setup variables for altitude 
float Xacel, Yacel, Zacel, XacelVal, YacelVal, ZacelVal, MagAcel; // Setup variavles for acceleration; First three are raw data and last three are filtered data
float Xgyro, Ygyro, Zgyro, XgyroVal, YgyroVal, ZgyroVal; // Setup variavles for acceleration; First three are raw data and last three are filtered data
float ZgUpper, ZgLowwer;    // Variables for upper and lowwer bounds for landing detection

void setup() 
{  
  Serial.begin(9600);
  while (!Serial);

  setupSDcard();  // Setup SD card
  setupSensors(); // Setup sensors
}

void loop() 
{ 
  readAltitude();             // Get pressure, filtered pressure and altitude
  readIMU();                  // Get IMU variable values
  launchProcedure(ZacelVal);  //  Get rocket launch status
  if (launched){              // Check if rocket has been launched
    writeSDcard();            // Write all data to SD card
  }
  Serial.println();
}

// ###############################################################################################################################################################

// Function to read pressure and get altitude
void readAltitude(){
  Pressure = BARO.readPressure(); // Read pressure

  filterPressure = PresFilter.updateEstimate(Pressure); // Get updated filtered pressure

  Altitude = 145366.45*(1-pow(((filterPressure*10)/1013.25),0.190284));  // Get updated altitude
  AltitudeAGL = Altitude - AltitudeGround;  // Get the altitude above ground
  Serial.print(Pressure*100);
  Serial.print('\t');
  Serial.print(filterPressure*100);
  Serial.print('\t');


}

// ###############################################################################################################################################################

// Function to read IMU data and filter it
void readIMU(){

  if (IMU.accelAvailable())                               // Read acceleration data 
  {  
    IMU.readAccel(Xacel, Yacel, Zacel);                   // Save raw data from accelrometer in X, Y, Z axis
    XacelVal = XacelFilter.updateEstimate(Xacel);         // Filter X axis data
    YacelVal = YacelFilter.updateEstimate(Yacel);         // Filter Y axis data
    ZacelVal = ZacelFilter.updateEstimate(Zacel);         // Filter Z axis data
  }
  MagAcel = sqrt(sq(XacelVal)+sq(YacelVal)+sq(ZacelVal));

  if (IMU.gyroAvailable())                                // Read gyroscope data
  {  
    IMU.readGyro(Xgyro, Ygyro, Zgyro);                    // Save raw data from gyroscope in X, Y, Z axis 
    XgyroVal = XgyroFilter.updateEstimate(Xgyro);         // Filter X axis data
    YgyroVal = YgyroFilter.updateEstimate(Ygyro);         // Filter Y axis data
    ZgyroVal = ZgyroFilter.updateEstimate(Zgyro);         // Filter Z axis data
  }
}

// ###############################################################################################################################################################

// Function for detecting launching and landing
void launchProcedure(float ZacelVal){
  if (ZacelVal > 3 && launched == false)  // Check for launch; If detected a large increase in Z axis force
  {
    launched = true;                        // Set launched equal to true; True means launched, False means not launched
    Serial.println("LAUNCHED!!!");
    CurTime = millis();                     // Get current time
    TimeCheck = CurTime + 2000;             // Find what time is two seconds ahead
    ZgUpper = (ZacelVal + 1) * 1.01;        // Find upper bounds of Z axis value (1% greater than current Z axis value)
    ZgLowwer = (ZacelVal + 1)  * 0.99;      // Find lowwer bounds of Z axis value (1% less than current Z axis value)
  }


  if (launched)                             // Check if launched
  {
    CurTime = millis();                     // Get current time
    if ((fabs((ZacelVal + 1) ) >= fabs(ZgUpper)) || (fabs((ZacelVal + 1) ) <= fabs(ZgLowwer)))  // Check if current Z axis value is outside of the bounds
    // If true then rocket is still moving
    {
      ZgUpper = (ZacelVal + 1)  * 1.01;     // Find upper bounds of Z axis value (1% greater than current Z axis value)
      ZgLowwer = (ZacelVal + 1)  * 0.99;    // Find lowwer bounds of Z axis value (1% less than current Z axis value)
      TimeCheck = CurTime + 2000;           // Find what time is two seconds ahead
      Serial.print("Moving");
      Serial.print('\t');
    }
    else
    {
      Serial.print("Not Moving");
      Serial.print('\t');
    }
    if (CurTime >= TimeCheck)   // Check if rocket has not moved for 2 seconds; Can assume landed if true
    {
        launched = false;       // Consider rocket as landed
        myFile.close();         // Close file once landed
        Serial.print("LANDED!!!");
        Serial.print('\t');
    }
  }
}

// ###############################################################################################################################################################

// Function to write to SD card
void writeSDcard(){
  String stringPres = String(Pressure);         // Make pressure into a string
  String stringKPres = String(filterPressure);  // Make filtered pressure into a string
  String stringAlt = String(Altitude);          // Make altitude into a string
  String stringXAcel = String(XacelVal);        // Make X axis acceleration into a string
  String stringYAcel = String(YacelVal);        // Make Y axis acceleration into a string        
  String stringZAcel = String(ZacelVal);        // Make Z axis acceleration into a string
  String stringXGyro = String(XgyroVal);        // Make X axis gyroscope into a string
  String stringYGyro = String(YgyroVal);        // Make Y axis gyroscope into a string
  String stringZGyro = String(ZgyroVal);        // Make Z axis gyroscope into a string

  String dataString = String(stringPres + "," + stringKPres + "," + stringAlt + "," + stringXAcel + "," + stringYAcel + "," + stringZAcel + "," 
  + stringXGyro + "," + stringYGyro + "," + stringZGyro + ",");
  Serial.println(dataString);
  myFile.println(dataString); // Write to SD card
}

// ###############################################################################################################################################################

// Function to setup SD card
void setupSDcard(){
  if (!SD.begin(10))                          // Check if correct chip select is chosen
  {
    Serial.println("initialization failed!"); 
    while (1);                                // Keep in while loop if SD chip select is wrong
  }
  Serial.println("initialization done.");

  myFile = SD.open("data2.csv", FILE_WRITE);  // Open a file with the given name

  if (!myFile)                                // Check if file did not open
  {
    Serial.println("error opening data2.txt");
    while (1);                                // Keep in while loop if file not opening
  }
}

// ###############################################################################################################################################################

// Fucntion to setup sensors
void setupSensors(){
  //******************************************  BARO  *******************************************************
  if (!BARO.begin()) // Check if barometer is available
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);      // If barometer not availble then keep in a while loop
  }
  Pressure = BARO.readPressure(); // Read pressure
  AltitudeGround = 145366.45*(1-pow(((Pressure*10)/1013.25),0.190284));  // Get altitude at ground level

    //******************************************  Accelerometer  *******************************************************
  if (!IMU.begin()) // Check if IMU is available 
  { 
    Serial.println("Failed to initialize IMU!");
    while (1);      // If sensor not available then keep in a while loop
  }

  
  IMU.setAccelFS(1);  // Set the full scale of the accelerometer
  IMU.setAccelODR(5); // Set the data rate of the accelerometer

  IMU.setAccelOffset(-0.010507, -0.015193, -0.010510);  // Calibration for accelerometer
  IMU.setAccelSlope (0.993033, 0.999312, 1.004281);     // Calibration for accelerometer

  /***********************************************************************************************************************************
  *******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
  ************************************************************************************************************************************/

   IMU.setGyroFS(1);  // Set the full scale of the gyroscope
   IMU.setGyroODR(5); // Set the data rate for the gyroscope
   IMU.setGyroOffset (0.884926, 0.944702, -1.495347); // Calibration for gyroscope
   IMU.setGyroSlope (1.184391, 1.239955, 1.191125);   // Calibration for gyroscope

/*****************************************************************************************************************************     
*********  FS  Full Scale       setting 0: ±245°/s | 1: ±500°/s | 2: ±1000°/s | 3: ±2000°/s       ****************************
*********  ODR Output Data Rate setting 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (not working 6:952Hz)   *******
*****************************************************************************************************************************/   
  IMU.gyroUnit= DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  

  IMU.accelUnit=  GRAVITY;    // or  METERPERSECOND2    

  Serial.print("Accelerometer Full Scale = ±");
  Serial.print(IMU.getAccelFS());
  Serial.println ("g");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.getAccelODR());        // alias  AccelerationSampleRate());
  Serial.println(" Hz \n");

  Serial.println("Gyroscope in degrees/second \n");
  Serial.print("Gyroscope Full Scale = ±");
  Serial.print(IMU.getGyroFS());
  Serial.println ("°/s");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.getGyroODR());        //alias  IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.println(" X \t Y \t Z ");

  myFile.println("Pressure, Kalman Pressure, Altitude, X Accel, Y Accel, Z Accel, X Gyro, Y Gyro, Z Gyro,");
}
