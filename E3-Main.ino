#include <Arduino_LPS22HB.h>
#include <SPI.h>                // SPI Communication 
#include <SD.h>                 // SD Card
#include <SimpleKalmanFilter.h> // Kalman Filter
#include <Arduino_LSM9DS1.h>    // IMU


boolean viewInSerialPlotter=true;      //true optimises for serial plotter, false for serial monitor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;

// Setup for Kalman Filters
SimpleKalmanFilter XgyroFilter(2, 2, 0.05);
SimpleKalmanFilter YgyroFilter(.6, .6, 0.3);
SimpleKalmanFilter ZgyroFilter(.6, .6, 0.3);
SimpleKalmanFilter XacelFilter(.1, .1, 0.17);
SimpleKalmanFilter YacelFilter(.1, .1, 0.17);
SimpleKalmanFilter ZacelFilter(.1, .1, 0.17);
SimpleKalmanFilter PresFilter(.1,.1,0.25);


// Setup variables
bool launched = false; 
unsigned long CurTime;
unsigned long TimeCheck;
float ZgStill, ZgUpper, ZgLowwer;
const int chipSelect = 10;
float Xacel, Yacel, Zacel, Xgyro, Ygyro, Zgyro, XgyroVal, YgyroVal, ZgyroVal, XacelVal, YacelVal, ZacelVal;


void setup() 
{  
  Serial.begin(9600);
  while (!Serial);

  //******************************************  SD  *******************************************************


  Serial.print("\nInitializing SD card...");


  if (!SD.begin(10)) 
  {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  myFile = SD.open("data2.csv", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("opened success");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data2.txt");
    while (1);
  }

  //******************************************  IMU  *******************************************************

  if (!IMU.begin())
  { 
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  
  IMU.setAccelFS(1);
  IMU.setAccelODR(5);
  IMU.setAccelOffset(-0.010507, -0.015193, -0.010510);
  IMU.setAccelSlope (0.993033, 0.999312, 1.004281);

  /***********************************************************************************************************************************
  *******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
  ************************************************************************************************************************************/

  //******************************************  BARO  *******************************************************


  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  // Gyroscope code
   IMU.setGyroFS(1);
   IMU.setGyroODR(5);
   IMU.setGyroOffset (0.884926, 0.944702, -1.495347);
   IMU.setGyroSlope (1.184391, 1.239955, 1.191125);

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

void loop() 
{
  float pressure = BARO.readPressure();  
  Serial.print(pressure);
  Serial.print('\t');

  float kpress;
  kpress = PresFilter.updateEstimate(pressure);
  Serial.print(kpress);
  Serial.print('\t');
  

  float alt = 145366.45*(1-pow(((kpress*10)/1013.25),0.190284));
  Serial.println(alt);
  Serial.print('\t');
  

  if (IMU.accelAvailable())                   // alias IMU.accelerationAvailable in library version 1.01
  {  
    IMU.readAccel(Xacel, Yacel, Zacel);                  // alias IMU.readAcceleration  in library version 1.01
    XacelVal = XacelFilter.updateEstimate(Xacel);
    YacelVal = YacelFilter.updateEstimate(Yacel);
    ZacelVal = ZacelFilter.updateEstimate(Zacel);
  }
  Serial.print('\t');
  
  if (IMU.gyroAvailable())    // alias IMU.gyroscopeAvailable
  {  
    IMU.readGyro(Xgyro, Ygyro, Zgyro);   // alias IMU.readGyroscope
    XgyroVal = XgyroFilter.updateEstimate(Xgyro);
    YgyroVal = YgyroFilter.updateEstimate(Ygyro);
    ZgyroVal = ZgyroFilter.updateEstimate(Zgyro);

  }
  
  if (ZacelVal > 1.1 && launched == false)
  {
    launched = true;
    Serial.println("LAUNCHED!!!");
    CurTime = millis();
    TimeCheck = CurTime + 2000;
    ZgUpper = (ZacelVal + 1) * 1.01;
    ZgLowwer = (ZacelVal + 1)  * 0.99;
  }
  Serial.print('\t');

  if (launched)
  {
    String stringPres = String(pressure);
    String stringKPres = String(kpress);
    String stringAlt = String(alt);
    String stringXAcel = String(XacelVal);
    String stringYAcel = String(YacelVal);
    String stringZAcel = String(ZacelVal);
    String stringXGyro = String(XgyroVal);
    String stringYGyro = String(YgyroVal);
    String stringZGyro = String(ZgyroVal);
    String dataString = String(stringPres + "," + stringKPres + "," + stringAlt + "," + stringXAcel + "," + stringYAcel + "," + stringZAcel + "," 
    + stringXGyro + "," + stringYGyro + "," + stringZGyro + ",");
    Serial.println(dataString);
    myFile.println(dataString);
    CurTime = millis();
    if ((fabs((ZacelVal + 1) ) >= fabs(ZgUpper)) || (fabs((ZacelVal + 1) ) <= fabs(ZgLowwer)))
    {

      ZgUpper = (ZacelVal + 1)  * 1.02;
      ZgLowwer = (ZacelVal + 1)  * 0.98;
      TimeCheck = CurTime + 2000;      
      Serial.print("Moving");
      Serial.print('\t');
    }
    else
    {
      Serial.print("Not Moving");
      Serial.print('\t');
    }
    if (CurTime >= TimeCheck)
    {
        launched = false;
        myFile.close();
        Serial.print("LANDED!!!");
        Serial.print('\t');
    }
  }
  Serial.println();
  
}
