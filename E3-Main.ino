#include <SimpleKalmanFilter.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>    // Baro


boolean viewInSerialPlotter=true;      //true optimises for serial plotter, false for serial monitor

// Setup for Kalman Filters
SimpleKalmanFilter XgyroFilter(2, 2, 0.05);
SimpleKalmanFilter YgyroFilter(.6, .6, 0.3);
SimpleKalmanFilter ZgyroFilter(.6, .6, 0.3);
SimpleKalmanFilter XacelFilter(.1, .1, 0.17);
SimpleKalmanFilter YacelFilter(.1, .1, 0.17);
SimpleKalmanFilter ZacelFilter(.1, .1, 0.17);
SimpleKalmanFilter Pres(3,3,0.25);


// Setup variables
bool launched = false; 
unsigned long MyTime;
unsigned long TimeCheck;

void setup() 
{  
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin())
  { Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BARO.begin()) 
  {
    Serial.println("Failed to initialize pressure sensor!");
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

  IMU.setGyroFS(2);
  IMU.setGyroODR(5);
  IMU.setGyroOffset (2.105325, 0.770813, -1.001007);
  IMU.setGyroSlope (1.193348, 1.159747, 1.147723);

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
}

void loop() {
  float Xacel, Yacel, Zacel, Xgyro, Ygyro, Zgyro, XgyroVal, YgyroVal, ZgyroVal, XacelVal, YacelVal, ZacelVal;
  /*
  float pressure = BARO.readPressure()*10;
  float kpress = Pres.updateEstimate(pressure);
  Serial.print(pressure);
  Serial.print('\t');
  Serial.print(kpress);
  Serial.print('\t');
  float alt = 145366.45*(1-pow((kpress/1013.25),0.190284));
  Serial.println(alt);
  */

  if (IMU.accelAvailable())                   // alias IMU.accelerationAvailable in library version 1.01
  {  
    IMU.readAccel(Xacel, Yacel, Zacel);                  // alias IMU.readAcceleration  in library version 1.01
    XacelVal = XacelFilter.updateEstimate(Xacel);
    YacelVal = YacelFilter.updateEstimate(Yacel);
    ZacelVal = ZacelFilter.updateEstimate(Zacel);
    
    Serial.print(-3);
    Serial.print('\t');
    Serial.print(3);
    Serial.print('\t');
    Serial.print(Xacel);
    Serial.print('\t');
    Serial.print(XacelVal);
    Serial.print('\t');
    Serial.print(Yacel);
    Serial.print('\t');
    Serial.print(YacelVal);
    Serial.print('\t');
    Serial.print(Zacel);
    Serial.print('\t');
    Serial.print(ZacelVal);
    Serial.println('\t');
    
  }
  
  if (IMU.gyroAvailable())    // alias IMU.gyroscopeAvailable
  {  
    IMU.readGyro(Xgyro, Ygyro, Zgyro);   // alias IMU.readGyroscope
    XgyroVal = XgyroFilter.updateEstimate(Xgyro);
    YgyroVal = YgyroFilter.updateEstimate(Ygyro);
    ZgyroVal = ZgyroFilter.updateEstimate(Zgyro);
    
    /*
    Serial.print(-3);
    Serial.print('\t');
    Serial.print(3);
    Serial.print('\t');
    Serial.print(Xgyro);
    Serial.print('\t');
    Serial.print(XgyroVal);
    Serial.print('\t');
    Serial.print(Ygyro);
    Serial.print('\t');
    Serial.print(YgyroVal);
    Serial.print('\t');
    Serial.print(Zgyro);
    Serial.print('\t');
    Serial.println(ZgyroVal);
    */

  }
  
  float ZgStill = ZacelVal;
  float ZgUpper;
  float ZgLowwer;

  if (ZacelVal > 1.1 && launched == false)
  {
    launched = true;
    Serial.println("LAUNCHED!!!");
    MyTime = millis();
    TimeCheck = MyTime + 2000;
  }

  if (launched)
  {
    MyTime = millis();
    ZgUpper = ZacelVal * 1.01;
    ZgLowwer = ZacelVal * 0.99;

    if (ZgLowwer > ZgStill || ZgStill > ZgUpper)
    {
      ZgStill = ZacelVal;
      TimeCheck = MyTime + 5000;
      Serial.println("Reset");
    }
    else
    {
      if (TimeCheck <= MyTime)
      {
        launched = false;
        Serial.println("LANDED!!!");
      }
    }
  }
}
