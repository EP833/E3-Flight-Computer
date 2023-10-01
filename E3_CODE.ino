#include <SimpleKalmanFilter.h> 
#include <Arduino_LPS22HB.h>    // Baro
#include <Arduino_LSM9DS1.h>    //  IMU

SimpleKalmanFilter Xkf(0.5,0.5,0.75);
SimpleKalmanFilter Ykf(0.1,0.1,0.75);
SimpleKalmanFilter Zkf(0.1,0.1,0.75);
SimpleKalmanFilter Pres(3,3,0.25);

bool launched = false; 
unsigned long MyTime;
unsigned long TimeCheck;
 
void setup() 
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BARO.begin()) 
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}

void loop() 
{
  float x, y, z;
  float pressure = BARO.readPressure()*10;
  float kpress = Pres.updateEstimate(pressure);

  /*
  Serial.print(pressure);
  Serial.print('\t');
  Serial.print(kpress);
  Serial.print('\t');
  float alt = 145366.45*(1-pow((kpress/1013.25),0.190284));
  Serial.print(alt);
  Serial.print('\t');
  */

  IMU.readAcceleration(x, y, z);
  float Xg = Xkf.updateEstimate(x);
  float Yg = Ykf.updateEstimate(y);
  float Zg = Zkf.updateEstimate(z);
  float ZgStill = Zg;
  float ZgUpper;
  float ZgLowwer;
  
  if (Zg > 1.1 && launched == false)
  {
    launched = true;
    Serial.println("LAUNCHED!!!");
    MyTime = millis();
    TimeCheck = MyTime + 2000;
  }
  if (launched)
  {
    MyTime = millis();
    ZgUpper = Zg * 1.02;
    ZgLowwer = Zg * 0.98;
    Serial.print(-4);
    Serial.print("\t");
    Serial.print(4);
    Serial.print('\t');
    Serial.print(Xg);
    Serial.print('\t');  
    Serial.print(Yg);
    Serial.print('\t'); 
    Serial.print(MyTime); 
    Serial.print('\t');
    Serial.println(TimeCheck);
    Serial.print(ZgLowwer);
    Serial.print("\t");
    Serial.print(ZgStill);
    Serial.print("\t");
    Serial.print(Zg);
    Serial.print("\t");
    Serial.println(ZgUpper);
    if (ZgLowwer > ZgStill || ZgStill > ZgUpper)
    {
      ZgStill = Zg;
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
