#include <SimpleKalmanFilter.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>

SimpleKalmanFilter Xkf(0.1,0.1,0.75);
SimpleKalmanFilter Ykf(0.1,0.1,0.75);
SimpleKalmanFilter Zkf(0.1,0.1,0.75);
SimpleKalmanFilter Pres(3,3,0.25);

 
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BARO.begin()) {
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

void loop() {
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
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    float Xg = Xkf.updateEstimate(x);
    float Yg = Ykf.updateEstimate(y);
    float Zg = Zkf.updateEstimate(z);

    Serial.print(-4);
    Serial.print("\t");
    Serial.print(4);
    Serial.print('\t');
    Serial.print(Xg);
    Serial.print('\t');  
    Serial.print(x);
    Serial.print('\t');  
    Serial.print(Yg);
    Serial.print('\t');  
    Serial.print(y);
    Serial.print('\t');  
    Serial.print(Zg);
    Serial.print('\t');  
    Serial.println(z);
  }
}