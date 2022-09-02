
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BME280.h>
#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

//For BMP388
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp; // I2C
Adafruit_BME280 bme; // BME280 I2C

//MPU
#ifdef defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif
MPU9250_DMP imu;

//GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);


//Global Variables
float bmp3Temp, bmp3Press, bmp3Alt;
float bmeTemp, bmePress, bmeAlt, bmeHum;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ ;
double GPSLat, GPSLong;
int GPSSat;
float GPSalt;
void setup()
{
  Serial.begin(115200);

  // FOR Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  //For Setting MPU
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at
  // +/- 4912 uT (micro-tesla's)
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  //For setting GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
}

void loop()
{
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  else
  {
    BMP388();
  }

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  else
  {
    BME280();
  }
  if (imu.begin() != INV_SUCCESS)
  {
    SerialPort.println("Unable to communicate with MPU-9250");
  }
  else
  {
    MPU();
  }
  GPS();
  delay(1000);
}
//---------------------------------------BMP388
void BMP388()
{
  if (! bmp.performReading())
  {
    Serial.println("Failed to perform reading :(");
  }
  else
  {
    bmp3Temp = bmp.temperature;
    bmp3Press = bmp.pressure / 100.0;
    bmp3Alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Temperature = ");
    Serial.print(bmp3Temp);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp3Press);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp3Alt);
    Serial.println(" m");
    Serial.println();
  }
}
//---------------------------------------BMP388


//---------------------------------------BME280
void BME280()
{
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  else
  {
    bmeTemp = bme.readTemperature();
    bmePress = bme.readPressure() / 100.0F;
    bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bmeHum = bme.readHumidity();
    Serial.print("Temperature = ");
    Serial.print(bmeTemp);
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bmePress);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmeAlt);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bmeHum);
    Serial.println(" %");
    Serial.println();
  }
}
//---------------------------------------BME280




//---------------------------------------MPU
void MPU()
{
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    float accelX = imu.calcAccel(imu.ax);
    float accelY = imu.calcAccel(imu.ay);
    float accelZ = imu.calcAccel(imu.az);
    float gyroX = imu.calcGyro(imu.gx);
    float gyroY = imu.calcGyro(imu.gy);
    float gyroZ = imu.calcGyro(imu.gz);
    float magX = imu.calcMag(imu.mx);
    float magY = imu.calcMag(imu.my);
    float magZ = imu.calcMag(imu.mz);
    SerialPort.println("Accel: " + String(accelX) + ", " +
                       String(accelY) + ", " + String(accelZ) + " g");
    SerialPort.println("Gyro: " + String(gyroX) + ", " +
                       String(gyroY) + ", " + String(gyroZ) + " dps");
    SerialPort.println("Mag: " + String(magX) + ", " +
                       String(magY) + ", " + String(magZ) + " uT");  SerialPort.println();
  }
  else
  {
    Serial.println("MPU Dataset is being ready");
  }
}
//---------------------------------------MPU



//---------------------------------------GPS
void GPS()
{
  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }
  delay(500);
  GPSLat = gps.location.lat();
  GPSLong = gps.location.lng();
  GPSSat = gps.satellites.value();
  GPSalt = gps.altitude.meters();
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  Serial.print("Sats=");  Serial.println(gps.satellites.value());
}
//---------------------------------------GPS
