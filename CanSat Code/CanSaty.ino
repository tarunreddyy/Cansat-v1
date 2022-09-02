#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

boolean debug = false;

//LoRa Module Connection Pins and Specs
int MOSI_loRa = 23, MISO_loRa = 19, CLK = 9999, NSS = 5, Dio0 = 17, RST = 4; //For ESP32_NodeMCU
int TxPower = 20, spreadingFactor = 7, CodingRate = 6;
long frequency = 868E6, bandWidth = 20.8E3, SPIFreq = 1E6;

//I2C pins connection
int sda = 99, scl = 99;

//indicator pins
int commIndicator = 32, buzzer = 99;

//SD module pins

//Global Variables for sensor values
float latitude, longitude, Altitude, Sat_No; //GPS
float gyx, gyy, gyz, acx, acy, acz, magx, magy, magz; //IMU
float humidity, Temp, pressure, bme_alt; //BME
float descentRate;

String latitude_st, longitude_st, Altitude_st, Sat_No_st; //GPS
String gyx_st, gyy_st, gyz_st, acx_st, acy_st, acz_st, magx_st, magy_st, magz_st; //IMU
String humidity_st, Temp_st, pressure_st, bme_alt_st; //BMP
String descentRate_st;

//For BME280 Definitions
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // BME280 I2C

//MPU-92050 Definitions
#ifdef defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif
MPU9250_DMP imu;

//GPS Module
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

//other global variables
int updateRate = 1500;
int IndicatorRate = 1000;
int Buzzer = 32;

boolean Ind1State = false, Ind2State = false;
String TLE_Str1="",TLE_Str2="";

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting CanSat...");
  //timer.setInterval(1000L, indicate1); //call indicate1 every second
  //timer.setInterval(IndicatorRate, indicate2);

  //Setup LoRa Module & Modulation Characteristics.................

  //LoRa.setSPIFrequency(SPIFreq);
  LoRa.setPins(NSS, RST, Dio0);
  LoRa.setTxPower(TxPower);
  //LoRa.setSpreadingFactor(spreadingFactor);//6 to 12
  //LoRa.setCodingRate4(CodingRate);//5-8
  //LoRa.setSignalBandwidth(20.8E3);
  if (!LoRa.begin(frequency))
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  else
  {
    Serial.print("LoRa Module Started");
  }
  LoRa.beginPacket();
  LoRa.print("LoRa OK!");
  LoRa.endPacket();

  //Setup Sensors..................................................
  //For Setting MPU-9250
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(16); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at
  // +/- 4912 uT (micro-tesla's)
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz


  //For setting GPS
  SerialGPS.begin(9600, SERIAL_8N1, 13, 12); //(RX, TX);
}

void loop()
{
  updateSensors();
  Calculate();
  generateString();
  TLE_Generate();
  TLE_Transmit();
  delay(1000);
}

void updateSensors()//fetch fresh sensor values and update the variables
{
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address!");
    LoRa.beginPacket();
    LoRa.println("Could not find a valid BME280 sensor, check wiring, address!");
    LoRa.endPacket();
  }
  else
  {
    BME280();
  }
  if (imu.begin() != INV_SUCCESS)
  {
    Serial.println("Unable to communicate with MPU-9250");
    LoRa.beginPacket();
    LoRa.println("Unable to communicate with MPU-9250");
    LoRa.endPacket();
  }
  else
  {
    MPU();
  }
  if (SerialGPS.available() > 0)
  {
    GPS();
  }
  else
  {
    Serial.println("GPS Data not ready");
    LoRa.beginPacket();
    LoRa.println("GPS Data not ready");
    LoRa.endPacket();
  }
}

void generateString()
{
  //GPS
  latitude_st = String(latitude);
  longitude_st = String(longitude);
  Altitude_st = String(Altitude);
  Sat_No_st = String(Sat_No);
  //IMU
  gyx_st = String(gyx);
  gyy_st = String(gyy);
  gyz_st = String(gyz);
  acx_st = String(acx);
  acy_st = String(acy);
  acz_st = String(acz);
  magx_st = String(magx);
  magy_st = String(magy);
  magz_st = String(magz);
  //ENV
  humidity_st = String(humidity);
  Temp_st = String(Temp);
  pressure_st = String(pressure);
  //Calculations
  bme_alt_st = String(bme_alt);
  descentRate_st = String(descentRate);
}
void Calculate()
{
  //Put all Calculations here
}

void TLE_Generate() //Generats a TLE string for transmission
{
  String st1 = "DL1 ";
  String st2 = "DL2 ";
  //Generate TLE Strings
  st1 = st1 + latitude_st +" "+longitude_st +" "+Altitude_st+" "+Sat_No_st;
  Serial.println(st1);
  st2 = st2 + humidity_st +" "+Temp_st+" "+pressure_st;
  Serial.println(st2);
  TLE_Str1= st1;
  TLE_Str2= st2;
}
void TLE_Transmit() //Transmits the latest Generated TLE String
{
  LoRa.beginPacket();
  LoRa.print(TLE_Str1);
  LoRa.endPacket();
  LoRa.beginPacket();
  LoRa.print(TLE_Str2);
  LoRa.endPacket();
}
void indicate1()//toggle indicator state (LED indicator on comm module)
{
  digitalWrite(commIndicator, !Ind1State);
}
void indicate2()//toggle indicator state (buzzer)
{
  digitalWrite(Buzzer, !Ind2State);
}
//---------------------------------------BME280_START
void BME280()
{
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  else
  {
    Temp = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    bme_alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
    Serial.print("Temperature = ");
    Serial.print(Temp);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme_alt);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println();
  }
}
//---------------------------------------BME280_END


//---------------------------------------MPU-9250_START
void MPU()
{
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    float acx = imu.calcAccel(imu.ax);
    float acy = imu.calcAccel(imu.ay);
    float accz = imu.calcAccel(imu.az);
    float gyx = imu.calcGyro(imu.gx);
    float gyy = imu.calcGyro(imu.gy);
    float gyz = imu.calcGyro(imu.gz);
    float magx = imu.calcMag(imu.mx);
    float magy = imu.calcMag(imu.my);
    float magz = imu.calcMag(imu.mz);
    SerialPort.println("Accel: " + String(acx) + ", " +
                       String(acy) + ", " + String(acz) + " g");
    SerialPort.println("Gyro: " + String(gyx) + ", " +
                       String(gyy) + ", " + String(gyz) + " dps");
    SerialPort.println("Mag: " + String(magx) + ", " +
                       String(magy) + ", " + String(magz) + " uT");
    SerialPort.println();
  }
  else
  {
    Serial.println("IMU Dataset is being ready");
  }
}
//---------------------------------------MPU-9250_END

//---------------------------------------GPS_START
void GPS()
{
  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }
  delay(500);
  latitude = gps.location.lat();
  longitude = gps.location.lng();
  Sat_No = gps.satellites.value();
  Altitude = gps.altitude.meters();
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  Serial.print("Sats=");  Serial.println(gps.satellites.value());
}
//---------------------------------------GPS_END
