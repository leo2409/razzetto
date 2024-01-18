// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// definire pin per connessioni
#define BUZZER_PIN 21
#define VOLTAGE_PIN 1
#define DET_SD 19
#define BNO08X_RESET 20

#define SEALEVELPRESSURE_HPA (1004.00)

Adafruit_BMP3XX bmp;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Wire.begin();
  Wire.setClock(350000);
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  Serial.println("Reading events");
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER, 4000)) {
    Serial.println("Could not enable raw accelerometer");
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500)) {
    Serial.println("Could not enable gyroscope");
  }
  
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
}

int inizio = 0;
int n_campioni_acc = 0;
int n_campioni_gyro = 0;
int n_campioni_magn = 0;
int n_campioni_baro = 0;
int last_sample_bmp = 0;

void loop() {
  if (inizio == 0) inizio = millis();
  else if (millis() - inizio > 10000){
    Serial.print("numero di campioni acc: "); Serial.println(n_campioni_acc);
    Serial.print("hz acc: "); Serial.println(n_campioni_acc/10);
    Serial.print("numero di campioni gyro: "); Serial.println(n_campioni_gyro);
    Serial.print("hz gyro: "); Serial.println(n_campioni_gyro/10);
    Serial.print("numero di campioni magn: "); Serial.println(n_campioni_magn);
    Serial.print("hz magn: "); Serial.println(n_campioni_magn/10);
    Serial.print("numero di campioni baro: "); Serial.println(n_campioni_baro);
    Serial.print("hz baro: "); Serial.println(n_campioni_baro/10);
    delay(20000);
  }

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  while (!bno08x.getSensorEvent(&sensorValue));

  
  switch (sensorValue.sensorId) {
    
  case SH2_GYROSCOPE_CALIBRATED:
    n_campioni_gyro++;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    n_campioni_magn++;
    break;
    
  case SH2_RAW_ACCELEROMETER:
    n_campioni_acc++;
    break;
  }
  
  if (last_sample_bmp == 0 || millis() - last_sample_bmp > 10) {
    bmp.performReading();
    n_campioni_baro++;
    last_sample_bmp = millis();
  }
}
