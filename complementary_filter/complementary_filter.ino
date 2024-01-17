// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <FS.h>
#include <stdlib.h>
#include "SPIFFS.h"

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET 20

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

double acc[3] = { 0, 0, 0 };
double A_acc[3][3] = { { -0.9991, -0.0056, -0.0104 },
                       { 0.0176, -1.0099, 0.0411 },
                       { 0.0148, 0.0233, -1.0033 } };
double b_acc[3] = { -0.1896, 0.1196, -0.2021 };

double mag[3] = { 0, 0, 0 };

double attitude_acc[3] = { 0, 0, 0 };

double attitude_gyro[3] = { 0, 0, 0 };

double comp_a = 0.5;
double attitude[3] = { 0, 0, 0 };

double gyro[3] = { 0, 0, 0 };
uint64_t gyro_millis;
double gyro_dt;

File file_flash;

//vedi Help di MATLAB per accelcal, solo che noi facciamo c=u*A+b con u e c e b con dim [1]x[3]
//usiamo una var di appoggio per non definirci c
void cal_acc(double acc[3]) {
  double app = 0;
  for (int j = 0; j < 3; j++) {
    app = 0;
    for (int k = 0; k < 3; k++) {
      app += acc[k] * A_acc[k][j];
    }
    acc[j] = app;
  }
  for (int i = 0; i < 3; i++) {
    acc[i] += b_acc[i];
  }
}

int bytesWritten = 0;
char* linea = (char *)malloc(100 * sizeof(char));
// TODO: LIBERARE LINEA

void print_su_flash() {
  sprintf(linea, "%f,%f,%f", attitude[0],attitude[1],attitude[2]);
  Serial.println(linea);
  bytesWritten += file_flash.println(linea);
}

long int inizio;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);  // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("Adafruit BNO08x test!");

  bool success = SPIFFS.begin();
 
  if (success) {
    Serial.println("File system mounted with success");
  } else {
    Serial.println("Error mounting the file system");
    return;
  }
 
  file_flash = SPIFFS.open("/dati_flash.txt", "w");
  bytesWritten += file_flash.println("attitude.x,attitude.y,attitude.z");
 
  if (!file_flash) {
    Serial.println("Error opening file for writing");
    return;
  }

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("impossibile trovare BNO085, aaaaaa! provo reset...");
    digitalWrite(BNO08X_RESET, LOW);
    delay(1000);
    if (!bno08x.begin_I2C()) {
      Serial.println("impossibile trovare BNO085");
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

  Serial.println("Reading events");
  delay(100);
  inizio = millis();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable raw magnetometer");
  }
}
void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc[0] = -sensorValue.un.rawAccelerometer.y / 65535. * 156.96;
      acc[1] = sensorValue.un.rawAccelerometer.x / 65535. * 156.96;
      acc[2] = sensorValue.un.rawAccelerometer.z / 65535. * 156.96;
      cal_acc(acc);
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag[0] = sensorValue.un.magneticField.x;
      mag[1] = sensorValue.un.magneticField.y;
      mag[2] = sensorValue.un.magneticField.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      uint64_t time_millis = millis();
      gyro[0] = sensorValue.un.gyroscope.x;
      gyro[1] = sensorValue.un.gyroscope.y;
      gyro[2] = sensorValue.un.gyroscope.z;
      if (gyro_millis != 0) {
        gyro_dt = abs(int(time_millis - gyro_millis)) * 0.001;
      }
      attitude_gyro[0] = attitude[0] + gyro[0] * gyro_dt;
      attitude_gyro[1] = attitude[1] + gyro[1] * gyro_dt;
      attitude_gyro[2] = attitude[2] + gyro[2] * gyro_dt;
      break;
  }
  attitude_acc[0] = atan2(mag[0] * sin(attitude[1]) - mag[1] * cos(attitude[1]), mag[2] * cos(attitude[2]) + sin(attitude[2]) * (mag[1] * sin(attitude[1]) + mag[0] * cos(attitude[1])));
  attitude_acc[1] = atan2(-acc[2], sqrt(pow(acc[1], 2) + pow(acc[0], 2)));  //questo è l'angolo sul y, pitch
  attitude_acc[2] = atan2(acc[1], -acc[0]);                                 // questo è l'angolo sulla z, roll

  attitude[0] = attitude_acc[0] * comp_a + attitude_gyro[0] * (1 - comp_a);
  attitude[1] = attitude_acc[1] * comp_a + attitude_gyro[1] * (1 - comp_a);
  attitude[2] = attitude_acc[2] * comp_a + attitude_gyro[2] * (1 - comp_a);

  Serial.print(degrees(attitude_gyro[0]));
  Serial.print('\t');
  Serial.print(degrees(attitude_gyro[1]));
  Serial.print('\t');
  Serial.print(degrees(attitude_gyro[2]));
  Serial.print('\t');

  Serial.print(degrees(attitude_acc[0]));
  Serial.print('\t');
  Serial.print(degrees(attitude_acc[1]));
  Serial.print('\t');
  Serial.print(degrees(attitude_acc[2]));
  Serial.print('\t');

  Serial.print(degrees(attitude[0]));
  Serial.print('\t');
  Serial.print(degrees(attitude[1]));
  Serial.print('\t');
  Serial.println(degrees(attitude[2]));

  if (millis() - inizio > 1000) {
    file_flash.close();
    Serial.println("finito!!!!");
    if (bytesWritten > 0) {
    Serial.println("File was written");
    Serial.println(bytesWritten);
  } else {
    Serial.println("File write failed");
  }
    delay(200000);
  } else {
    print_su_flash();
  }
  
}
