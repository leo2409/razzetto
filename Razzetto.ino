#include <math.h>
#include <stdlib.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
// sensori
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
// file sistem
#include <FS.h>
#include <SPIFFS.h>
// linear algebra
#include <BasicLinearAlgebra.h>
#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)
#define G_CONST (9.81)
using namespace BLA;

// definire pin per connessioni
#define BUZZER_PIN 21
#define VOLTAGE_PIN 1
#define DET_SD 19
#define BNO08X_RESET 20
// PYRO CHANNELS
#define PYRO_1 4
#define PYRO_2 5
#define PYRO_3 6
#define PYRO_4 7
// SPI
#define MISO 35
#define MOSI 37
#define CS_SD 34
#define SCK 36
// I2C
#define SCL 9
#define SDA 8
// voltaggio pin voltage batteria
#define MAX_VOLTAGE 0.7368  // Batteria 100% 4.2V -> 0.7368V
#define MIN_VOLTAGE 0.6491  // batteria   0% 3.7V -> 0.6491V
#define SEALEVELPRESSURE_HPA (1013.25)

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Razzetto"
#define REMOTEXY_WIFI_PASSWORD ""
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 276 bytes
  { 255, 0, 0, 21, 0, 13, 1, 16, 31, 1, 70, 16, 6, 10, 7, 7, 1, 121, 0, 70,
    16, 22, 10, 7, 7, 1, 121, 0, 129, 0, 3, 5, 13, 3, 8, 77, 73, 67, 82, 79,
    45, 83, 68, 0, 129, 0, 21, 5, 9, 3, 8, 80, 89, 82, 79, 32, 49, 0, 129, 0,
    35, 5, 9, 3, 8, 80, 89, 82, 79, 32, 49, 0, 129, 0, 48, 5, 9, 3, 8, 80,
    89, 82, 79, 32, 49, 0, 70, 16, 36, 10, 7, 7, 1, 121, 0, 70, 16, 49, 10, 7,
    7, 1, 121, 0, 66, 1, 6, 25, 7, 16, 2, 26, 129, 0, 3, 20, 13, 3, 8, 66,
    65, 84, 84, 69, 82, 89, 0, 71, 56, 1, 46, 19, 19, 0, 2, 24, 255, 0, 0, 52,
    195, 0, 0, 52, 67, 0, 0, 52, 66, 0, 0, 32, 65, 0, 0, 160, 64, 24, 0, 71,
    56, 41, 46, 19, 19, 0, 2, 24, 255, 0, 0, 52, 195, 0, 0, 52, 67, 0, 0, 52,
    66, 0, 0, 32, 65, 0, 0, 160, 64, 24, 0, 71, 56, 21, 46, 19, 19, 0, 2, 24,
    255, 0, 0, 52, 195, 0, 0, 52, 67, 0, 0, 52, 66, 0, 0, 32, 65, 0, 0, 160,
    64, 24, 0, 129, 0, 4, 43, 12, 3, 8, 66, 65, 84, 84, 69, 82, 89, 0, 129, 0,
    24, 43, 12, 3, 8, 66, 65, 84, 84, 69, 82, 89, 0, 129, 0, 45, 43, 12, 3, 8,
    66, 65, 84, 84, 69, 82, 89, 0, 68, 17, 3, 67, 57, 32, 8, 36 };

// this structure defines all the variables and events of your control interface
struct {

  // output variables
  uint8_t sd_check;           // led state 0 .. 1
  uint8_t pyro_1_continuity;  // led state 0 .. 1
  uint8_t pyro_2_continuity;  // led state 0 .. 1
  uint8_t pyro_3_continuity;  // led state 0 .. 1
  int8_t battery_percentage;  // =0..100 level position
  float pitch;                // from -180 to 180
  float yaw;                  // from -180 to 180
  float roll;                 // from -180 to 180
  float height;

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

//SENSORI
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValueIMU;

// STATO
uint8_t stato = 1;

// FILE NELLA FLASH
File file_flash;

// vettori di stato
// accelerometro vettore colonna
BLA::Matrix<3> acc = { 0, 0, 0 };
// accelerazione verticale
float acc_vert = 0;
// velocità verticale e altezza
float base_altitude = 0;
float altitude_baro = 0;
float h_baro = 0;
float v_vert = 0;
BLA::Matrix<2,1>  S_h       = { 0, 0 };
BLA::Matrix<2,2>  A_h = {   1.000,  0.004,
                            0.000,  1.000  };
BLA::Matrix<2,1>  B_h = {0.5 * 0.004 * 0.004, 0.004};
BLA::Matrix<1,2>  C_h = {  1,  0   };
BLA::Matrix<2,2>  U_h = {  0,  0,
                           0,  0  };
float std_dev_acc = pow(0.50,2); // 0.10^2 m/s^2 deviazione standard acc
BLA::Matrix<1,1> std_dev_baro = { pow(0.2 , 2) }; // 0.2 m incertezza sul barometro
BLA::Matrix<2,1> K_h = {  0,  0   };
BLA::Matrix<2,2> I = {  1,  0,
                        0,  1,  };
BLA::Matrix<1,1> M = { 0 };
// giroscopio
BLA::Matrix<3> gyro = { 0, 0, 0 };
// magnetometro
BLA::Matrix<3> mag = { 0, 0, 0 };
// attitude
BLA::Matrix<3> attitude_kalman = { 0, 90, 90 };
BLA::Matrix<3> attitude_acc = { 0, 0, 0 };
BLA::Matrix<3> uncertainty_attitude = { 5 * 5, 2 * 2, 2 * 2 };
BLA::Matrix<3> kalman_gain = { 0, 0, 0, };

long int last_sample_gyro = 0;
float dt_gyro = 0;
long int last_sample_acc = 0;
float dt_acc = 0;
// altezza e velocità verticale


// matrici calibrazione accelerometro
BLA::Matrix<3, 3> A_cal_acc = { -0.9991, 0.0176, 0.0148,
                                -0.0056, -1.0099, 0.0233,
                                -0.0104, 0.0411, -1.0033 };
BLA::Matrix<3> b_cal_acc = { -0.1896, 0.1196, -0.2021 };  // vettore colonna



void setup() {
  tone(BUZZER_PIN, 4000, 1000);

  Serial.begin(115200);
  while (!Serial) delay(100);
  delay(3000);
  Serial.println("RAZZETTO");

  RemoteXY_Init();

  // PIN DA SETTARE
  // digitali
  pinMode(DET_SD, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  // analogici
  adcAttachPin(VOLTAGE_PIN);
  adcAttachPin(PYRO_1);
  adcAttachPin(PYRO_2);
  adcAttachPin(PYRO_3);
  adcAttachPin(PYRO_4);
  analogSetAttenuation(ADC_0db);  // lettura voltaggi da 0.0V - 0.8V
  Serial.println("ADC: OK");

  // FILESISTEM
  if (SPIFFS.begin()) {
    Serial.println("SPIFFS:\tOK");
  } else {
    Serial.println("impossibile montare il file system SPIFFS flash");
    while (1)
      ;
  }
  // apertura file flash
  file_flash = SPIFFS.open("/dati_flash.csv", "w");
  if (!file_flash) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1)
      ;
  }

  // INIZIALIZZO SENSORI
  // BNO085 IMU
  if (!bno08x.begin_I2C()) {
    Serial.println("impossibile trovare BNO085!");
    while (1)
      ;
  }
  Serial.println("BNO085:\tOK");
  // set up report IMU
  setReports();
  // BMP390 barometro
  if (!bmp.begin_I2C()) {
    Serial.println("Impossibile trovare il bmp390!");
    while (1)
      ;
  }
  Serial.println("BMP390:\tOK");
  // set up parametri barometro
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("FINE SETUP");
  delay(1000);

  // TODO: SPOSTARE IN LOOP
  // setto altezza di partenza
  // tolgo i primi 10 valori perché sono difettati
  for (int i = 0; i < 10; i++) {
    while (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
    }
  }
  // prendo 100 misurazioni e faccio la media
  for (int i = 0; i < 100; i++) {
    while (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
    }
    altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    base_altitude += altitude_baro;
  }
  base_altitude /= 100;
  // altitudine terra
  Serial << "altitudine terra: " << base_altitude << "\n";
  delay(10000);
}

// ========================= LOOP =========================

void loop() {
  RemoteXY_Handler();
  //leggo valori da IMU

  // battery percentage
  RemoteXY.battery_percentage = calc_batt_percentage();
  // controllo che sia inserita la sd
  RemoteXY.sd_check = sd_check();
  // continuità sui pyro channel
  pyro_continuity();

  readIMU();
  // calcolo attitude con accelerometro e magnetometro
  calc_attitude_acc();
  RemoteXY.yaw = attitude_acc(0);
  RemoteXY.roll = attitude_acc(1);
  RemoteXY.pitch = attitude_acc(2);

  readBaro();

  kalman_filter_attitude();
  kalman_filter_hight();
  Serial << "g:9.8,"
         << "g_neg:-9.8,"
     //   << "acc_z_inertial:"     << acc_vert            << ","
         << "vertical_velocity:"  << v_vert               << ","
     //    << "altitude_baro:"      << altitude_baro        << ","
         << "h_baro:"             << h_baro               << ","
         << "h_kalman:"           << S_h(0,0)             << ","
         << "v_kalman:"           << S_h(1,0)             << "\n";
  //Serial << attitude_kalman << '\n';
}

// ========================= FUNZIONI =========================

void kalman_filter_attitude() {
  // prediction
  attitude_kalman = attitude_kalman + gyro * dt_gyro;

  // uncertainty
  uncertainty_attitude(0) = uncertainty_attitude(0) + pow(dt_gyro, 2) * 2 * 2;  // 4*4 è la deviazione standard del giroscopio
  uncertainty_attitude(1) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;
  uncertainty_attitude(2) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;

  // calcolo kalman gain
  kalman_gain(0) = uncertainty_attitude(0) / (uncertainty_attitude(0) + 5 * 5);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
  // new attitude
  attitude_kalman(0) = attitude_kalman(0) + kalman_gain(0) * (attitude_acc(0) - attitude_kalman(0));
  // calcolo kalman gain
  kalman_gain(1) = uncertainty_attitude(1) / (uncertainty_attitude(1) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
  // new attitude
  attitude_kalman(1) = attitude_kalman(1) + kalman_gain(1) * (attitude_acc(1) - attitude_kalman(1));
  // calcolo kalman gain
  kalman_gain(2) = uncertainty_attitude(2) / (uncertainty_attitude(2) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
  // new attitude
  attitude_kalman(2) = attitude_kalman(2) + kalman_gain(2) * (attitude_acc(2) - attitude_kalman(2));

  // update prediction with current state from accelerometer
  uncertainty_attitude(0) = (1 - kalman_gain(0)) * uncertainty_attitude(0);
  uncertainty_attitude(1) = (1 - kalman_gain(1)) * uncertainty_attitude(1);
  uncertainty_attitude(2) = (1 - kalman_gain(2)) * uncertainty_attitude(2);
}

void kalman_filter_hight() {
  // calcolo l'accelerazione sull'asse z inerziale
  acc_vert = -sin(degToRad(attitude_kalman(1))) * acc(2)
                   + cos(degToRad(attitude_kalman(1))) * sin(degToRad(attitude_kalman(2))) * acc(1)
                   + cos(degToRad(attitude_kalman(1))) * cos(degToRad(attitude_kalman(2))) * acc(0);

  // tolgo la gravità
  acc_vert = acc_vert - G_CONST;

  // calcolo la velocità verticale
  v_vert = v_vert + acc_vert * dt_acc;

  Serial << dt_acc << "\n";

  // aggiorno il tempo di campionamento nella matrice di transizione dello stato 
  A_h(0,1) = dt_acc;
  // aggiorno la matrice degli ingressi B
  B_h(0) = 0.5 * pow(dt_acc,2);
  B_h(1) = dt_acc;
  // prediction altitude velocity
  S_h = A_h * S_h + B_h * acc_vert;
  // uncertainty
  U_h = A_h * U_h * ~A_h + (B_h * ~B_h) * std_dev_acc;
  // calcolo il guadagno di kalman
  K_h = U_h * ~C_h * Inverse(C_h * U_h * ~C_h + std_dev_baro);
  // nuovo stato
  M = {h_baro};
  S_h = S_h + K_h * (M - C_h * S_h);
  // aggiorno la uncertainty
  U_h = (I - K_h * C_h) * U_h;

}

void pyro_continuity() {
  float pyro_1, pyro_2, pyro_3;  //, pyro_4;
  pyro_1 = analogRead(PYRO_1) / 8191. * 0.8;
  pyro_2 = analogRead(PYRO_2) / 8191. * 0.8;
  pyro_3 = analogRead(PYRO_3) / 8191. * 0.8;
  //pyro_4 = analogRead(PYRO_4) / 8191. * 0.8;
  RemoteXY.pyro_1_continuity = pyro_1 > 0.04;
  RemoteXY.pyro_2_continuity = pyro_2 > 0.04;
  RemoteXY.pyro_3_continuity = pyro_3 > 0.04;
  //RemoteXY.pyro_4_continuity = pyro_4 > 0.04;
}

// controllo che la sd sia inserita
bool sd_check() {
  return digitalRead(DET_SD) == HIGH;
}

// lettura percentuale batteria
int calc_batt_percentage() {
  double voltage = 0;
  for (int i = 0; i < 10; i++) {
    voltage += analogRead(VOLTAGE_PIN) / 8191. * 0.8;
  }
  voltage /= 100;
  return (int)((voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100));
}

void readBaro() {
  while (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  h_baro = altitude_baro - base_altitude;
}

void readIMU() {
  if (bno08x.wasReset()) {
    Serial.println("bno085 è stato resettato");
    setReports();
  }

  // prendo un valore dall'imu
  while (!bno08x.getSensorEvent(&sensorValueIMU)) ;
  
  long int mill;
  switch (sensorValueIMU.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc(0) = -sensorValueIMU.un.rawAccelerometer.y / 65535. * 156.96;
      acc(1) = sensorValueIMU.un.rawAccelerometer.x / 65535. * 156.96;
      acc(2) = sensorValueIMU.un.rawAccelerometer.z / 65535. * 156.96;
      mill = millis();
      if (last_sample_acc != 0) {
        dt_acc = (mill - last_sample_acc) * 0.001;
      }
      last_sample_acc = mill;
      cal_acc();
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag(0) = sensorValueIMU.un.magneticField.x;
      mag(1) = sensorValueIMU.un.magneticField.y;
      mag(2) = sensorValueIMU.un.magneticField.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:  // rad/s
      gyro(0) = degrees(sensorValueIMU.un.gyroscope.x);
      gyro(1) = degrees(sensorValueIMU.un.gyroscope.y);
      gyro(2) = degrees(sensorValueIMU.un.gyroscope.z);
      mill = millis();
      if (last_sample_gyro != 0) {
        dt_gyro = (mill - last_sample_gyro) * 0.001;
      }
      last_sample_gyro = mill;
      break;
  }
}

void calc_attitude_acc() {
  attitude_acc(1) = atan2(-acc(2), sqrt(pow(acc(1), 2) + pow(acc(0), 2)));
  attitude_acc(2) = atan2(acc(1), -acc(0));
  attitude_acc(0) = atan2(mag(0) * sin(attitude_acc(1)) - mag(1) * cos(attitude_acc(1)), mag(2) * cos(attitude_acc(2)) + sin(attitude_acc(2)) * (mag(1) * sin(attitude_acc(1)) + mag(0) * cos(attitude_acc(1))));
  attitude_acc(0) = degrees(attitude_acc(0));
  attitude_acc(1) = degrees(attitude_acc(1));
  attitude_acc(2) = degrees(attitude_acc(2));
}

// calibrazione accelerometro c'=u'*A'+b' se traspongo c = A*u + b con u e c e b con dim [3]x[1]
// u (uncal) = c (cal) = a (acc) -> acc = A * acc + b
void cal_acc() {
  acc = A_cal_acc * acc + b_cal_acc;
}

void setReports(void) {
  Serial.println("setReports");
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