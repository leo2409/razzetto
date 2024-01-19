#include <math.h>
#include <string>
#include <iostream>
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

//colori led
#define ledbianco() ()(digitalWrite(RGB_BUILTIN, HIGH))  // Turn the RGB LED white
#define ledspento() (digitalWrite(RGB_BUILTIN, LOW))   // Turn the RGB LED off
#define ledrosso()  (neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0)) // Red
#define ledverde()  (neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0)) // Green
#define ledblue()  (neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS)) // Blue
#define ledazzurro() (neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,RGB_BRIGHTNESS)) // Azzurro
#define ledviola()   (neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,RGB_BRIGHTNESS)) // Viola
#define ledgiallo()  (neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,0)) // Giallo
#define ledoffblack() (neopixelWrite(RGB_BUILTIN,0,0,0)) // Off / black

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
#define ACCTRASHOLD 20

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG  

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "Razzetto"
#define REMOTEXY_WIFI_PASSWORD "nimbus2024"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD "nimbus2024"


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 355 bytes
  { 255,5,0,43,0,92,1,16,31,1,70,16,6,8,7,7,1,121,0,70,
  16,22,8,7,7,1,121,0,129,0,3,3,13,3,8,77,73,67,82,79,
  45,83,68,0,129,0,21,3,9,3,8,80,89,82,79,32,49,0,129,0,
  35,3,9,3,8,80,89,82,79,32,50,0,129,0,48,3,9,3,8,80,
  89,82,79,32,51,0,70,16,36,8,7,7,1,121,0,70,16,49,8,7,
  7,1,121,0,66,129,5,22,15,4,2,26,129,0,10,18,13,3,8,66,
  65,84,84,69,82,89,0,71,56,2,34,19,19,0,2,24,255,0,0,52,
  195,0,0,52,67,0,0,52,66,0,0,32,65,0,0,160,64,24,0,71,
  56,42,34,19,19,0,2,24,255,0,0,52,195,0,0,52,67,0,0,52,
  66,0,0,32,65,0,0,160,64,24,0,71,56,22,34,19,19,0,2,24,
  255,0,0,52,195,0,0,52,67,0,0,52,66,0,0,32,65,0,0,160,
  64,24,0,129,0,7,30,8,3,8,80,73,84,67,72,0,129,0,28,30,
  7,3,8,82,79,76,76,0,129,0,48,30,6,3,8,89,65,87,0,68,
  17,6,54,53,30,8,36,67,4,21,22,10,5,2,31,11,7,44,37,22,
  23,4,2,26,2,3,129,0,41,18,14,3,8,83,69,65,32,76,69,86,
  69,76,0,129,0,45,90,8,3,31,83,84,65,82,84,0,10,48,46,86,
  12,12,4,1,31,79,78,0,31,79,70,70,0,129,0,36,91,8,3,24,
  83,84,65,82,84,0,67,4,7,89,20,5,2,26,11 };


// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  float sea_level;
  uint8_t start; // =1 if state is ON, else =0 

    // output variables
  uint8_t sd_check; // led state 0 .. 1 
  uint8_t pyro_1_continuity; // led state 0 .. 1 
  uint8_t pyro_2_continuity; // led state 0 .. 1 
  uint8_t pyro_4_continuity; // led state 0 .. 1 
  int8_t battery_percentage; // =0..100 level position 
  float pitch;  // from -180 to 180 
  float yaw;  // from -180 to 180 
  float roll;  // from -180 to 180 
  float height;
  char battery_percentage_string[11];  // string UTF8 end zero 
  char errore[30];  // string UTF8 end zero 


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
int stato = 1;

// FILE NELLA FLASH
File file_flash;
int bytesWritten_flash=0;
char* linea = (char *)malloc(1000 * sizeof(char));

File log_flash;
int bytesWritten_log=0;

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
float sealevelpressure=0;
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
float h_kalman;
float v_kalman;


long int last_sample_gyro = 0;
float dt_gyro = 0;
long int last_sample_acc = 0;
float dt_acc = 0;

// variabili che indicano se nella corrente iterazione del loop sono arrivati dei campioni dei sensori indicati
bool new_acc = false;
bool new_gyro = false;
bool new_mag = false;
bool new_baro = false;
unsigned long t_accensionemotore=0;


// matrici calibrazione accelerometro
BLA::Matrix<3, 3> A_cal_acc = { -0.9991, 0.0176, 0.0148,
                                -0.0056, -1.0099, 0.0233,
                                -0.0104, 0.0411, -1.0033 };
BLA::Matrix<3> b_cal_acc = { -0.1896, 0.1196, -0.2021 };  // vettore colonna*/

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);
  delay(3000);
  Serial.println("RAZZETTO");

  //Settaggio di connessione secondo il sito
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
  analogSetAttenuation(ADC_0db); // lettura voltaggi da 0.0V - 0.8V
  Serial.println("ADC: OK");

  // I2C SET-UP
  Wire.begin();
  Wire.setClock(3400000); // ottimo trovato con 10 ore di lavoro

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
  bytesWritten_flash += file_flash.println("v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z");
  if (!file_flash) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1)
      ;
  }

  log_flash= SPIFFS.open("/log_flash.csv", "w");
  bytesWritten_log += file_flash.println("Eventi importanti");
  if (!log_flash) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1);
  }
  
  // INIZIALIZZO SENSORI
  // BNO085 IMU
  if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("impossibile trovare BNO085!");
    while (1) ;
  }
  Serial.println("BNO085:\tOK");
  // set up report IMU
  setReports();
  // BMP390 barometro
  if (!bmp.begin_I2C(119, &Wire)) {
    Serial.println("Impossibile trovare il bmp390!");
    while (1) ;
  }
  Serial.println("BMP390:\tOK");
  // set up parametri barometro
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

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
  // altitudine terra
  Serial << "altitudine terra: " << base_altitude << "\n";
  delay(10000);
  
}



void setReports(void) {
  RemoteXY.battery_percentage = 0;
  Serial.println("setReports");
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER, 4000)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable raw magnetometer");
  }
}

// parametri utili per i sensori e il conteggio dei campioni
int inizio = 0;
int n_campioni_acc = 0;
int n_campioni_gyro = 0;
int n_campioni_magn = 0;
int n_campioni_baro = 0;
int last_sample_bmp = 0;
char stampa[100];

// ========================= FUNZIONI =========================
// stimo lo stato del razzo (orientazione, velocità verticale e altezza dal suolo) 
void stima_stato_razzo() {
  readIMU();
  readBaro();
  if (new_gyro) {
    // predizione dell'orientazione
    attitude_kalman = attitude_kalman + gyro * dt_gyro;
    // aggiorno incertezza sulla predizione
    uncertainty_attitude(0) = uncertainty_attitude(0) + pow(dt_gyro, 2) * 2 * 2;  // 4*4 è la deviazione standard del giroscopio
    uncertainty_attitude(1) = uncertainty_attitude(1) + pow(dt_gyro, 2) * 2 * 2;
    uncertainty_attitude(2) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;
  }

  if (new_acc) {
    calc_attitude_acc();
    // calcolo kalman gain x
    kalman_gain(0) = uncertainty_attitude(0) / (uncertainty_attitude(0) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(0) = attitude_kalman(0) + kalman_gain(0) * (attitude_acc(0) - attitude_kalman(0));
    // calcolo kalman gain y
    kalman_gain(1) = uncertainty_attitude(1) / (uncertainty_attitude(1) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(1) = attitude_kalman(1) + kalman_gain(1) * (attitude_acc(1) - attitude_kalman(1));
    // calcolo kalman gain z
    kalman_gain(2) = uncertainty_attitude(2) / (uncertainty_attitude(2) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(2) = attitude_kalman(2) + kalman_gain(2) * (attitude_acc(2) - attitude_kalman(2));

    // aggiorno incertezza sulla stima stima
    uncertainty_attitude(0) = (1 - kalman_gain(0)) * uncertainty_attitude(0);
    uncertainty_attitude(1) = (1 - kalman_gain(1)) * uncertainty_attitude(1);
    uncertainty_attitude(2) = (1 - kalman_gain(2)) * uncertainty_attitude(2);

    // calcolo acc vert
    calc_acc_vert();
    // aggiorno il tempo di campionamento nella matrice di transizione dello stato 
    A_h(0,1) = dt_acc;
    // aggiorno la matrice degli ingressi B
    B_h(0) = 0.5 * pow(dt_acc,2);
    B_h(1) = dt_acc;
    // predizione sulla velocità e sull'altezza
    S_h = A_h * S_h + B_h * acc_vert;
    // aggiorno incertezza sulla predizione
    U_h = A_h * U_h * ~A_h + (B_h * ~B_h) * std_dev_acc;
    }
    
  if (new_baro) {
    // faccio update sulla stima velocità verticale e altezza
    // update incertezza
    // calcolo il guadagno di kalman
    K_h = U_h * ~C_h * Inverse(C_h * U_h * ~C_h + std_dev_baro);
    // M è la misura sull'altezza
    M = {h_baro};
    // aggiorno la stima
    S_h = S_h + K_h * (M - C_h * S_h);
    // aggiorno l'incertezza sulla stima
    U_h = (I - K_h * C_h) * U_h;
  }
  h_kalman=S_h(0,0);
  v_kalman=S_h(1,0);
}

void calc_acc_vert() {
  // calcolo l'accelerazione sull'asse z inerziale
  acc_vert =  + sin(degToRad(attitude_kalman(1))) * acc(2)
              - cos(degToRad(attitude_kalman(1))) * sin(degToRad(attitude_kalman(2))) * acc(1)
              + cos(degToRad(attitude_kalman(1))) * cos(degToRad(attitude_kalman(2))) * acc(0);
  
  // tolgo la gravità
  acc_vert = acc_vert - 9.81;
}

void readBaro() {
  new_baro = false;
  if (last_sample_bmp == 0 || millis() - last_sample_bmp > 10) {
    if(sealevelpressure==0){
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }else{
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    n_campioni_baro++;
    new_baro = true;
    last_sample_bmp = millis();
  }
  h_baro = altitude_baro - base_altitude;
}


void setGroundAltitude(){
  ledgiallo();
  for (int i = 0; i < 1000; i++) {
    readBaro();
    base_altitude += altitude_baro;
  }
  base_altitude /= 1000;
  ledspento();
}

void readIMU() {
  new_acc = false;
  new_gyro = false;
  new_mag = false;

  if (bno08x.wasReset()) {
    Serial.println("bno085 è stato resettato");
    setReports();
  }
  
  // prendo un valore dall'imu
  while (!bno08x.getSensorEvent(&sensorValueIMU)) ;
  //Serial.print("while readImu: "); Serial.println(millis() - t2);
  
  long int mill;
  switch (sensorValueIMU.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc(0) = sensorValueIMU.un.rawAccelerometer.y / 65535. * 156.96;
      acc(1) = -sensorValueIMU.un.rawAccelerometer.x / 65535. * 156.96;
      acc(2) = -sensorValueIMU.un.rawAccelerometer.z / 65535. * 156.96;
      
      mill = millis();
      if (last_sample_acc != 0) {
        dt_acc = (mill - last_sample_acc) * 0.001;
      }
      last_sample_acc = mill;
      //cal_acc();
      
      n_campioni_acc++;
      new_acc = true;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag(0) = sensorValueIMU.un.magneticField.x;
      mag(1) = sensorValueIMU.un.magneticField.y;
      mag(2) = sensorValueIMU.un.magneticField.z;
      n_campioni_magn++;
      new_mag = true;
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
      
      n_campioni_gyro++;
      new_gyro = true;
      break;
  }
}

// calibrazione accelerometro c'=u'*A'+b' se traspongo c = A*u + b con u e c e b con dim [3]x[1]
// u (uncal) = c (cal) = a (acc) -> acc = A * acc + b
void cal_acc() {
  acc = A_cal_acc * acc + b_cal_acc;
}

void calc_attitude_acc() {
  attitude_acc(1) = atan2(acc(2), sqrt(pow(acc(1), 2) + pow(acc(0), 2)));
  attitude_acc(2) = atan2(-acc(1), acc(0));
  attitude_acc(0) = atan2(mag(0) * sin(attitude_acc(1)) - mag(1) * cos(attitude_acc(1)), mag(2) * cos(attitude_acc(2)) + sin(attitude_acc(2)) * (mag(1) * sin(attitude_acc(1)) + mag(0) * cos(attitude_acc(1))));
  attitude_acc(0) = degrees(attitude_acc(0));
  attitude_acc(1) = degrees(attitude_acc(1));
  attitude_acc(2) = degrees(attitude_acc(2));
}

void print_su_flash() {
  // tempo, v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
  long unsigned int t= millis();
  sprintf(linea, "%lu, %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", t, v_kalman, h_kalman, attitude_kalman(0), attitude_kalman(1), attitude_kalman(2), gyro(0), gyro(1), gyro(2), h_baro, acc(0), acc(1), acc(2), mag(0), mag(1),mag(2));
  Serial.println(linea);
  bytesWritten_flash += file_flash.println(linea);
}


// ======== kalman ==========

/*
void kalman_filter_attitude() {
  // prediction
  attitude_kalman = attitude_kalman + gyro * dt_gyro;

  // uncertainty
  uncertainty_attitude(0) = uncertainty_attitude(0) + pow(dt_gyro, 2) * 2 * 2;  // 4*4 è la deviazione standard del giroscopio
  uncertainty_attitude(1) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;
  uncertainty_attitude(2) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;

  // calcolo kalman gain
  kalman_gain(0) = uncertainty_attitude(0) / (uncertainty_attitude(0) + 3 * 3);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
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

  //Serial << dt_acc << "\n";

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
*/

// ============= funzioni utili negli stati ============

void updateUI(){
  RemoteXY_Handler();
  // battery percentage
  calc_batt_percentage();
  sprintf(RemoteXY.battery_percentage_string,"%d", RemoteXY.battery_percentage);
  //continuità micce
  pyro_continuity();
  //controllo sd_check;
  sd_check();
  RemoteXY.roll=attitude_kalman(0);
  RemoteXY.pitch=attitude_kalman(1);
  RemoteXY.yaw=attitude_kalman(2);
}

float mod(BLA::Matrix<3> a){
  float a0=pow(a(0),2);
  float a1=pow(a(1),2);
  float a2=pow(a(2),2);
  return sqrt(a0+a1+a2);
}

void pyro_continuity() {
  float pyro_1, pyro_2, pyro_4;  //, pyro_4;
  pyro_1 = analogRead(PYRO_1) / 8191. * 0.8;
  pyro_2 = analogRead(PYRO_2) / 8191. * 0.8;
  pyro_4 = analogRead(PYRO_4) / 8191. * 0.8;
  RemoteXY.pyro_1_continuity = pyro_1 > 0.04;
  RemoteXY.pyro_2_continuity = pyro_2 > 0.04;
  RemoteXY.pyro_4_continuity = pyro_4 > 0.04;
}

// controllo che la sd sia inserita
void sd_check() {
  RemoteXY.sd_check=digitalRead(DET_SD) == HIGH;
}

// lettura percentuale batteria
void calc_batt_percentage() {
  double voltage = 0;
  for (int i = 0; i < 10; i++) {
    voltage += analogRead(VOLTAGE_PIN) / 8191. * 0.8;
  }
  voltage /= 10;
  RemoteXY.battery_percentage=(int)((voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100));
}


// ============ LOOP ============

void loop() {
  if (inizio == 0) inizio = millis();
  else if (millis() - inizio > 100000){
    
    Serial.print("numero di campioni acc: "); Serial.println(n_campioni_acc);
    Serial.print("hz acc: "); Serial.println(n_campioni_acc/100);
    Serial.print("numero di campioni gyro: "); Serial.println(n_campioni_gyro);
    Serial.print("hz gyro: "); Serial.println(n_campioni_gyro/100);
    Serial.print("numero di campioni magn: "); Serial.println(n_campioni_magn);
    Serial.print("hz magn: "); Serial.println(n_campioni_magn/100);
    Serial.print("numero di campioni baro: "); Serial.println(n_campioni_baro);
    Serial.print("hz baro: "); Serial.println(n_campioni_baro/100);
    delay(10000);
    
    inizio = millis();
    n_campioni_acc = 0;
    n_campioni_gyro = 0;
    n_campioni_magn = 0;
    n_campioni_baro = 0;
    last_sample_bmp = 0;
  }
  stima_stato_razzo();
  RemoteXY.yaw = attitude_acc(0);
  RemoteXY.roll = attitude_acc(1);
  RemoteXY.pitch = attitude_acc(2);

  Serial << "g:2,"
         << "g_neg:-2,"
     //    << "acc_x:" << acc(0) << ","
     //    << "acc_y:" << acc(1) << ","
     //    << "acc_z:" << acc(2) << ","
     //      << "gyro_x:" << gyro(0) << ","
     //      << "gyro_y:" << gyro(1) << ","
     //      << "gyro_z:" << gyro(2) << ","
     //     << "att_x_acc:" << attitude_acc(0) << ","
     //     << "att_y_acc:" << attitude_acc(1) << ","
     //     << "att_z_acc:" << attitude_acc(2) << ","
     //     << "att_x_kalman:" << attitude_kalman(0) << ","
     //     << "att_y_kalman:" << attitude_kalman(1) << ","
     //     << "att_z_kalman:" << attitude_kalman(2) << ","
            << "acc_vert:"     << acc_vert            << ","
     //    << "altitude_baro:"      << altitude_baro        << ","
            << "h_baro:"             << h_baro               << ","
            << "h_kalman:"           << h_kalman             << ","
            << "v_kalman:"           << v_kalman             << "\n";
  //Serial << attitude_kalman << '\n';*/
  
  //STATI
  Serial.println("Switch");
  Serial.println(stato);
  switch(stato){
    case 1:{
    Serial.println("Case 1");
    //instaurazione connessione utente-razzo set_up
    //Aggiornamento interfaccia
    updateUI();
    //if connessione stabilita {stato=2;}
    if(RemoteXY_isConnected()){
      stato=2;
    }
    }break;
    case 2:{
    Serial.println("Case 2");
    //Aggiornamento interfaccia utente 
    updateUI();
    //if passaggio pressione al livello del mare{stato=3;}
    if(RemoteXY.sea_level){
      stato=3;
      tone(BUZZER_PIN, 2000, 500);
    }
    }break;
    case 3:{
    //Inizio Calcolo Assetto e altezza da terra, azione che continua per tutta la durata del volo
    stima_stato_razzo();
    //Aggiornamento interfaccia utente 
    updateUI();
    //Inizializzazione del ground
    // TODO: FARE UNA MEDIA
    sealevelpressure= RemoteXY.sea_level;
    setGroundAltitude();
    //Feedback positivo dell'utente per partite{stato=4;}
    if(RemoteXY.start){
      stato=4;
      tone(BUZZER_PIN, 3000, 1000);
    }
    }break; 
    case 4:{ //Controlli preparatori
    //Aggiornamento interfaccia utente 
    updateUI();
    char* s = (char *)malloc(100 * sizeof(char));
    bool controllo=true;
    //Controllo livello batteria LIPO tramite voltometro
    if(RemoteXY.battery_percentage<=30){
      //Errore batteria scarica
      strcat(s, "KO Batteria ");
      controllo=false;
    }
    //Controllo SD
    if(!RemoteXY.sd_check){
      //Errore SD non inserita
      strcat(s,"NO SD ");
      controllo=false;
    }
    //Corpo totalmente fermo: Accelerazione gravitazionale su un solo asse
    stima_stato_razzo();
    if(v_kalman<=-0.2 || v_kalman>=0.2){
      //ERRORE corpo non totalmente fermo
      strcat(s, "NO fermo ");
      controllo=false;
    }
    //Controllo angoli Roll e Pitch con valore 0 , accelerometro e giroscopio
    if(RemoteXY.yaw>=30 || RemoteXY.yaw<=-30 || RemoteXY.pitch<=-30 || RemoteXY.pitch>=30){
      //ERRORE Roll e/o Pitch non sono a zero
      strcat(s, "NO Posizione ");
      controllo=false;
    }
    //Controllo della continuità sulle micce 
    if(RemoteXY.pyro_1_continuity==0 || RemoteXY.pyro_2_continuity==0 || RemoteXY.pyro_4_continuity==0){            
      //ERRORE Non continuità delle micce
      strcat(s, "NO continuità");
      controllo=false;
    }
    //strcpy(RemoteXY.errore, s);
    if(controllo){
      stato=5;
    } 
    }break;
    case 5:{
    //stima razzo
    stima_stato_razzo();
    //raccolta dati(S):
    file_flash.println();
    // count down con aggiorno interfaccia
    for(int i=0; i<9; i++){
      int n=i/2+100;
      tone(BUZZER_PIN, n, 100);
    }
    //Interfaccia utente R
    updateUI();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    // accensione motore
    pinMode(PYRO_4, OUTPUT);
    ledrosso();
    //accensione motore
    t_accensionemotore=millis();//quando acceso motore
    // log provo ad accendere il motore
    bytesWritten_log+= log_flash.println(t_accensionemotore+"provo ad accendere il motore");
    bool acceso = false;
    digitalWrite(PYRO_4, HIGH);
    while (millis() - t_accensionemotore < 2000 && !acceso) {
      stima_stato_razzo();
      if (acc(0)>=ACCTRASHOLD){
        acceso=true;
      }
    }
    digitalWrite(PYRO_4, LOW);
    tone(BUZZER_PIN, 0);

    //accensione corretta FA DEI TEST
    stima_stato_razzo();
    if(acceso || acc(0)>=ACCTRASHOLD){
      bytesWritten_log+= log_flash.println(millis()+"accensione corretta del motore");
      stato=6;
    }else if(millis()-t_accensionemotore>3000){
      bytesWritten_log+= log_flash.println(millis()+"accensione non corretta del motore");
      stato=11;
    }
    }break; /*
    case 6:{ 
  
    //Controllo pressione tramite il barometro (convertito in un’altezza approssimativa)
    //Controllo accensione motore (registrazione del t in cui si spegne)
    //Controllo angoli Roll e Pitch imponendo dei valori massimi di oscillazione (y e z) con Logg
    //unione delle misure con un algoritmo di sensor fusion
    kalman_filter_hight();
    //condizione: if( (h(t-1)-h(t)<0 && deltaT>100) || 6.T>t){stato=7;}
    if(h0-h1<0 && deltaT>100){
      stato=7;
    }
    }break;
    case 7:
    //Accensione prima carica di espulsione
    if(7.T>1){stato=8;
    }else if(|a|=9.81 || h>10){stato=9;}
    break;
    case 8:{
    //Accensione seconda carica di espulsione
    // if(|a|=9.81 || h>10){stato=9;}
    }break;
    case 9:{
    //Logging apertura paracadute 
    //if(|a|=9.81 && h<2.5){stato=10;}
    }break;
    case 10:{
    //Raccolta dati(R)
    //trasferimento di dati su flash su SD
    //segnale acustico
    }break;
    case 11:{
    //si accende segnale acustico
    }break;
   */ 
  }
}