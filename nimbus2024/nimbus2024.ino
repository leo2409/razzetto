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
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SPIFFS.h"
#include <FS.h>
#include <SPIFFS.h>
// preference per variabili su flash
#include <Preferences.h>
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
#define VOLTAGE_PIN 1


#define SEALEVELPRESSURE_HPA (1013.25)
// valori specifici della configurazione e del lancio (razzo e motore)
#define ACCTRESHOLD 15
#define ACC_SEPARAZIONE_CONO 20
#define DURATA_MOTORE 0
#define MAX_DURATA_FLIGHT 6800


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


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 355 bytes
  { 255,5,0,133,0,92,1,16,31,1,70,16,6,8,7,7,1,121,0,70,
  16,22,8,7,7,1,121,0,129,0,3,3,13,3,8,77,73,67,82,79,
  45,83,68,0,129,0,21,3,9,3,8,80,89,82,79,32,49,0,129,0,
  35,3,9,3,8,80,89,82,79,32,50,0,129,0,48,3,9,3,8,80,
  89,82,79,32,52,0,70,16,36,8,7,7,1,121,0,70,16,49,8,7,
  7,1,121,0,66,129,5,21,15,4,2,26,129,0,10,17,13,3,8,66,
  65,84,84,69,82,89,0,71,56,2,33,19,19,0,2,24,255,0,0,52,
  195,0,0,52,67,0,0,52,66,0,0,32,65,0,0,160,64,24,0,71,
  56,42,33,19,19,0,2,24,255,0,0,52,195,0,0,52,67,0,0,52,
  66,0,0,32,65,0,0,160,64,24,0,71,56,22,33,19,19,0,2,24,
  255,0,0,52,195,0,0,52,67,0,0,52,66,0,0,32,65,0,0,160,
  64,24,0,129,0,7,29,8,3,8,80,73,84,67,72,0,129,0,28,29,
  7,3,8,82,79,76,76,0,129,0,48,29,6,3,8,89,65,87,0,68,
  17,6,53,53,30,8,36,67,4,21,21,10,5,2,31,11,7,44,37,21,
  23,4,2,26,2,3,129,0,41,17,14,3,8,83,69,65,32,76,69,86,
  69,76,0,129,0,45,90,8,3,31,83,84,65,82,84,0,10,48,26,89,
  10,10,4,1,31,79,78,0,31,79,70,70,0,129,0,37,92,8,3,24,
  83,84,65,82,84,0,67,4,6,84,53,4,2,26,101 };
  
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
  char errore[101];  // string UTF8 end zero 

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

//stringa caso 4
char s[100] = "";

// FILE NELLA FLASH
File file_flash;
int bytesWritten_flash=0;
char* linea = (char *)malloc(1000 * sizeof(char));

File file_log;
int bytesWritten_log=0;

//FILE SD
File flash_sd;
File log_sd;

// numero del lancio
Preferences preferences;
unsigned int n_flight;


// vettori di stato
// accelerometro vettore colonna
BLA::Matrix<3> acc = { 0, 0, 0 };
// accelerazione verticale
float acc_vert = 0;
// velocità verticale e altezza
float base_altitude = 0;
float altitude_baro = 0;
float h_baro = 0;
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
float std_dev_acc_attitude = pow(3,2); // deviazione standard acc su attitude
float std_dev_mag_attitude = pow(5,2); // deviazione standard mag su attitude
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
float h_kalman = 0;
float v_kalman = 0;


long int last_sample_gyro = 0;
float dt_gyro = 0;
long int last_sample_acc = 0;
float dt_acc = 0;
long int last_sample_bmp = 0;

// variabili che indicano se nella corrente iterazione del loop sono arrivati dei campioni dei sensori indicati
bool new_acc = false;
bool new_gyro = false;
bool new_mag = false;
bool new_baro = false;
unsigned long t_accensionemotore=0;
unsigned long int t6;
unsigned long int t7;
unsigned long int t8;
unsigned long int t9;


// matrici calibrazione accelerometro
BLA::Matrix<3, 3> A_cal_acc = { 1.0084 ,  0.0111,   -0.0143,
                                0.0119,   0.9996,   0.0145,
                                0.0028,   0.0139,   1.0010 };
BLA::Matrix<3> b_cal_acc = { 0.1498,   0.1510,   0.1626 };

typedef struct {
  long unsigned int time_millis;
  float attitude_x;
  float attitude_y;
  float attitude_z;
  float h_kalman;
  float v_kalman;
  float acc_x;
  float acc_vert;
  float h_baro;
} riga_flash;

long unsigned int last_print_flash = 0;

riga_flash riga_nuova;

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
  Wire.setClock(350000);

  // PREFERENCE
  // preference per variabili su flash
  if (!preferences.begin("my-app", false)) {
    Serial.println("Errore nel begin di preference");
    while (1)
      ;
  }
  n_flight = preferences.getUInt("n_flight", 0);
  Serial.println("PREFERENCES:\tOK");
  Serial.print("numero di flight: ");
  Serial.println(n_flight);
  // FILESISTEM
  if (SPIFFS.begin()) {
    Serial.println("SPIFFS:\tOK");
  } else {
    Serial.println("impossibile montare il file system SPIFFS flash");
    while (1)
      ;
  }
  // list dei file sulla flash
  list_file_flash();
  // se ci sta qualcosa sposto tutto su sd 
  if (move_file_flash_sd("/dati_vecchi")) {
    // se riesco a spostare i file elimino tutti i file sulla flash
    remove_file_flash();
  }

  // apertura file flash
  file_flash = SPIFFS.open(String("/dati_flash_binario_") + n_flight, "w");
  if (!file_flash) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1)
      ;
  }
  file_flash.setBufferSize(800);

  file_log = SPIFFS.open(String("/file_log_") + n_flight + String(".txt"), "w");
  if (!file_log) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1);
  }
  file_flash.setBufferSize(1000);
  bytesWritten_log += file_log.println("Eventi importanti");
  
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
  delay(5000);
  
}



void setReports(void) {
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

// ================== FILESYSTEM ===========================

void list_file_flash() {
  // list dei file
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  Serial.println("list dei file su flash: ");
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());

    file = root.openNextFile();
  }
  Serial.println();
  file.close();
  root.close();
}

void traduci_file_dati_volo(File file_flash_binario) {
  ledgiallo();
  Serial.println("inizio traduzione dati di volo");
  File file_flash_tradotto = SPIFFS.open(String("/dati_flash_") + n_flight + String(".csv"), "w");
  if (!file_flash_tradotto) {
    Serial.println("Errore aprendo il file tradotto dati_flash in scrittura");
    while (1)
      ;
  }

  file_flash_tradotto.println("time_millis, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, h_kalman, v_kalman, acc_x, acc_vert, h_baro");
  Serial.print("riapro il file binario: ");
  String nome_file = String("/") + file_flash.name();
  Serial.println(nome_file);
  file_flash_binario.close();
  file_flash_binario = SPIFFS.open(nome_file);
  if (!file_flash_tradotto) {
    Serial.println("Errore aprendo il file dati_flash_binario in lettura");
    while (1)
      ;
  }
  while (file_flash_binario.available()) {
    // leggo una riga
    file_flash_binario.read((uint8_t*) &riga_nuova, sizeof(riga_flash));
    // impagino la riga
    sprintf(linea, "%lu,%f,%f,%f,%f,%f,%f,%f,%f\n",
        riga_nuova.time_millis,
        riga_nuova.attitude_x,
        riga_nuova.attitude_y,
        riga_nuova.attitude_z,
        riga_nuova.h_kalman,
        riga_nuova.v_kalman,
        riga_nuova.acc_x,
        riga_nuova.acc_vert,
        riga_nuova.h_baro);
    file_flash_tradotto.print(linea);
  }
  file_flash_tradotto.close();
  file_flash_binario.close();
  ledoffblack();
  Serial.println("fine traduzione dati di volo");
}

bool move_file_flash_sd(String dir) {
  ledgiallo();
  // mount della sd
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return false;
  }
  // creazione della dir se non esiste già
  SD.mkdir(dir);
  // list dei file
  File root_flash = SPIFFS.open("/");
  File file = root_flash.openNextFile();
  while (file) {
    Serial.print("sposto il file: ");
    Serial.println(file.name());
    // apro un file su sd con lo stesso nome del file su flash
    String file_path = file.name();
    file_path = dir + "/" + file_path;
    // nome del file sulla sd
    Serial.print("creazione file: ");
    Serial.println(file_path);
    // creazione file su sd
    File file_sd = SD.open(file_path, FILE_WRITE);
    // controllo che si sia aperto
    if (!file_sd) {
      Serial.println("Impossibile aprire il file su sd in scrittura");
      while (1)
        ;
    }
    Serial.println("inizio a spostare il file sulla sd");
    String riga_da_spostare = "";
    while (file.available()) {
      riga_da_spostare = file.readStringUntil('\n');
      file_sd.println(riga_da_spostare);
    }
    // chiudo file sd
    file_sd.close();
    // chiudo file su flash
    file.close();
    // passo al file successivo
    file = root_flash.openNextFile();
  }
  root_flash.close();
  ledspento();
  return true;
}

void remove_file_flash() {
  Serial.println("remove");
  ledgiallo();
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("elimino il file: ");
    Serial.println("/" + String(file.name()));
    // elimino il file
    SPIFFS.remove("/" + String(file.name()));
    // passo al file successivo
    file.close();
    file = root.openNextFile();
  }
  root.close();
  ledspento();
}


void print_su_flash() {
  if (millis() - last_print_flash > 9) {
    riga_nuova.time_millis = millis();
    riga_nuova.attitude_x = attitude_kalman(0);
    riga_nuova.attitude_y = attitude_kalman(1);
    riga_nuova.attitude_z = attitude_kalman(2);
    riga_nuova.h_kalman = h_kalman;
    riga_nuova.v_kalman= v_kalman;
    riga_nuova.acc_x = acc(0);
    riga_nuova.acc_vert = acc_vert;
    riga_nuova.h_baro = h_baro;

    file_flash.write((uint8_t*) &riga_nuova, sizeof(riga_flash));
    // aggiorno il tempo dell'ultima scrittura
    last_print_flash = millis();
  }
}

void log_flash(String log) {
  log = String(millis()) + String(": ") + log + String(" altezza: ") + String(h_kalman)+ String("\n");
  bytesWritten_log += file_log.print(log);
}

// ========================= KALMAN =========================
// stimo lo stato del razzo (orientazione, velocità verticale e altezza dal suolo) 
void stima_stato_razzo() {
  readIMU();
  readBaro();
  if (new_gyro) {
    // predizione dell'orientazione
    attitude_kalman = attitude_kalman + gyro * dt_gyro;
    // aggiorno incertezza sulla predizione
    uncertainty_attitude(0) = uncertainty_attitude(0) + pow(dt_gyro, 2) * 2 * 2;  // 2*2 è la deviazione standard del giroscopio
    uncertainty_attitude(1) = uncertainty_attitude(1) + pow(dt_gyro, 2) * 2 * 2;
    uncertainty_attitude(2) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;
  }
  // calcolo gli angoli da accelerometro e magnetometro
  calc_attitude_acc();

  if (new_acc) {
    // calcolo kalman gain y
    kalman_gain(1) = uncertainty_attitude(1) / (uncertainty_attitude(1) + std_dev_acc_attitude);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(1) = attitude_kalman(1) + kalman_gain(1) * (attitude_acc(1) - attitude_kalman(1));
    // calcolo kalman gain z
    kalman_gain(2) = uncertainty_attitude(2) / (uncertainty_attitude(2) + std_dev_acc_attitude);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(2) = attitude_kalman(2) + kalman_gain(2) * (attitude_acc(2) - attitude_kalman(2));

    // aggiorno incertezza sulla stima stima
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

  if (new_mag) {
     // calcolo kalman gain x
    kalman_gain(0) = uncertainty_attitude(0) / (uncertainty_attitude(0) + std_dev_mag_attitude);  // 3*3 è la deviazione standard dell'angolo misurato dall'accelerometro
    // update stima
    attitude_kalman(0) = attitude_kalman(0) + kalman_gain(0) * (attitude_acc(0) - attitude_kalman(0));
    // aggiorno incertezza sulla stima stima
    uncertainty_attitude(0) = (1 - kalman_gain(0)) * uncertainty_attitude(0);
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
  // Calcolo dell'accelerazione sull'asse x inerziale
  acc_vert =  + cos(degToRad(attitude_kalman(1)))*cos(degToRad(attitude_kalman(2))) * acc(0)
              - sin(degToRad(attitude_kalman(2))) * cos(degToRad(attitude_kalman(1))) * acc(1)
              +sin(degToRad(attitude_kalman(1))) * acc(2);

  // Tolgo la gravità
  acc_vert = acc_vert - 9.81;
}



void readBaro() {
  new_baro = false;
  if ( millis() - last_sample_bmp > 10) {
    if(sealevelpressure==0){
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }else{
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    new_baro = true;
    last_sample_bmp = millis();
  }
  h_baro = altitude_baro - base_altitude;
}


void setGroundAltitude() {
  ledgiallo();
  // faccio 50 letture per togliere eventuali primi errori
  for (int i = 0; i < 50; i++) {
    altitude_baro = bmp.readAltitude(sealevelpressure);
    Serial.println(altitude_baro);
  }
  // prendo come base_altitude la media su 500 misurazioni
  int n = 1000;
  for (int i = 0; i < n; i++) {
    altitude_baro = bmp.readAltitude(sealevelpressure);
    Serial.println(altitude_baro);
    base_altitude += altitude_baro;
  }
  base_altitude /= n;
  Serial.print("altezza terreno: "); Serial.println(base_altitude);
  sprintf(RemoteXY.errore, "altezza terreno: %f", base_altitude);
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
  
  long int mill;
  switch (sensorValueIMU.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc(0) = -sensorValueIMU.un.rawAccelerometer.y  / 65535. * 156.96;
      acc(1) = sensorValueIMU.un.rawAccelerometer.x  / 65535. * 156.96;
      acc(2) = sensorValueIMU.un.rawAccelerometer.z  / 65535. * 156.96;

      cal_acc();
      
      mill = millis();
      if (last_sample_acc != 0) {
        dt_acc = (mill - last_sample_acc) * 0.001;
      }
      last_sample_acc = mill;
      
      new_acc = true;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag(0) = sensorValueIMU.un.magneticField.x;
      mag(1) = sensorValueIMU.un.magneticField.y;
      mag(2) = sensorValueIMU.un.magneticField.z;
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
  if (new_acc) {
    // calcolo gli angoli di eulero di y e z
    attitude_acc(1) = atan2(acc(2), sqrt(pow(acc(1), 2) + pow(acc(0), 2)));
    attitude_acc(2) = atan2(-acc(1), acc(0));
    // converto gli angoli in gradi
    attitude_acc(1) = degrees(attitude_acc(1));
    attitude_acc(2) = degrees(attitude_acc(2));
  }
  if (new_mag) {
    // trasformo gli angoli su y e z in
    float acc_rad_y = degToRad(attitude_acc(1));
    float acc_rad_z = degToRad(attitude_acc(2));
    attitude_acc(0) = - atan2(mag(0) * sin(acc_rad_y) - mag(1) * cos(acc_rad_y), mag(2) * cos(acc_rad_z) + sin(acc_rad_z) * (mag(1) * sin(acc_rad_y) + mag(0) * cos(acc_rad_y)));
    attitude_acc(0) = degrees(attitude_acc(0));
  }
}

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
  RemoteXY.height=h_kalman;
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
  RemoteXY.sd_check = digitalRead(DET_SD) == HIGH;
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
      log_flash("si è connesso");
      stato=2;
    }
    }break;
    case 2:{
    Serial.println("Case 2");
    //Aggiornamento interfaccia utente 
    updateUI();
    //if passaggio pressione al livello del mare{stato=3;}
    if(RemoteXY.sea_level){
      log_flash("nuova pressione del mare fornita da utente");
      //Inizializzazione del ground
      sealevelpressure= RemoteXY.sea_level;
      setGroundAltitude();
      stato=3;
      tone(BUZZER_PIN, 2000, 500);
    }
    }break;
    case 3:{
    Serial.println("Caso 3");
    //Inizio Calcolo Assetto e altezza da terra, azione che continua per tutta la durata del volo
    stima_stato_razzo();
    //Aggiornamento interfaccia utente 
    updateUI();
    
    //Feedback positivo dell'utente per partite{stato=4;}
    if(RemoteXY.start){
      log_flash("Feedback positivo");
      stato=4;
      tone(BUZZER_PIN, 3000, 1000);
    }
    }break; 
    case 4:{ //Controlli preparatori
    Serial.println("Caso 4");
    //Aggiornamento interfaccia utente 
    stima_stato_razzo();
    updateUI();
    
    
    // variabile di errore
    sprintf(s," ");
    bool controllo = true;
    //Controllo livello batteria LIPO tramite voltometro
    /*
    if(RemoteXY.battery_percentage<=30){
      //Errore batteria scarica
      strcat(s, "NO Batt ");
      controllo = false;
    }
    */
    //Controllo SD
    if(!RemoteXY.sd_check){
      //Errore SD non inserita
      strcat(s,"NO SD ");
      controllo = false;
    }
    //Corpo totalmente fermo: Accelerazione gravitazionale su un solo asse
    stima_stato_razzo();
    /*
    if(v_kalman<=-0.6 || v_kalman>=0.2){
      //ERRORE corpo non totalmente fermo
      strcat(s, "NO Ferm ");
      controllo = false;
    }
    */
    //Controllo angoli Yaw e Pitch con valore minore di una soglia
    if(RemoteXY.yaw>=30 || RemoteXY.yaw<=-30 || RemoteXY.pitch<=-30 || RemoteXY.pitch>=30){
      //ERRORE Roll e/o Pitch non sono a zero
      strcat(s, "NO Pos ");
      controllo = false;
    }
    
    //Controllo della continuità sulle micce 
    if(RemoteXY.pyro_1_continuity==0 || RemoteXY.pyro_2_continuity==0 || RemoteXY.pyro_4_continuity==0){
      //ERRORE Non continuità delle micce
      strcat(s, "NO Pyro");
      controllo = false;
    }

    updateUI();
    if(controllo){ //passa tutti i controlli
      stato=5;
      log_flash("controlli passati");
    } else {
      stato=11;
      log_flash("controlli non passati");
    }
    }break; 
    case 5:{
    //stima razzo
    stima_stato_razzo();
    //raccolta dati(S):
    print_su_flash();
    // count down con aggiorno interfaccia
    for(int i=1; i<10; i++){
      int n=(i/2)*1000;
      tone(BUZZER_PIN, n, 100);
      //stima razzo
      stima_stato_razzo();
      //raccolta dati(S):
      print_su_flash();
      RemoteXY_delay(1000);
    }
    //Interfaccia utente R
    updateUI();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    // accensione motore
    pinMode(PYRO_4, OUTPUT);
    //accensione motore
    t_accensionemotore=millis();//quando acceso motore
    // log provo ad accendere il motore
    log_flash(String(" prova ad accendere il motore"));
    bool acceso = false;
    digitalWrite(PYRO_4, HIGH);
    while (!acceso && millis() - t_accensionemotore < 2000) {
      stima_stato_razzo();
      print_su_flash();
      if (acc(0) >= ACCTRESHOLD){
        acceso=true;
        t6 = millis();
      }
    }
    digitalWrite(PYRO_4, LOW);
    tone(BUZZER_PIN, 0);

    while (!acceso && millis() - t_accensionemotore < 4000) {
      stima_stato_razzo();
      print_su_flash();
      if (acc(0)>=ACCTRESHOLD){
        acceso=true;
        t6 = millis();
      }
    }

    stima_stato_razzo();
    print_su_flash();
    if(acceso){
      log_flash(String("corretta accensione del motore"));
      stato=6;
      ledblue();
    }else {
      log_flash(String("non corretta accensione del motore"));
      stato=11;
    }
    }break;
    case 6:{ 
    //Controllo pressione tramite il barometro (convertito in un’altezza approssimativa)
    //Controllo accensione motore (registrazione del t in cui si spegne)
    stima_stato_razzo();
    print_su_flash();
    //Controllo angoli Roll e Pitch imponendo dei valori massimi di oscillazione (y e z) con Logg
    //unione delle misure con un algoritmo di sensor fusion
    //condizione: apogeo però dopo spegnimento motore o tempo in torno a 6.7 secondi{stato=7;}
    if (millis() - t6 > MAX_DURATA_FLIGHT || 
      (v_kalman<=0 && millis() - t6 > DURATA_MOTORE)) {
      //tone(BUZZER_PIN, 5000, 1000);
      ledgiallo();
      log_flash(String("apogeo"));
      stima_stato_razzo();
      //raccolta dati:
      print_su_flash();
      stato=7;
      }
    }break;
    case 7:{
    //Accensione prima carica di espulsione
    bool carica1 = false;
    tone(BUZZER_PIN, 2000);
    t7 = millis();
    log_flash("accensione prima miccia");
    pinMode(PYRO_1, OUTPUT);
    digitalWrite(PYRO_1, HIGH);
    while (carica1 || millis() - t7 < 800) {
      // aggiorno stato razzo
      stima_stato_razzo();
      print_su_flash();
      if(acc(0) > ACC_SEPARAZIONE_CONO){
        carica1=true;
        ledviola();
      }
    }
    digitalWrite(PYRO_1, LOW);
    noTone(BUZZER_PIN);

    if(carica1){
      log_flash("corretta accensione del paracadute");
      stato = 9;
    } else {
      stato = 8;
    }
    }break;
    case 8:{
    stima_stato_razzo();
    print_su_flash();
    //Accensione seconda carica di espulsione
    bool carica2 = false;
    t8=millis();
    tone(BUZZER_PIN, 2000);
    log_flash("accensione seconda miccia");
    pinMode(PYRO_2, OUTPUT);
    digitalWrite(PYRO_2, HIGH);
    while (millis()-t8< 1000) {
      if(acc(0) > ACC_SEPARAZIONE_CONO){
        carica2=true;
      }
    }
    digitalWrite(PYRO_2, LOW);
    noTone(BUZZER_PIN);
    if(carica2){
      log_flash("corretta apertura del paracadute per il nostro test");
      ledazzurro();
      stato=9;
      t9 = millis();
    } else {
      // identico allo stato 13 dell'sfc
      stato = 12;
    }
    }break;
    case 9:{

    stima_stato_razzo();
    print_su_flash();
    //se è fermo
    if (v_kalman >= -0.3 && v_kalman <= 0.1) {
        if (millis() - t9 > 200) {
          ledazzurro();
          tone(BUZZER_PIN, 2000, 1000);
          stato = 10;
        }
    } else {
      t9 = millis();
    }
    }break;
    case 10:{
    //Raccolta dati(R)
    stima_stato_razzo();
    print_su_flash();
    ledazzurro();
    log_flash("sono a terra");
    //trasferimento di dati su flash su SD
    //salvo su sd e gestisco file chiudendoli
    file_log.close();
    traduci_file_dati_volo(file_flash);
    // sposto i file su sd
    log_flash("sposto tutti i file su SD");
    if (move_file_flash_sd(String("/flight_") + n_flight)) {
      // rimuovo i file
      remove_file_flash();
    }
    //aumento il contatore dei lanci
    n_flight++;
    // salvo il contatore
    preferences.putUInt("n_flight", n_flight);
    preferences.end();
    //segnale acustico
    ledverde();
    tone(BUZZER_PIN, 2000, 2000);
    stato=13;
    }break;
    //caso di errore
    case 11:{
    //si accende segnale acustico
    tone(BUZZER_PIN, 4000, 8000);
    ledrosso();
    //salva su sd
    //salvo su sd e gestisco file chiudendoli
    file_log.close();
    traduci_file_dati_volo(file_flash);
    
    // mando l'errore a remoteXY
    Serial.println(s);
    sprintf(RemoteXY.errore, s);
    updateUI();

    if (move_file_flash_sd(String("/flight_") + n_flight)) {
      // rimuovo i file
      remove_file_flash();
    }
    ledrosso();

    //aumento il contatore dei lanci
    n_flight++;
    // salvo il contatore
    preferences.putUInt("n_flight", n_flight);
    preferences.end();

    while(1) {
      updateUI();
      if (!RemoteXY.start) {
        stato = 3;
      }
    }
    }break;
    case 12:{
    //si accende segnale acustico
    tone(BUZZER_PIN, 4000, 8000);
    ledrosso();
    //salva su sd
    //salvo su sd e gestisco file chiudendoli
    file_log.close();
    traduci_file_dati_volo(file_flash);
    
    // mando l'errore a remoteXY
    Serial.println(s);

    if (move_file_flash_sd(String("/flight_") + n_flight)) {
      // rimuovo i file
      remove_file_flash();
      log_flash("spostati tutti i file su SD");
    }
    ledrosso();

    //aumento il contatore dei lanci
    n_flight++;
    // salvo il contatore
    preferences.putUInt("n_flight", n_flight);
    preferences.end();

    // fine esecuzione
    stato = 13;
    }break;
    // fine esecuzione
    case 13:{}
  }
}
