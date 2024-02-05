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
#define REMOTEXY_WIFI_SSID "Test Lancio"
#define REMOTEXY_WIFI_PASSWORD "nimbus2024"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 51 bytes
  { 255, 1, 0, 0, 0, 44, 0, 16, 31, 1, 10, 48, 23, 40, 15, 15, 4, 26, 31, 79,
    78, 0, 31, 79, 70, 70, 0, 129, 0, 6, 28, 52, 6, 24, 83, 84, 79, 32, 80, 69,
    82, 32, 76, 65, 78, 67, 73, 65, 82, 69, 0 };

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  uint8_t lanciato;  // =1 if state is ON, else =0

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

// preference per variabili su flash
#include <Preferences.h>

Preferences preferences;

// FILE SYSTEM INCLUDE
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <FS.h>
#include "SPIFFS.h"
#include <SPIFFS.h>
// sensori
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
// linear algebra
#include <BasicLinearAlgebra.h>
#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)
#define G_CONST (9.81)
using namespace BLA;
//colori led
#define ledbianco() ()(digitalWrite(RGB_BUILTIN, HIGH))                               // Turn the RGB LED white
#define ledspento() (digitalWrite(RGB_BUILTIN, LOW))                                  // Turn the RGB LED off
#define ledrosso() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0))                 // Red
#define ledverde() (neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0))                 // Green
#define ledblue() (neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS))                  // Blue
#define ledazzurro() (neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, RGB_BRIGHTNESS))  // Azzurro
#define ledviola() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS))    // Viola
#define ledgiallo() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0))   // Giallo
#define ledoffblack() (neopixelWrite(RGB_BUILTIN, 0, 0, 0))                           // Off / black

#define BUZZER_PIN 21
#define BNO08X_RESET 20
#define SEALEVELPRESSURE_HPA (1013.25)
#define ACCTRASHOLD 20


//FILE SYSTEM
File file_flash;
int bytesWritten_flash = 0;
char* linea = (char*)malloc(1000 * sizeof(char));

File file_log;
int bytesWritten_log = 0;

//FILE SD
File flash_sd;
File log_sd;


// vettori di stato
// accelerometro vettore colonna
BLA::Matrix<3> acc = { 0, 0, 0 };
// accelerazione verticale
float acc_vert = 0;
// velocità verticale e altezza
float base_altitude = 0;
float altitude_baro = 0;
float h_baro = 0;
float sealevelpressure = 0;
BLA::Matrix<2, 1> S_h = { 0, 0 };
BLA::Matrix<2, 2> A_h = { 1.000, 0.004,
                          0.000, 1.000 };
BLA::Matrix<2, 1> B_h = { 0.5 * 0.004 * 0.004, 0.004 };
BLA::Matrix<1, 2> C_h = { 1, 0 };
BLA::Matrix<2, 2> U_h = { 0, 0,
                          0, 0 };
float std_dev_acc = pow(0.50, 2);                  // 0.10^2 m/s^2 deviazione standard acc
BLA::Matrix<1, 1> std_dev_baro = { pow(0.2, 2) };  // 0.2 m incertezza sul barometro
float std_dev_acc_attitude = pow(3,2); // deviazione standard acc su attitude
float std_dev_mag_attitude = pow(5,2); // deviazione standard mag su attitude
BLA::Matrix<2, 1> K_h = { 0, 0 };
BLA::Matrix<2, 2> I = {
  1,
  0,
  0,
  1,
};
BLA::Matrix<1, 1> M = { 0 };
// giroscopio
BLA::Matrix<3> gyro = { 0, 0, 0 };
// magnetometro
BLA::Matrix<3> mag = { 0, 0, 0 };
// attitude
BLA::Matrix<3> attitude_kalman = { 0, 90, 90 };
BLA::Matrix<3> attitude_acc = { 0, 0, 0 };
BLA::Matrix<3> uncertainty_attitude = { 5 * 5, 2 * 2, 2 * 2 };
BLA::Matrix<3> kalman_gain = {
  0,
  0,
  0,
};
float h_kalman;
float v_kalman;


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
unsigned long t_accensionemotore = 0;
unsigned long int t5;


// matrici calibrazione accelerometro
BLA::Matrix<3, 3> A_cal_acc = { 1.0084 ,  0.0111,   -0.0143,
                                0.0119,   0.9996,   0.0145,
                                0.0028,   0.0139,   1.0010 };
BLA::Matrix<3> b_cal_acc = { 0.1498,   0.1510,   0.1626 };


// SENSORI
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValueIMU;

// stato controllore
int stato = 1;
// numero del lancio
unsigned int n_flight;


void setup() {
  // seriale
  Serial.begin(115200);
  while (!Serial) delay(100);
  delay(3000);
  Serial.println("RAZZETTO");

  // REMOTE XY
  RemoteXY_Init();

  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // I2C SET-UP
  Wire.begin();
  Wire.setClock(3400000);  // ottimo trovato con 10 ore di lavoro
  // TODO you setup code

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

  // FILESYSTEM
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
  file_flash = SPIFFS.open(String("/dati_flash_") + n_flight + String(".csv"), "w");
  if (!file_flash) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1)
      ;
  }
  bytesWritten_flash += file_flash.print("time_millis, v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z");

  // apertura file di log
  file_log = SPIFFS.open(String("/file_log_") + n_flight + String(".txt"), "w");
  if (!file_log) {
    Serial.println("Errore aprendo il file dati_flash in scrittura");
    while (1)
      ;
  }
  bytesWritten_log += file_log.println("Eventi importanti");

  // INIZIALIZZO SENSORI
  // BNO085 IMU
  if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("impossibile trovare BNO085!");
    while (1)
      ;
  }
  Serial.println("BNO085:\tOK");
  // set up report IMU
  setReports();
  // BMP390 barometro
  if (!bmp.begin_I2C(119, &Wire)) {
    Serial.println("Impossibile trovare il bmp390!");
    while (1)
      ;
  }
  Serial.println("BMP390:\tOK");
  // set up parametri barometro
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // nizializzo altezza del terreno"
  Serial.println("inizializzo altezza del terreno");
  setGroundAltitude();

  Serial.println("FINE SETUP");

  delay(1000);
}

// ========================= FUNZIONI =========================

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
  // tempo, v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
  long unsigned int t = millis();
  sprintf(linea, "%lu, %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, v_kalman, h_kalman, attitude_kalman(0), attitude_kalman(1), attitude_kalman(2), gyro(0), gyro(1), gyro(2), h_baro, acc(0), acc(1), acc(2), mag(0), mag(1), mag(2));
  //Serial.println(linea);
  bytesWritten_flash += file_flash.print(linea);
}

void log_flash(String log) {
  log = String(millis()) + String(": ") + log + String("\n");
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
  acc_vert = +cos(degToRad(attitude_kalman(1))) * cos(degToRad(attitude_kalman(2))) * acc(0)
             - sin(degToRad(attitude_kalman(2))) * cos(degToRad(attitude_kalman(1))) * acc(1)
             + sin(degToRad(attitude_kalman(1))) * acc(2);

  // Tolgo la gravità
  acc_vert = acc_vert - 9.81;
}



void readBaro() {
  new_baro = false;
  if (millis() - last_sample_bmp > 10) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    // n_campioni_baro++;
    new_baro = true;
    last_sample_bmp = millis();
  }
  h_baro = altitude_baro - base_altitude;
}


void setGroundAltitude() {
  ledgiallo();
  // faccio 50 letture per togliere eventuali primi errori
  for (int i = 0; i < 50; i++) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
      Serial.println(altitude_baro);
    }
  }
  // prendo come base_altitude la media su 500 misurazioni
  int n = 500;
  for (int i = 0; i < n; i++) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    base_altitude += altitude_baro;
  }
  base_altitude /= n;
  Serial.print("altezza terreno: "); Serial.println(base_altitude);
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
      acc(0) = -sensorValueIMU.un.rawAccelerometer.y / 65535. * 156.96;
      acc(1) = sensorValueIMU.un.rawAccelerometer.x / 65535. * 156.96;
      acc(2) = sensorValueIMU.un.rawAccelerometer.z / 65535. * 156.96;

      cal_acc();

      mill = millis();
      if (last_sample_acc != 0) {
        dt_acc = (mill - last_sample_acc) * 0.001;
      }
      last_sample_acc = mill;

      //n_campioni_acc++;
      new_acc = true;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag(0) = sensorValueIMU.un.magneticField.x;
      mag(1) = sensorValueIMU.un.magneticField.y;
      mag(2) = sensorValueIMU.un.magneticField.z;
      //n_campioni_magn++;
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

      //n_campioni_gyro++;
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



// ========================= LOOP =========================

void loop() {
  RemoteXY_Handler();
  switch (stato) {/Users/leo/Documents/Arduino/Razzetto/test_lancio/test_lancio.ino
    case 1:
      {
        //aspetti che sia conesso l'interfaccia utente
        if (RemoteXY_isConnected()) {
          stato = 2;
        }
        break;
      }
    case 2:
      {
        RemoteXY_Handler();
        //stima razzo
        stima_stato_razzo();
        //aspetta fedback utente
        if (RemoteXY.lanciato) {
          stato = 3;
          tone(BUZZER_PIN, 3000, 1000);
          ledverde();
          //stima razzo
          stima_stato_razzo();
          //raccolta dati:
          print_su_flash();
          // log provo il lancio
          //bytesWritten_log += file_log.print(millis()) + file_log.println(": prova lancio");
          log_flash("prova lancio");
        }
        break;
      }
    case 3:
      {
        //stima razzo
        stima_stato_razzo();
        //raccolta dati:
        print_su_flash();
        //accensione motore
        t_accensionemotore = millis();  //quando acceso motore
        // log provo ad accendere il motore
        //bytesWritten_log += file_log.print(millis()) + file_log.println(": prova ad accendere il motore");
        log_flash("prova ad accendere il motore");
        bool acceso = false;
        while (!acceso && millis() - t_accensionemotore < 2000) {
          stima_stato_razzo();
          print_su_flash();
          if (acc(0) >= ACCTRASHOLD) {
            acceso = true;
          }
        }
        if (acceso) {
          //bytesWritten_log += file_log.print(millis()) + file_log.println(": partito");
          log_flash("partito");
          stato = 4;
          ledblue();
        }
      }
      break;
    case 4:
      {
        //stima razzo
        stima_stato_razzo();
        //raccolta dati:
        print_su_flash();
        //apogeo
        if (v_kalman < 0) {
          tone(BUZZER_PIN, 4000, 1000);
          ledgiallo();
          //bytesWritten_log += file_log.print(millis()) + file_log.println(": apogeo");
          log_flash("apogeo");
          stima_stato_razzo();
          //raccolta dati:
          print_su_flash();
          t5=millis();
          stato = 5;
          //calcolo tempo che poi servirà per capire se fermo
        }
        break;
      }
    case 5:
      {
        //stima razzo
        stima_stato_razzo();
        //raccolta dati(S):
        print_su_flash();
        //controllare quando è fermo
        if (v_kalman >= -0.3 && v_kalman <= 0.1) {
          if (millis() - t5 > 3000) {
            ledverde();
            tone(BUZZER_PIN, 2000, 1000);
            // chiudo i file dei dati e di log
            file_log.close();
            file_flash.close();
            // sposto i file su sd
            if (move_file_flash_sd(String("/flight_") + n_flight)) {
              // rimuovo i file
              remove_file_flash();
            }
            //aumento il contatore dei lanci
            n_flight++;
            // salvo il contatore
            preferences.putUInt("n_flight", n_flight);
            preferences.end();
            stato = 6;
          } else {
            t5 = millis();
          }
        }
        break;
      }
    case 6:
      {
      }
  }
}
