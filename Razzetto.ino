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
  uint8_t pyro_3_continuity; // led state 0 .. 1 
  int8_t battery_percentage; // =0..100 level position 
  float pitch;  // from -180 to 180 
  float yaw;  // from -180 to 180 
  float roll;  // from -180 to 180 
  float height;
  char battery_percentage_string[11];  // string UTF8 end zero 
  char Errore[11];  // string UTF8 end zero 


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
  analogSetPinAttenuation(PYRO_1, ADC_0db);
  analogSetPinAttenuation(PYRO_2, ADC_0db);
  analogSetPinAttenuation(PYRO_3, ADC_0db);
  analogSetPinAttenuation(PYRO_4, ADC_0db);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db); // lettura voltaggi da 0.0V - 0.8V
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
  /*
  // battery percentage
  RemoteXY.battery_percentage = calc_batt_percentage();
  // controllo che sia inserita la sd
  RemoteXY.sd_check = sd_check();
  
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
*/

  //STATI
  Serial.println("Switch");
  Serial.println(stato);
  switch(stato){
    case 1:
    Serial.println("Case 1");
    //instaurazione connessione utente-razzo set_up
    RemoteXY_Handler();
    // battery percentage
    RemoteXY.battery_percentage = calc_batt_percentage();
    //RemoteXY.battery_percentage_string[11]= (char) calc_batt_percentage();
    //if connessione stabilita {stato=2;}
    if(RemoteXY_isConnected()){
      stato=2;
    }
    break;
    case 2:
    Serial.println("Case 2");
    //Aggiornamento interfaccia utente 
    RemoteXY_Handler();
    // battery percentage
    RemoteXY.battery_percentage = calc_batt_percentage();
    RemoteXY.battery_percentage_string[11]= to_string(calc_batt_percentage());
    //if passaggio pressione al livello del mare{stato=3;}
    if(RemoteXY.sea_level){
      stato=3;
      tone(BUZZER_PIN, 2000, 500);
    }
    break;
    case 3:
    //Aggiornamento interfaccia utente 
    RemoteXY_Handler();
    // battery percentage
    RemoteXY.battery_percentage = calc_batt_percentage();
    RemoteXY.battery_percentage_string[11]= to_string(calc_batt_percentage());
    //Inizializzazione del ground
    readBaro();
    base_altitude=h_baro;
    //Inizio Calcolo Assetto e altezza da terra, azione che continua per tutta la durata del volo
    calc_attitude_acc();
    //aggiorna interfaccia con assetto 
    RemoteXY_Handler();
    //Feedback positivo dell'utente per partite{stato=4;}
    if(RemoteXY.start){
      stato=4;
      tone(BUZZER_PIN, 3000, 1000);
    }
    break; /*
    case(stato==4): //Controlli preparatori
    //Controllo livello batteria LIPO tramite voltometro
    if(calc_batt_percentage()>60){
      //Controllo SD
      if(sd_check()){
        //Corpo totalmente fermo: Accelerazione gravitazionale su un solo asse
        if(acc(0)<=0.2 && acc(2)<=0.2 && acc(1)>=0 && acc(1)<=10)){
          //Controllo angoli Roll e Pitch con valore 0 , accelerometro e giroscopio
          if(RemoteXY.yaw==0 && RemoXY.pitch==0){
            //Controllo della continuità sulle micce 
            if(pyro_continuity()){
              stato=5;
            }else{
              //ERRORE Non continuità delle micce
            }
          }else{
            //ERRORE Roll e/o Pitch non sono a zero
          }
        }else{
          //ERRORE corpo non totalmente fermo
        }
      }else{
        //Errore SD non inserita
      }
    }else{
      //Errore batteria scarica
    }
    break;
    case(stato==5):
    //Calcolo Assetto e altezza da terra
    calc_attitude_acc();

    //Interfaccia utente R
    //accensione motore
    t0=millis();//quando acceso motore
    //raccolta dati(S)
    
    //accensione corretta
    if(acc(0)>10){
      stato=6;
      
    }else if(millis()-t0>3000){
      stato=11;
    }
    break;
    case(stato==6):
    //Controllo pressione tramite il barometro (convertito in un’altezza approssimativa)
    //Controllo accensione motore (registrazione del t in cui si spegne)
    //Controllo angoli Roll e Pitch imponendo dei valori massimi di oscillazione (y e z) con Logg
    //unione delle misure con un algoritmo di sensor fusion
    kalman_filter_hight();
    //condizione: if( (h(t-1)-h(t)<0 && deltaT>100) || 6.T>t){stato=7;}
    if(h0-h1<0 && deltaT>100){
      stato=7;
    }
    break;
    case(stato==7):
    //Accensione prima carica di espulsione
    if(7.T>1){stato=8;
    }else if(|a|=9.81 || h>10){stato=9;}
    break;
    case(stato==8):
    //Accensione seconda carica di espulsione
    // if(|a|=9.81 || h>10){stato=9;}
    break;
    case(stato==9):
    //Logging apertura paracadute 
    //if(|a|=9.81 && h<2.5){stato=10;}
    break;
    case(stato==10):
    //Raccolta dati(R)
    //trasferimento di dati su flash su SD
    //segnale acustico
    break;
    case(stato==11):
    //si accende segnale acustico
    break;*/
  }
    
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
    voltage += analogRead(VOLTAGE_PIN) / 8191. * 2.6;
    Serial << voltage << "\n";
  }
  voltage /= 100;
  return (int)((voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100));
}

void readBaro() {
  while (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  altitude_baro = bmp.readAltitude(RemoteXY.sea_level);
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