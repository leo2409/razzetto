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
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "nimbus2024"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 224 bytes
  { 255,4,0,15,0,217,0,16,31,1,129,0,34,27,18,6,24,80,89,82,
  79,50,0,70,16,33,33,6,6,1,149,0,10,48,36,37,15,15,4,26,
  31,79,78,0,31,79,70,70,0,10,48,37,67,15,15,4,26,31,79,78,
  0,31,79,70,70,0,129,0,35,57,18,6,24,80,89,82,79,52,0,70,
  16,34,63,6,6,1,149,0,10,48,12,67,15,15,4,26,31,79,78,0,
  31,79,70,70,0,129,0,10,57,18,6,24,80,89,82,79,51,0,70,16,
  9,63,6,6,1,149,0,10,48,11,37,15,15,4,26,31,79,78,0,31,
  79,70,70,0,70,16,8,33,6,6,1,149,0,129,0,9,27,18,6,24,
  80,89,82,79,49,0,129,0,4,5,53,8,36,71,82,79,85,78,68,32,
  84,69,83,84,0,67,4,27,86,23,6,2,26,11,129,0,12,14,36,8,
  1,65,67,67,95,77,65,88,0,129,0,10,87,15,4,24,97,99,99,95,
  109,97,120,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t test_ejection_charge2; // =1 if state is ON, else =0 
  uint8_t test_ejection_charge4; // =1 if state is ON, else =0 
  uint8_t test_ejection_charge3; // =1 if state is ON, else =0 
  uint8_t test_ejection_charge1; // =1 if state is ON, else =0 

    // output variables
  uint8_t pyro_2_continuity; // led state 0 .. 1 
  uint8_t pyro_4_continuity; // led state 0 .. 1 
  uint8_t pyro_3_continuity; // led state 0 .. 1 
  uint8_t pyro_1_continuity; // led state 0 .. 1 
  char acc_max[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define ledrosso()  (neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0)) // Red

// ========= TUTTO QUELLO CHE SERVE PER CALCOLARE ACCELERAZIONE =================
// sensori
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
//accelerazione x massima
float acc_max=0;
// accelerometro vettore colonna
BLA::Matrix<3> acc = { 0, 0, 0 };
// matrici calibrazione accelerometro
BLA::Matrix<3, 3> A_cal_acc = { 1.0084 ,  0.0111,   -0.0143,
                                0.0119,   0.9996,   0.0145,
                                0.0028,   0.0139,   1.0010 };
BLA::Matrix<3> b_cal_acc = { 0.1498,   0.1510,   0.1626 };



void setup() {
  RemoteXY_Init ();
  Serial.begin(115200);
  while (!Serial) delay(100);
  delay(2000);
  Serial.println("GROUND TEST NIMBUS2024");

  digitalWrite(RGB_BUILTIN, HIGH);   // Turn the RGB LED white
  delay(1000);
  digitalWrite(RGB_BUILTIN, LOW);    // Turn the RGB LED off
  delay(1000);
  
  
  adcAttachPin(PYRO_1);
  adcAttachPin(PYRO_2);
  adcAttachPin(PYRO_3);
  adcAttachPin(PYRO_4);
  analogSetAttenuation(ADC_0db); // lettura voltaggi da 0.0V - 0.8V
  Serial.println("SETUP FINITO");
  delay(5000);
}

void calc_batt_percentage() {
  double voltage = 0;
  for (int i = 0; i < 10; i++) {
    voltage += analogRead(VOLTAGE_PIN) / 8191. * 0.8;
  }
  voltage /= 10;
  RemoteXY.battery_percentage=(int)((voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100));
}

void loop() { 
  RemoteXY_Handler ();
  calc_batt_percentage();
  sprintf(RemoteXY.battery_percentage_string,"%d", RemoteXY.battery_percentage);
  pyro_continuity();
  if (!RemoteXY_isConnected()) {
    neopixelWrite(RGB_BUILTIN,0,0,0);
    return;
  }
  else neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0);
  if (RemoteXY.test_ejection_charge1 && RemoteXY.pyro_1_continuity) test(&RemoteXY.test_ejection_charge1, PYRO_1);
  if (RemoteXY.test_ejection_charge2 && RemoteXY.pyro_2_continuity) test(&RemoteXY.test_ejection_charge2, PYRO_2);
  if (RemoteXY.test_ejection_charge3 && RemoteXY.pyro_3_continuity) test(&RemoteXY.test_ejection_charge3, PYRO_3);
  if (RemoteXY.test_ejection_charge4 && RemoteXY.pyro_4_continuity) test(&RemoteXY.test_ejection_charge4, PYRO_4);
  RemoteXY.test_ejection_charge1 = 0;
  RemoteXY.test_ejection_charge2 = 0;
  RemoteXY.test_ejection_charge3 = 0;
  RemoteXY.test_ejection_charge4 = 0;
}

void test(uint8_t* test_ejection_charge, int pyro_channel) {
  Serial.println(pyro_channel);
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,0);
  tone(BUZZER_PIN, 1000, 100); // 10
  Serial.println("10");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 9
  Serial.println("9");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 8
  Serial.println("8");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 7
  Serial.println("7");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 6
  Serial.println("6");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 5
  Serial.println("5");
  delay(900);
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 4
  Serial.println("4");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 3
  Serial.println("3");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 2
  Serial.println("2");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 1000, 100); // 1
  Serial.println("1");
  delay(900);
  RemoteXY_Handler ();
  if (*test_ejection_charge == 0) return;
  tone(BUZZER_PIN, 4000);
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0);
  Serial.println("acceso");
  pinMode(pyro_channel, OUTPUT);
  ledrosso();
  digitalWrite(pyro_channel, HIGH); // sets the pyro on
  long unsigned int t=millis();
  while(millis()-t<1000){
    calc_acc_max();
  }
  digitalWrite(pyro_channel, LOW); // sets the pyro off
  t=millis();
  while(millis()-t<3000){
    calc_acc_max();
  }
  pinMode(pyro_channel, INPUT);
  tone(BUZZER_PIN, 0);
  neopixelWrite(RGB_BUILTIN,0,0,0);
  *test_ejection_charge = 0;
  Serial.println("spento");
  sprintf(RemoteXY.acc_max, "%f", acc_max);
  
}

void pyro_continuity() {
  float pyro_1, pyro_2, pyro_3, pyro_4;
  pyro_1 = analogRead(PYRO_1) / 8191. * 0.8;
  pyro_2 = analogRead(PYRO_2) / 8191. * 0.8;
  pyro_3 = analogRead(PYRO_3) / 8191. * 0.8;
  pyro_4 = analogRead(PYRO_4) / 8191. * 0.8;
  RemoteXY.pyro_1_continuity = pyro_1 > 0.04;
  RemoteXY.pyro_2_continuity = pyro_2 > 0.04;
  RemoteXY.pyro_3_continuity = pyro_3 > 0.04;
  RemoteXY.pyro_4_continuity = pyro_4 > 0.04;
}

//funzioni utili al calcolo dell'accelerazione maggiore
void readIMU_acc() {
  if (bno08x.wasReset()) {
    Serial.println("bno085 è stato resettato");
    setReports();
  }
  
  // prendo un valore dall'imu
  while (!bno08x.getSensorEvent(&sensorValueIMU)) ;
  
  switch (sensorValueIMU.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc(0) = -sensorValueIMU.un.rawAccelerometer.y  / 65535. * 156.96;
      acc(1) = sensorValueIMU.un.rawAccelerometer.x  / 65535. * 156.96;
      acc(2) = sensorValueIMU.un.rawAccelerometer.z  / 65535. * 156.96;

      cal_acc();
      break;
  }
}

// calibrazione accelerometro c'=u'*A'+b' se traspongo c = A*u + b con u e c e b con dim [3]x[1]
// u (uncal) = c (cal) = a (acc) -> acc = A * acc + b
void cal_acc() {
  acc = A_cal_acc * acc + b_cal_acc;
}

void cal_acc_max(){
  readIMU_acc();
  acc_max=acc(0)>acc_max?acc(0):acc_max;
}