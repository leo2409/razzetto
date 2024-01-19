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

// PYRO CHANNELS
#define PYRO_1 4
#define PYRO_2 5
#define PYRO_3 6
#define PYRO_4 7
#define BUZZER_PIN 21

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 185 bytes
  { 255,4,0,4,0,178,0,16,31,1,129,0,34,27,18,6,24,80,89,82,
  79,50,0,70,16,33,33,6,6,1,149,0,10,48,36,37,15,15,4,26,
  31,79,78,0,31,79,70,70,0,10,48,37,67,15,15,4,26,31,79,78,
  0,31,79,70,70,0,129,0,35,57,18,6,24,80,89,82,79,52,0,70,
  16,34,63,6,6,1,149,0,10,48,12,67,15,15,4,26,31,79,78,0,
  31,79,70,70,0,129,0,10,57,18,6,24,80,89,82,79,51,0,70,16,
  9,63,6,6,1,149,0,10,48,11,37,15,15,4,26,31,79,78,0,31,
  79,70,70,0,70,16,8,33,6,6,1,149,0,129,0,9,27,18,6,24,
  80,89,82,79,49,0,129,0,5,6,53,8,36,71,82,79,85,78,68,32,
  84,69,83,84,0 };
  
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

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


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

void loop() { 
  RemoteXY_Handler ();
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
  digitalWrite(pyro_channel, HIGH); // sets the pyro on
  delay(2000);
  digitalWrite(pyro_channel, LOW); // sets the pyro off
  pinMode(pyro_channel, INPUT);
  tone(BUZZER_PIN, 0);
  neopixelWrite(RGB_BUILTIN,0,0,0);
  *test_ejection_charge = 0;
  Serial.println("spento");
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