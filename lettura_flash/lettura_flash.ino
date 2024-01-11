/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include "SPIFFS.h"

File file;

String prova = "ciao io mi chiamo leo";
 
void setup() {
  Serial.begin(115200);
  
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  file = SPIFFS.open("/dati_flash.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  
  file.close();
}
 
void loop() {
  
}
