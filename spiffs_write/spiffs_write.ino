#include <FS.h>
#include "SPIFFS.h"
 
void setup() {
 
  Serial.begin(115200);
  Serial.println();  
 
  bool success = SPIFFS.begin();
 
  if (success) {
    Serial.println("File system mounted with success");
  } else {
    Serial.println("Error mounting the file system");
    return;
  }
 
  File file = SPIFFS.open("/file.txt", "w");
 
  if (!file) {
    Serial.println("Error opening file for writing");
    return;
  }
  
 
  int bytesWritten = file.println(56877589.0);
  file.println(0.0);
 
  if (bytesWritten > 0) {
    Serial.println("File was written");
    Serial.println(bytesWritten);
 
  } else {
    Serial.println("File write failed");
  }
 
  file.close();
 
}
 
void loop() {
  File file = SPIFFS.open("/file.txt", "r");

  if (!file) {
    Serial.println("Error opening file for reading");
    return;
  }

  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }

  file.close();

  delay(1000);
}