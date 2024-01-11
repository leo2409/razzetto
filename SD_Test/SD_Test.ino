/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <FS.h>
#include "SPIFFS.h"

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

File file_flash;

void setup(){
    Serial.begin(115200);
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  
    file_flash = SPIFFS.open("/dati_flash.txt");
    if(!file_flash){
      Serial.println("Failed to open file for reading");
      return;
    }
    // TODO: dare un coso a 2
    pinMode(2, INPUT);
    if (analogRead(2) / 8191. * 3.3 > 0.5) {
      Serial.println("sd inserita");
    }
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);



    if(SD.mkdir("/dati_volo")){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
    //TODO: INCREMENTALE PER FILE CSV CON REFERENCES
    File file_sd =  SD.open("/dati_volo/volo_1.csv", FILE_WRITE);
    if(!file_sd){
        Serial.println("Failed to open file for writing");
        return;
    }
    Serial.println("inizio a spostare i file sulla sd");
    //TODO: da aggiungere il resto
    file_sd.println("\"Time (s)\",\"Height (m)\",\"attitude_x (deg)\",\"attitude_y (deg)\",\"attitude_z (deg)\"");
    while (file_flash.available()) {
      char* roba = (char*) file_flash.readStringUntil('\n').c_str();
      file_sd.println(roba);
    }
    file_sd.close();
    Serial.println("fineee!!");
}

void loop(){
  
}
