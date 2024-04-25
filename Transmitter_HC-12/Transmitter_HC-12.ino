#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h>

#define HC12_Rx 16
#define HC12_Tx 4
#define GPS_Rx 35
#define GPS_Tx 34
#define SDA_PIN 33
#define SCL_PIN 32
#define HC12_Set 15
#define SD_CS_PIN 5
static const uint32_t GPSBaud = 9600;

char serialZnak;
char HC12Znak;
String serialZprava = "";
String HC12Zprava = "";
boolean serialKonecZpravy = false;
boolean HC12KonecZpravy = false;
boolean communicationEnabled = true;
boolean sdEnabled = true; // Flag to control communication
unsigned long lastTransmissionTime = 0;

Adafruit_BME280 bme;
TinyGPSPlus gps;

SoftwareSerial HC12(12, 13);
SoftwareSerial GPSs(GPS_Rx, GPS_Tx);
File myFile;
void setup()
{
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bme.begin(0x76, &Wire))
  {
    Serial.println("BME280 sensor not found. Check wiring!");
    while (1);
  }
  if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed");
        sdEnabled = false;
        return;
    }
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.print("negr");
  myFile.close();
  /*myFile = SD.open("/data.csv", FILE_WRITE);
  if (!myFile) {
    Serial.println("Error opening data.csv");
  } 
  else {
    // Check if file is empty
    if (myFile.size() == 0) {
      // Write header to the file
      myFile.println("Temperature;Humidity;Pressure;Latitude;Longitude;Time;Speed;Altitude;");
    }
    myFile.close(); // Close the file after writing header
  }*/
  pinMode(HC12_Set, OUTPUT);
  digitalWrite(HC12_Set, HIGH);
  delay(80);
  HC12.begin(9600);
  GPSs.begin(GPSBaud);
}

void loop()
{
  unsigned long currentTime = millis();
  while (HC12.available())
  {
    HC12Znak = HC12.read();
    HC12Zprava += char(HC12Znak);
    if (HC12Znak == '\n')
    {
      HC12KonecZpravy = true;
    }
  }

  while (Serial.available())
  {
    serialZnak = Serial.read();
    serialZprava += char(serialZnak);
    if (serialZnak == '\n')
    {
      serialKonecZpravy = true;
    }
  }

  if (serialKonecZpravy)
  {
    if (serialZprava.startsWith("COM+ON"))
    {
      communicationEnabled = true;
      Serial.println("Communication enabled.");
      HC12.println("Communication enabled.");
    }
    else if (serialZprava.startsWith("COM+OFF"))
    {
      communicationEnabled = false;
      Serial.println("Communication disabled.");
      HC12.println("Communication disabled.");
    }
    else if (serialZprava.startsWith("COM+SD+ON"))
    {
      sdEnabled = true;
      Serial.println("SD enabled.");
      HC12.println("COM+OFF");
      
    }
    else if (serialZprava.startsWith("COM+SD+OFF"))
    {
      sdEnabled = false;
      Serial.println("SD disabled.");
      HC12.println("COM+OFF");
    }
    else if (serialZprava.startsWith("AT")) 
    {
      HC12.print(serialZprava);
      delay(100);
      digitalWrite(HC12_Set, LOW);
      delay(100);
      Serial.print(serialZprava);
      HC12.print(serialZprava);
      delay(500);
      digitalWrite(HC12_Set, HIGH);
      delay(100);
    }
    else
    {
      HC12.print(serialZprava);
    }

    serialZprava = "";
    serialKonecZpravy = false;
  }

  if (HC12KonecZpravy)
  {
    if (HC12Zprava.startsWith("COM+ON"))
    {
      communicationEnabled = true;
      Serial.println("Communication enabled.");
    }
    else if (HC12Zprava.startsWith("COM+OFF"))
    {
      communicationEnabled = false;
      Serial.println("Communication disabled.");
    }
    else if (HC12Zprava.startsWith("AT")) 
     {
      // nastavení konfiguračního módu s pauzou pro zpracování
      digitalWrite(HC12_Set, LOW);
      delay(100);
      // vytištění konfigurační zprávy po sériové lince pro kontrolu
      Serial.print(serialZprava);
      // nastavení konfigurace pro lokálně připojený modul s pauzou pro zpracování
      HC12.print(HC12Zprava);
      delay(500);
      digitalWrite(HC12_Set, HIGH);
      // přechod zpět do transparentního módu s pauzou na zpracování
      delay(100);
      // odeslání informace do druhého modulu o úspěšném novém nastavení
      HC12.println("Vzdalena konfigurace probehla v poradku!");
    }
    else
    {
      Serial.print(HC12Zprava);
    }

    HC12Zprava = "";
    HC12KonecZpravy = false;
  }

    if (GPSs.available() > 0)
      {
        if (gps.encode(GPSs.read()))
        {
          if (gps.location.isValid())
          {
            // Transmit data via HC12
            //transmitDataHC12();
            /*myFile = SD.open("test.txt", FILE_WRITE);
             myFile.println("negr1");
             myFile.close();
             */
            // Write data to SD card
            /*if (sdEnabled) {
              writeDataToFile();
            }
            // Print data to Serial
            printDataToSerial();
          }*/
          /*
          else
          {
            Serial.println(F("INVALID"));
            HC12.println(F("NEGR"));
          }*/
          
        }
      }
    else{
      printBME();
      writeSD();
      delay(5000);
    }
  }
}
void printBME() {
  HC12.print(bme.readTemperature());
  HC12.print(";");
  HC12.print(bme.readHumidity());
  HC12.print(";");
  HC12.print(bme.readPressure() / 100.0F);
  HC12.println();

  Serial.print(bme.readTemperature());
  Serial.print(";");
  Serial.print(bme.readHumidity());
  Serial.print(";");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println();
}
void writeSD(){
  // Open a file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.

    myFile = SD.open("/test.csv", FILE_APPEND);

    // if the file opened okay, write to it.
    if (myFile) 
    {
      Serial.print("Writing to test.csv...");
      myFile.print(bme.readTemperature());
      myFile.print(";");
      myFile.print(bme.readHumidity());
      myFile.print(";");
      myFile.print(bme.readPressure() / 100.0F);
      myFile.println();
      // close the file:
      myFile.close();
      Serial.println("done.");
    } 
    else 
    {

      // if the file didn't open, print an error.
      Serial.println("error opening test.csv");

    }
}
void transmitDataHC12() {
  // Code to transmit data via HC12
      HC12.print(bme.readTemperature());
      HC12.print(";");
      HC12.print(bme.readHumidity());
      HC12.print(";");
      HC12.print(bme.readPressure() / 100.0F);
      HC12.print(";");
      HC12.print(gps.location.lat(), 6);
      HC12.print(F(";"));
      HC12.print(gps.location.lng(), 6);
      HC12.print(F(";"));
      HC12.print(gps.time.hour());
      HC12.print(F(":"));
      HC12.print(gps.time.minute());
      HC12.print(F(":"));
      HC12.print(gps.time.second());
      HC12.print(F("."));
      HC12.print(gps.time.centisecond());
      HC12.print(";");
      HC12.print(gps.speed.kmph());
      HC12.print(";");
      HC12.print(gps.altitude.meters());
      HC12.println();
}

void writeDataToFile() {
  // Write data to the file
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    Serial.println(F("okkk"));
     myFile.println("negr1");
    myFile.print(bme.readTemperature() );
    myFile.print(";");
    myFile.print(bme.readHumidity());
    myFile.print(";");
    myFile.print(bme.readPressure() / 100.0F);
    myFile.print(";");
    myFile.print(gps.location.lat(), 6);
    myFile.print(F(";"));
    myFile.print(gps.location.lng(), 6);
    myFile.print(F(";"));
    myFile.print(gps.time.hour());
    myFile.print(F(":"));
    myFile.print(gps.time.minute());
    myFile.print(F(":"));
    myFile.print(gps.time.second());
    myFile.print(F("."));
    myFile.print(gps.time.centisecond());
    myFile.print(";");
    myFile.print(gps.speed.kmph());
    myFile.print(";");
    myFile.print(gps.altitude.meters());
    myFile.println();
    myFile.close();
  }
  else {
    Serial.println(F("negr"));
    myFile.close();
  }
}

void printDataToSerial() {
  // Print data to Serial
  Serial.print(bme.readTemperature());
  Serial.print(";");
  Serial.print(bme.readHumidity());
  Serial.print(";");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(";");
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(";"));
  Serial.print(gps.location.lng(), 6);
  Serial.print(F(";"));
  Serial.print(gps.time.hour());
  Serial.print(F(":"));
  Serial.print(gps.time.minute());
  Serial.print(F(":"));
  Serial.print(gps.time.second());
  Serial.print(F("."));
  Serial.print(gps.time.centisecond());
  Serial.print(";");
  Serial.print(gps.speed.kmph());
  Serial.print(";");
  Serial.print(gps.altitude.meters());
  Serial.println();
}