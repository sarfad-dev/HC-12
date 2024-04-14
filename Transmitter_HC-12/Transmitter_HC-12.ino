//tx
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>

#define RXD2 16 //(RX2)
#define TXD2 17 //(TX2)
#define pinSet 5 //set
#define HC12 Serial2 //Hardware serial 2 on the ESP32
static const int RXPin = 19, TXPin = 18;
static const uint32_t GPSBaud = 9600;

char serialZnak;
char HC12Znak;
String serialZprava = "";
String HC12Zprava = "";
boolean serialKonecZpravy = false;
boolean HC12KonecZpravy = false;
boolean communicationEnabled = true; // Flag to control communication

Adafruit_BME280 bme;
TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  if (!bme.begin(0x76))
  {
    Serial.println("BME280 sensor not found. Check wiring!");
    while (1);
  }

  pinMode(pinSet, OUTPUT);
  digitalWrite(pinSet, HIGH);
  delay(80);
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);
  ss.begin(GPSBaud);
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
    else if (serialZprava.startsWith("AT")) 
    {
      HC12.print(serialZprava);
      delay(100);
      digitalWrite(pinSet, LOW);
      delay(100);
      Serial.print(serialZprava);
      HC12.print(serialZprava);
      delay(500);
      digitalWrite(pinSet, HIGH);
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
      digitalWrite(pinSet, LOW);
      delay(100);
      // vytištění konfigurační zprávy po sériové lince pro kontrolu
      Serial.print(serialZprava);
      // nastavení konfigurace pro lokálně připojený modul s pauzou pro zpracování
      HC12.print(HC12Zprava);
      delay(500);
      digitalWrite(pinSet, HIGH);
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

  
  if (communicationEnabled && currentTime - lastTransmissionTime >= 500)
    {
      if (ss.available() > 0)
      {
        if (gps.encode(ss.read()))
        {
          if (gps.location.isValid())
          {
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
          else
          {
            Serial.println(F("INVALID"));
          }
          
        }
      }
    }
}