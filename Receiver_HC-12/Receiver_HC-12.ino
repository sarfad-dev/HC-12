//rx
#define RXD2 16 //(RX2)
#define TXD2 17 //(TX2)
#define pinSet 5 //set
#define HC12 Serial2 //Hardware serial 2 on the ESP32
char serialZnak;
char HC12Znak;
String serialZprava = "";
String HC12Zprava = ""; //to co mi vyplivne HC-12
boolean serialKonecZpravy = false;
boolean HC12KonecZpravy = false;

void setup()
{
  HC12Zprava.reserve(64);
  serialZprava.reserve(64);
  // nastavení pinu Set jako výstupního
  pinMode(pinSet, OUTPUT);
  // nastavení transparentního módu pro komunikaci
  digitalWrite(pinSet, HIGH);
  // pauza pro spolehlivé nastavení módu
  delay(80);
  // zahájení komunikace po sériové lince
  Serial.begin(2400);
  // zahájení komunikace s modulem HC-12
  HC12.begin(2400, SERIAL_8N1, RXD2, TXD2); // Serial port to HC12
}

void loop()
{
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
      Serial.println("Communication enabled.");
      HC12.println("COM+ON");
    }
    else if (serialZprava.startsWith("COM+OFF"))
    {
      Serial.println("Communication disabled.");
      HC12.println("COM+OFF");
    }
    else if (serialZprava.startsWith("AT")) {
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
    if (HC12Zprava.startsWith("AT")) {
      digitalWrite(pinSet, LOW);
      delay(100);
      Serial.print(serialZprava);
      HC12.print(HC12Zprava);
      delay(500);
      digitalWrite(pinSet, HIGH);
      delay(100);
      HC12.println("Vzdalena konfigurace probehla v poradku!");
    }
    else
    {
      Serial.print(HC12Zprava);
    }
    HC12Zprava = "";
    HC12KonecZpravy = false;
  }
}
