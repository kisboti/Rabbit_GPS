#include <EEPROM.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CS_PIN 29
#define SCK_PIN 2
#define MOSI_PIN 3
#define MISO_PIN 4

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

String count="";
String coord="###";
String coordprev="no fix\n---";

#include <Adafruit_NeoPixel.h>
#define RED_PIN    17
#define GREEN_PIN  16
#define BLUE_PIN   25

int Power = 11;
#define LED_PIN    12
#define NUM_PIXELS 1
Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define FreqPIN 26

void setup() {
  int32_t freq = 864;
  Serial.begin(115200);
  delay(5000);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setRX(MISO_PIN);
  SPI.begin();
  delay(300);
  Serial.println("LoRa Receiver");

  pixel.begin();
  pinMode(Power,OUTPUT);
  digitalWrite(Power, HIGH);
  pixel.setBrightness(255);

  EEPROM.begin(512);

  pinMode(FreqPIN, INPUT_PULLUP);
  String freqSTR = "";

  Serial.println(digitalRead(FreqPIN));

  if(!digitalRead(FreqPIN))
  {
    char c;
    pixel.setPixelColor(0, pixel.Color(255, 255, 255));
    pixel.show();
    do {
      if(Serial.available()>0)
      { 
        c = Serial.read();
        freqSTR=freqSTR+c;
      }
    }while (c!='\n');
    freq = freqSTR.toInt();
    Serial.println(freq);
    EEPROM.put(0, freq);
    EEPROM.commit();
    delay(1000);
  }
  int32_t storedNumber;
  EEPROM.get(0, storedNumber);
  Serial.print("Stored freq: ");
  Serial.println(storedNumber);
  freq=storedNumber;

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  LoRa.setPins(CS_PIN,28,27);
  if (!LoRa.begin(freq*1000000)) {
    Serial.println("Starting LoRa failed!");
    display.setCursor(0, 0);
    display.println("LoRa fail");
    display.display();
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();
  Serial.println("LoRa Start");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa Start");
  display.display();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("radio rx ");
    String data="";

    while (LoRa.available()) {
      char c=LoRa.read();
      Serial.print(c);
      data=data+c;
    }

    // Discard corrupt packets
    if(data.charAt(data.length() - 1) != ';') {
      Serial.println("\nCorrupt packet, discarded");
      return;
    }

    display.setTextSize(2);
    display.clearDisplay();
    display.setCursor(0, 0);

    int startIndex = 0;
    int endIndex;

    endIndex = data.indexOf(';', startIndex);
    String count = data.substring(startIndex, endIndex);
    startIndex = endIndex + 1;

    endIndex = data.indexOf(';', startIndex);
    String coord = data.substring(startIndex, endIndex);
    startIndex = endIndex + 1;

    endIndex = data.indexOf(';', startIndex);
    String temp = data.substring(startIndex, endIndex);
    startIndex = endIndex + 1;

    endIndex = data.indexOf(';', startIndex);
    String alt = data.substring(startIndex, endIndex);
    startIndex = endIndex + 1;

    if(coord=="no_fix")
    {
      display.println(coordprev);
    }
    else 
    {
      display.println(coord);
      coordprev=coord;
    }
      
    display.display();

    display.setTextSize(1);
    display.println("ID: " + count);
    display.println("Alt: " + alt + " m");
    display.println("Temp: " + temp + " C");
    display.println("RSSI: "+ String(LoRa.packetRssi()) + " dBi");

    display.display();

    Serial.print("\n\rRSSI ");
    Serial.println(LoRa.packetRssi());
  }
}