#include <EEPROM.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP085.h>   // Handles both BMP085 and BMP180

#define CS_PIN 29  // Chip Select pin
#define SCK_PIN 2 // Clock pin
#define MOSI_PIN 3 // Master Out Slave In pin
#define MISO_PIN 4 // Master In Slave Out pin

#define RED_PIN    17
#define GREEN_PIN  16
#define BLUE_PIN   25

int Power = 11;
#define LED_PIN    12     // Built-in RGB LED pin on XIAO RP2040
#define NUM_PIXELS 1      // Only one built-in LED

#define FreqPIN 26

Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;

// Both sensor objects are declared; only one gets used at runtime,
// decided by auto-detection in setup().
Adafruit_BMP280 bmp280; // I2C
Adafruit_BMP085 bmp180; // I2C (Adafruit_BMP085 lib also drives BMP180)

// Which sensor is actually present
enum BaroType { BARO_NONE, BARO_280, BARO_180 };
BaroType baroType = BARO_NONE;

String pos;
String posprev = "no_fix";
String latlon;
int counter=0;
double basealt;

// ---- Unified barometer helpers ---------------------------------------
// readBaroTemperature(): returns °C regardless of which chip is fitted
double readBaroTemperature() {
  if (baroType == BARO_280) {
    return bmp280.readTemperature();
  } else if (baroType == BARO_180) {
    return bmp180.readTemperature();
  }
  return NAN;
}

// readBaroAltitude(): returns meters above the sea-level pressure given in hPa,
// regardless of which chip is fitted (handles the Pa/hPa unit difference
// between the two Adafruit libraries internally)
double readBaroAltitude(double seaLevelhPa) {
  if (baroType == BARO_280) {
    return bmp280.readAltitude(seaLevelhPa);
  } else if (baroType == BARO_180) {
    // Adafruit_BMP085 library's readAltitude() expects Pascals, not hPa
    return bmp180.readAltitude(seaLevelhPa * 100.0);
  }
  return NAN;
}

// Tries BMP280 first (addr 0x76, then 0x77), then falls back to BMP180/BMP085 (addr 0x77)
void initBarometer() {
  if (bmp280.begin(0x76) || bmp280.begin(0x77)) {
    baroType = BARO_280;
    Serial.println("BMP280 detected.");
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  } else if (bmp180.begin()) {
    baroType = BARO_180;
    Serial.println("BMP180 detected.");
  } else {
    baroType = BARO_NONE;
    Serial.println("No barometer (BMP280/BMP180) detected!");
  }
}
// ------------------------------------------------------------------------

void setup() {
  int32_t freq = 864;
  Serial.begin(115200);
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);
  delay(5000);
  Serial.println("Hello");

  //LED =============================================================================
  
  pixel.begin();           // Initialize NeoPixel
  pinMode(Power,OUTPUT);
  digitalWrite(Power, HIGH);
  pixel.setBrightness(255); // Optional: set brightness (0-255)


  // Initialize EEPROM emulation (allocate 512 bytes)
  EEPROM.begin(512);


  //Frequency change =================================================================
  pinMode(FreqPIN, INPUT_PULLUP);
  String freqSTR = "";

  Serial.println(digitalRead(FreqPIN));

  if(!digitalRead(FreqPIN))
  {
    char c;
    pixel.setPixelColor(0, pixel.Color(255, 255, 255));
    pixel.show();
    //delay(10000);
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



  //LoRa ============================================================================
  SPI.setSCK(SCK_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setRX(MISO_PIN);
  SPI.begin();
  delay(300);
  LoRa.setPins(CS_PIN,28,27);
  Serial.println("LoRa Sender");
  Serial.print(freq);
  if (!LoRa.begin(freq*1000000)) {
    Serial.println("Starting LoRa failed!");
    

    //Set neopixel to red
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(20);
  pixel.setPixelColor(0, pixel.Color(0, 255, 255));
  pixel.show();

  //BMP (auto-detect BMP280 or BMP180) ==============================================
  initBarometer();

  delay(5000);
  basealt = readBaroAltitude(1013.25);
  Serial.print("Base: ");
  Serial.println(basealt);
}

void loop() {
  String transmit;
  String nmeaSentence = "";
  // Read data from GPS
  char c;

  if (Serial1.available()) 
  {
    nmeaSentence=Serial1.readStringUntil('\n');
  }
  while(Serial1.available())
  {
    c = Serial1.read();
  }
  

  Serial.println(nmeaSentence);
  for (size_t i = 0; i < nmeaSentence.length(); i++) gps.encode(nmeaSentence[i]);

  // If valid GPS data is available
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    
    latlon = String(latitude, 6) + "\n" + String(longitude, 6);
    //Serial.println(latlon);
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));  // Green
    pixel.show();
    posprev = latlon;
  } else {
    latlon = posprev;
    pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Yellow
    pixel.show();
  }

  transmit=String(counter)+";"+latlon+";";

  //Barometer data (BMP280 or BMP180) ========================================================================

  double temp = readBaroTemperature();
  Serial.print(F("Temperature = "));
  Serial.print(temp);
  Serial.println(" *C");
  transmit=transmit+String(temp)+';';

  double alt = readBaroAltitude(1013.25)-basealt;
  Serial.print(F("Approx altitude = "));
  Serial.print(alt); /* Adjusted to local forecast! */
  Serial.println(" m");
  transmit=transmit+String(alt)+';';
  
  // send packet
  //Serial.println(transmit);
  Serial.println(transmit);
  LoRa.beginPacket();
  LoRa.print(transmit);
  LoRa.endPacket();
  counter++;
  delay(500);

}
