// Dual deploy pyro v2 - ported from RP2040/BMP180 to ESP32-S3/BMP280
// Rabbit_Fire-v2 board: 2 pyro channels only
//   PYRO1 = drogue/apogee channel
//   PYRO2 = main channel
//
// *** IMPORTANT BOARD SETTINGS ***
// This board uses native USB (D+/D- go straight to GPIO19/20), so you MUST
// enable "USB CDC On Boot: Enabled" in Arduino IDE > Tools, or Serial will
// never show up on your PC.
//
// All Serial output is mirrored to WirelessSerial (BLE terminal).

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <McuBleTerminal.h>

TinyGPSPlus gps;

#define RGB_PIN 38
#define NUMPIXELS 1

Adafruit_NeoPixel rgbLed(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

#define CS_PIN   16  // Chip Select pin (net CS-1)
#define SCK_PIN   7  // Clock pin (net SCK)
#define MOSI_PIN  6  // Master Out Slave In pin (net MOSI)
#define MISO_PIN  5  // Master In Slave Out pin (net MISO)


Adafruit_BMP280 bmp;
#define SEALEVELPRESSURE_HPA (1013.25)

//ground level altitude:
long initialAltitude;
//current altitude
long currAltitude;
long liftoffAltitude;
long lastAltitude;
//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 10;
long currTemp;
long currPressure;
int setAltitude = 200;    // fixed main deploy altitude (no DIP switches on this board)


//BMP sensor internal altitude data:

#define CALIB_SAMPLES 10
#define SAMPLE_DELAY_MS 200
float BMPbaseAltitude = 0.0;
long BMPmaxAltitude = -1000.0;
float calibrateBaseAltitude(int samples, int delayMs);

bool LoRaON = false;

//Kalman Variables
float f_1=1.00000;
float kalman_x;
float kalman_x_last;
float kalman_p;
float kalman_p_last;
float kalman_k;
float kalman_q;
float kalman_r;
float kalman_x_temp;
float kalman_p_temp;
//end of Kalman Variables


// I2C: SDA = GPIO1, SCL = GPIO2 (set in Wire.begin() below)
const int PyroDrogue   = 10;   // net PYC1-CTRL  (schematic PYRO1 = apogee/drogue)
const int PyroMain     = 11;   // net PYC2-CTRL  (schematic PYRO2 = main)
const int ContDrogue   = 12;   // net PYC1-Continuity
const int ContMain     = 13;   // net PYC2-Continuity
const int pinSpeaker   = 9;    // net Buzz
const int ButtonPin    = 21;   // net Button


unsigned long previousTimeLED5s = millis();
unsigned long previousTimeLEDoff = millis();
long timeIntervalLed5s = 5000;
long timeIntervalLedOff = 100;
unsigned long previousTimeLED1s = millis();
long timeIntervalLed1s = 1000;
int ledState = LOW;
int blinkCount = 0;

float currTime = 0;
unsigned long previousTimeAltimeter = millis();
long timeIntervalAltimeter = 100;
long launchDeltaMeter = 30;
bool baroApogee = false;
bool apogeeMarker = false;
unsigned long DrogueProhib = millis();
long DrogueTimer = 4000;
bool Launchmarker = false;
bool Drogue = false;
bool Main = false;
bool FullDeploy = false;
bool Landing = false;
unsigned long LandingTime = millis();
int i;
int j;
int k;

//Pyro timer settings:
long Position1 = 2000;   // drogue pyro ON time
long Position2 = 2000;   // main pyro ON time
unsigned long TimerPosition1 = millis();
unsigned long TimerPosition2 = millis();

bool markLanding = false;

bool DrogueCont = false;
bool MainCont = false;


//Apogee blinks - Kalman filter:
int n;
int num1 = 0;
int num2 = 0;
int num3 = 0;
int num4 = 0;
long apogeeAltitude;


//Apogee blinks - BMP raw data:
int m;
int num5 = 0;
int num6 = 0;
int num7 = 0;
int num8 = 0;
long BMPapogeeAltitude;


// LittleFS file paths
const char *LOGFILE = "/flight.csv";
const char *IDFILE  = "/flight_id.txt";

// LittleFS state variables
uint32_t      flightID        = 0;
bool          apogeeSaved     = false;
bool          landingWritten  = false;
unsigned long launchTime      = 0;
unsigned long apogeeTime      = 0;
unsigned long fullDeployTime  = 0;
bool          awaitingConfirm = false;

// GPS module TX -> ESP32-S3 GPIO18 (RX)
// GPS module RX -> ESP32-S3 GPIO17 (TX)
#define GPS_TX_PIN 17   // ESP32-S3 TX -> GPS RX
#define GPS_RX_PIN 18   // ESP32-S3 RX <- GPS TX
#define GPS_BAUD   9600 // SAM-M8Q default UART baud rate
HardwareSerial GPSSerial(1); // use UART1 peripheral

String pos;
String posprev = "no_fix";
String latlon;
int counter=0;

volatile bool loraTxDone = true;

void onTxDone() {
  loraTxDone = true;
}

void setup(void) {
  Serial.begin(115200);
  WirelessSerial.begin("MyESP32");
  delay(5000);

  pinMode(PyroDrogue, OUTPUT);
  pinMode(PyroMain, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);
  pinMode(ContDrogue, INPUT);
  pinMode(ContMain, INPUT);

  //Make sure the outputs are turned off
  digitalWrite(PyroDrogue, LOW);
  digitalWrite(PyroMain, LOW);
  digitalWrite(pinSpeaker, LOW);

  // ESP32-S3 ADC setup - needed to read the full 0-3.3V range on GPIO12/13
  analogReadResolution(12);          // 0-4095
  analogSetAttenuation(ADC_11db);    // allows measuring up to ~3.3V

  //Setup internal RGB LED:
  rgbLed.begin();
  rgbLed.clear();
  rgbLed.show();

  //init Kalman filter
  KalmanInit();

  // ESP32 I2C init - SDA=GPIO1, SCL=GPIO2 per schematic
  Wire.begin(1, 2);

  // BMP280 init - try both common I2C addresses
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("FATAL: BMP280 not found at 0x76 or 0x77. Check wiring.");
      WirelessSerial.println("FATAL: BMP280 not found at 0x76 or 0x77. Check wiring.");
      while (1) { delay(1000); }
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,   // temperature oversampling
                   Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
                   Adafruit_BMP280::FILTER_X16,
                   Adafruit_BMP280::STANDBY_MS_1);

  // let's do some dummy altitude reading to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    KalmanCalc(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  }
  //let's read the launch site altitude
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += KalmanCalc(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(50);
  }
  initialAltitude = (sum / 10.0);

  lastAltitude = 0;
  liftoffAltitude = 20;
  measures = 10;

  // Set BMP280 internal base altitude (before start)
  BMPbaseAltitude = calibrateBaseAltitude(10, 200);

  Serial.print("BMP internal base altitude is set");
  Serial.print(BMPbaseAltitude);
  Serial.println(" m");

  WirelessSerial.print("BMP internal base altitude is set");
  WirelessSerial.print(BMPbaseAltitude);
  WirelessSerial.println(" m");


  // --- Pyro continuity check ---

  int adcValDrogue = analogRead(ContDrogue);
  float voltageDrogue = adcValDrogue * (3.3 / 4095.0);
  Serial.print("ContDrogue: ");
  Serial.print(adcValDrogue);
  Serial.print(";");
  Serial.println(voltageDrogue);

  WirelessSerial.print("ContDrogue: ");
  WirelessSerial.print(adcValDrogue);
  WirelessSerial.print(";");
  WirelessSerial.println(voltageDrogue);


  if (voltageDrogue >= 0.8) {
    DrogueCont = true;
    Serial.println("Drogue cont true");
    WirelessSerial.println("Drogue cont true");
  }

  int adcValMain = analogRead(ContMain);
  float voltageMain = adcValMain * (3.3 / 4095.0);
  Serial.print("ContMain: ");
  Serial.print(adcValMain);
  Serial.print(";");
  Serial.println(voltageMain);

  WirelessSerial.print("ContMain: ");
  WirelessSerial.print(adcValMain);
  WirelessSerial.print(";");
  WirelessSerial.println(voltageMain);

  if (voltageMain >= 0.8) {
    MainCont = true;
    Serial.println("Main cont true");
    WirelessSerial.println("Main cont true");
  }


  // --- Beep the status: ---
  if (DrogueCont == true) {
    beep(1);
  } else {
    longbeep(1);
  }

  if (MainCont == true) {
    beep(1);
  } else {
    longbeep(1);
  }


  // Set the LED to red (for Setup phase)
  rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 0));
  rgbLed.show();

  delay(500);

  rgbLed.clear();
  rgbLed.show();


  // LittleFS init
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed.");
    WirelessSerial.println("FATAL: LittleFS mount failed.");
    while (1);
  }
  Serial.println("LittleFS OK");
  WirelessSerial.println("LittleFS OK");

  flightID = fsLoadFlightID();
  Serial.print("Last flight ID: "); Serial.println(flightID);
  WirelessSerial.print("Last flight ID: "); WirelessSerial.println(flightID);

  fsEnsureLogHeader();

  Serial.setTimeout(50);


  //lora part
  String freqSTR = "";

  int32_t freq = 864;
  if (!digitalRead(ButtonPin))
{
  char c;
  rgbLed.setPixelColor(0, rgbLed.Color(255, 255, 255));
  rgbLed.show();
  //delay(10000);
  do {
    if (Serial.available() > 0) {
      c = Serial.read();
      freqSTR = freqSTR + c;
    }
  } while (c != '\n');
  freq = freqSTR.toInt();
  Serial.println(freq);
  WirelessSerial.println(freq);

  File f = LittleFS.open("/freq.txt", "w");
  if (f) {
    f.println(freq);
    f.close();
  } else {
    Serial.println("ERROR: could not open /freq.txt for writing.");
    WirelessSerial.println("ERROR: could not open /freq.txt for writing.");
  }

  freqSTR = "";
  delay(1000);
}

int32_t storedNumber = 868;
if (LittleFS.exists("/freq.txt")) {
  File f = LittleFS.open("/freq.txt", "r");
  storedNumber = f.parseInt();
  f.close();
} else {
  Serial.println("No stored freq found on LittleFS, storedNumber = 868");
  WirelessSerial.println("No stored freq found on LittleFS, storedNumber = 868");
}
Serial.print("Stored freq: ");
Serial.println(storedNumber);
WirelessSerial.print("Stored freq: ");
WirelessSerial.println(storedNumber);
freq = storedNumber;

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  delay(300);
  LoRa.setPins(CS_PIN,4,15);
  Serial.println("LoRa Sender");
  Serial.print(freq);
  WirelessSerial.println("LoRa Sender");
  WirelessSerial.print(freq);
  if (!LoRa.begin(freq*1000000)) {
    Serial.println("Starting LoRa failed!");
    WirelessSerial.println("Starting LoRa failed!");


    //Set neopixel to red
    rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 0));
    rgbLed.show();
    delay(20000);
  }
  else {
    Serial.println("LoRa started!");
    WirelessSerial.println("LoRa started!");
    LoRa.onTxDone(onTxDone);
    LoRa.setSpreadingFactor(12);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);
    rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 255));
    rgbLed.show();
    LoRaON = true;
  }

//GPS setup

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("ESP32-S3 <-> SAM-M8Q starting...");
  WirelessSerial.println("ESP32-S3 <-> SAM-M8Q starting...");


}     //End of Setup phase


//**LED Blink Function**
void blinkLED(int count, int duration, int r = 255, int g = 255, int b = 255) {
  for (int i = 0; i < count; i++) {
    rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
    rgbLed.show();
    digitalWrite(pinSpeaker, HIGH);
    delay(duration);

    rgbLed.clear();
    rgbLed.show();
    digitalWrite(pinSpeaker, LOW);
    delay(duration);
  }
}


void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pinSpeaker, HIGH);
    delay(200);
    digitalWrite(pinSpeaker, LOW);
    delay(500);
  }
}

void longbeep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pinSpeaker, HIGH);
    delay(800);
    digitalWrite(pinSpeaker, LOW);
    delay(500);
  }
}


// LittleFS helper functions

uint32_t fsLoadFlightID() {
  if (!LittleFS.exists(IDFILE)) return 0;
  File f = LittleFS.open(IDFILE, "r");
  uint32_t id = f.parseInt();
  f.close();
  return id;
}

void fsSaveFlightID() {
  File f = LittleFS.open(IDFILE, "w");
  f.print(flightID);
  f.close();
}

void fsEnsureLogHeader() {
  if (!LittleFS.exists(LOGFILE)) {
    File f = LittleFS.open(LOGFILE, "w");
    f.println("# Flight Logger - ESP32-S3");
    f.println("# Format: one event per line");
    f.close();
  }
}

void fsDumpLog() {
  if (!LittleFS.exists(LOGFILE)) {
    Serial.println("(no log file found)");
    WirelessSerial.println("(no log file found)");
    return;
  }
  File f = LittleFS.open(LOGFILE, "r");
  if (!f) {
    Serial.println("ERROR: cannot open log.");
    WirelessSerial.println("ERROR: cannot open log.");
    return;
  }
  if (f.size() == 0) {
    Serial.println("(log file is empty)");
    WirelessSerial.println("(log file is empty)");
    f.close();
    return;
  }
  Serial.println("--- LOG START ---");
  WirelessSerial.println("--- LOG START ---");
  while (f.available()) {
    char ch = f.read();
    Serial.write(ch);
    WirelessSerial.print(ch);
  }
  Serial.println("--- LOG END ---");
  WirelessSerial.println("--- LOG END ---");
  f.close();
}

void fsStatusLog() {
  if (!LittleFS.exists(LOGFILE)) {
    Serial.println("No log file.");
    WirelessSerial.println("No log file.");
    return;
  }
  File f = LittleFS.open(LOGFILE, "r");
  size_t size    = f.size();
  int    flights = 0;
  String line;
  while (f.available()) {
    line = f.readStringUntil('\n');
    if (line.startsWith("Flight ID:")) flights++;
  }
  f.close();
  Serial.print("Size:    "); Serial.print(size);    Serial.println(" bytes");
  Serial.print("Flights: "); Serial.println(flights);
  WirelessSerial.print("Size:    "); WirelessSerial.print(size);    WirelessSerial.println(" bytes");
  WirelessSerial.print("Flights: "); WirelessSerial.println(flights);
}

void fsClearLog() {
  Serial.println("Type YES to confirm deletion:");
  WirelessSerial.println("Type YES to confirm deletion:");
  awaitingConfirm = true;
}

void fsDoConfirmedClear(const String &response) {
  if (response == "YES") {
    LittleFS.remove(LOGFILE);
    fsEnsureLogHeader();
    Serial.println("Log cleared.");
    WirelessSerial.println("Log cleared.");
  } else {
    Serial.println("Canceled.");
    WirelessSerial.println("Canceled.");
  }
}

void fsPrintHelp() {
  Serial.println("Commands: HELP, DUMP, STATUS, CLEAR");
  WirelessSerial.println("Commands: HELP, DUMP, STATUS, CLEAR");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  Serial.print("> "); Serial.println(cmd);
  WirelessSerial.print("> "); WirelessSerial.println(cmd);
  if (awaitingConfirm) {
    awaitingConfirm = false;
    fsDoConfirmedClear(cmd);
    return;
  }
  if      (cmd == "HELP")   fsPrintHelp();
  else if (cmd == "DUMP")   fsDumpLog();
  else if (cmd == "STATUS") fsStatusLog();
  else if (cmd == "CLEAR")  fsClearLog();
  else {
    Serial.print("Unknown: "); Serial.println(cmd);
    WirelessSerial.print("Unknown: "); WirelessSerial.println(cmd);
  }
}

void handleSerial() {
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
  if (WirelessSerial.availableLine()) {
    processCommand(WirelessSerial.readLine());
  }
}

void fsWriteLaunch() {
  File f = LittleFS.open(LOGFILE, "a");
  if (!f) return;
  f.println("---");
  f.print("Flight ID:        "); f.println(flightID);
  f.close();
}

void fsWriteApogee(float altKalman, float altBMP, float tApogee) {
  File f = LittleFS.open(LOGFILE, "a");
  if (!f) return;
  f.print("ALT:              "); f.print((long)altKalman); f.println(" m");
  f.print("BMP Alt:          "); f.print((long)altBMP);    f.println(" m");
  f.print("Time to apogee:   "); f.print(tApogee, 1);   f.println(" s");
  f.close();
}

void fsWriteLanding(float tDesc, float tTotal) {
  File f = LittleFS.open(LOGFILE, "a");
  if (!f) return;
  f.print("Descend time:     "); f.print(tDesc, 1);  f.println(" s");
  f.print("Total flight time:"); f.print(tTotal, 1); f.println(" s");
  f.close();
}


void deployment() {

  handleSerial();

  //Green LED blinks until start:
  if (Launchmarker == false) {
    if ((millis() - previousTimeLED1s) >= timeIntervalLed1s) {
      previousTimeLED1s = millis();
      previousTimeLEDoff = millis();
      rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0));
      rgbLed.show();
    }
    if ((millis() - previousTimeLEDoff) >= timeIntervalLedOff) {
      rgbLed.clear();
      rgbLed.show();
    }
  }


  //read current altitude
  if ((millis() - previousTimeAltimeter) >= timeIntervalAltimeter) {
    previousTimeAltimeter = millis();

    currAltitude = (KalmanCalc(bmp.readAltitude(SEALEVELPRESSURE_HPA)) - initialAltitude);
    currTemp = bmp.readTemperature();
    currPressure = bmp.readPressure();

    // BMP internal altitude:
    float BMPcurrAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    float BMPrelAltitude = BMPcurrAltitude - BMPbaseAltitude;

    if (BMPrelAltitude > BMPmaxAltitude) {
      BMPmaxAltitude = BMPrelAltitude;
    }

    currTime = currTime + 0.1;

    if (Launchmarker) {
      Serial.print(currTime);
      Serial.print(";");
      Serial.print(currAltitude);
      Serial.print(";");
      Serial.print(BMPrelAltitude);
      Serial.print(";");
      Serial.println(currTemp);

      WirelessSerial.print(currTime);
      WirelessSerial.print(";");
      WirelessSerial.print(currAltitude);
      WirelessSerial.print(";");
      WirelessSerial.print(BMPrelAltitude);
      WirelessSerial.print(";");
      WirelessSerial.println(currTemp);
    }

    if (apogeeMarker && !apogeeSaved) {
      apogeeSaved = true;
      float tApogee = (apogeeTime - launchTime) / 1000.0f;
      fsWriteApogee((float)apogeeAltitude, (float)BMPmaxAltitude, tApogee);
      Serial.print("** APOGEE saved: "); Serial.print(apogeeAltitude); Serial.println(" m");
      WirelessSerial.print("** APOGEE saved: "); WirelessSerial.print(apogeeAltitude); WirelessSerial.println(" m");
    }
  }

  // Start detection
  if (currAltitude > launchDeltaMeter && Launchmarker == false) {

    DrogueProhib = millis();
    Launchmarker = true;

    launchTime     = millis();
    apogeeTime     = millis();
    apogeeSaved    = false;
    landingWritten = false;

    flightID++;
    fsSaveFlightID();
    fsWriteLaunch();

    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255));
    rgbLed.show();
  }


  // Apogee detection:
  if (currAltitude > liftoffAltitude && Launchmarker == true && (millis() - DrogueProhib) > DrogueTimer && apogeeMarker == false) {

    rgbLed.clear();
    rgbLed.show();

    if (currAltitude < lastAltitude && baroApogee == false) {
      measures = measures - 1;
      if (measures == 0) {
        baroApogee = true;
        apogeeMarker = true;
        apogeeAltitude = currAltitude;
        apogeeTime = millis();
        Serial.print("APOGEE, ");
        WirelessSerial.print("APOGEE, ");
      }
    } else {
      lastAltitude = currAltitude;
      measures = 10;
    }
  }


  // Drogue open - Main still locked:
  if (apogeeMarker == true && Drogue == false) {
    TimerPosition1 = millis();
    digitalWrite(PyroDrogue, HIGH);
    Drogue = true;
  }


  if ((millis() - TimerPosition1) > Position1 && Drogue == true && Main == false) {
    digitalWrite(PyroDrogue, LOW);
  }


  // Main open - fixed setAltitude
  if (Drogue == true && Main == false) {
    if (currAltitude <= setAltitude) {
      TimerPosition2 = millis();
      digitalWrite(PyroMain, HIGH);
      Main = true;
    }
  }

  if ((millis() - TimerPosition2) > Position2 && Main == true && FullDeploy == false) {
    digitalWrite(PyroMain, LOW);
    FullDeploy = true;
    fullDeployTime = millis();
  }


  //Blink apogees:
  if (FullDeploy == true) {

    if (!landingWritten) {
      landingWritten  = true;
      float tApogee   = (apogeeTime      - launchTime)  / 1000.0f;
      float tDesc     = (fullDeployTime  - apogeeTime)  / 1000.0f;
      float tTotal    = tApogee + tDesc;
      fsWriteLanding(tDesc, tTotal);
      Serial.print("** FLIGHT #"); Serial.print(flightID); Serial.println(" COMPLETE **");
      Serial.print("   Descend time:    "); Serial.print(tDesc, 1);  Serial.println(" s");
      Serial.print("   Total flight:    "); Serial.print(tTotal, 1); Serial.println(" s");

      WirelessSerial.print("** FLIGHT #"); WirelessSerial.print(flightID); WirelessSerial.println(" COMPLETE **");
      WirelessSerial.print("   Descend time:    "); WirelessSerial.print(tDesc, 1);  WirelessSerial.println(" s");
      WirelessSerial.print("   Total flight:    "); WirelessSerial.print(tTotal, 1); WirelessSerial.println(" s");
    }
    //Kalman filter apogee:
    num1 = (apogeeAltitude / 1000) % 10;
    num2 = (apogeeAltitude / 100) % 10;
    num3 = (apogeeAltitude / 10) % 10;
    num4 = (apogeeAltitude / 1) % 10;

    //BMP raw apogee:
    num5 = (BMPmaxAltitude / 1000) % 10;
    num6 = (BMPmaxAltitude / 100) % 10;
    num7 = (BMPmaxAltitude / 10) % 10;
    num8 = (BMPmaxAltitude / 1) % 10;

    // (apogee blink sequence still commented out, as in your version)

    handleSerial();

  }       //End of blink cycle


}    //End of void deployment

//================================================================
// Kalman functions
//================================================================

void KalmanInit()
{
  kalman_q = 4.0001;
  kalman_r = .20001;

  kalman_x = 0;
  kalman_p = 0;
  kalman_x_temp = 0;
  kalman_p_temp = 0;

  kalman_x_last = 0;
  kalman_p_last = 0;
}

float KalmanCalc(float altitude)
{
  kalman_x_temp = kalman_x_last;
  kalman_p_temp = kalman_p_last + kalman_r;

  kalman_k = (f_1 / (kalman_p_temp + kalman_q)) * kalman_p_temp;
  kalman_x = kalman_x_temp + (kalman_k * (altitude - kalman_x_temp));
  kalman_p = (f_1 - kalman_k) * kalman_p_temp;

  kalman_x_last = kalman_x;
  kalman_p_last = kalman_p;

  return kalman_x;
}


//BMP base altitude calculation:
float calibrateBaseAltitude(int samples, int delayMs) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    sum += alt;
    Serial.print("Meres ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(alt);
    Serial.println(" m");

    WirelessSerial.print("Meres ");
    WirelessSerial.print(i + 1);
    WirelessSerial.print(": ");
    WirelessSerial.print(alt);
    WirelessSerial.println(" m");
    delay(delayMs);
  }
  return sum / samples;
}

bool firstlockdone = false;
int gpsBase = 0;

void loratracking(){
  String transmit;

  // ==================== GPS ====================
  while (GPSSerial.available())
  {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isValid())
  {
    latlon = String(gps.location.lat(), 6) + "\n" +
             String(gps.location.lng(), 6);

    posprev = latlon;
  }
  else
  {
    latlon = posprev;
  }


  int numSats = 0;
  if (gps.satellites.isValid())
    numSats = gps.satellites.value();



  // ==================== Altitude ====================

  int gpsAlt = 0;
  if (gps.altitude.isValid()) {
    gpsAlt = round(gps.altitude.meters());
    if(!firstlockdone)
    {
      gpsBase = gpsAlt;
      Serial.print("GPS base is: ");
      Serial.println(gpsBase);
      WirelessSerial.print("GPS base is: ");
      WirelessSerial.println(gpsBase);
      firstlockdone = true;
    }
    gpsAlt = gpsAlt-gpsBase;

  }
  int baroAlt = round(bmp.readAltitude(1013.25)-BMPbaseAltitude);

  // ==================== Temperature ====================
  float temp = bmp.readTemperature();


  // ==================== Build packet ====================
  transmit =
      String(counter) + ";" +
      latlon + ";" +
      String(temp) + ";" +
      String(gpsAlt) + "/" +
      String(baroAlt) + ";" +
      String(numSats) + ";" ;

  if(counter%10==0 && !Launchmarker)
  {
    transmit = transmit + String(DrogueCont) + "/" + String(MainCont) + ";" ;
  }

  // ==================== Send only if radio is free ====================
  if (loraTxDone) {
  loraTxDone = false;
  LoRa.beginPacket();
  LoRa.print(transmit);
  LoRa.endPacket(true);      // async, will fire onTxDone via IRQ when finished
  counter++;
  Serial.println(transmit);
  WirelessSerial.println(transmit);
}
}

void loop() {

  deployment();
  if (LoRaON) {
    loratracking();
  }
}
