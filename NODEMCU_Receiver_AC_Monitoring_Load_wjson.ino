/*
 * ZMPT101B ACS712 IOT MONITORING using nodemcu and arduino uno board
 * 
 * this is for board NodeMCU
 * 
 * author: massagita 2021
 */
 
#include <ArduinoJson.h>

#include <SoftwareSerial.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial

#include <SimpleTimer.h>
SimpleTimer timer;

const int relay0 = 13;
const int relay1 = 2;

String key = "123456789";

char auth[ ] = "Bt4qkcUCE8OMqySt98SiX0OuVC6_WtW_";
char ssid[ ] = "Aswad Blaugrana";
char pass[ ] = "punyarizal";

// Declare the "link" serial port
// Please see SoftwareSerial library for detail
SoftwareSerial linkSerial(14, 12); // RX, TX

const int led = 16;
int countDisplay = 0;
int refresh = 0;

int swRel0 = 1, swRel1 = 1;

float arusKiriman, teganganKiriman;

//Variabel untuk json doc
float dsValFromArdu, phValFromArdu, pressBarFromArdu;
float pressPsiFromArdu, dhtTempFromArdu, dhtHumiFromArdu;

int daya;

unsigned long nextMillis;
float energi;
long detik;
unsigned long cost;
int perkwh = 1352;

BLYNK_WRITE(V0) { swRel0 = param.asInt(); }
BLYNK_WRITE(V1) { swRel1 = param.asInt(); }

void sendUptime() {
  Blynk.virtualWrite(V2, teganganKiriman);
  Blynk.virtualWrite(V3, arusKiriman);
  Blynk.virtualWrite(V4, daya);
  Blynk.virtualWrite(V5, energi);
  Blynk.virtualWrite(V6, cost);
}

void setup() {

  Serial.begin(115200);
  Serial.println("lol");
  pinMode(led, OUTPUT);
  pinMode(relay0, OUTPUT);
  pinMode(relay1, OUTPUT);

  digitalWrite(relay0, 1);
  digitalWrite(relay1, 1);

  lcd.init();

  for (int i = 0; i < 5; i++) {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }

  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("~MONITORING LOAD");
  delay(1000);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("~MONITORING LOAD");
    lcd.setCursor(0, 1);
    lcd.print("connect");
    Serial.println("Connecting..");
    if (++refresh > 6) {
      lcd.setCursor(refresh, 1);
      lcd.print(".");
      delay(30);
    }
    if(refresh > 16) {
      refresh = 0; lcd.clear();
    }
    
    digitalWrite(led, digitalRead(led) ^ 1);
  }

  Serial.print((String)"Connected to " + ssid + "\t");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());

  delay(1500);

  refresh = 0;

  lcd.clear();
  //while (!Serial) continue;
  // Initialize the "link" serial port
  // Use the lowest possible data rate to reduce error ratio
  linkSerial.begin(4800);

  //memulai blynk
  Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8080);

  //interval sendUptime 1 detik
  timer.setInterval(1000, sendUptime);
}

void loop() {
  if (linkSerial.available()) {
    countDisplay++;
    if (WiFi.status() == WL_CONNECTED) {

      if (++refresh > 5) {
        lcd.clear();
        refresh = 0;
      }

      Blynk.run(); //run blynk
      timer.run(); //run timer blynk

      digitalWrite(relay0, swRel0);
      digitalWrite(relay1, swRel1);

      daya = (arusKiriman * 240 / 1.3);
      hitung();

      // Allocate the JSON document
      // This one must be bigger than for the sender because it must store the strings

      StaticJsonDocument<500> docReceive;
      //deserializeJson(doc, json);

      // Read the JSON document from the "link" serial port
      DeserializationError err = deserializeJson(docReceive, linkSerial);

      if (err == DeserializationError::Ok) {
        teganganKiriman = docReceive["sensorTegangan"].as<float>();
        arusKiriman = docReceive["sensorArus"].as<float>();

        // Print the values
        // (we must use as<T>() to resolve the ambiguity)
        Serial.print("TX -> V: ");
        Serial.print(teganganKiriman);
        Serial.print("\t A: ");
        Serial.println(arusKiriman);
      }

      else {
        // Print error to the "debug" serial port
        Serial.print("deserializeJson() returned ");
        Serial.println(err.c_str());
        lcd.clear();
        lcd.setCursor(15, 1);
        lcd.print("E");
        // Flush all bytes in the "link" serial port buffer
        while (linkSerial.available() > 0)
          linkSerial.read();
      }

      lcd.setCursor(0, 0);
      lcd.print("V:"); lcd.print(teganganKiriman);
      lcd.setCursor(9, 0);
      lcd.print("A:"); lcd.print(arusKiriman);

      lcd.setCursor(0, 1);
      lcd.print("P:"); lcd.print(daya);
      lcd.print(" kWh:"); lcd.print(energi);

    }
    else {
      Serial.println("Error in WiFi connection");
    }
  }
  digitalWrite(led, digitalRead(led) ^ 1);
}

void hitung() {
  while (millis() < nextMillis) {}
  nextMillis = millis() + 1000;
  energi += daya / 3600000.000;
  detik++;
  cost = energi * perkwh;
}

