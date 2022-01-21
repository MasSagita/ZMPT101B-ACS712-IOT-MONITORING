/*
 * ZMPT101B ACS712 IOT MONITORING using nodemcu and arduino uno board
 * 
 * this is for board Arduino Uno
 * 
 * author: massagita 2021
 */

//library
#include <ArduinoJson.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <SoftwareSerial.h>
SoftwareSerial linkSerial(6, 7);

#define sensorArus      A0
#define sensorTegangan  A1

//pin input output
const int pinLed[] = {12, 13};
const int pinBuzzer = 11;

//store zero arus dan tegangan
int zero_tegangan, zero_arus;

// this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int mVperAmp = 185; 
//variabel buat acs
int Watt = 0;
double voltase = 0;
double VRMS = 0;
double AmpsRMS = 0;

//variable led indikator
bool ledIndikator = true;

//variable buat nyimpen nilai
float acVoltage;
float acCurrent;
float daya;

//buat refresh screen
float refresh = 0;

//variable buat hitung energi dan cost
unsigned long nextMillis;
float energi;
long detik;
unsigned long cost;
int perkwh = 1352;

float miliAmpere;

void setup() {
  //serial.begin(9600);
  linkSerial.begin(4800);
  Serial.begin(115200);
  lcd.init();
  
  pinMode(pinBuzzer, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(pinLed[i], OUTPUT);
  }

  //indikator led berkedip
  for (int i = 0; i < 9; i++) {
    bool state;
    led(1, state);
    digitalWrite(pinBuzzer, state);
    delay(70);
    state = !state;
  }

  lcd.backlight();

  //set zero tegangan dan arus
  zero_tegangan = 504;                //nilai dari kalibrasi
  zero_arus = analogRead(sensorArus); //nilai dari analog reading yang pertama
  digitalWrite(pinBuzzer, LOW);
}

void loop() {

  //refresh screen
  if (++refresh > 5) {
    lcd.clear();
    refresh = 0;
  }
  
  ledIndikator = !ledIndikator;
  
  acCurrent = acs712(0.05);                 //nilai arus
  acVoltage = get_voltage(sensorTegangan);  //nilai tegangan
  
  hitungEnergi(); //manggil fungsi hitung energi
  daya = (AmpsRMS * 240 / 1.3); //ngitung daya
  
  Serial.println((String) " P:" + daya + "\tkWh:" + energi
                 + "\tC:" + cost);

  //buat json dokumen
  StaticJsonDocument<500> docSending;
  
  docSending["sensorTegangan"]  = acVoltage;
  docSending["sensorArus"]      = acCurrent;
  docSending["dayaSend"]        = daya;
  docSending["energiSend"]      = energi;
  docSending["costSend"]        = cost;
  
  // Send the JSON 
  serializeJson(docSending, linkSerial);
  
  led(1, ledIndikator);
  led(0, 0);
}

//fungsi untuk led
void led(int ch, bool on) {
  digitalWrite(pinLed[ch], on);
}

//fungsi baca tegangan
float get_voltage(int ampPin) {
  int sampleDuration = 100;
  int sampleCount = 0;
  unsigned long rSquaredSum = 0;
  int rawZero = zero_tegangan;
  uint32_t startTime = millis();
  while ((millis() - startTime) < sampleDuration) {
    int RawVoltageIn = analogRead(ampPin) - rawZero;
    rSquaredSum += abs(RawVoltageIn);
    sampleCount++;
  }
  rSquaredSum = rSquaredSum / sampleCount ;
  // y = 2,4653x - 20,575
  // y = 2.419x - 19.303
  double voltage  = (2.419 * rSquaredSum) - 19.303;
  if (voltage < 0) voltage = 0;
  return voltage;
}

//untuk dapat nilai absolute
int absolute(int a) {
  if (a < 0) {
    a = a * -1;
  }
  return a;
}

//fungsi baca sensor arus
float acs712(float calib){
  voltase = getVPP();
  VRMS = ((voltase / 2.0) * 0.707); //root 2 = 0.707
  AmpsRMS = ((VRMS * 1000) / mVperAmp) - calib;
  
  if(AmpsRMS < 0.04) AmpsRMS = 0;
  miliAmpere = AmpsRMS / 1000; //mili ampere
  
  return AmpsRMS;
}

//function to get analogread value
float getVPP() {
  float result;
  int readValue;                
  int maxValue = 0;             
  int minValue = 1023;          

  uint32_t start_time = millis();
  
  while ((millis() - start_time) < 500) {
    readValue = analogRead(sensorArus);
    if (readValue > maxValue) maxValue = readValue;
    if (readValue < minValue) minValue = readValue;
  }  
  result = ((maxValue - minValue) * 5.0) / 1023.0;
  return result;
}

// fungsi hitung energi
void hitungEnergi() {
  while (millis() < nextMillis) {}
  nextMillis = millis() + 1000;
  energi += daya / 3600000.000;
  detik++;
  cost = energi * perkwh;
}

void serial_send( float sensor1, float sensor2) {

  String data_string = (String)sensor1 + "," + (String)sensor2;
  const char *data = data_string.c_str();

  //  serial.write("*");
  //  serial.write(data);
  //  serial.write("|");
  //  serial.write(data);
  //  serial.write("#");
  millis();

}


