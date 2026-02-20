/*
 * FinOrchard Pro - TB Node med Light Sleep
 * Arduino MKR WAN 1310
 */
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MKRWAN.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>

#define WM1_ANALOG  A0
#define WM1_FAS1    0
#define WM2_ANALOG  A1
#define WM2_FAS1    1
#define WM_FAS2     2
#define ONEWIRE_PIN 3
#define TB_PIN      6
#define BL1_ANALOG  A4
#define BL2_ANALOG  A5
#define SEND_INTERVAL_SEC 900
#define WM_MEASURE_MS 1000
#define WM_REST_MS 500

String appEui = "0000000000000000";
String appKey = "2D80A14858997AF2B79DFC86AAE94A65";

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensors(&oneWire);
LoRaModem modem;
RTCZero rtc;

volatile uint16_t tipCount = 0;
bool shouldSend = false;

struct SensorData {
  int wm1_raw, wm2_raw;
  float temp1, temp2;
  int bl1_raw, bl2_raw;
  uint16_t tb_tips;
  float air_temp, air_rh;
};

void countTip() { tipCount++; }
void alarmMatch() { shouldSend = true; }

void setNextAlarm() {
  rtc.setAlarmEpoch(rtc.getEpoch() + SEND_INTERVAL_SEC);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);
}

int measureWatermark(int fas1Pin, int analogPin) {
  int r1, r2;
  digitalWrite(fas1Pin, HIGH); digitalWrite(WM_FAS2, LOW);
  delay(WM_MEASURE_MS); r1 = analogRead(analogPin);
  digitalWrite(fas1Pin, LOW); delay(WM_REST_MS);
  digitalWrite(WM_FAS2, HIGH);
  delay(WM_MEASURE_MS); r2 = analogRead(analogPin);
  digitalWrite(WM_FAS2, LOW); delay(WM_REST_MS);
  return (r1 + r2) / 2;
}

bool readSHT30(float &t, float &h) {
  Wire.beginTransmission(0x44); Wire.write(0x2C); Wire.write(0x06);
  if (Wire.endTransmission() != 0) { t=-999; h=-999; return false; }
  delay(20);
  Wire.requestFrom(0x44, 6);
  if (Wire.available() != 6) { t=-999; h=-999; return false; }
  uint16_t rt = (Wire.read()<<8)|Wire.read(); Wire.read();
  uint16_t rh = (Wire.read()<<8)|Wire.read(); Wire.read();
  t = -45.0 + 175.0*(rt/65535.0);
  h = 100.0*(rh/65535.0);
  return true;
}

SensorData readAllSensors() {
  SensorData d;
  tempSensors.begin(); delay(750);
  tempSensors.requestTemperatures(); delay(100);
  d.temp1 = tempSensors.getTempCByIndex(0);
  d.temp2 = tempSensors.getTempCByIndex(1);
  d.wm1_raw = measureWatermark(WM1_FAS1, WM1_ANALOG);
  d.wm2_raw = measureWatermark(WM2_FAS1, WM2_ANALOG);
  d.bl1_raw = analogRead(BL1_ANALOG);
  d.bl2_raw = analogRead(BL2_ANALOG);
  noInterrupts(); d.tb_tips = tipCount; tipCount = 0; interrupts();
  readSHT30(d.air_temp, d.air_rh);
  return d;
}

void sendPayload(SensorData &d) {
  int16_t t1=(int16_t)(d.temp1*100), t2=(int16_t)(d.temp2*100), at=(int16_t)(d.air_temp*100);
  uint8_t rh=(uint8_t)constrain(d.air_rh,0,100);
  uint8_t p[16];
  p[0]=(d.wm1_raw>>8)&0xFF; p[1]=d.wm1_raw&0xFF;
  p[2]=(d.wm2_raw>>8)&0xFF; p[3]=d.wm2_raw&0xFF;
  p[4]=(t1>>8)&0xFF; p[5]=t1&0xFF;
  p[6]=(t2>>8)&0xFF; p[7]=t2&0xFF;
  p[8]=(d.bl1_raw>>8)&0xFF; p[9]=d.bl1_raw&0xFF;
  p[10]=(d.bl2_raw>>8)&0xFF; p[11]=d.bl2_raw&0xFF;
  p[12]=(uint8_t)min(d.tb_tips,255);
  p[13]=(at>>8)&0xFF; p[14]=at&0xFF; p[15]=rh;
  modem.beginPacket();
  for(int i=0;i<16;i++) modem.write(p[i]);
  int err=modem.endPacket(true);
  Serial.println(err>0?"  LoRa: OK":"  LoRa: FEL");
}

void printData(SensorData &d) {
  Serial.print("WM:"); Serial.print(d.wm1_raw); Serial.print("/"); Serial.println(d.wm2_raw);
  Serial.print("T1:"); Serial.print(d.temp1); Serial.print(" T2:"); Serial.println(d.temp2);
  Serial.print("BL:"); Serial.print(d.bl1_raw); Serial.print("/"); Serial.println(d.bl2_raw);
  Serial.print("TB:"); Serial.println(d.tb_tips);
  Serial.print("Luft:"); Serial.print(d.air_temp); Serial.print("C "); Serial.print(d.air_rh); Serial.println("%");
}

void setup() {
  Serial.begin(9600); while(!Serial && millis()<10000){}
  Serial.println("FinOrchard TB - startar...");
  pinMode(WM1_FAS1,OUTPUT); digitalWrite(WM1_FAS1,LOW);
  pinMode(WM2_FAS1,OUTPUT); digitalWrite(WM2_FAS1,LOW);
  pinMode(WM_FAS2,OUTPUT);  digitalWrite(WM_FAS2,LOW);
  Wire.begin();
  rtc.begin(); rtc.setEpoch(0);
  pinMode(TB_PIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(TB_PIN, countTip, FALLING);
  if (!modem.begin(EU868)) { Serial.println("FEL: modem!"); while(1); }
  if (!modem.joinOTAA(appEui, appKey)) { Serial.println("FEL: TTN!"); while(1); }
  Serial.println("TTN: Ansluten!");
  shouldSend = true;
}

void loop() {
  if (shouldSend) {
    shouldSend = false;
    Serial.println("Mater...");
    SensorData data = readAllSensors();
    printData(data);
    sendPayload(data);
    setNextAlarm();
  }
  Serial.println("Sover...");
  Serial.flush();
  LowPower.sleep();
}
