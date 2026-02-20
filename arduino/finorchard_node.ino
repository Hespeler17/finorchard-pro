/*
 * FinOrchard Pro - Sensor Node
 * Arduino MKR WAN 1310
 * 
 * SENSORER:
 *   WM1 + WM2   : Watermark markfukt (AC-metod)
 *   DS18B20 x2  : Temperatur OneWire (D3)
 *   BL1 + BL2   : Bladfuktighet analog (A4, A5)
 *   TB          : Regnsensor puls (D6, interrupt)
 *   SHT30       : Lufttemp + RH (I2C D11/D12)
 * 
 * PIN MAPPING (enligt kopplingsschema):
 *   D0  = WM1 FAS1 (Lila W2)
 *   D1  = WM2 FAS1 (Lila W4)
 *   D2  = WM gemensam FAS2 (Grön W5)
 *   D3  = DS18B20 OneWire (Gul W7)
 *   D6  = TB signal (Vit)
 *   D11 = SHT30 SDA
 *   D12 = SHT30 SCL
 *   A0  = WM1 mätpunkt (Grön W1 via 10k)
 *   A1  = WM2 mätpunkt (Grön W3 via 10k)
 *   A4  = BL1 analog (Vit från chip 1)
 *   A5  = BL2 analog (Vit från chip 2)
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MKRWAN.h>

// =============================================
// PIN DEFINITIONS
// =============================================
#define WM1_ANALOG  A0
#define WM1_FAS1    0       // D0
#define WM2_ANALOG  A1
#define WM2_FAS1    1       // D1
#define WM_FAS2     2       // D2 - gemensam FAS2
#define ONEWIRE_PIN 3       // D3
#define TB_PIN      6       // D6
#define BL1_ANALOG  A4
#define BL2_ANALOG  A5
// SHT30 använder hårdvaru I2C på D11/D12

// =============================================
// INSTÄLLNINGAR
// =============================================
#define SEND_INTERVAL_MS  900000UL  // 15 minuter
#define WM_MEASURE_MS     1000      // 1 sek per AC-fas
#define WM_REST_MS        500       // 0.5 sek vila mellan faser
#define TB_MM_PER_TIP     0.2       // mm per puls (kolla på sensorn!)

// TTN credentials - fyll i dina egna
const char* appEui = "0000000000000000";
const char* appKey = "00000000000000000000000000000000";

// =============================================
// GLOBALA VARIABLER
// =============================================
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensors(&oneWire);
LoRaModem modem;

volatile uint16_t tipCount = 0;    // TB pulsräknare (interrupt)
unsigned long lastSendTime = 0;

// Sensordata struct
struct SensorData {
  int     wm1_raw;        // Watermark 1 råvärde (0-1023)
  int     wm2_raw;        // Watermark 2 råvärde (0-1023)
  float   temp1;          // DS18B20 #1 temperatur
  float   temp2;          // DS18B20 #2 temperatur
  int     bl1_raw;        // Bladfukt 1 råvärde (0-1023)
  int     bl2_raw;        // Bladfukt 2 råvärde (0-1023)
  uint16_t tb_tips;       // Tipping bucket pulsantal
  float   air_temp;       // SHT30 lufttemperatur
  float   air_rh;         // SHT30 luftfuktighet
};

// =============================================
// INTERRUPT - TIPPING BUCKET
// =============================================
void countTip() {
  tipCount++;
}

// =============================================
// WATERMARK AC-METOD
// =============================================
int measureWatermark(int fas1Pin, int analogPin) {
  /*
   * AC-metod för att förhindra elektrolys:
   * FAS 1: fas1Pin=HIGH, FAS2=LOW  → Mät
   * VILA:  båda LOW
   * FAS 2: fas1Pin=LOW,  FAS2=HIGH → Omvänd polaritet
   * VILA:  båda LOW
   * Returnerar medelvärde av FAS1-mätning
   */
  int reading1 = 0;
  int reading2 = 0;

  // FAS 1
  digitalWrite(fas1Pin, HIGH);
  digitalWrite(WM_FAS2, LOW);
  delay(WM_MEASURE_MS);
  reading1 = analogRead(analogPin);
  digitalWrite(fas1Pin, LOW);
  delay(WM_REST_MS);

  // FAS 2 (omvänd polaritet)
  digitalWrite(fas1Pin, LOW);
  digitalWrite(WM_FAS2, HIGH);
  delay(WM_MEASURE_MS);
  reading2 = analogRead(analogPin);
  digitalWrite(WM_FAS2, LOW);
  delay(WM_REST_MS);

  // Medelvärde av båda faserna
  return (reading1 + reading2) / 2;
}

// =============================================
// SHT30 VIA I2C
// =============================================
bool readSHT30(float &temperature, float &humidity) {
  Wire.beginTransmission(0x44);  // SHT30 default adress
  Wire.write(0x2C);              // Single shot, high repeatability
  Wire.write(0x06);
  if (Wire.endTransmission() != 0) {
    temperature = -999.0;
    humidity = -999.0;
    return false;
  }
  delay(20);

  Wire.requestFrom(0x44, 6);
  if (Wire.available() != 6) {
    temperature = -999.0;
    humidity = -999.0;
    return false;
  }

  uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
  Wire.read(); // CRC
  uint16_t rawHum = (Wire.read() << 8) | Wire.read();
  Wire.read(); // CRC

  temperature = -45.0 + 175.0 * ((float)rawTemp / 65535.0);
  humidity    = 100.0 * ((float)rawHum / 65535.0);
  return true;
}

// =============================================
// LÄS ALLA SENSORER
// =============================================
SensorData readAllSensors() {
  SensorData data;

  // --- DS18B20 ---
  tempSensors.requestTemperatures();
  delay(100);
  data.temp1 = tempSensors.getTempCByIndex(0);
  data.temp2 = tempSensors.getTempCByIndex(1);

  // --- WATERMARK (AC) ---
  data.wm1_raw = measureWatermark(WM1_FAS1, WM1_ANALOG);
  data.wm2_raw = measureWatermark(WM2_FAS1, WM2_ANALOG);

  // --- BLADFUKTIGHET (analog) ---
  data.bl1_raw = analogRead(BL1_ANALOG);
  data.bl2_raw = analogRead(BL2_ANALOG);

  // --- TIPPING BUCKET (interrupt-räknare) ---
  noInterrupts();
  data.tb_tips = tipCount;
  tipCount = 0;   // Nollställ efter läsning
  interrupts();

  // --- SHT30 ---
  readSHT30(data.air_temp, data.air_rh);

  return data;
}

// =============================================
// SKICKA PAYLOAD VIA LORAWAN
// =============================================
void sendPayload(SensorData &d) {
  /*
   * Payload format (14 bytes):
   * Byte 0-1:  WM1 raw (uint16)
   * Byte 2-3:  WM2 raw (uint16)
   * Byte 4-5:  Temp1 (int16, *100, t.ex. 2350 = 23.50°C)
   * Byte 6-7:  Temp2 (int16, *100)
   * Byte 8-9:  BL1 raw (uint16)
   * Byte 10-11: BL2 raw (uint16)
   * Byte 12:   TB tips (uint8, max 255 per intervall)
   * Byte 13-14: Air temp (int16, *100)
   * Byte 15:   Air RH (uint8, 0-100%)
   */
  
  int16_t t1 = (int16_t)(d.temp1 * 100);
  int16_t t2 = (int16_t)(d.temp2 * 100);
  int16_t at = (int16_t)(d.air_temp * 100);
  uint8_t rh = (uint8_t)constrain(d.air_rh, 0, 100);

  uint8_t payload[16];
  payload[0]  = (d.wm1_raw >> 8) & 0xFF;
  payload[1]  = d.wm1_raw & 0xFF;
  payload[2]  = (d.wm2_raw >> 8) & 0xFF;
  payload[3]  = d.wm2_raw & 0xFF;
  payload[4]  = (t1 >> 8) & 0xFF;
  payload[5]  = t1 & 0xFF;
  payload[6]  = (t2 >> 8) & 0xFF;
  payload[7]  = t2 & 0xFF;
  payload[8]  = (d.bl1_raw >> 8) & 0xFF;
  payload[9]  = d.bl1_raw & 0xFF;
  payload[10] = (d.bl2_raw >> 8) & 0xFF;
  payload[11] = d.bl2_raw & 0xFF;
  payload[12] = (uint8_t)min(d.tb_tips, 255);
  payload[13] = (at >> 8) & 0xFF;
  payload[14] = at & 0xFF;
  payload[15] = rh;

  modem.beginPacket();
  modem.write(payload, 16);
  int err = modem.endPacket(true);  // true = confirmed uplink

  if (err > 0) {
    Serial.println("  LoRa: Skickat OK");
  } else {
    Serial.println("  LoRa: FEL vid sändning");
  }
}

// =============================================
// DEBUG PRINT
// =============================================
void printData(SensorData &d) {
  Serial.println("--- Sensorvärden ---");
  Serial.print("  WM1 råvärde:  "); Serial.println(d.wm1_raw);
  Serial.print("  WM2 råvärde:  "); Serial.println(d.wm2_raw);
  Serial.print("  Temp1 (DS18): "); Serial.print(d.temp1); Serial.println(" °C");
  Serial.print("  Temp2 (DS18): "); Serial.print(d.temp2); Serial.println(" °C");
  Serial.print("  BL1 råvärde:  "); Serial.println(d.bl1_raw);
  Serial.print("  BL2 råvärde:  "); Serial.println(d.bl2_raw);
  Serial.print("  TB pulsar:    "); Serial.println(d.tb_tips);
  Serial.print("  Lufttemp:     "); Serial.print(d.air_temp); Serial.println(" °C");
  Serial.print("  Luftfukt:     "); Serial.print(d.air_rh); Serial.println(" %");
  Serial.println("--------------------");
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("FinOrchard Pro - Startar...");

  // Watermark pinnar som output (START låga)
  pinMode(WM1_FAS1, OUTPUT); digitalWrite(WM1_FAS1, LOW);
  pinMode(WM2_FAS1, OUTPUT); digitalWrite(WM2_FAS1, LOW);
  pinMode(WM_FAS2,  OUTPUT); digitalWrite(WM_FAS2,  LOW);

  // Tipping bucket interrupt
  pinMode(TB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TB_PIN), countTip, FALLING);

  // I2C för SHT30
  Wire.begin();

  // DS18B20
  tempSensors.begin();
  Serial.print("  DS18B20 funna: ");
  Serial.println(tempSensors.getDeviceCount());

  // LoRaWAN
  Serial.println("  Ansluter till TTN...");
  if (!modem.begin(EU868)) {
    Serial.println("  FEL: LoRa modem startar ej!");
    while (1);
  }
  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("  FEL: TTN join misslyckades!");
    while (1);
  }
  Serial.println("  TTN: Ansluten!");

  lastSendTime = millis();
  Serial.println("Setup klar - kör!");
}

// =============================================
// LOOP
// =============================================
void loop() {
  unsigned long now = millis();

  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;

    Serial.println("Mäter sensorer...");
    SensorData data = readAllSensors();
    printData(data);
    sendPayload(data);
  }

  // Inget annat att göra mellan mätningar
  // (TB räknas automatiskt via interrupt)
  delay(100);
}
