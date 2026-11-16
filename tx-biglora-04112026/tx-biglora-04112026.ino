// ============================================================
//  TX-MiniLoRa-GPS.ino  —  v7
//  Store-and-Forward + Watchdog HW + Hard-reset RA-02
//
//  Cambios respecto a v6:
//   - BURST_GAP_MS 350 (un paquete dura ~300 ms en el aire)
//   - MAX_BURST: ráfagas limitadas por REQ (el RX pide el siguiente tramo)
//   - BUF_SIZE potencia de 2 (2048) para que seq % BUF_SIZE sea
//     consistente al dar la vuelta en 65535
//   - Buffer UART del GPS ampliado (1024) y lectura del GPS durante
//     las ráfagas (pumpGPS) para no corromper NMEA
//   - seqInBuffer más estricto y ajuste de from_seq si ya se sobrescribió
//   - Reinicio si LoRa no inicia tras 3 intentos
// ============================================================
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include "esp_task_wdt.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ─── LoRa Pins ───────────────────────────────────────────────
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// ─── GPS Pins ────────────────────────────────────────────────
#define GPS_RX 34
//#define GPS_TX 12

// ─── Configuración ───────────────────────────────────────────
#define DEVICE_ID 103
#define HDOP_MAX 3.0f
#define INTERVAL_MOVING 5000UL
#define INTERVAL_STATIC 5000UL
#define SPEED_THRESHOLD 2.0f

// ─── Protocolo ───────────────────────────────────────────────
#define LISTEN_WINDOW_MS 1200
#define BURST_GAP_MS 350   // antes 80: el RX necesita tiempo para publicar por MQTT
#define MAX_BURST 15       // máximo de paquetes por REQ (igual a MAX_BURST_REQ del RX)

// 2048 registros × 5 s/registro = 10240 s ≈ 2 h 50 min de historia
// Memoria: 2048 × 36 bytes = 73.7 KB
// Potencia de 2: el índice seq % BUF_SIZE no se rompe al dar la vuelta en 65535
#define BUF_SIZE 2048

// ─── Watchdog / recuperación ─────────────────────────────────
#define WDT_TIMEOUT_SEC 60
#define MAX_LORA_FAILS 5

// ─── Tipos de paquete ────────────────────────────────────────
#define PKT_DATA 0x01
#define PKT_REQ 0x02

// ─── Estructuras packed ───────────────────────────────────────
typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char timestamp[20];
  int32_t lat;
  int32_t lon;
  uint16_t speed;
  uint8_t flags;
} data_pkt_t;  // 36 bytes

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;  // 7 bytes

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t) == 7, "req_pkt_t size mismatch");

// ─── Buffer circular ─────────────────────────────────────────
data_pkt_t tx_buf[BUF_SIZE];
uint16_t current_seq = 0;
uint16_t oldest_seq = 0;

// ─── Globals generales ───────────────────────────────────────
TinyGPSPlus gps;

XPowersPMU PMU;
HardwareSerial GPSSerial(1);
uint32_t pktSent = 0;
uint32_t burstSent = 0;
unsigned long lastSend = 0;
uint8_t loraFailCount = 0;

// ─── Forward declarations ─────────────────────────────────────
void initPMU();
void loraHardReset();
bool loraApplyConfig();
bool loraReinit();
bool loraSendPacket(const uint8_t* data, size_t len);
bool buildAndBuffer();
void sendLatest();
void listenForReq();
void sendBurst(uint16_t from_seq, uint16_t to_seq);
bool seqInBuffer(uint16_t seq);
void pumpGPS(unsigned long ms);
void printDebug(const data_pkt_t& p);
void addDays(int& year, int& month, int& day, int n);
void gpsToISO_VET(int year, int month, int day,
                  int hour, int minute, int second, char* out);


// ============================================================
void initPMU() {
  Wire.begin(21, 22);
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
    Serial.println("[PMU] AXP2101 no encontrado");
    return;
  }
  PMU.setDC1Voltage(3300);
  PMU.enableDC1();
  PMU.setALDO2Voltage(3300);
  PMU.enableALDO2();
  PMU.setALDO3Voltage(3300);
  PMU.enableALDO3();
  Serial.println("[PMU] OK");
}

// ===========================================================
void loraHardReset() {
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(150);
}

bool loraApplyConfig() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) return false;
  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  return true;
}

bool loraReinit() {
  Serial.println("[RST] Reiniciando modulo LoRa...");
  LoRa.end();
  delay(50);
  loraHardReset();
  if (!loraApplyConfig()) {
    Serial.println("[RST] LoRa FALLO — reinicio pendiente");
    return false;
  }
  loraFailCount = 0;
  Serial.println("[RST] LoRa reiniciado OK");
  return true;
}

bool loraSendPacket(const uint8_t* data, size_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  int ok = LoRa.endPacket(false);
  return (ok == 1);
}


// ============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(300);

  initPMU();   // sube rails de LoRa y GPS
  delay(500);  // deja estabilizar

  // Buffer UART grande: loraSendPacket bloquea ~300 ms y el GPS manda ~960 B/s
  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, -1);
  delay(100);
  Serial.println("[GPS] Serial iniciado");

  delay(200);
  // ── Watchdog de hardware ─────────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // ── LoRa: hard-reset SIEMPRE al arrancar ─────────────────────
  loraHardReset();

  bool ok = false;
  for (int i = 0; i < 3 && !ok; i++) {
    ok = loraApplyConfig();
    if (!ok) {
      Serial.printf("[BOOT] LoRa intento %d/3 fallo\n", i + 1);
      loraHardReset();
      delay(300);
    }
  }

  if (!ok) {
    Serial.println("[BOOT] LoRa no inicia -> restart");
    delay(500);
    ESP.restart();
  }

  Serial.printf("[BOOT] TX v7 listo. DEVICE_ID=%u BUF=%u\n", DEVICE_ID, BUF_SIZE);
}

// ============================================================
void loop() {
  esp_task_wdt_reset();

  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  bool moving = gps.location.isValid() && gps.speed.isValid() && (gps.speed.kmph() >= SPEED_THRESHOLD);

  unsigned long interval = moving ? INTERVAL_MOVING : INTERVAL_STATIC;

  if (millis() - lastSend >= interval) {
    lastSend = millis();
    if (buildAndBuffer()) {
      sendLatest();
      listenForReq();
    }
  }
}


// ============================================================
bool buildAndBuffer() {
  bool hasFix = gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  bool hdopOk = hasFix && gps.hdop.isValid() && (gps.hdop.hdop() < HDOP_MAX);
  bool moving = true;

  if (!hasFix) {

    static unsigned long lastNoFix = 0;

    if (millis() - lastNoFix > 2000) {

      lastNoFix = millis();

      Serial.println("[GPS] Esperando FIX...");
      Serial.printf("location=%d date=%d time=%d sats=%d hdop=%.2f\n",
                    gps.location.isValid(),
                    gps.date.isValid(),
                    gps.time.isValid(),
                    gps.satellites.value(),
                    gps.hdop.hdop());
    }

    return false;
  }

  data_pkt_t& p = tx_buf[current_seq % BUF_SIZE];
  p.pkt_type = PKT_DATA;
  p.device_id = DEVICE_ID;
  p.seq = current_seq;
  p.flags = 0;
  p.lat = (int32_t)(gps.location.lat() * 1e7);
  p.lon = (int32_t)(gps.location.lng() * 1e7);
  p.speed = (uint16_t)(gps.speed.kmph() * 10);

  gpsToISO_VET(gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second(),
               p.timestamp);

  p.flags |= 0x01;
  if (hdopOk) p.flags |= 0x02;
  if (moving) p.flags |= 0x04;

  current_seq++;
  if ((uint16_t)(current_seq - oldest_seq) > BUF_SIZE) oldest_seq++;

  return true;
}


// ============================================================
void sendLatest() {
  const data_pkt_t& p = tx_buf[(uint16_t)(current_seq - 1) % BUF_SIZE];

  bool ok = loraSendPacket((uint8_t*)&p, sizeof(data_pkt_t));

  char msg[100];
  if (ok) {
    pktSent++;
    loraFailCount = 0;
    snprintf(msg, sizeof(msg),
             "[OK] seq=%u | %s | %.6f %.6f | %.1f km/h",
             p.seq, p.timestamp,
             p.lat / 1e7f, p.lon / 1e7f,
             p.speed / 10.0f);
    Serial.println(msg);
  } else {
    loraFailCount++;
    snprintf(msg, sizeof(msg),
             "[ERR] seq=%u fallos=%u/%u",
             p.seq, loraFailCount, MAX_LORA_FAILS);
    Serial.println(msg);

    if (loraFailCount >= MAX_LORA_FAILS) {
      if (!loraReinit()) {
        Serial.println("[ERR] loraReinit fallo -> ESP.restart");
        delay(500);
        ESP.restart();
      }
    }
  }
}


// ============================================================
void listenForReq() {
  LoRa.receive();
  unsigned long t0 = millis();

  while (millis() - t0 < LISTEN_WINDOW_MS) {
    esp_task_wdt_reset();
    while (GPSSerial.available()) gps.encode(GPSSerial.read());

    int sz = LoRa.parsePacket();
    if (sz == 0) continue;
    if (sz != sizeof(req_pkt_t)) {
      while (LoRa.available()) LoRa.read();
      continue;
    }

    req_pkt_t req;
    LoRa.readBytes((uint8_t*)&req, sizeof(req));
    if (req.pkt_type != PKT_REQ || req.device_id != DEVICE_ID) continue;

    char msg[60];
    snprintf(msg, sizeof(msg), "[REQ] seq %u..%u (%u faltantes)",
             req.from_seq, req.to_seq,
             (uint16_t)(req.to_seq - req.from_seq + 1));
    Serial.println(msg);

    LoRa.idle();
    sendBurst(req.from_seq, req.to_seq);
    LoRa.receive();
    t0 = millis();  // sigue escuchando tras la ráfaga (el RX pide el siguiente tramo)
  }
  LoRa.idle();
}


// ============================================================
// Espera ms milisegundos pero sigue leyendo el GPS y alimentando el watchdog
void pumpGPS(unsigned long ms) {
  unsigned long t0 = millis();
  do {
    esp_task_wdt_reset();
    while (GPSSerial.available()) gps.encode(GPSSerial.read());
    delay(5);
  } while (millis() - t0 < ms);
}


// ============================================================
void sendBurst(uint16_t from_seq, uint16_t to_seq) {
  uint16_t sent = 0, skipped = 0, count = 0;

  // Si piden algo más viejo de lo que tengo, empiezo por lo más viejo disponible
  if (!seqInBuffer(from_seq)) {
    uint16_t d = (uint16_t)(current_seq - from_seq);
    if (d > BUF_SIZE && d < 32768) from_seq = oldest_seq;  // ya se sobrescribió
  }

  for (uint16_t s = from_seq;
       s != (uint16_t)(to_seq + 1) && count < MAX_BURST;
       s++, count++) {

    esp_task_wdt_reset();
    while (GPSSerial.available()) gps.encode(GPSSerial.read());

    if (!seqInBuffer(s)) {
      skipped++;
      continue;
    }

    const data_pkt_t& p = tx_buf[s % BUF_SIZE];
    if (p.seq != s) {
      skipped++;
      continue;
    }

    if (loraSendPacket((uint8_t*)&p, sizeof(data_pkt_t))) {
      sent++;
      burstSent++;
      loraFailCount = 0;
    } else {
      skipped++;
      loraFailCount++;
      if (loraFailCount >= MAX_LORA_FAILS) {
        loraReinit();
        break;
      }
    }

    pumpGPS(BURST_GAP_MS);  // en vez de delay(): sigue leyendo el GPS
  }

  char msg[60];
  snprintf(msg, sizeof(msg), "[BURST] %u enviados %u omitidos", sent, skipped);
  Serial.println(msg);
}


// ============================================================
bool seqInBuffer(uint16_t seq) {
  uint16_t d = (uint16_t)(current_seq - seq);
  return d >= 1 && d <= BUF_SIZE;  // excluye seq == current_seq (aún no escrito) y futuros
}

void printDebug(const data_pkt_t& p) {
  Serial.printf("[PKT] seq=%-5u fix=%c | %s | %+.6f %+.6f | %.1f km/h | buf[%u..%u]\n",
                p.seq, (p.flags & 0x01) ? 'Y' : 'N', p.timestamp,
                p.lat / 1e7f, p.lon / 1e7f, p.speed / 10.0f,
                oldest_seq, (uint16_t)(current_seq - 1));
}

void addDays(int& year, int& month, int& day, int n) {
  static const uint8_t dim[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  day += n;
  while (true) {
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    int d = (month == 2 && leap) ? 29 : dim[month - 1];
    if (day <= d) break;
    day -= d;
    if (++month > 12) {
      month = 1;
      year++;
    }
  }
}

void gpsToISO_VET(int year, int month, int day,
                  int hour, int minute, int second, char* out) {
  static const uint8_t dim[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  if (year < 2019) addDays(year, month, day, 7168);
  hour -= 4;
  if (hour < 0) {
    hour += 24;
    day--;
    if (day < 1) {
      if (--month < 1) {
        month = 12;
        year--;
      }
      bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
      day = (month == 2 && leap) ? 29 : dim[month - 1];
    }
  }
  snprintf(out, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
           year, month, day, hour, minute, second);
}