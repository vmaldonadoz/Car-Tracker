// ============================================================
//  TX-MiniLoRa-GPS-v9.ino
//  Store-and-Forward + Watchdog HW + Hard-reset RA-02
//  + HISTORIAL PERSISTENTE en anillo sobre flash cruda
//
//  Por qué v9: la v8 usaba LittleFS + flush() y el historial NO
//  era durable (el seq retrocedió tras quitar la energía).
//
//  Persistencia v9:
//   - Se usa la partición "spiffs" DIRECTAMENTE (esp_partition_*),
//     sin sistema de archivos, sin caché, sin flush.
//   - Anillo de LOG_SECTORS sectores de 4 KB. Cada registro = 40 B
//     (paquete 36 B + CRC8 + relleno), 102 registros por sector.
//   - Cada punto se escribe en el siguiente slot libre ANTES de
//     enviarlo por LoRa. Al entrar a un sector nuevo se borra
//     (contiene lo más viejo del anillo).
//   - Al arrancar se recorre el anillo, se validan los CRC, se
//     busca el seq más nuevo y se restauran tx_buf, current_seq
//     y oldest_seq. El seq CONTINÚA donde quedó.
//   - Un registro cortado por falta de energía se detecta por CRC
//     y se salta.
//   - Si la partición no existe, funciona como v7 (solo RAM).
//
//  Mejoras de GPS para antena pequeña:
//   1. ALDO1 activo (backup del GPS → hot/warm start, requiere 18650)
//   2. configureGPS(): GPS+BeiDou+GLONASS y modo vehículo (L76K),
//      con TX temporal en GPIO12 solo durante el arranque
//   3. Fix válido solo si es reciente (GPS_MAX_AGE_MS)
//   4. Diagnóstico: chars / sats / hdop / age / csFail
// ============================================================
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include "esp_partition.h"
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
#define GPS_TX_CFG 12         // TX hacia el GPS: SOLO se usa unos ms al arrancar para configurarlo
#define GPS_CONFIG_ENABLE 1   // 1 = configurar el GPS al arrancar; 0 = desactivar (si hay problemas con GPIO12)

// ─── Configuración ───────────────────────────────────────────
#define DEVICE_ID 101
#define HDOP_MAX 3.0f
#define INTERVAL_MOVING 5000UL
#define INTERVAL_STATIC 5000UL
#define SPEED_THRESHOLD 2.0f
#define GPS_MAX_AGE_MS 4000   // el fix debe ser reciente

// ─── Protocolo ───────────────────────────────────────────────
#define LISTEN_WINDOW_MS 1200
#define BURST_GAP_MS 350   // un paquete dura ~300 ms en el aire
#define MAX_BURST 15       // paquetes por REQ (igual a MAX_BURST_REQ del RX)

// 2048 registros × 5 s = 10240 s ≈ 2 h 50 min de historia
// RAM: 2048 × 36 B = 73.7 KB. Potencia de 2 para que seq % BUF_SIZE sea consistente.
#define BUF_SIZE 2048

// ─── Persistencia (anillo en flash) ──────────────────────────
#define LOG_SECTOR_SIZE 4096
#define LOG_REC_SIZE 40
#define LOG_REC_PER_SECTOR (LOG_SECTOR_SIZE / LOG_REC_SIZE)        // 102
#define LOG_SECTORS 24                                              // 96 KB de flash
#define LOG_CAPACITY (LOG_SECTORS * LOG_REC_PER_SECTOR)             // 2448 registros
// Al borrar un sector quedan LOG_CAPACITY - 102 = 2346 registros (> BUF_SIZE)

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

typedef struct __attribute__((packed)) {
  data_pkt_t pkt;   // 36
  uint8_t crc;      // 1
  uint8_t pad[3];   // 3  → 40 bytes (múltiplo de 4)
} log_rec_t;

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t) == 7, "req_pkt_t size mismatch");
static_assert(sizeof(log_rec_t) == LOG_REC_SIZE, "log_rec_t size mismatch");
static_assert(LOG_CAPACITY - LOG_REC_PER_SECTOR > BUF_SIZE, "anillo demasiado chico");

// ─── Buffer circular ─────────────────────────────────────────
data_pkt_t tx_buf[BUF_SIZE];
uint16_t current_seq = 0;
uint16_t oldest_seq = 0;

// ─── Estado de la persistencia ───────────────────────────────
const esp_partition_t* logPart = nullptr;
uint32_t logWriteIdx = 0;   // próximo slot a escribir (0..LOG_CAPACITY-1)
bool fsOk = false;

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
void gpsSendNMEA(const char* body);
void configureGPS();
uint8_t crc8(const uint8_t* d, size_t n);
bool logInit();
void logLoad();
void logAppend(const data_pkt_t& p);
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
  PMU.setALDO1Voltage(3300);
  PMU.enableALDO1();   // backup/RTC del GPS → permite hot/warm start
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
  LoRa.setSpreadingFactor(7);
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
//  PERSISTENCIA: anillo de registros en la partición "spiffs"
// ============================================================
uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
  }
  return c;
}

static bool recValid(const log_rec_t& r) {
  return r.pkt.pkt_type == PKT_DATA &&
         r.pkt.device_id == DEVICE_ID &&
         r.crc == crc8((const uint8_t*)&r.pkt, sizeof(r.pkt));
}

static bool slotEmpty(uint32_t addr) {
  uint8_t b[LOG_REC_SIZE];
  if (esp_partition_read(logPart, addr, b, sizeof(b)) != ESP_OK) return false;
  for (size_t i = 0; i < sizeof(b); i++) {
    if (b[i] != 0xFF) return false;
  }
  return true;
}

bool logInit() {
  logPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                     ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
  if (!logPart) return false;
  if (logPart->size < (uint32_t)LOG_SECTORS * LOG_SECTOR_SIZE) return false;
  return true;
}

// Guarda un registro en el siguiente slot libre del anillo
void logAppend(const data_pkt_t& p) {
  if (!fsOk) return;

  alignas(4) log_rec_t r;
  memset(&r, 0xFF, sizeof(r));
  r.pkt = p;
  r.crc = crc8((const uint8_t*)&p, sizeof(p));

  for (int tries = 0; tries < LOG_REC_PER_SECTOR + 2; tries++) {
    uint32_t sector = logWriteIdx / LOG_REC_PER_SECTOR;
    uint32_t slot = logWriteIdx % LOG_REC_PER_SECTOR;
    uint32_t addr = sector * LOG_SECTOR_SIZE + slot * LOG_REC_SIZE;

    if (slot == 0) {
      // Entrando a un sector nuevo: se borra (tiene los registros más viejos del anillo)
      if (esp_partition_erase_range(logPart, sector * LOG_SECTOR_SIZE, LOG_SECTOR_SIZE) != ESP_OK) {
        Serial.println("[FS] ERROR borrando sector");
        logWriteIdx = (logWriteIdx + LOG_REC_PER_SECTOR) % LOG_CAPACITY;  // saltar sector
        continue;
      }
    } else if (!slotEmpty(addr)) {
      // Slot sucio (escritura cortada o basura previa): se salta
      logWriteIdx = (logWriteIdx + 1) % LOG_CAPACITY;
      continue;
    }

    esp_err_t e = esp_partition_write(logPart, addr, &r, sizeof(r));
    logWriteIdx = (logWriteIdx + 1) % LOG_CAPACITY;
    if (e == ESP_OK) return;

    Serial.println("[FS] ERROR escribiendo registro");
  }
  Serial.println("[FS] No se pudo guardar el registro");
}

// Restaura tx_buf / current_seq / oldest_seq desde la flash
void logLoad() {
  uint8_t* sec = (uint8_t*)malloc(LOG_SECTOR_SIZE);
  if (!sec) {
    Serial.println("[FS] Sin memoria para leer el historial");
    return;
  }

  // ── Pasada 1: buscar el seq más nuevo (comparación circular) ──
  bool any = false;
  uint16_t newest = 0;
  uint32_t newestIdx = 0;

  for (uint32_t s = 0; s < LOG_SECTORS; s++) {
    esp_task_wdt_reset();
    if (esp_partition_read(logPart, s * LOG_SECTOR_SIZE, sec, LOG_SECTOR_SIZE) != ESP_OK) continue;

    for (uint32_t i = 0; i < LOG_REC_PER_SECTOR; i++) {
      const log_rec_t* r = (const log_rec_t*)(sec + i * LOG_REC_SIZE);
      if (!recValid(*r)) continue;
      if (!any || (int16_t)(r->pkt.seq - newest) > 0) {
        newest = r->pkt.seq;
        newestIdx = s * LOG_REC_PER_SECTOR + i;
        any = true;
      }
    }
  }

  if (!any) {
    logWriteIdx = 0;
    free(sec);
    Serial.println("[FS] Sin historial previo");
    return;
  }

  // ── Pasada 2: cargar los últimos BUF_SIZE registros ──
  uint16_t maxAge = 0;
  uint32_t loaded = 0;

  for (uint32_t s = 0; s < LOG_SECTORS; s++) {
    esp_task_wdt_reset();
    if (esp_partition_read(logPart, s * LOG_SECTOR_SIZE, sec, LOG_SECTOR_SIZE) != ESP_OK) continue;

    for (uint32_t i = 0; i < LOG_REC_PER_SECTOR; i++) {
      const log_rec_t* r = (const log_rec_t*)(sec + i * LOG_REC_SIZE);
      if (!recValid(*r)) continue;

      uint16_t age = (uint16_t)(newest - r->pkt.seq);
      if (age >= BUF_SIZE) continue;

      tx_buf[r->pkt.seq % BUF_SIZE] = r->pkt;
      if (age > maxAge) maxAge = age;
      loaded++;
    }
  }
  free(sec);

  current_seq = (uint16_t)(newest + 1);
  oldest_seq = (uint16_t)(newest - maxAge);
  logWriteIdx = (newestIdx + 1) % LOG_CAPACITY;

  Serial.printf("[FS] Restaurados %lu registros (seq %u..%u). Proximo seq=%u, slot=%lu\n",
                (unsigned long)loaded, oldest_seq, newest, current_seq,
                (unsigned long)logWriteIdx);
}


// ============================================================
//  Configuración del módulo GPS (L76K): más constelaciones =
//  más satélites a la vista, clave con una antena pequeña.
// ============================================================
void gpsSendNMEA(const char* body) {  // body sin '$' ni '*'
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= *p;
  char out[96];
  snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
  GPSSerial.print(out);
  GPSSerial.flush();
  delay(100);
}

void configureGPS() {
  gpsSendNMEA("PCAS04,7");  // GPS + BeiDou + GLONASS
  gpsSendNMEA("PCAS11,3");  // modo vehículo (mejor seguimiento en movimiento)
  gpsSendNMEA("PCAS00");    // guarda la configuración en el GPS
  Serial.println("[GPS] Configurado: GPS+BDS+GLONASS, modo vehiculo");
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
#if GPS_CONFIG_ENABLE
  // TX temporal hacia el GPS solo para configurarlo; luego se libera GPIO12
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX_CFG);
  delay(300);  // el GPS necesita unos ms tras recibir energía
  configureGPS();
  GPSSerial.end();
  pinMode(GPS_TX_CFG, INPUT);  // libera GPIO12 (pin de arranque)
  delay(50);
  GPSSerial.setRxBufferSize(1024);
#endif
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, -1);  // normal: solo RX
  delay(100);
  Serial.println("[GPS] Serial iniciado");

  delay(200);
  // ── Watchdog de hardware ─────────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // ── Historial persistente ────────────────────────────────────
  fsOk = logInit();
  if (fsOk) {
    logLoad();
  } else {
    Serial.println("[FS] Particion 'spiffs' no disponible -> SIN persistencia (solo RAM)");
  }

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

  Serial.printf("[BOOT] TX v9 listo. DEVICE_ID=%u BUF=%u next_seq=%u\n",
                DEVICE_ID, BUF_SIZE, current_seq);
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
  bool hasFix = gps.location.isValid() && gps.location.age() < GPS_MAX_AGE_MS &&
                gps.date.isValid() && gps.time.isValid();
  bool hdopOk = hasFix && gps.hdop.isValid() && (gps.hdop.hdop() < HDOP_MAX);
  bool moving = true;

  if (!hasFix) {

    static unsigned long lastNoFix = 0;

    if (millis() - lastNoFix > 2000) {

      lastNoFix = millis();

      Serial.printf("[GPS] Esperando FIX... chars=%lu sats=%d hdop=%.2f age=%lu csFail=%lu\n",
                    gps.charsProcessed(),
                    gps.satellites.value(),
                    gps.hdop.hdop(),
                    gps.location.age(),
                    gps.failedChecksum());
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

  logAppend(p);   // ← persistir en flash ANTES de enviar, haya RX o no

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
    if (p.pkt_type != PKT_DATA || p.seq != s) {   // slot vacío o sobrescrito
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