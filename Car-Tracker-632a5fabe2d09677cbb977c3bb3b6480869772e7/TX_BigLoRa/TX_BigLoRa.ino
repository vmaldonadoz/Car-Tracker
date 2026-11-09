// ============================================================
//  TX-BigLoRa-GPS.ino  —  v10
//  Multi-transmisor: RAM + LittleFS + NVS + ACK de burst
//
//  PERSISTENCIA (ningún dato se pierde al apagar):
//    · NVS       : guarda seq actual, oldest_seq y device_id
//    · LittleFS  : guarda cada paquete GPS en flash (hasta FS_BUF_SIZE)
//    · RAM       : buffer circular de los últimos BUF_SIZE paquetes
//    · Al arrancar: restaura la ventana RAM desde LittleFS
//
//  GARANTÍA DE ENTREGA:
//    1. TX envía último paquete DATA
//    2. TX escucha LISTEN_WINDOW_MS
//       a) PKT_REQ → retransmite burst (RAM o flash) → espera PKT_ACK
//       b) PKT_ACK → confirma recepción y guarda último seq confirmado
//    3. Sin ACK → paquete permanece en RAM/flash para el próximo ciclo
//
//  CAPACIDAD:
//    · RAM   : BUF_SIZE=2048 paquetes × 36 bytes = 73 KB (~2.8 h)
//    · Flash : FS_BUF_SIZE paquetes (ajustar según partición LittleFS)
//              Con partición 1 MB → ~27 000 paquetes (~37 h)
//              Con partición 4 MB → ~110 000 paquetes (~6 días)
//
//  IMPORTANTE — Partition Scheme en Arduino IDE:
//    Tools → Partition Scheme → "8M with spiffs" o "4MB (1.2MB APP/1.5MB SPIFFS)"
//    Si LittleFS no monta, el TX funciona igual pero sin persistencia en flash.
// ============================================================
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <SPI.h>
#include "esp_task_wdt.h"
#include <Preferences.h>
#include <LittleFS.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ─── LoRa Pins ───────────────────────────────────────────────
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS   18
#define LORA_RST  23
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// ─── GPS Pins ────────────────────────────────────────────────
#define GPS_RX 34

// ─── Identificación del dispositivo ──────────────────────────
// CAMBIAR este valor para cada transmisor (1, 2, 3, ...)
#define DEVICE_ID_DEFAULT 100

// ─── Forzar ID en NVS ────────────────────────────────────────
// Útil si NVS tiene un ID incorrecto de firmware anterior.
//   0  → usa el valor guardado en NVS (producción)
//   N  → fuerza device_id=N en NVS al arrancar (luego poner a 0)
#define DEVICE_ID_FORCE 0

// ─── Parámetros GPS ──────────────────────────────────────────
#define HDOP_MAX         3.0f
#define INTERVAL_MOVING  5000UL
#define INTERVAL_STATIC  5000UL
#define SPEED_THRESHOLD  2.0f

// ─── Buffer RAM ──────────────────────────────────────────────
// 2048 × 36 bytes = 73 KB. Límite seguro para DRAM del ESP32.
#define BUF_SIZE 2048

// ─── Buffer LittleFS (flash) ─────────────────────────────────
// Ajustar según el tamaño de la partición LittleFS.
// Cálculo: FS_BUF_SIZE × 36 bytes = tamaño del archivo en flash.
//   10 000 paquetes × 36 B = 360 KB  (~13.8 h a 5 s/paquete)
//   27 000 paquetes × 36 B = 972 KB  (~37.5 h, cabe en 1 MB)
//   55 000 paquetes × 36 B = 1.98 MB (~6.4 días, necesita 4 MB)
#define FS_BUF_SIZE  10000
#define FS_BUF_FILE  "/buf.bin"

// ─── Protocolo ───────────────────────────────────────────────
#define LISTEN_WINDOW_MS  3000UL  // ventana de escucha tras TX
#define BURST_GAP_MS        80UL  // pausa entre paquetes del burst
#define ACK_WAIT_MS       4000UL  // espera máxima del ACK tras burst

// ─── Anti-duplicado de burst ─────────────────────────────────
#define BURST_COOLDOWN_PER_PKT 2500UL
#define BURST_COOLDOWN_MIN    20000UL
#define RECENT_BURSTS             8

// ─── Watchdog ────────────────────────────────────────────────
// 90 s da margen suficiente para GPS sin fix al arrancar
#define WDT_TIMEOUT_SEC 90
#define MAX_LORA_FAILS   5

// ─── Tipos de paquete ────────────────────────────────────────
#define PKT_DATA 0x01
#define PKT_REQ  0x02
#define PKT_ACK  0x03

// ─── Estructuras packed ───────────────────────────────────────
typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char     timestamp[20];
  int32_t  lat;
  int32_t  lon;
  uint16_t speed;
  uint8_t  flags;
} data_pkt_t;  // 36 bytes

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;  // 7 bytes

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t acked_seq;
  uint8_t  rx_id[4];
} ack_pkt_t;  // 9 bytes

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t)  ==  7, "req_pkt_t size mismatch");
static_assert(sizeof(ack_pkt_t)  ==  9, "ack_pkt_t size mismatch");

// ─── Buffer circular RAM ─────────────────────────────────────
data_pkt_t   tx_buf[BUF_SIZE];
uint16_t     current_seq    = 0;
uint16_t     oldest_seq     = 0;
uint16_t     last_acked_seq = 0;

// ─── NVS ─────────────────────────────────────────────────────
Preferences  prefs;
uint16_t     deviceId = DEVICE_ID_DEFAULT;

void seqLoad() {
  prefs.begin("tracker", false);  // read-write (para guardar ID forzado)
  current_seq    = prefs.getUShort("seq",  0);
  oldest_seq     = prefs.getUShort("oseq", 0);
  last_acked_seq = prefs.getUShort("ack",  0);

#if DEVICE_ID_FORCE > 0
  deviceId = (uint16_t)DEVICE_ID_FORCE;
  prefs.putUShort("did", deviceId);
  Serial.printf("[CFG] *** Device ID FORZADO a %u ***\n", deviceId);
  Serial.println("[CFG]     Pon DEVICE_ID_FORCE 0 para produccion");
#else
  deviceId = prefs.getUShort("did", DEVICE_ID_DEFAULT);
#endif

  prefs.end();
  Serial.printf("[NVS] seq=%u oldest=%u acked=%u device_id=%u\n",
                current_seq, oldest_seq, last_acked_seq, deviceId);
}

void seqSave() {
  prefs.begin("tracker", false);
  prefs.putUShort("seq",  current_seq);
  prefs.putUShort("oseq", oldest_seq);
  prefs.putUShort("ack",  last_acked_seq);
  prefs.putUShort("did",  deviceId);
  prefs.end();
}

// ─── LittleFS: persistencia de paquetes GPS en flash ─────────
//  Layout: paquete con seq S → offset = (S % FS_BUF_SIZE) × 36
//  Archivo circular de tamaño fijo: sobrescribe los más antiguos.
//  Si LittleFS no está disponible, fs_ok=false y todo es no-op.

bool fs_ok = false;

bool fsWrite(const data_pkt_t& p) {
  if (!fs_ok) return false;
  uint32_t offset = (uint32_t)(p.seq % FS_BUF_SIZE) * sizeof(data_pkt_t);
  const char* mode = LittleFS.exists(FS_BUF_FILE) ? "r+" : "w+";
  File f = LittleFS.open(FS_BUF_FILE, mode);
  if (!f) return false;
  f.seek(offset);
  f.write((uint8_t*)&p, sizeof(data_pkt_t));
  f.close();
  return true;
}

bool fsRead(uint16_t seq, data_pkt_t& out) {
  if (!fs_ok) return false;
  uint32_t offset = (uint32_t)(seq % FS_BUF_SIZE) * sizeof(data_pkt_t);
  File f = LittleFS.open(FS_BUF_FILE, "r");
  if (!f) return false;
  if (f.size() < offset + sizeof(data_pkt_t)) { f.close(); return false; }
  f.seek(offset);
  bool ok = (f.read((uint8_t*)&out, sizeof(data_pkt_t)) == sizeof(data_pkt_t));
  f.close();
  return ok && (out.seq == seq) && (out.pkt_type == PKT_DATA);
}

void fsInit() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS no disponible — sin persistencia en flash");
    Serial.println("[FS]   Verifica: Tools -> Partition Scheme -> incluya SPIFFS/LittleFS");
    return;
  }
  fs_ok = true;

  if (LittleFS.exists(FS_BUF_FILE)) {
    File tmp = LittleFS.open(FS_BUF_FILE, "r");
    uint32_t pkts = tmp.size() / sizeof(data_pkt_t);
    Serial.printf("[FS] Buffer flash: %u KB / %u paquetes almacenados\n",
                  (uint32_t)tmp.size() / 1024, pkts);
    tmp.close();
  } else {
    Serial.println("[FS] Buffer flash vacio — se llenara con datos GPS");
  }

  // ── Restaurar ventana RAM desde flash al arrancar ──────────
  if (current_seq > 0) {
    uint16_t restore_from = (uint16_t)(current_seq - oldest_seq) > BUF_SIZE
                            ? (uint16_t)(current_seq - BUF_SIZE)
                            : oldest_seq;
    uint16_t restored = 0;
    for (uint16_t s = restore_from; s != current_seq; s++) {
      data_pkt_t tmp;
      if (fsRead(s, tmp)) {
        tx_buf[s % BUF_SIZE] = tmp;
        restored++;
      }
    }
    Serial.printf("[FS] %u paquetes restaurados en RAM desde flash\n", restored);
  }

  Serial.printf("[FS] LittleFS OK — %u / %u KB usados\n",
                LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);
}

bool seqInFlash(uint16_t seq) {
  return (uint16_t)(current_seq - seq) <= FS_BUF_SIZE &&
         (uint16_t)(current_seq - seq) >= 1;
}

// ─── Anti-duplicado de burst ──────────────────────────────────
struct RecentBurst {
  uint16_t      from_seq;
  uint16_t      to_seq;
  unsigned long sent_at;
  unsigned long cooldown_ms;
};
RecentBurst recentBursts[RECENT_BURSTS];
uint8_t     recentBurstIdx = 0;

bool wasRecentlyBursted(uint16_t from_seq, uint16_t to_seq) {
  unsigned long now = millis();
  for (int i = 0; i < RECENT_BURSTS; i++) {
    if (recentBursts[i].sent_at == 0) continue;
    if (now - recentBursts[i].sent_at > recentBursts[i].cooldown_ms) continue;
    if (from_seq <= recentBursts[i].to_seq &&
        to_seq   >= recentBursts[i].from_seq) return true;
  }
  return false;
}

void registerBurst(uint16_t from_seq, uint16_t to_seq, unsigned long cooldown_ms) {
  recentBursts[recentBurstIdx] = { from_seq, to_seq, millis(), cooldown_ms };
  recentBurstIdx = (recentBurstIdx + 1) % RECENT_BURSTS;
}

// ─── Globals ─────────────────────────────────────────────────
TinyGPSPlus    gps;
XPowersPMU     PMU;
HardwareSerial GPSSerial(1);
uint32_t       pktSent      = 0;
uint32_t       burstSent    = 0;
unsigned long  lastSend     = 0;
uint8_t        loraFailCount = 0;

// ─── Forward declarations ─────────────────────────────────────
void  initPMU();
void  loraHardReset();
bool  loraApplyConfig();
bool  loraReinit();
bool  loraSendPacket(const uint8_t* data, size_t len);
bool  buildAndBuffer();
void  sendLatest();
void  listenForReq();
void  sendBurst(uint16_t from_seq, uint16_t to_seq);
bool  waitForAck(uint16_t to_seq);
bool  seqInBuffer(uint16_t seq);
void  addDays(int& year, int& month, int& day, int n);
void  gpsToISO_VET(int y, int mo, int d, int h, int mi, int s, char* out);

// ─── PMU ─────────────────────────────────────────────────────
void initPMU() {
  Wire.begin(21, 22);
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
    Serial.println("[PMU] AXP2101 no encontrado"); return;
  }
  PMU.setDC1Voltage(3300);   PMU.enableDC1();
  PMU.setALDO2Voltage(3300); PMU.enableALDO2();
  PMU.setALDO3Voltage(3300); PMU.enableALDO3();
  Serial.println("[PMU] OK");
}

// ─── LoRa ────────────────────────────────────────────────────
void loraHardReset() {
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);  delay(20);
  digitalWrite(LORA_RST, HIGH); delay(150);
}

bool loraApplyConfig() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) return false;
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  return true;
}

bool loraReinit() {
  Serial.println("[RST] Reiniciando modulo LoRa...");
  LoRa.end(); delay(50);
  loraHardReset();
  if (!loraApplyConfig()) {
    Serial.println("[RST] LoRa FALLO"); return false;
  }
  loraFailCount = 0;
  Serial.println("[RST] LoRa reiniciado OK");
  return true;
}

bool loraSendPacket(const uint8_t* data, size_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  return (LoRa.endPacket(false) == 1);
}

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(300);
  Serial.println("\n====== TX-BigLoRa v10 ======");

  memset(recentBursts, 0, sizeof(recentBursts));

  // ── NVS: restaurar secuencia e ID ────────────────────────
  seqLoad();

  initPMU();
  delay(500);

  // ── LittleFS: restaurar paquetes desde flash ──────────────
  // (debe ir DESPUÉS de seqLoad para conocer current_seq)
  fsInit();

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, -1);
  delay(100);
  Serial.println("[GPS] Serial iniciado");

  // ── Watchdog HW ──────────────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // ── LoRa ─────────────────────────────────────────────────
  loraHardReset();
  bool ok = false;
  for (int i = 0; i < 3 && !ok; i++) {
    ok = loraApplyConfig();
    if (!ok) {
      Serial.printf("[BOOT] LoRa intento %d/3 fallo\n", i + 1);
      loraHardReset(); delay(300);
    }
  }
  Serial.printf("[BOOT] LoRa %s — device_id=%u RAM=%u pkts flash=%u pkts\n",
                ok ? "OK" : "FALLO",
                deviceId, BUF_SIZE, FS_BUF_SIZE);
  Serial.printf("[MEM] Heap libre: %u bytes\n", ESP.getFreeHeap());
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  esp_task_wdt_reset();

  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  bool moving = gps.location.isValid() && gps.speed.isValid()
                && (gps.speed.kmph() >= SPEED_THRESHOLD);
  unsigned long interval = moving ? INTERVAL_MOVING : INTERVAL_STATIC;

  if (millis() - lastSend >= interval) {
    lastSend = millis();
    if (buildAndBuffer()) {
      sendLatest();
      listenForReq();
    }
  }
}

// ─── Construir y almacenar paquete GPS ───────────────────────
bool buildAndBuffer() {
  bool hasFix = gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  bool hdopOk = hasFix && gps.hdop.isValid() && (gps.hdop.hdop() < HDOP_MAX);

  if (!hasFix) {
    static unsigned long lastNoFix = 0;
    if (millis() - lastNoFix > 2000) {
      lastNoFix = millis();
      Serial.printf("[GPS] Esperando FIX... sats=%d hdop=%.2f\n",
                    gps.satellites.value(), gps.hdop.hdop());
    }
    return false;
  }

  data_pkt_t& p = tx_buf[current_seq % BUF_SIZE];
  p.pkt_type  = PKT_DATA;
  p.device_id = deviceId;
  p.seq       = current_seq;
  p.flags     = 0;
  p.lat       = (int32_t)(gps.location.lat() * 1e7);
  p.lon       = (int32_t)(gps.location.lng() * 1e7);
  p.speed     = (uint16_t)(gps.speed.kmph() * 10);

  gpsToISO_VET(gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second(),
               p.timestamp);

  p.flags |= 0x01;  // has_fix
  if (hdopOk) p.flags |= 0x02;
  bool moving = gps.speed.isValid() && (gps.speed.kmph() >= SPEED_THRESHOLD);
  if (moving)  p.flags |= 0x04;

  current_seq++;
  // oldest_seq avanza cuando el buffer de FLASH está lleno
  if ((uint16_t)(current_seq - oldest_seq) > FS_BUF_SIZE) oldest_seq++;

  seqSave();   // persiste contadores en NVS
  fsWrite(p);  // persiste paquete en LittleFS (36 bytes)

  return true;
}

// ─── Enviar último paquete ───────────────────────────────────
void sendLatest() {
  const data_pkt_t& p = tx_buf[(current_seq - 1) % BUF_SIZE];
  bool ok = loraSendPacket((uint8_t*)&p, sizeof(data_pkt_t));

  if (ok) {
    pktSent++;
    loraFailCount = 0;
    Serial.printf("[OK] seq=%u | %s | %.6f %.6f | %.1f km/h\n",
                  p.seq, p.timestamp,
                  p.lat / 1e7f, p.lon / 1e7f, p.speed / 10.0f);
  } else {
    loraFailCount++;
    Serial.printf("[ERR] seq=%u fallos=%u/%u\n",
                  p.seq, loraFailCount, MAX_LORA_FAILS);
    if (loraFailCount >= MAX_LORA_FAILS) {
      if (!loraReinit()) { delay(500); ESP.restart(); }
    }
  }
}

// ─── Ventana de escucha: REQ + ACK ───────────────────────────
void listenForReq() {
  LoRa.receive();
  unsigned long t0 = millis();

  while (millis() - t0 < LISTEN_WINDOW_MS) {
    esp_task_wdt_reset();
    while (GPSSerial.available()) gps.encode(GPSSerial.read());

    int sz = LoRa.parsePacket();
    if (sz == 0) continue;

    uint8_t pkt_type_peek = LoRa.peek();

    // ── ACK directo ─────────────────────────────────────────
    if (pkt_type_peek == PKT_ACK && sz == sizeof(ack_pkt_t)) {
      ack_pkt_t ack;
      LoRa.readBytes((uint8_t*)&ack, sizeof(ack));
      if (ack.device_id == deviceId) {
        last_acked_seq = ack.acked_seq;
        seqSave();
        Serial.printf("[ACK] RX confirmo hasta seq=%u\n", ack.acked_seq);
        t0 = millis();
      }
      continue;
    }

    // ── REQ ─────────────────────────────────────────────────
    if (pkt_type_peek == PKT_REQ && sz == sizeof(req_pkt_t)) {
      req_pkt_t req;
      LoRa.readBytes((uint8_t*)&req, sizeof(req));
      if (req.pkt_type != PKT_REQ || req.device_id != deviceId) continue;

      uint16_t missing = (uint16_t)(req.to_seq - req.from_seq + 1);
      if (missing == 0 || missing > 500) {
        Serial.printf("[REQ] Rango invalido %u..%u, ignorado\n",
                      req.from_seq, req.to_seq); continue;
      }
      if (wasRecentlyBursted(req.from_seq, req.to_seq)) {
        Serial.printf("[REQ] Ignorado — burst reciente\n"); continue;
      }

      Serial.printf("[REQ] seq %u..%u (%u faltantes)\n",
                    req.from_seq, req.to_seq, missing);

      unsigned long cooldownMs = (unsigned long)missing * BURST_COOLDOWN_PER_PKT
                                 + BURST_COOLDOWN_MIN;
      registerBurst(req.from_seq, req.to_seq, cooldownMs);

      LoRa.idle(); delay(20);
      esp_task_wdt_reset();
      sendBurst(req.from_seq, req.to_seq);

      LoRa.receive();
      if (waitForAck(req.to_seq)) {
        Serial.printf("[ACK] Burst confirmado hasta seq=%u\n", req.to_seq);
      } else {
        Serial.println("[ACK] Sin ACK — datos conservados en RAM/flash");
      }

      LoRa.receive();
      t0 = millis();
      continue;
    }

    while (LoRa.available()) LoRa.read();
  }
  LoRa.idle();
}

// ─── Retransmisión en burst (RAM primero, luego flash) ───────
void sendBurst(uint16_t from_seq, uint16_t to_seq) {
  uint16_t sent = 0, skipped = 0;
  uint16_t total = (uint16_t)(to_seq - from_seq + 1);
  Serial.printf("[BURST] Iniciando %u pkts (seq %u..%u)\n", total, from_seq, to_seq);

  for (uint16_t s = from_seq; s != (uint16_t)(to_seq + 1); s++) {
    esp_task_wdt_reset();

    data_pkt_t tmp;
    const data_pkt_t* pkt = nullptr;
    const char* src = "?";

    // Caso 1: en RAM
    if (seqInBuffer(s)) {
      const data_pkt_t& ramPkt = tx_buf[s % BUF_SIZE];
      if (ramPkt.seq == s) { pkt = &ramPkt; src = "RAM"; }
    }

    // Caso 2: en flash (fuera de ventana RAM o corrupción)
    if (!pkt && seqInFlash(s)) {
      if (fsRead(s, tmp)) { pkt = &tmp; src = "FS"; }
    }

    if (!pkt) {
      skipped++;
      if (skipped <= 3)
        Serial.printf("[BURST] seq=%u no encontrado (RAM=%c FS=%c)\n",
                      s, seqInBuffer(s) ? 'Y' : 'N', seqInFlash(s) ? 'Y' : 'N');
      continue;
    }

    if (loraSendPacket((uint8_t*)pkt, sizeof(data_pkt_t))) {
      sent++; burstSent++; loraFailCount = 0;
    } else {
      skipped++; loraFailCount++;
      if (loraFailCount >= MAX_LORA_FAILS) { loraReinit(); break; }
    }
    delay(BURST_GAP_MS);
  }

  Serial.printf("[BURST] OK=%u/%u skip=%u\n", sent, total, skipped);
}

// ─── Esperar ACK del RX tras burst ───────────────────────────
bool waitForAck(uint16_t to_seq) {
  unsigned long t0 = millis();
  while (millis() - t0 < ACK_WAIT_MS) {
    esp_task_wdt_reset();
    int sz = LoRa.parsePacket();
    if (sz == 0) continue;
    if (sz != sizeof(ack_pkt_t)) { while (LoRa.available()) LoRa.read(); continue; }

    ack_pkt_t ack;
    LoRa.readBytes((uint8_t*)&ack, sizeof(ack));
    if (ack.pkt_type != PKT_ACK || ack.device_id != deviceId) continue;

    if (ack.acked_seq == to_seq ||
        (uint16_t)(ack.acked_seq - to_seq) <= 32768U) {
      last_acked_seq = ack.acked_seq;
      seqSave();
      return true;
    }
  }
  return false;
}

// ─── Helpers ─────────────────────────────────────────────────
bool seqInBuffer(uint16_t seq) {
  return (uint16_t)(current_seq - seq) <= BUF_SIZE &&
         (uint16_t)(current_seq - seq) >= 1;
}

void addDays(int& year, int& month, int& day, int n) {
  static const uint8_t dim[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  day += n;
  while (true) {
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    int d = (month == 2 && leap) ? 29 : dim[month - 1];
    if (day <= d) break;
    day -= d;
    if (++month > 12) { month = 1; year++; }
  }
}

void gpsToISO_VET(int year, int month, int day,
                  int hour, int minute, int second, char* out) {
  static const uint8_t dim[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  if (year < 2019) addDays(year, month, day, 7168);
  hour -= 4;
  if (hour < 0) {
    hour += 24; day--;
    if (day < 1) {
      if (--month < 1) { month = 12; year--; }
      bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
      day = (month == 2 && leap) ? 29 : dim[month - 1];
    }
  }
  snprintf(out, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
           year, month, day, hour, minute, second);
}
