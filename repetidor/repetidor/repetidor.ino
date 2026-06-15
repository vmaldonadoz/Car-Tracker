// ============================================================
//  REPEATER-LoRa.ino  —  v1
//  Repetidor transparente para sistema de rastreo vehicular
//
//  Hardware : LilyGO TTGO T-Beam v1.1
//             ESP32 + SX1278 (433 MHz) + AXP2101
//  Alimentación: batería 18650 + panel solar
//
//  Estrategia de ahorro energético:
//    1. WiFi y Bluetooth completamente deshabilitados
//    2. CPU a 80 MHz (en lugar de 240 MHz)
//    3. GPS desconectado desde el PMU
//    4. Light sleep entre recepciones (DIO0 como wakeup GPIO)
//
//  Compatibilidad:
//    - Reenvía PKT_DATA  (TX → RX, o TX → Repetidor → RX)
//    - Reenvía PKT_REQ   (RX → TX, o RX → Repetidor → TX)
//    - Cache de deduplicación: descarta paquetes ya reenviados
//      para evitar loops entre múltiples repetidores
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_task_wdt.h"

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ─── Pines LoRa (T-Beam v1.1 — idénticos al RX) ─────────────
#define LORA_SCK    5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   23
#define LORA_DIO0  26
#define LORA_FREQ  433E6

// ─── PMU I2C (T-Beam v1.1) ───────────────────────────────────
#define PMU_SDA    21
#define PMU_SCL    22

// ─── Tipos de paquete (idénticos a TX y RX) ──────────────────
#define PKT_DATA   0x01
#define PKT_REQ    0x02

// ─── Configuración del repetidor ─────────────────────────────
//  El delay aleatorio reduce la probabilidad de colisión con
//  otros nodos que puedan estar transmitiendo al mismo tiempo.
#define REPEAT_DELAY_MIN_MS   60UL
#define REPEAT_DELAY_MAX_MS  160UL

//  Cache de deduplicación: evita reenviar el mismo paquete dos
//  veces (e.g. si hay dos repetidores en cadena o el TX hace
//  burst y el mismo paquete llega por dos caminos).
#define DEDUP_CACHE_SIZE      32
#define DEDUP_TTL_MS       60000UL   // olvidar entrada tras 60 s

// ─── Watchdog ────────────────────────────────────────────────
#define WDT_TIMEOUT_SEC   90

// ─── Estructuras packed (deben coincidir byte a byte con TX/RX)
typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char     timestamp[20];
  int32_t  lat;
  int32_t  lon;
  uint16_t speed;
  uint8_t  flags;
} data_pkt_t;   // 36 bytes

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;    // 7 bytes

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t)  ==  7, "req_pkt_t size mismatch");

// ─── Cache de deduplicación ───────────────────────────────────
struct DedupEntry {
  uint8_t       pkt_type;
  uint16_t      device_id;
  //  Para DATA: almacena seq
  //  Para REQ:  almacena from_seq (identifica unívocamente la solicitud)
  uint16_t      key_seq;
  unsigned long seen_at_ms;
  bool          valid;
};

DedupEntry dedup_cache[DEDUP_CACHE_SIZE];

// ─── PMU ─────────────────────────────────────────────────────
XPowersPMU PMU;
bool       pmuOk = false;

// ─── Estadísticas (Serial) ───────────────────────────────────
uint32_t stat_rx        = 0;
uint32_t stat_forwarded = 0;
uint32_t stat_dropped   = 0;
uint32_t stat_errors    = 0;

// ─── Flag de interrupción ─────────────────────────────────────
volatile bool packetReady = false;

void IRAM_ATTR onDIO0Rise() {
  packetReady = true;
}

// ─── Forward declarations ─────────────────────────────────────
void  initPMU();
void  initLoRa();
bool  isDuplicate(uint8_t pkt_type, uint16_t device_id, uint16_t key_seq);
void  addToDedup (uint8_t pkt_type, uint16_t device_id, uint16_t key_seq);
void  forwardPacket(const uint8_t* buf, size_t len,
                    uint8_t pkt_type, uint16_t dev, uint16_t seq_key);
void  printStats();


// ============================================================
//  Cache de deduplicación
// ============================================================
bool isDuplicate(uint8_t pkt_type, uint16_t device_id, uint16_t key_seq) {
  unsigned long now = millis();
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    DedupEntry& e = dedup_cache[i];
    if (!e.valid) continue;

    // Expirar entradas antiguas al paso
    if (now - e.seen_at_ms > DEDUP_TTL_MS) {
      e.valid = false;
      continue;
    }
    if (e.pkt_type == pkt_type && e.device_id == device_id && e.key_seq == key_seq)
      return true;
  }
  return false;
}

void addToDedup(uint8_t pkt_type, uint16_t device_id, uint16_t key_seq) {
  unsigned long now = millis();

  // Buscar slot vacío o expirado primero
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    DedupEntry& e = dedup_cache[i];
    if (!e.valid || (now - e.seen_at_ms > DEDUP_TTL_MS)) {
      e = { pkt_type, device_id, key_seq, now, true };
      return;
    }
  }

  // Cache llena → reemplazar la entrada más antigua
  int oldest_idx = 0;
  for (int i = 1; i < DEDUP_CACHE_SIZE; i++) {
    if (dedup_cache[i].seen_at_ms < dedup_cache[oldest_idx].seen_at_ms)
      oldest_idx = i;
  }
  dedup_cache[oldest_idx] = { pkt_type, device_id, key_seq, now, true };
}


// ============================================================
//  PMU — encender solo los rieles necesarios para el LoRa
//        y configurar carga para 18650 + solar
// ============================================================
void initPMU() {
  Wire.begin(PMU_SDA, PMU_SCL);

  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL)) {
    Serial.println("[PMU] AXP2101 no encontrado — sin gestion de energia");
    return;
  }
  pmuOk = true;

  // ── Rieles activos: solo lo que alimenta al LoRa ─────────────
  PMU.setDC1Voltage(3300);    PMU.enableDC1();    // VCC principal ESP32
  PMU.setALDO2Voltage(3300);  PMU.enableALDO2();  // LoRa VCC
  PMU.setALDO3Voltage(3300);  PMU.enableALDO3();  // LoRa VCC (backup/lógica)

  // ── Deshabilitar GPS: ahorra ~25 mA ──────────────────────────
  //    En T-Beam v1.1 el GPS se alimenta por ALDO4
  PMU.disableALDO4();

  // ── Otros LDOs no necesarios ─────────────────────────────────
  PMU.disableDLDO1();
  PMU.disableDLDO2();
  PMU.disableCPUSLDO();   // periférico interno del AXP

  // ── Carga: optimizado para panel solar pequeño ───────────────
  //    500 mA es seguro para paneles de 2–5 W
  //    Voltaje de corte 4.2 V para batería estándar 18650
  PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
  PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

  // ── Protección: ADC de batería activo para monitoreo ─────────
  PMU.enableBattDetection();

  Serial.printf("[PMU] OK | Bat: %u mV | Cargando: %s\n",
                PMU.getBattVoltage(),
                PMU.isCharging() ? "Si" : "No");
}


// ============================================================
//  LoRa — mismos parámetros que TX y RX
// ============================================================
void initLoRa() {
  Serial.println("[LoRa] Iniciando SPI...");  Serial.flush();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  Serial.println("[LoRa] Configurando pines...");  Serial.flush();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  Serial.println("[LoRa] Llamando LoRa.begin()...");  Serial.flush();
  bool ok = false;
  for (int attempt = 1; attempt <= 3 && !ok; attempt++) {
    ok = LoRa.begin(LORA_FREQ);
    Serial.printf("[LoRa] Intento %d: %s\n", attempt, ok ? "OK" : "FALLO");
    Serial.flush();
    if (!ok) delay(500);
  }
  if (!ok) { delay(2000); ESP.restart(); }

  Serial.println("[LoRa] setBandwidth...");   Serial.flush();
  LoRa.setSignalBandwidth(125E3);

  Serial.println("[LoRa] setCodingRate...");  Serial.flush();
  LoRa.setCodingRate4(6);

  Serial.println("[LoRa] setTxPower...");     Serial.flush();
  LoRa.setTxPower(20);

  Serial.println("[LoRa] enableCrc...");      Serial.flush();
  LoRa.enableCrc();

  Serial.println("[LoRa] setSF...");          Serial.flush();
  LoRa.setSpreadingFactor(8);

  Serial.println("[LoRa] receive()...");      Serial.flush();
  
  LoRa.receive();
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onDIO0Rise, RISING);
  Serial.println("[LoRa] OK — RX:SF8 TX:SF9 BW125 CR4/6");

  Serial.println("[LoRa] OK completo");       Serial.flush();
}

// ============================================================
//  Reenviar un paquete con delay anti-colisión
// ============================================================
void forwardPacket(const uint8_t* buf, size_t len,
                   uint8_t pkt_type, uint16_t dev, uint16_t seq_key) {

  uint32_t wait_ms = random(REPEAT_DELAY_MIN_MS, REPEAT_DELAY_MAX_MS);
  delay(wait_ms);

  // ── Log detallado de lo que se va a reenviar ─────────────────
  if (pkt_type == PKT_DATA) {
    const data_pkt_t* p = (const data_pkt_t*)buf;
    Serial.printf("[FWD→] DATA dev=%u seq=%u ts=%s lat=%.6f lon=%.6f "
                  "spd=%.1f km/h flags=0x%02X (SF8→SF9)\n",
                  p->device_id, p->seq, p->timestamp,
                  p->lat / 1e7f, p->lon / 1e7f,
                  p->speed / 10.0f, p->flags);
  } else {
    const req_pkt_t* r = (const req_pkt_t*)buf;
    Serial.printf("[FWD→] REQ  dev=%u seq %u..%u (%u faltantes) (SF8→SF9)\n",
                  r->device_id, r->from_seq, r->to_seq,
                  (uint16_t)(r->to_seq - r->from_seq + 1));
  }

  // ── Cambiar a SF9 solo para transmitir ───────────────────────
  LoRa.idle();
  LoRa.setSpreadingFactor(9);

  LoRa.beginPacket();
  LoRa.write(buf, len);
  bool ok = (LoRa.endPacket(false) == 1);

  // ── Volver a SF8 para seguir escuchando ──────────────────────
  LoRa.setSpreadingFactor(8);
  LoRa.receive();

  if (ok) {
    stat_forwarded++;
    Serial.printf("[FWD✓] fwd=%lu drop=%lu err=%lu\n",
                  stat_forwarded, stat_dropped, stat_errors);
  } else {
    stat_errors++;
    Serial.println("[FWD✗] TX fallo");
  }
}


// ============================================================
//  Light sleep hasta que DIO0 suba (RxDone del SX1278)
//
//  Consumo aproximado:
//    SX1278 en RX continua  ~10 mA   (no cambia, siempre activo)
//    ESP32 activo a 80 MHz  ~30 mA
//    ESP32 en light sleep    ~0.8 mA
//    ────────────────────────────────
//    Ahorro por light sleep  ~29 mA  (muy significativo con batería)
//
//  El SPI y los periféricos se mantienen en light sleep, a
//  diferencia de deep sleep. Al despertar el código continúa
//  en la línea siguiente sin necesidad de re-inicializar nada.
// ============================================================


// ============================================================
void printStats() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 30000) return;
  lastPrint = millis();

  Serial.printf("[STAT] rx=%lu fwd=%lu drop=%lu err=%lu",
                stat_rx, stat_forwarded, stat_dropped, stat_errors);
  if (pmuOk) {
    Serial.printf(" | bat=%u mV chg=%s",
                  PMU.getBattVoltage(),
                  PMU.isCharging() ? "Y" : "N");
  }
  Serial.println();
}


// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[REPEATER] T-Beam v1.1 — Iniciando");
  Serial.println("[REPEATER] PKT_DATA=0x01  PKT_REQ=0x02");
  Serial.printf ("[REPEATER] data_pkt_t=%u bytes  req_pkt_t=%u bytes\n",
                 sizeof(data_pkt_t), sizeof(req_pkt_t));

  // ── 1. Apagar WiFi y BT: ahorra ~80-120 mA ──────────────────
  WiFi.mode(WIFI_OFF);
  btStop();

  // ── 2. Reducir CPU a 80 MHz: ahorra ~20-30 mA vs 240 MHz ────
  //       80 MHz es más que suficiente para procesar 36 bytes
  setCpuFrequencyMhz(80);
  Serial.printf("[SYS] CPU: %u MHz\n", getCpuFrequencyMhz());

  // ── 3. Inicializar caché ─────────────────────────────────────
  memset(dedup_cache, 0, sizeof(dedup_cache));

  // ── 4. PMU: encender LoRa, apagar GPS ───────────────────────
  initPMU();

  // ── 5. LoRa ─────────────────────────────────────────────────
  initLoRa();

  // ── 6. Watchdog de hardware ─────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  Serial.println("[REPEATER] Listo — modo RX continuo + light sleep");
  // al final del setup(), justo antes de cerrar la llave
  Serial.println("[SETUP] Completo — entrando al loop");
  Serial.flush();
}


// ============================================================
//  LOOP
// ============================================================
void loop() {
  esp_task_wdt_reset();
  printStats();

  // Si no hay paquete listo, dormir 10 ms y salir
  // Esto permite al ESP32 usar su idle task interno
  if (!packetReady) {
    delay(10);
    return;
  }
  packetReady = false;

  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  if (!LoRa.available()) return;

  stat_rx++;
  uint8_t pkt_type = LoRa.peek();

  if (pkt_type == PKT_DATA && packetSize == (int)sizeof(data_pkt_t)) {
    data_pkt_t pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
    int   rssi = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();

    if (pkt.device_id == 0 || pkt.device_id > 9999) { stat_errors++; return; }

    if (isDuplicate(PKT_DATA, pkt.device_id, pkt.seq)) {
      stat_dropped++;
      Serial.printf("[DUP] DATA dev=%u seq=%u\n", pkt.device_id, pkt.seq);
      return;
    }

    addToDedup(PKT_DATA, pkt.device_id, pkt.seq);
    Serial.printf("[RX] DATA dev=%u seq=%u ts=%s lat=%.6f lon=%.6f "
                  "spd=%.1f km/h RSSI=%d SNR=%.1f\n",
                  pkt.device_id, pkt.seq, pkt.timestamp,
                  pkt.lat / 1e7f, pkt.lon / 1e7f,
                  pkt.speed / 10.0f, rssi, snr);

    forwardPacket((uint8_t*)&pkt, sizeof(pkt),
                  PKT_DATA, pkt.device_id, pkt.seq);

  } else if (pkt_type == PKT_REQ && packetSize == (int)sizeof(req_pkt_t)) {
    req_pkt_t req;
    LoRa.readBytes((uint8_t*)&req, sizeof(req));

    if (isDuplicate(PKT_REQ, req.device_id, req.from_seq)) {
      stat_dropped++;
      return;
    }

    addToDedup(PKT_REQ, req.device_id, req.from_seq);
    Serial.printf("[RX] REQ  dev=%u seq %u..%u (%u faltantes)\n",
                  req.device_id, req.from_seq, req.to_seq,
                  (uint16_t)(req.to_seq - req.from_seq + 1));

    forwardPacket((uint8_t*)&req, sizeof(req),
                  PKT_REQ, req.device_id, req.from_seq);

  } else {
    Serial.printf("[IGN] type=0x%02X size=%d\n", pkt_type, packetSize);
    while (LoRa.available()) LoRa.read();
    stat_errors++;
  }
}