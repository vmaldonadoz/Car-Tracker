// ============================================================
//  TX-MiniLoRa-GPS.ino  —  v8
//  Store-and-Forward + Watchdog HW + Hard-reset RA-02
//  + Persistencia en LittleFS (buffer sobrevive a pérdida de energía)
//
//  Cambios respecto a v7:
//   - Buffer circular persistido en LittleFS (/txbuf.bin)
//   - NVS (Preferences) guarda current_seq, oldest_seq y acked_seq
//   - PKT_ACK: el TX escucha ACKs del RX durante listenForReq()
//     y marca los paquetes confirmados (libera espacio en disco)
//   - Al arrancar se restaura el buffer desde LittleFS si existe
//   - Solo se borra un paquete del disco cuando el RX lo confirma
//   - Escritura en LittleFS: solo el slot nuevo (no todo el buffer)
// ============================================================
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Preferences.h>
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
#define GPS_TX 12

// ─── Configuración ───────────────────────────────────────────
#define DEVICE_ID 103
#define HDOP_MAX 3.0f
#define INTERVAL_MOVING 5000UL
#define INTERVAL_STATIC 5000UL
#define SPEED_THRESHOLD 2.0f

// ─── Protocolo ───────────────────────────────────────────────
#define LISTEN_WINDOW_MS 1200
#define BURST_GAP_MS 350
#define MAX_BURST 15

// 2048 registros × 5 s/registro = 10240 s ≈ 2 h 50 min de historia
// Memoria flash: 2048 × 36 bytes = 73.7 KB
// Potencia de 2: el índice seq % BUF_SIZE no se rompe al dar la vuelta en 65535
#define BUF_SIZE 2048

// ─── Watchdog / recuperación ─────────────────────────────────
#define WDT_TIMEOUT_SEC 60
#define MAX_LORA_FAILS 5

// ─── Tipos de paquete ────────────────────────────────────────
#define PKT_DATA 0x01
#define PKT_REQ  0x02
#define PKT_ACK  0x03   // ← ACK del RX al TX (nuevo en v8)

// ─── Persistencia ────────────────────────────────────────────
// Archivo binario en LittleFS: BUF_SIZE registros de data_pkt_t
// Cada slot[i] corresponde exactamente al índice i del buffer circular
#define TX_BUF_FILE "/txbuf.bin"
#define NVS_NAMESPACE "txstate"

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
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t acked_seq;
  uint8_t  rx_id[4];
} ack_pkt_t;  // 10 bytes  (igual que en el RX)

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t)  ==  7, "req_pkt_t size mismatch");
static_assert(sizeof(ack_pkt_t)  ==  9, "ack_pkt_t size mismatch");

// ─── Buffer circular (RAM — cargado desde LittleFS al arrancar) ──
data_pkt_t tx_buf[BUF_SIZE];
bool       slot_valid[BUF_SIZE];  // true = slot tiene datos no descartados
uint16_t   current_seq = 0;
uint16_t   oldest_seq  = 0;
uint16_t   acked_seq   = 0;       // último seq confirmado por el RX

// ─── Globals generales ───────────────────────────────────────
TinyGPSPlus gps;
XPowersPMU PMU;
Preferences prefs;
HardwareSerial GPSSerial(1);
uint32_t pktSent  = 0;
uint32_t burstSent = 0;
unsigned long lastSend = 0;
uint8_t loraFailCount  = 0;

// ─── Forward declarations ─────────────────────────────────────
void initPMU();
void initStorage();
void loadBufferFromFlash();
void saveSlotToFlash(uint16_t idx);
void freeConfirmedSlots(uint16_t up_to_seq);
void saveState();
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


void gpsSendNMEA(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= *p;
  char out[96];
  snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
  GPSSerial.print(out);
  delay(100);
}

void configureGPS() {
  gpsSendNMEA("PCAS04,7");   // GPS + BeiDou + GLONASS
  gpsSendNMEA("PCAS11,3");   // modo vehículo
  Serial.println("[GPS] Configurado: GPS+BDS+GLONASS, modo vehiculo");
}

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
  PMU.setALDO1Voltage(3300);
  PMU.enableALDO1();  // backup/RTC del GPS → permite hot/warm start
  Serial.println("[PMU] OK");
}

// ============================================================
//  PERSISTENCIA EN LITTLEFS
// ============================================================

// Inicializa LittleFS y crea el archivo del buffer si no existe
void initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Error montando LittleFS — datos NO persistentes");
    return;
  }
  Serial.printf("[FS] LittleFS OK — %u / %u KB usados\n",
                LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);

  // Si el archivo no existe, crear uno vacío con BUF_SIZE slots en cero
  if (!LittleFS.exists(TX_BUF_FILE)) {
    File f = LittleFS.open(TX_BUF_FILE, "w");
    if (f) {
      data_pkt_t empty;
      memset(&empty, 0, sizeof(empty));
      for (uint16_t i = 0; i < BUF_SIZE; i++) {
        f.write((uint8_t*)&empty, sizeof(data_pkt_t));
      }
      f.close();
      Serial.println("[FS] Archivo de buffer creado (vacío)");
    } else {
      Serial.println("[FS] No se pudo crear el archivo de buffer");
    }
  }
}

// Carga el buffer desde LittleFS y restaura current_seq/oldest_seq/acked_seq desde NVS
void loadBufferFromFlash() {
  // ── Leer estado de secuencias desde NVS ──────────────────
  prefs.begin(NVS_NAMESPACE, true);
  current_seq = prefs.getUShort("cur_seq", 0);
  oldest_seq  = prefs.getUShort("old_seq", 0);
  acked_seq   = prefs.getUShort("ack_seq", 0);
  prefs.end();

  Serial.printf("[NVS] Restaurado: current_seq=%u oldest_seq=%u acked_seq=%u\n",
                current_seq, oldest_seq, acked_seq);

  // Siempre limpiar RAM antes de cargar desde flash
  memset(tx_buf,    0, sizeof(tx_buf));
  memset(slot_valid, 0, sizeof(slot_valid));

  // FIX: usar oldest_seq == current_seq como "buffer vacío"
  // (antes: "== 0 && == 0" fallaba si el primer punto se guardó en flash
  //  pero la energía se cortó antes de que el NVS se actualizara)
  if (current_seq == oldest_seq) {
    Serial.println("[FS] Buffer vacío — sin datos previos");
    return;
  }

  // ── Leer buffer desde LittleFS ───────────────────────────
  File f = LittleFS.open(TX_BUF_FILE, "r");
  if (!f) {
    Serial.println("[FS] No se pudo abrir buffer — reiniciando secuencias");
    current_seq = 0; oldest_seq = 0; acked_seq = 0;
    return;
  }

  if (f.size() < (size_t)(BUF_SIZE * sizeof(data_pkt_t))) {
    Serial.println("[FS] Archivo de buffer corrupto — reiniciando secuencias");
    f.close();
    current_seq = 0; oldest_seq = 0; acked_seq = 0;
    return;
  }

  f.read((uint8_t*)tx_buf, BUF_SIZE * sizeof(data_pkt_t));
  f.close();

  // Marcar como válidos los slots dentro del rango [oldest_seq, current_seq)
  // donde el seq almacenado en flash coincide (verificación de integridad)
  uint32_t recovered = 0;
  for (uint16_t s = oldest_seq; s != current_seq; s++) {
    uint16_t idx = s % BUF_SIZE;
    if (tx_buf[idx].seq == s && tx_buf[idx].pkt_type == PKT_DATA) {
      slot_valid[idx] = true;
      recovered++;
    }
  }

  // FIX: avanzar oldest_seq más allá de slots inválidos (liberados o corruptos)
  // para que el buffer no quede referenciando huecos que nunca llegarán
  while (oldest_seq != current_seq && !slot_valid[oldest_seq % BUF_SIZE]) {
    oldest_seq++;
  }

  Serial.printf("[FS] Buffer restaurado: %u/%u paquetes válidos (oldest=%u cur=%u)\n",
                recovered, (uint16_t)(current_seq - oldest_seq + recovered),
                oldest_seq, current_seq);
}

// Escribe un único slot en la posición correcta del archivo (sin reescribir todo)
void saveSlotToFlash(uint16_t idx) {
  File f = LittleFS.open(TX_BUF_FILE, "r+");
  if (!f) {
    // Si no existe (primera escritura), crear
    f = LittleFS.open(TX_BUF_FILE, "w");
    if (!f) {
      Serial.println("[FS] Error abriendo buffer para escritura");
      return;
    }
    // Rellenar con ceros hasta el offset necesario
    data_pkt_t empty; memset(&empty, 0, sizeof(empty));
    for (uint16_t i = 0; i < BUF_SIZE; i++) f.write((uint8_t*)&empty, sizeof(empty));
    f.close();
    f = LittleFS.open(TX_BUF_FILE, "r+");
    if (!f) return;
  }

  uint32_t offset = (uint32_t)idx * sizeof(data_pkt_t);
  f.seek(offset);
  f.write((uint8_t*)&tx_buf[idx], sizeof(data_pkt_t));
  f.close();
}

// Guarda current_seq, oldest_seq y acked_seq en NVS
void saveState() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putUShort("cur_seq", current_seq);
  prefs.putUShort("old_seq", oldest_seq);
  prefs.putUShort("ack_seq", acked_seq);
  prefs.end();
}

// Libera del buffer (marca slot_valid=false) todos los slots confirmados hasta up_to_seq
// También borra el contenido del slot en flash poniéndolo a cero
void freeConfirmedSlots(uint16_t up_to_seq) {
  // Recorrer desde oldest_seq hasta up_to_seq (inclusive)
  for (uint16_t s = oldest_seq; s != (uint16_t)(up_to_seq + 1); s++) {
    uint16_t idx = s % BUF_SIZE;
    if (slot_valid[idx] && tx_buf[idx].seq == s) {
      // Borrar slot en flash (sobrescribir con ceros)
      memset(&tx_buf[idx], 0, sizeof(data_pkt_t));
      slot_valid[idx] = false;
      saveSlotToFlash(idx);
    }
  }

  // Avanzar oldest_seq más allá de los confirmados
  if ((int16_t)(up_to_seq - oldest_seq) >= 0) {
    oldest_seq = (uint16_t)(up_to_seq + 1);
  }

  acked_seq = up_to_seq;
  saveState();

  Serial.printf("[ACK] Confirmados hasta seq=%u — oldest_seq ahora=%u\n",
                up_to_seq, oldest_seq);
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
  delay(500);

  // ── Almacenamiento persistente ───────────────────────────
  initStorage();
  loadBufferFromFlash();   // restaura buffer y secuencias desde flash

  // Buffer UART grande
  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100);
  configureGPS();
  Serial.println("[GPS] Serial iniciado");

  delay(200);
  // ── Watchdog de hardware ─────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // ── LoRa: hard-reset SIEMPRE al arrancar ─────────────────
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

  Serial.printf("[BOOT] TX v8 listo. DEVICE_ID=%u BUF=%u cur=%u old=%u ack=%u\n",
                DEVICE_ID, BUF_SIZE, current_seq, oldest_seq, acked_seq);
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
  bool hasFix = gps.location.isValid() && gps.location.age() < 4000 &&
              gps.date.isValid() && gps.time.isValid();
  bool hdopOk = hasFix && gps.hdop.isValid() && (gps.hdop.hdop() < HDOP_MAX);
  bool moving = true;

  if (!hasFix) {
    static unsigned long lastNoFix = 0;
    if (millis() - lastNoFix > 2000) {
      lastNoFix = millis();
      Serial.println("[GPS] Esperando FIX...");
      Serial.printf("[GPS] chars=%lu sats=%d hdop=%.2f age=%lu csFail=%lu\n",
              gps.charsProcessed(), gps.satellites.value(),
              gps.hdop.hdop(), gps.location.age(), gps.failedChecksum());
      Serial.printf("location=%d date=%d time=%d sats=%d hdop=%.2f\n",
                    gps.location.isValid(),
                    gps.date.isValid(),
                    gps.time.isValid(),
                    gps.satellites.value(),
                    gps.hdop.hdop());
    }
    return false;
  }

  uint16_t idx = current_seq % BUF_SIZE;
  data_pkt_t& p = tx_buf[idx];
  p.pkt_type  = PKT_DATA;
  p.device_id = DEVICE_ID;
  p.seq       = current_seq;
  p.flags     = 0;
  p.lat       = (int32_t)(gps.location.lat() * 1e7);
  p.lon       = (int32_t)(gps.location.lng() * 1e7);
  p.speed     = (uint16_t)(gps.speed.kmph() * 10);

  gpsToISO_VET(gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second(),
               p.timestamp);

  p.flags |= 0x01;
  if (hdopOk) p.flags |= 0x02;
  if (moving)  p.flags |= 0x04;

  // ── Persistir en flash con orden correcto ────────────────
  //
  // ORDEN CRÍTICO: NVS primero, flash después.
  //
  // Si la energía se corta ENTRE los dos pasos:
  //   · Después de NVS pero antes de flash → el slot quedará con
  //     pkt_type=0 en disco; la validación lo descartará. Se pierde
  //     1 punto como máximo. ✓ aceptable.
  //   · Después de flash pero sin NVS → imposible: NVS ya se guardó. ✓
  //
  // Con el orden inverso (flash → NVS), si se corta entre ellos,
  // el NVS quedaba con cur_seq viejo y el punto en flash nunca
  // se recuperaba (bug original).

  // 1. Actualizar secuencias en RAM
  slot_valid[idx] = true;
  current_seq++;
  if ((uint16_t)(current_seq - oldest_seq) > BUF_SIZE) {
    // El slot más antiguo fue sobrescrito — invalidar y avanzar oldest_seq
    slot_valid[oldest_seq % BUF_SIZE] = false;
    oldest_seq++;
  }

  // 2. Guardar secuencias en NVS PRIMERO
  saveState();

  // 3. Escribir el slot en flash
  saveSlotToFlash(idx);

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
//  Escucha ventana: REQ → retransmite burst / ACK → libera slots
// ============================================================
void listenForReq() {
  LoRa.receive();
  unsigned long t0 = millis();

  while (millis() - t0 < LISTEN_WINDOW_MS) {
    esp_task_wdt_reset();
    while (GPSSerial.available()) gps.encode(GPSSerial.read());

    int sz = LoRa.parsePacket();
    if (sz == 0) continue;

    // ── ACK del RX ───────────────────────────────────────
    if (sz == sizeof(ack_pkt_t)) {
      ack_pkt_t ack;
      LoRa.readBytes((uint8_t*)&ack, sizeof(ack));
      if (ack.pkt_type == PKT_ACK && ack.device_id == DEVICE_ID) {
        Serial.printf("[ACK<] seq confirmado hasta %u\n", ack.acked_seq);
        // Liberar slots confirmados.
        // Condición: acked_seq < ack.acked_seq (aritmética de 16 bits mod 65536)
        // La comparación (int16_t)(ack.acked_seq - acked_seq) > 0 cubre
        // correctamente los wraparounds de uint16_t.
        // Caso especial: acked_seq==0 y ack.acked_seq==0 → primer ACK al seq=0
        if ((int16_t)(ack.acked_seq - acked_seq) > 0 ||
            (acked_seq == 0 && oldest_seq == 0 && current_seq > 0)) {
          freeConfirmedSlots(ack.acked_seq);
        }
      }
      continue;
    }

    // ── REQ del RX ───────────────────────────────────────
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
    t0 = millis();  // seguir escuchando (el RX puede pedir más tramos)
  }
  LoRa.idle();
}


// ============================================================
// Espera ms milisegundos pero sigue leyendo el GPS y el watchdog
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

  // Si piden algo más viejo de lo que tenemos, empezar por lo más viejo disponible
  if (!seqInBuffer(from_seq)) {
    uint16_t d = (uint16_t)(current_seq - from_seq);
    if (d > BUF_SIZE && d < 32768) from_seq = oldest_seq;
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

    uint16_t idx = s % BUF_SIZE;
    const data_pkt_t& p = tx_buf[idx];
    if (p.seq != s || !slot_valid[idx]) {
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

    pumpGPS(BURST_GAP_MS);
  }

  char msg[60];
  snprintf(msg, sizeof(msg), "[BURST] %u enviados %u omitidos", sent, skipped);
  Serial.println(msg);
}


// ============================================================
bool seqInBuffer(uint16_t seq) {
  uint16_t d = (uint16_t)(current_seq - seq);
  return d >= 1 && d <= BUF_SIZE;
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
