// ============================================================
//  TX-MiniLoRa-GPS.ino  —  v8
//  Store-and-Forward + Watchdog HW + Hard-reset RA-02 + BLE UART
//
//  BLE: implementa Nordic UART Service (NUS)
//  Usa la app "Serial Bluetooth Terminal" (Android/iOS)
//  o "nRF Connect" para recibir los mensajes.
//
//  Dispositivo BLE visible como: "GPS-TX-1"  (cambia DEVICE_ID)
//
//  Mensajes que envía por BLE:
//    [OK]  seq=5 | 2024-01-15T14:23:01 | 10.491200 -66.879100 | 0.0 km/h
//    [ERR] seq=5 fallos=1/5
//    [RST] LoRa reiniciado OK
//    [WDT] Boot - LoRa OK
//    [WDT] Boot - LoRa FALLO
// ============================================================
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <SPI.h>
#include "esp_task_wdt.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <LittleFS.h>

// ─── LoRa Pins ───────────────────────────────────────────────
#define LORA_SCK   4
#define LORA_MISO  5
#define LORA_MOSI  6
#define LORA_SS    7
#define LORA_RST   10
#define LORA_DIO0  3
#define LORA_FREQ  433E6

// ─── GPS Pins ────────────────────────────────────────────────
#define GPS_RX 20
#define GPS_TX 21

// ─── Configuración ───────────────────────────────────────────
#define DEVICE_ID        1
#define HDOP_MAX         3.0f
#define INTERVAL_MOVING  5000UL
#define INTERVAL_STATIC  30000UL
#define SPEED_THRESHOLD  2.0f

// ─── Protocolo ───────────────────────────────────────────────
#define LISTEN_WINDOW_MS  1200
#define BURST_GAP_MS      80
#define BUF_SIZE          2400     // ventana RAM: últimos N paquetes (~84 KB, ~3.3 h)
#define FS_BUF_SIZE       29000   // capacidad flash (~1 MB, ajustar según partición)
#define FS_BUF_FILE       "/buf.bin"

// ─── Watchdog / recuperación ─────────────────────────────────
#define WDT_TIMEOUT_SEC   60
#define MAX_LORA_FAILS    5

// ─── BLE: Nordic UART Service (NUS) ──────────────────────────
//  Estos UUIDs son estándar — reconocidos por la mayoría de apps
#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify → teléfono
#define BLE_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write  ← teléfono

// ─── Tipos de paquete ────────────────────────────────────────
#define PKT_DATA 0x01
#define PKT_REQ  0x02

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
} data_pkt_t;   // 36 bytes

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;    // 7 bytes

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t)  ==  7, "req_pkt_t size mismatch");

// ─── Buffer circular (RAM) ───────────────────────────────────
data_pkt_t tx_buf[BUF_SIZE];   // ventana deslizante de los últimos BUF_SIZE paquetes
uint16_t   current_seq = 0;
uint16_t   oldest_seq  = 0;

// ─── NVS: persistencia de secuencia ──────────────────────────
Preferences prefs;

void seqLoad() {
  prefs.begin("tracker", true);
  current_seq = prefs.getUShort("seq", 0);
  oldest_seq  = prefs.getUShort("oseq", 0);
  prefs.end();
  Serial.printf("[NVS] seq cargado: current=%u oldest=%u\n", current_seq, oldest_seq);
}

void seqSave() {
  prefs.begin("tracker", false);
  prefs.putUShort("seq",  current_seq);
  prefs.putUShort("oseq", oldest_seq);
  prefs.end();
}

// ─── LittleFS: buffer en flash ───────────────────────────────
//  Layout: registro i ocupa offset = (seq % FS_BUF_SIZE) * sizeof(data_pkt_t)
//  El archivo crece 36 bytes por cada punto GPS — sin pre-creación en el arranque.
//  Si LittleFS no está disponible, fs_ok=false y todas las ops son no-op.

bool fs_ok = false;   // LittleFS montado correctamente

bool fsWrite(const data_pkt_t& p) {
  if (!fs_ok) return false;
  uint32_t offset = (uint32_t)(p.seq % FS_BUF_SIZE) * sizeof(data_pkt_t);

  // "w+" crea el archivo si no existe; "r+" reutiliza y puede extenderlo
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
  // Verificar que el offset ya fue escrito (el archivo puede estar incompleto al inicio)
  if (f.size() < offset + sizeof(data_pkt_t)) { f.close(); return false; }
  f.seek(offset);
  bool ok = (f.read((uint8_t*)&out, sizeof(data_pkt_t)) == sizeof(data_pkt_t));
  f.close();
  return ok && (out.seq == seq) && (out.pkt_type == PKT_DATA);
}

void fsInit() {
  // true = formatear si la partición no está inicializada (solo en primer arranque)
  // IMPORTANTE: requiere "8M with spiffs" o similar en Tools → Partition Scheme
  if (!LittleFS.begin(true)) {
    bleNotify("[FS] LittleFS no disponible — verifica Partition Scheme");
    bleNotify("[FS] El sistema funciona sin persistencia en flash");
    return;   // no fatal: el dispositivo sigue enviando normalmente
  }

  fs_ok = true;   // LittleFS listo — el archivo crece con cada paquete GPS

  if (LittleFS.exists(FS_BUF_FILE)) {
    File tmp = LittleFS.open(FS_BUF_FILE, "r");
    char msg[50];
    snprintf(msg, sizeof(msg), "[FS] Buffer existente (%u KB / %u pkts)",
             (uint32_t)tmp.size() / 1024, (uint32_t)(tmp.size() / sizeof(data_pkt_t)));
    tmp.close();
    bleNotify(msg);
  } else {
    bleNotify("[FS] Buffer vacio — se llenara con datos GPS");
  }

  // Restaurar la ventana RAM con los últimos BUF_SIZE paquetes del historial
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
    char msg[50];
    snprintf(msg, sizeof(msg), "[FS] %u paquetes restaurados en RAM", restored);
    bleNotify(msg);
  }
}

// ─── BLE globals ─────────────────────────────────────────────
BLEServer*         bleServer   = nullptr;
BLECharacteristic* bleTxChar   = nullptr;
bool               bleConnected = false;

// ─── Globals generales ───────────────────────────────────────
TinyGPSPlus    gps;
HardwareSerial GPSSerial(1);
uint32_t       pktSent       = 0;
uint32_t       burstSent     = 0;
unsigned long  lastSend      = 0;
uint8_t        loraFailCount = 0;

// ─── Forward declarations ─────────────────────────────────────
void  bleNotify(const char* msg);
void  bleSetup();
void  loraHardReset();
bool  loraApplyConfig();
bool  loraReinit();
bool  loraSendPacket(const uint8_t* data, size_t len);
bool  buildAndBuffer();
void  sendLatest();
void  listenForReq();
void  sendBurst(uint16_t from_seq, uint16_t to_seq);
bool  seqInBuffer(uint16_t seq);
bool  seqInFlash(uint16_t seq);
void  printDebug(const data_pkt_t& p);
void  addDays(int& year, int& month, int& day, int n);
void  gpsToISO_VET(int year, int month, int day,
                   int hour, int minute, int second, char* out);


// ============================================================
//  BLE: callbacks de conexión/desconexión
// ============================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("[BLE] Cliente conectado");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("[BLE] Cliente desconectado — re-advertising");
    BLEDevice::startAdvertising();  // volver a ser visible
  }
};


// ============================================================
//  bleNotify: envía string por BLE UART si hay cliente conectado
//  También lo imprime por Serial siempre
// ============================================================
void bleNotify(const char* msg) {
  Serial.println(msg);
  if (bleConnected && bleTxChar) {
    bleTxChar->setValue((uint8_t*)msg, strlen(msg));
    bleTxChar->notify();
    delay(10);  // pequeña pausa para que el stack BLE procese
  }
}


// ============================================================
//  Inicializar BLE con Nordic UART Service
// ============================================================
void bleSetup() {
  char devName[20];
  snprintf(devName, sizeof(devName), "GPS-TX-%u", DEVICE_ID);

  BLEDevice::init(devName);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  // TX characteristic: ESP32 → teléfono (notify)
  bleTxChar = svc->createCharacteristic(
    BLE_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  bleTxChar->addDescriptor(new BLE2902());

  // RX characteristic: teléfono → ESP32 (write) — por si se quiere enviar comandos
  BLECharacteristic* rxChar = svc->createCharacteristic(
    BLE_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  (void)rxChar;  // no usado por ahora

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.printf("[BLE] Advertising como '%s'\n", devName);
}


// ============================================================
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
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  return true;
}

bool loraReinit() {
  bleNotify("[RST] Reiniciando modulo LoRa...");
  LoRa.end();
  delay(50);
  loraHardReset();
  if (!loraApplyConfig()) {
    bleNotify("[RST] LoRa FALLO — reinicio pendiente");
    return false;
  }
  loraFailCount = 0;
  bleNotify("[RST] LoRa reiniciado OK");
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
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[TX] ID=%u  BUF=%u pts (~%lu min)\n",
                DEVICE_ID, BUF_SIZE, (unsigned long)(BUF_SIZE * 5 / 60));

  // ── Recuperar secuencia desde NVS ────────────────────────────
  seqLoad();

  // ── Montar LittleFS y restaurar buffer ───────────────────────
  fsInit();

  // ── BLE primero: así ya podemos notificar el estado del boot ──
  bleSetup();

  // ── Watchdog de hardware ─────────────────────────────────────
  esp_task_wdt_deinit();
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // ── GPS ──────────────────────────────────────────────────────
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // ── LoRa: hard-reset SIEMPRE al arrancar ─────────────────────
  bleNotify("[BOOT] Hard-reset RA-02...");
  loraHardReset();

  bool ok = false;
  for (int i = 0; i < 3 && !ok; i++) {
    ok = loraApplyConfig();
    if (!ok) {
      char msg[40];
      snprintf(msg, sizeof(msg), "[BOOT] LoRa intento %d/3 fallo", i + 1);
      bleNotify(msg);
      loraHardReset();
      delay(300);
    }
  }

  if (!ok) {
    bleNotify("[BOOT] LoRa sin respuesta — WDT reiniciara");
    return;
  }

  bleNotify("[BOOT] LoRa OK — sistema listo");
}


// ============================================================
void loop() {
  esp_task_wdt_reset();

  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  bool moving = gps.location.isValid() &&
                gps.speed.isValid()    &&
                (gps.speed.kmph() >= SPEED_THRESHOLD);

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
  bool moving = hasFix && (gps.speed.kmph() >= SPEED_THRESHOLD);

  if (!hasFix) {
    // Notificar solo cada 30 s para no saturar BLE
    static unsigned long lastNoFix = 0;
    if (millis() - lastNoFix > 30000) {
      bleNotify("[GPS] Sin fix");
      lastNoFix = millis();
    }
    return false;
  }

  data_pkt_t& p = tx_buf[current_seq % BUF_SIZE];
  p.pkt_type  = PKT_DATA;
  p.device_id = DEVICE_ID;
  p.seq       = current_seq;
  p.flags     = 0;
  p.lat       = (int32_t)(gps.location.lat() * 1e7);
  p.lon       = (int32_t)(gps.location.lng() * 1e7);
  p.speed     = (uint16_t)(gps.speed.kmph() * 10);

  gpsToISO_VET(gps.date.year(),  gps.date.month(),  gps.date.day(),
               gps.time.hour(),  gps.time.minute(), gps.time.second(),
               p.timestamp);

  p.flags |= 0x01;
  if (hdopOk) p.flags |= 0x02;
  if (moving) p.flags |= 0x04;

  current_seq++;
  // oldest_seq avanza cuando el buffer de FLASH está lleno (no el de RAM)
  if ((uint16_t)(current_seq - oldest_seq) > FS_BUF_SIZE) oldest_seq++;
  seqSave();      // persiste contadores en NVS
  fsWrite(p);     // persiste paquete en LittleFS (36 bytes)

  return true;
}


// ============================================================
void sendLatest() {
  const data_pkt_t& p = tx_buf[(current_seq - 1) % BUF_SIZE];

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
    bleNotify(msg);
  } else {
    loraFailCount++;
    snprintf(msg, sizeof(msg),
             "[ERR] seq=%u fallos=%u/%u",
             p.seq, loraFailCount, MAX_LORA_FAILS);
    bleNotify(msg);

    if (loraFailCount >= MAX_LORA_FAILS) {
      if (!loraReinit()) {
        bleNotify("[ERR] loraReinit fallo -> ESP.restart");
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
    if (sz != sizeof(req_pkt_t)) { while (LoRa.available()) LoRa.read(); continue; }

    req_pkt_t req;
    LoRa.readBytes((uint8_t*)&req, sizeof(req));
    if (req.pkt_type != PKT_REQ || req.device_id != DEVICE_ID) continue;

    char msg[60];
    snprintf(msg, sizeof(msg), "[REQ] seq %u..%u (%u faltantes)",
             req.from_seq, req.to_seq,
             (uint16_t)(req.to_seq - req.from_seq + 1));
    bleNotify(msg);

    LoRa.idle();
    sendBurst(req.from_seq, req.to_seq);
    LoRa.receive();
    t0 = millis();
  }
  LoRa.idle();
}


// ============================================================
void sendBurst(uint16_t from_seq, uint16_t to_seq) {
  uint16_t sent = 0, skipped = 0;

  for (uint16_t s = from_seq; s != (uint16_t)(to_seq + 1); s++) {
    esp_task_wdt_reset();

    data_pkt_t tmp;
    const data_pkt_t* pkt = nullptr;

    if (seqInBuffer(s)) {
      // ── Caso 1: está en la ventana RAM ───────────────────────
      const data_pkt_t& ramPkt = tx_buf[s % BUF_SIZE];
      if (ramPkt.seq == s) pkt = &ramPkt;
    }

    if (!pkt && seqInFlash(s)) {
      // ── Caso 2: fuera de RAM, buscar en LittleFS ──────────────
      if (fsRead(s, tmp)) pkt = &tmp;
    }

    if (!pkt) { skipped++; continue; }

    if (loraSendPacket((uint8_t*)pkt, sizeof(data_pkt_t))) {
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
    delay(BURST_GAP_MS);
  }

  char msg[60];
  snprintf(msg, sizeof(msg), "[BURST] %u enviados %u omitidos", sent, skipped);
  bleNotify(msg);
}


// ============================================================
//  seqInBuffer: el paquete está en la ventana RAM
bool seqInBuffer(uint16_t seq) {
  return (uint16_t)(current_seq - seq) <= BUF_SIZE &&
         (uint16_t)(current_seq - seq) >= 1;
}

//  seqInFlash: el paquete puede estar en LittleFS
bool seqInFlash(uint16_t seq) {
  return (uint16_t)(current_seq - seq) <= FS_BUF_SIZE &&
         (uint16_t)(current_seq - seq) >= 1;
}

void printDebug(const data_pkt_t& p) {
  Serial.printf("[PKT] seq=%-5u fix=%c | %s | %+.6f %+.6f | %.1f km/h | buf[%u..%u]\n",
                p.seq, (p.flags & 0x01) ? 'Y' : 'N', p.timestamp,
                p.lat / 1e7f, p.lon / 1e7f, p.speed / 10.0f,
                oldest_seq, (uint16_t)(current_seq - 1));
}

void addDays(int& year, int& month, int& day, int n) {
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  day += n;
  while (true) {
    bool leap = (year%4==0 && year%100!=0) || (year%400==0);
    int d = (month==2 && leap) ? 29 : dim[month-1];
    if (day <= d) break;
    day -= d;
    if (++month > 12) { month=1; year++; }
  }
}

void gpsToISO_VET(int year, int month, int day,
                  int hour, int minute, int second, char* out) {
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (year < 2019) addDays(year, month, day, 7168);
  hour -= 4;
  if (hour < 0) {
    hour += 24; day--;
    if (day < 1) {
      if (--month < 1) { month=12; year--; }
      bool leap = (year%4==0 && year%100!=0)||(year%400==0);
      day = (month==2 && leap) ? 29 : dim[month-1];
    }
  }
  snprintf(out, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
           year, month, day, hour, minute, second);
}