// ============================================================
//  MQTT-BLE-RX.ino  —  v2 con Store-and-Forward
//  Detecta gaps de secuencia y pide retransmisión al TX
// ============================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

#include <LittleFS.h>

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

// ─── Botón ───────────────────────────────────────────────────
#define BUTTON_PIN 38
#define LONG_PRESS_MS 3000
#define DEBOUNCE_MS 50

// ─── BLE UUIDs ───────────────────────────────────────────────
#define BLE_DEVICE_NAME "LoRa-1"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CREDS_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_STATUS_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ─── Reconexión ──────────────────────────────────────────────
#define WIFI_RETRY_MS 15000
#define MQTT_RETRY_MS 5000

// ─── Número máximo de vehículos distintos a rastrear ─────────
#define MAX_VEHICLES 16

// ─── Tipos de paquete (idénticos al TX) ──────────────────────
#define PKT_DATA 0x01
#define PKT_REQ 0x02

#define TOPIC_GLOBAL "tracker/global"

// ─── Cola offline ─────────────────────────────────────────────
#define OFFLINE_DATA_FILE "/ofq.bin"
#define OFFLINE_MAX_RECORDS 400


bool bleJsonReady = false;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char timestamp[20];
  int32_t lat;
  int32_t lon;
  uint16_t speed;
  uint8_t flags;
} data_pkt_t;  // 20 bytes

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;  // 7 bytes

struct __attribute__((packed)) OfflineRecord {
  data_pkt_t pkt;
  int16_t rssi;
  float snr;
  char stationId[32];
};


static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t) == 7, "req_pkt_t size mismatch");

// ─── Estado de secuencia por vehículo ────────────────────────
struct VehicleState {
  uint16_t device_id;
  uint16_t last_seq;  // último seq recibido correctamente
  uint16_t global_seq;
  bool initialized;  // false = nunca visto este vehículo
  uint32_t rx_count;
  uint32_t gap_count;  // cuántas veces detectamos un gap
};
VehicleState veh_states[MAX_VEHICLES];

// ─── Credenciales y configuración ────────────────────────────
struct Credentials {
  char ssid[64];
  char pass[64];
  char broker[64];
  int port;
  char topicBase[96];
  char stationId[32];
};

// ─── Globals ─────────────────────────────────────────────────
Preferences prefs;
Credentials creds;
WiFiClient espClient;
PubSubClient mqtt(espClient);
XPowersPMU PMU;

BLEServer* bleServer = nullptr;
BLECharacteristic* charCreds = nullptr;
BLECharacteristic* charStatus = nullptr;
bool bleRunning = false;
bool bleClientConnected = false;

unsigned long buttonPressStart = 0;
bool buttonHeld = false;
bool longPressHandled = false;
unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;

uint32_t totalRx = 0;
uint32_t totalPub = 0;
uint32_t totalErr = 0;



// ─── Forward declarations ─────────────────────────────────────
VehicleState* findOrCreateVehicle(uint16_t device_id);
void handleDataPacket(const data_pkt_t& p, int rssi, float snr);
void sendReq(uint16_t device_id, uint16_t from_seq, uint16_t to_seq);
void publishPacket(const data_pkt_t& p, int rssi, float snr);
void printPacket(const data_pkt_t& p, int rssi, float snr, bool gap);
void startBLE();
bool loadCredentials();
void saveCredentials();
void connectWiFi();
void connectMQTT();
void checkButton();
void initPMU();
void initLoRa();


// ============================================================
//  BLE
// ============================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleClientConnected = true;
    charStatus->setValue("Listo. Envía JSON con credenciales.");
    charStatus->notify();
  }
  void onDisconnect(BLEServer*) override {
    bleClientConnected = false;
    BLEDevice::startAdvertising();
  }
};

static std::string bleBuffer = "";

class CredsCallback : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* c) override {

    std::string rx = c->getValue();

    // 🔥 reset si empieza nuevo JSON
    if (rx.find('{') != std::string::npos) {
      bleBuffer.clear();
    }

    bleBuffer.append(rx);

    Serial.printf("[BLE] chunk %d bytes\n", rx.size());

    if (bleBuffer.find('}') != std::string::npos) {
      bleJsonReady = true;
    }
  }
};

void processBLEJson() {

  if (!bleJsonReady) return;

  Serial.println("[BLE] Parsing JSON...");

  std::string clean;
  clean.reserve(bleBuffer.size());

  for (size_t i = 0; i < bleBuffer.size(); i++) {

    unsigned char c = (unsigned char)bleBuffer[i];

    // ❌ eliminar NULL
    if (c == 0x00) continue;

    // ❌ eliminar NBSP UTF-8 (0xC2 0xA0)
    if (c == 0xC2 && i + 1 < bleBuffer.size() && (unsigned char)bleBuffer[i + 1] == 0xA0) {
      i++;  // saltar el 0xA0
      continue;
    }

    // ❌ eliminar cualquier otro control raro
    if (c < 32 && c != '\n' && c != '\r' && c != '\t') continue;

    clean += (char)c;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, clean.c_str());

  if (err) {
    Serial.printf("[BLE] JSON error: %s\n", err.c_str());

    Serial.println("[BLE] RAW CLEAN:");
    Serial.println(clean.c_str());

    bleBuffer.clear();
    bleJsonReady = false;
    return;
  }

  // ✔️ validar campos mínimos
  const char* req[] = { "ssid", "pass", "broker", "topic" };

  for (auto f : req) {
    if (!doc.containsKey(f)) {
      Serial.printf("[BLE] Falta campo: %s\n", f);
      bleBuffer.clear();
      bleJsonReady = false;
      return;
    }
  }

  // ✔️ guardar credenciales
  strlcpy(creds.ssid, doc["ssid"] | "", sizeof(creds.ssid));
  strlcpy(creds.pass, doc["pass"] | "", sizeof(creds.pass));
  strlcpy(creds.broker, doc["broker"] | "", sizeof(creds.broker));
  creds.port = doc["port"] | 1883;
  strlcpy(creds.topicBase, doc["topic"] | "gps/vehiculos", sizeof(creds.topicBase));
  strlcpy(creds.stationId, doc["station"] | "parada", sizeof(creds.stationId));

  saveCredentials();

  Serial.println("[BLE] JSON OK → credenciales guardadas");

  charStatus->setValue("OK guardado");
  charStatus->notify();

  bleBuffer.clear();
  bleJsonReady = false;

  if (bleRunning) {
    BLEDevice::stopAdvertising();
    delay(200);
    BLEDevice::deinit(true);  // libera la radio completamente
    bleRunning = false;
  }

  delay(300);
  ESP.restart();  // reinicio directo, sin flag en NVS
}

//

void startBLE() {
  if (bleRunning) return;
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  BLEService* service = bleServer->createService(SERVICE_UUID);
  charCreds = service->createCharacteristic(CHAR_CREDS_UUID, BLECharacteristic::PROPERTY_WRITE);
  charCreds->setCallbacks(new CredsCallback());
  charStatus = service->createCharacteristic(CHAR_STATUS_UUID,
                                             BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  charStatus->addDescriptor(new BLE2902());
  charStatus->setValue("Esperando JSON: {ssid,pass,broker,topic,station,port}");
  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  bleRunning = true;
  Serial.println("[BLE] Activo → " BLE_DEVICE_NAME);
}


// ============================================================
//  NVS
// ============================================================
bool loadCredentials() {
  prefs.begin("config", true);
  bool exists = prefs.isKey("ssid");
  if (exists) {
    strlcpy(creds.ssid, prefs.getString("ssid", "").c_str(), sizeof(creds.ssid));
    strlcpy(creds.pass, prefs.getString("pass", "").c_str(), sizeof(creds.pass));
    strlcpy(creds.broker, prefs.getString("broker", "").c_str(), sizeof(creds.broker));
    creds.port = prefs.getInt("port", 1883);
    strlcpy(creds.topicBase, prefs.getString("topic", "gps/vehiculos").c_str(), sizeof(creds.topicBase));
    strlcpy(creds.stationId, prefs.getString("station", "parada").c_str(), sizeof(creds.stationId));
  }
  prefs.end();
  bool valid = exists && strlen(creds.ssid) > 0 && strlen(creds.broker) > 0;
  Serial.printf("[NVS] %s\n", valid ? "OK" : "Sin credenciales — usar BLE");
  return valid;
}

void saveCredentials() {
  prefs.begin("config", false);
  prefs.putString("ssid", creds.ssid);
  prefs.putString("pass", creds.pass);
  prefs.putString("broker", creds.broker);
  prefs.putInt("port", creds.port);
  prefs.putString("topic", creds.topicBase);
  prefs.putString("station", creds.stationId);
  prefs.end();
}


// ============================================================
//  Botón
// ============================================================
void checkButton() {
  static unsigned long lastDebounce = 0;
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  if (pressed && !buttonHeld) {
    if (millis() - lastDebounce < DEBOUNCE_MS) return;
    buttonHeld = true;
    longPressHandled = false;
    buttonPressStart = millis();
  }
  if (buttonHeld && !longPressHandled && millis() - buttonPressStart >= LONG_PRESS_MS) {
    longPressHandled = true;
    Serial.println("[BTN] → BLE activado");
    startBLE();
  }
  if (!pressed && buttonHeld) {
    lastDebounce = millis();
    buttonHeld = false;
  }
}


// ============================================================
//  WiFi / MQTT
// ============================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect(true);  // ← limpia estado anterior
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(creds.ssid, creds.pass);
  Serial.printf("[WiFi] Conectando a '%s'...\n", creds.ssid);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char* lastSlash = strrchr(topic, '/');
  if (!lastSlash) return;

  uint16_t device_id = (uint16_t)atoi(lastSlash + 1);
  if (device_id == 0) return;

  char buf[16] = { 0 };
  memcpy(buf, payload, min((unsigned int)15, length));
  uint16_t global_seq = (uint16_t)atoi(buf);

  VehicleState* v = findOrCreateVehicle(device_id);
  if (!v) return;

  // Actualizar global_seq SIEMPRE (sirve para dedup en publishPacket)
  if ((uint16_t)(global_seq - v->global_seq) < 32000) {
    v->global_seq = global_seq;
  }

  // Actualizar last_seq SOLO si nunca hemos visto este vehículo
  // Una vez inicializado, last_seq lo maneja solo la recepción LoRa
  if (!v->initialized) {
    v->last_seq = global_seq;
    v->initialized = true;
    Serial.printf("[MQTT] Bus #%u inicializado desde retained: last_seq=%u\n",
                  device_id, global_seq);
  }
  // ← NO hay else: si ya está inicializado, last_seq no se toca aquí
}

void connectMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String clientId = String("RX_") + creds.stationId + "_" + mac.substring(6);

  char lwtTopic[128];
  snprintf(lwtTopic, sizeof(lwtTopic), "tracker/status/%s", creds.stationId);

  // ← registrar callback ANTES de connect

  if (mqtt.connect(clientId.c_str(), lwtTopic, 1, true, "offline")) {
    mqtt.publish(lwtTopic, "online", true);
    Serial.printf("[MQTT] Conectado como '%s'\n", clientId.c_str());

    // Suscribirse al topic global de todos los vehículos
    char subTopic[64];
    snprintf(subTopic, sizeof(subTopic), "%s/+", TOPIC_GLOBAL);
    mqtt.subscribe(subTopic);
    Serial.printf("[MQTT] Suscrito a %s\n", subTopic);
  } else {
    Serial.printf("[MQTT] Fallo rc=%d\n", mqtt.state());
    totalErr++;
  }
}


// ============================================================
//  PMU / LoRa
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

void initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LoRa] ERROR");
    while (1) delay(1000);
  }
  // ─── MISMOS parámetros que el TX ────────────────────────
  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.enableCrc();
  Serial.println("[LoRa] Receptor listo (SF9 BW125 CR4/6)");
}

// ============================================================
//  Almacenamiento offline en LittleFS
// ============================================================
void initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Error montando LittleFS");
  } else {
    size_t used = LittleFS.usedBytes();
    size_t total = LittleFS.totalBytes();
    Serial.printf("[FS] LittleFS OK — %u / %u KB usados\n",
                  used / 1024, total / 1024);
  }
}

void saveOffline(const data_pkt_t& pkt, int rssi, float snr) {
  // Contar registros pendientes
  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  uint32_t count = f ? (f.size() / sizeof(OfflineRecord)) : 0;
  if (f) f.close();

  if (count >= OFFLINE_MAX_RECORDS) {
    Serial.println("[FS] Cola llena — paquete descartado");
    return;
  }

  File fw = LittleFS.open(OFFLINE_DATA_FILE, "a");
  if (!fw) {
    Serial.println("[FS] Error al abrir cola para escritura");
    return;
  }

  OfflineRecord rec;
  memcpy(&rec.pkt, &pkt, sizeof(data_pkt_t));
  rec.rssi = (int16_t)rssi;
  rec.snr = snr;
  strlcpy(rec.stationId, creds.stationId, sizeof(rec.stationId));

  fw.write((uint8_t*)&rec, sizeof(OfflineRecord));
  fw.close();

  Serial.printf("[FS] Guardado offline: seq=%u bus#%u (pendientes=%u)\n",
                pkt.seq, pkt.device_id, count + 1);
}

void flushOfflineQueue() {
  if (!mqtt.connected()) return;
  if (!LittleFS.exists(OFFLINE_DATA_FILE)) return;

  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  if (!f) return;

  uint32_t total = f.size() / sizeof(OfflineRecord);

  if (total == 0) {
    f.close();
    LittleFS.remove(OFFLINE_DATA_FILE);
    return;
  }

  Serial.printf("[FS] Flushing %u paquetes offline...\n", total);

  File tmp = LittleFS.open("/ofq_tmp.bin", "w");
  if (!tmp) {
    f.close();
    return;
  }

  uint32_t sent = 0, kept = 0;
  bool mqttFailed = false;

  while (f.available() >= (int)sizeof(OfflineRecord)) {

    OfflineRecord rec;
    f.readBytes((char*)&rec, sizeof(OfflineRecord));

    if (!mqttFailed && mqtt.connected()) {

      char topic[128];

      snprintf(topic, sizeof(topic), "%s/%u",
               creds.topicBase, rec.pkt.device_id);

      char buf[320];

      snprintf(buf, sizeof(buf),
               "{\"device_id\":%u,\"ts\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,"
               "\"speed\":%.1f,\"flags\":%u,\"rssi\":%d,\"snr\":%.1f,"
               "\"station\":\"%s\",\"seq\":%u,\"stored\":true}",
               rec.pkt.device_id,
               rec.pkt.timestamp,
               rec.pkt.lat / 1e7f,
               rec.pkt.lon / 1e7f,
               rec.pkt.speed / 10.0f,
               rec.pkt.flags,
               rec.rssi,
               rec.snr,
               rec.stationId,
               rec.pkt.seq);

      if (mqtt.publish(topic, buf)) {

        sent++;
        mqtt.loop();
        delay(20);

      } else {

        mqttFailed = true;
        tmp.write((uint8_t*)&rec, sizeof(OfflineRecord));
        kept++;
      }

    } else {

      tmp.write((uint8_t*)&rec, sizeof(OfflineRecord));
      kept++;
    }
  }

  f.close();
  tmp.close();

  LittleFS.remove(OFFLINE_DATA_FILE);

  if (kept > 0) {

    LittleFS.rename("/ofq_tmp.bin", OFFLINE_DATA_FILE);

    Serial.printf("[FS] Flush parcial: %u enviados, %u pendientes\n",
                  sent, kept);

  } else {

    LittleFS.remove("/ofq_tmp.bin");

    Serial.printf("[FS] Flush completo: %u paquetes enviados\n",
                  sent);
  }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n====== TRACKER RX v2 (Store-and-Forward) ======");
  memset(veh_states, 0, sizeof(veh_states));
  pinMode(BUTTON_PIN, INPUT);
  initPMU();
  initLoRa();
  initStorage();  // ← AGREGAR esta línea
  if (loadCredentials()) {
    mqtt.setServer(creds.broker, creds.port);
    mqtt.setBufferSize(512);
    mqtt.setCallback(mqttCallback);
    connectWiFi();
  } else {
    startBLE();
  }
}

void loop() {
  processBLEJson();
  //checkRestartFlag();
  checkButton();


  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiRetry > WIFI_RETRY_MS) {
      lastWifiRetry = millis();
      connectWiFi();
    }
    return;
  }
  static bool wifiLogged = false;
  if (!wifiLogged) {
    Serial.printf("[WiFi] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
    wifiLogged = true;
  }

  if (!mqtt.connected()) {

    if (millis() - lastMqttRetry > MQTT_RETRY_MS) {
      lastMqttRetry = millis();
      connectMQTT();
    }

  } else {

    mqtt.loop();

    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 30000) {  // cada 30 s
      lastFlush = millis();
      flushOfflineQueue();
    }
  }

  // ─── Recepción LoRa ──────────────────────────────────────
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  // Leer el primer byte para saber el tipo sin consumir el resto
  if (!LoRa.available()) return;

  uint8_t pkt_type_peek = LoRa.peek();  // no avanza el buffer

  if (pkt_type_peek == PKT_DATA && packetSize == sizeof(data_pkt_t)) {
    data_pkt_t pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
    handleDataPacket(pkt, LoRa.packetRssi(), LoRa.packetSnr());
  } else {
    // Paquete desconocido o tamaño inesperado
    Serial.printf("[LoRa] Ignorado: type=0x%02X size=%d\n", pkt_type_peek, packetSize);
    while (LoRa.available()) LoRa.read();
    totalErr++;
  }
}


// ============================================================
//  Manejo de un paquete DATA recibido
// ============================================================
void handleDataPacket(const data_pkt_t& pkt, int rssi, float snr) {
  totalRx++;

  // Validación básica
  if (pkt.device_id == 0 || pkt.device_id > 9999) {
    totalErr++;
    return;
  }
  if (pkt.lat < -900000000 || pkt.lat > 900000000) {
    totalErr++;
    return;
  }
  if (pkt.lon < -1800000000 || pkt.lon > 1800000000) {
    totalErr++;
    return;
  }
  if (pkt.timestamp[0] == '\0') {
    totalErr++;
    return;
  }

  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  if (!v) {
    Serial.println("[RX] MAX_VEHICLES alcanzado");
    return;
  }

  bool hasGap = false;

  if (!v->initialized) {
    // Primer paquete de este vehículo: inicializar sin pedir retransmisión
    v->last_seq = pkt.seq;
    v->initialized = true;
    Serial.printf("[RX] Vehículo #%u visto por primera vez. seq=%u\n",
                  pkt.device_id, pkt.seq);
  } else {
    uint16_t expected = (uint16_t)(v->last_seq + 1);

    if (pkt.seq == expected) {
      // Secuencia correcta
      v->last_seq = pkt.seq;

    } else if ((uint16_t)(pkt.seq - v->last_seq) > 1) {
      // GAP detectado: falta desde (last_seq+1) hasta (pkt.seq-1)
      uint16_t from = (uint16_t)(v->last_seq + 1);
      uint16_t to = (uint16_t)(pkt.seq - 1);
      uint16_t missing = (uint16_t)(to - from + 1);

      Serial.printf("[GAP] Bus #%u: faltan seq %u..%u (%u paquetes)\n",
                    pkt.device_id, from, to, missing);
      v->gap_count++;
      hasGap = true;

      // Enviar REQ — el TX responderá en su próxima ventana de escucha
      // Pequeño delay aleatorio para evitar colisiones si hay múltiples RX
      delay(random(20, 80));
      sendReq(pkt.device_id, from, to);

      // Aceptar el paquete actual aunque haya gap
      v->last_seq = pkt.seq;

    } else {
      // Paquete duplicado o muy antiguo (seq <= last_seq): ignorar silenciosamente
      return;
    }
  }

  v->rx_count++;
  printPacket(pkt, rssi, snr, hasGap);
  publishPacket(pkt, rssi, snr);
}


// ============================================================
//  Enviar paquete REQ al TX (RX → TX, half-duplex)
// ============================================================
void sendReq(uint16_t device_id, uint16_t from_seq, uint16_t to_seq) {
  req_pkt_t req;
  req.pkt_type = PKT_REQ;
  req.device_id = device_id;
  req.from_seq = from_seq;
  req.to_seq = to_seq;

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&req, sizeof(req));
  LoRa.endPacket(false);

  Serial.printf("[REQ→] Bus #%u: pidiendo seq %u..%u\n",
                device_id, from_seq, to_seq);
}


// ============================================================
//  Publicar paquete a MQTT como JSON
// ============================================================
void publishPacket(const data_pkt_t& pkt, int rssi, float snr) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);

  // Dedup best-effort
  if (v && v->global_seq == pkt.seq) {
    Serial.printf("[DEDUP] Seq %u bus #%u ya publicado, skip\n",
                  pkt.seq, pkt.device_id);
    return;
  }

  // ── Sin conexión MQTT: guardar en flash ──────────────────────
  if (!mqtt.connected()) {
    saveOffline(pkt, rssi, snr);
    return;
  }

  char topic[128];
  snprintf(topic, sizeof(topic), "%s/%u", creds.topicBase, pkt.device_id);

  char buf[300];
  snprintf(buf, sizeof(buf),
           "{\"device_id\":%u,\"ts\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,"
           "\"speed\":%.1f,\"flags\":%u,\"rssi\":%d,\"snr\":%.1f,"
           "\"station\":\"%s\",\"seq\":%u}",
           pkt.device_id, pkt.timestamp,
           pkt.lat / 1e7f, pkt.lon / 1e7f,
           pkt.speed / 10.0f, pkt.flags,
           rssi, snr, creds.stationId, pkt.seq);

  bool ok = mqtt.publish(topic, buf);
  if (ok) {
    totalPub++;
    char globalTopic[64];
    snprintf(globalTopic, sizeof(globalTopic), "tracker/global/%u", pkt.device_id);
    char seqStr[8];
    snprintf(seqStr, sizeof(seqStr), "%u", pkt.seq);
    mqtt.publish(globalTopic, seqStr, true);
    Serial.printf("[MQTT] Publicado → %s\n", topic);
  } else {
    totalErr++;
    saveOffline(pkt, rssi, snr);  // ← publish falló, guardar en flash
  }
}
// ============================================================
//  Buscar o crear entrada de vehículo
// ============================================================
VehicleState* findOrCreateVehicle(uint16_t device_id) {
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (veh_states[i].initialized && veh_states[i].device_id == device_id)
      return &veh_states[i];
  }
  // No encontrado: crear en slot vacío
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (!veh_states[i].initialized) {
      veh_states[i].device_id = device_id;
      return &veh_states[i];
    }
  }
  return nullptr;  // tabla llena
}


// ============================================================
void printPacket(const data_pkt_t& pkt, int rssi, float snr, bool gap) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  Serial.printf("%s[PKT] Bus#%-4u seq=%-5u %s fix=%c | %+.5f,%+.5f | "
                "%.1f km/h | RSSI=%d SNR=%.1f | rx=%lu gaps=%lu\n",
                gap ? "⚠️  " : "",
                pkt.device_id, pkt.seq,
                pkt.timestamp,  // ← nuevo
                (pkt.flags & 0x01) ? 'Y' : 'N',
                pkt.lat / 1e7f, pkt.lon / 1e7f,
                pkt.speed / 10.0f,
                rssi, snr,
                v ? v->rx_count : 0UL,
                v ? v->gap_count : 0UL);
}