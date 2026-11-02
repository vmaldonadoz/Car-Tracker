// ============================================================
//  MQTT-RX.ino  —  v3
//  Store-and-Forward + Portal WiFi (reemplaza BLE) + OTA MQTT
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ─── Versión de firmware (cambiar en cada release OTA) ───────
#define FIRMWARE_VERSION "1.2"

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

// ─── Portal WiFi (config AP) ──────────────────────────────────
#define CONFIG_AP_SSID_PREFIX "LoRa-Config-"
#define CONFIG_AP_PASS ""  // sin password; cambia si quieres seguridad

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char timestamp[20];
  int32_t lat;
  int32_t lon;
  uint16_t speed;
  uint8_t flags;
} data_pkt_t;  // 20 bytes (con padding -> 36)

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
  uint16_t last_seq;
  uint16_t global_seq;
  bool initialized;
  uint32_t rx_count;
  uint32_t gap_count;
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

WebServer configServer(80);
bool configMode = false;

unsigned long buttonPressStart = 0;
bool buttonHeld = false;
bool longPressHandled = false;
unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;

uint32_t totalRx = 0;
uint32_t totalPub = 0;
uint32_t totalErr = 0;

// ─── OTA topics ───────────────────────────────────────────────
#define OTA_TOPIC_PREFIX "tracker/cmd/"
#define OTA_TOPIC_SUFFIX "/ota"
#define OTA_STATUS_PREFIX "tracker/status/"

// ─── Forward declarations ─────────────────────────────────────
VehicleState* findOrCreateVehicle(uint16_t device_id);
void handleDataPacket(const data_pkt_t& p, int rssi, float snr);
void sendReq(uint16_t device_id, uint16_t from_seq, uint16_t to_seq);
void publishPacket(const data_pkt_t& p, int rssi, float snr);
void printPacket(const data_pkt_t& p, int rssi, float snr, bool gap);
bool loadCredentials();
void saveCredentials();
void connectWiFi();
void connectMQTT();
void checkButton();
void initPMU();
void initLoRa();
void publishHeartbeat();
void doOTA(const String& version, const String& url);

// ============================================================
//  PORTAL WIFI DE CONFIGURACIÓN  (reemplaza BLE)
// ============================================================
const char* CONFIG_PAGE_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Configurar Nodo RX</title>
  <style>
    body { font-family: sans-serif; max-width: 420px; margin: 30px auto; padding: 0 16px; }
    h2 { text-align: center; }
    label { display: block; margin-top: 12px; font-weight: bold; }
    input { width: 100%; padding: 8px; box-sizing: border-box; margin-top: 4px; }
    button { width: 100%; padding: 12px; margin-top: 20px; background: #2563eb; color: white;
             border: none; border-radius: 6px; font-size: 16px; }
  </style>
</head>
<body>
  <h2>Configurar Nodo RX</h2>
  <form action="/save" method="POST">
    <label>WiFi SSID</label>
    <input name="ssid" required>
    <label>WiFi Password</label>
    <input name="pass" type="password">
    <label>Broker MQTT (IP o dominio)</label>
    <input name="broker" required>
    <label>Puerto MQTT</label>
    <input name="port" type="number" value="1883">
    <label>Topic base (ej: gps/vehiculos)</label>
    <input name="topic" value="gps/vehiculos">
    <label>ID de esta estación</label>
    <input name="station" required>
    <button type="submit">Guardar y Reiniciar</button>
  </form>
</body>
</html>
)HTML";

void handleConfigRoot() {
  configServer.send(200, "text/html", CONFIG_PAGE_HTML);
}

void handleConfigSave() {
  strlcpy(creds.ssid, configServer.arg("ssid").c_str(), sizeof(creds.ssid));
  strlcpy(creds.pass, configServer.arg("pass").c_str(), sizeof(creds.pass));
  strlcpy(creds.broker, configServer.arg("broker").c_str(), sizeof(creds.broker));
  creds.port = configServer.arg("port").toInt();
  if (creds.port == 0) creds.port = 1883;
  strlcpy(creds.topicBase, configServer.arg("topic").c_str(), sizeof(creds.topicBase));
  strlcpy(creds.stationId, configServer.arg("station").c_str(), sizeof(creds.stationId));

  saveCredentials();

  configServer.send(200, "text/html",
    "<html><body style='font-family:sans-serif;text-align:center;margin-top:50px'>"
    "<h2>Guardado correctamente</h2><p>Reiniciando...</p></body></html>");

  delay(1000);
  ESP.restart();
}

void startConfigPortal() {
  configMode = true;

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);

  String apName = String(CONFIG_AP_SSID_PREFIX) + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.softAP(apName.c_str(), CONFIG_AP_PASS);

  Serial.printf("[CONFIG] AP iniciado: %s\n", apName.c_str());
  Serial.printf("[CONFIG] IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("[CONFIG] Conéctate y abre http://192.168.4.1");

  configServer.on("/", handleConfigRoot);
  configServer.on("/save", HTTP_POST, handleConfigSave);
  configServer.begin();
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
  Serial.printf("[NVS] %s\n", valid ? "OK" : "Sin credenciales -> portal config");
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
//  Botón → activa portal de configuración
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
    Serial.println("[BTN] -> Portal de configuracion activado");
    startConfigPortal();
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
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(creds.ssid, creds.pass);
  Serial.printf("[WiFi] Conectando a '%s'...\n", creds.ssid);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr(topic);

  // ── OTA ──────────────────────────────────────────────────
  String otaTopic = String(OTA_TOPIC_PREFIX) + creds.stationId + OTA_TOPIC_SUFFIX;
  if (topicStr == otaTopic) {
    String payloadStr;
    for (unsigned int i = 0; i < length; i++) payloadStr += (char)payload[i];

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, payloadStr) != DeserializationError::Ok) return;

    const char* ver = doc["version"];
    const char* url = doc["url"];
    if (ver && url) doOTA(String(ver), String(url));
    return;
  }

  // ── tracker/global/{device_id} (sync de secuencia) ────────
  char* lastSlash = strrchr(topic, '/');
  if (!lastSlash) return;

  uint16_t device_id = (uint16_t)atoi(lastSlash + 1);
  if (device_id == 0) return;

  char buf[16] = { 0 };
  memcpy(buf, payload, min((unsigned int)15, length));
  uint16_t global_seq = (uint16_t)atoi(buf);

  VehicleState* v = findOrCreateVehicle(device_id);
  if (!v) return;

  if ((uint16_t)(global_seq - v->global_seq) < 32000) {
    v->global_seq = global_seq;
  }

  if (!v->initialized) {
    v->last_seq = global_seq;
    v->initialized = true;
    Serial.printf("[MQTT] Bus #%u inicializado desde retained: last_seq=%u\n",
                  device_id, global_seq);
  }
}

void connectMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String clientId = String("RX_") + creds.stationId + "_" + mac.substring(6);

  char lwtTopic[128];
  snprintf(lwtTopic, sizeof(lwtTopic), "tracker/status/%s", creds.stationId);

  if (mqtt.connect(clientId.c_str(), lwtTopic, 1, true, "offline")) {
    mqtt.publish(lwtTopic, "online", true);
    Serial.printf("[MQTT] Conectado como '%s'\n", clientId.c_str());

    char subTopic[64];
    snprintf(subTopic, sizeof(subTopic), "%s/+", TOPIC_GLOBAL);
    mqtt.subscribe(subTopic);
    Serial.printf("[MQTT] Suscrito a %s\n", subTopic);

    // OTA
    char otaTopic[96];
    snprintf(otaTopic, sizeof(otaTopic), "%s%s%s", OTA_TOPIC_PREFIX, creds.stationId, OTA_TOPIC_SUFFIX);
    mqtt.subscribe(otaTopic);
    Serial.printf("[MQTT] Suscrito a %s (firmware v%s)\n", otaTopic, FIRMWARE_VERSION);

  } else {
    Serial.printf("[MQTT] Fallo rc=%d\n", mqtt.state());
    totalErr++;
  }
}

// ============================================================
//  OTA vía MQTT + GitHub
// ============================================================
void publishOtaStatus(const String& json) {
  char topic[128];
  snprintf(topic, sizeof(topic), "%s%s", OTA_STATUS_PREFIX, creds.stationId);
  mqtt.publish(topic, json.c_str(), false);
  Serial.printf("[OTA] Status: %s\n", json.c_str());
}

void doOTA(const String& version, const String& url) {
  Serial.println("\n[OTA] ========== INICIANDO OTA ==========");
  Serial.printf("[OTA] Version actual : %s\n", FIRMWARE_VERSION);
  Serial.printf("[OTA] Version nueva  : %s\n", version.c_str());
  Serial.printf("[OTA] URL            : %s\n", url.c_str());

  if (version == FIRMWARE_VERSION) {
    publishOtaStatus("{\"info\":\"ya_tengo_esta_version\"}");
    return;
  }

  publishOtaStatus("{\"status\":\"descargando\",\"target\":\"" + version + "\"}");
  delay(300);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);
  http.addHeader("User-Agent", "ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();
  Serial.printf("[OTA] HTTP Code: %d\n", httpCode);

  if (httpCode != 200) {
    publishOtaStatus("{\"error\":\"http_" + String(httpCode) + "\"}");
    http.end();
    return;
  }

  int totalSize = http.getSize();
  Serial.printf("[OTA] Tamano del firmware: %d bytes\n", totalSize);

  if (!Update.begin(totalSize > 0 ? totalSize : UPDATE_SIZE_UNKNOWN)) {
    publishOtaStatus("{\"error\":\"no_space\"}");
    Update.printError(Serial);
    http.end();
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[1024];
  int written = 0;

  while (http.connected() && written < totalSize) {
    int available = stream->available();
    if (available > 0) {
      int toRead = min(available, (int)sizeof(buf));
      int bytesRead = stream->readBytes(buf, toRead);
      written += Update.write(buf, bytesRead);
      Serial.printf("[OTA] %d / %d bytes (%.0f%%)\r",
                    written, totalSize,
                    totalSize ? (written * 100.0 / totalSize) : 0);
    }
    delay(1);
  }
  Serial.println();

  if (Update.end() && Update.isFinished()) {
    publishOtaStatus("{\"status\":\"ok\",\"version\":\"" + version + "\"}");
    Serial.println("[OTA] Actualizacion exitosa. Reiniciando...");
    delay(500);
    ESP.restart();
  } else {
    publishOtaStatus("{\"error\":\"update_incompleto\"}");
    Update.printError(Serial);
  }

  http.end();
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
  LoRa.setSpreadingFactor(9);
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
    Serial.printf("[FS] LittleFS OK - %u / %u KB usados\n",
                  used / 1024, total / 1024);
  }
}

void saveOffline(const data_pkt_t& pkt, int rssi, float snr) {
  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  uint32_t count = f ? (f.size() / sizeof(OfflineRecord)) : 0;
  if (f) f.close();

  if (count >= OFFLINE_MAX_RECORDS) {
    Serial.println("[FS] Cola llena - paquete descartado");
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
      snprintf(topic, sizeof(topic), "%s/%u", creds.topicBase, rec.pkt.device_id);

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
    Serial.printf("[FS] Flush parcial: %u enviados, %u pendientes\n", sent, kept);
  } else {
    LittleFS.remove("/ofq_tmp.bin");
    Serial.printf("[FS] Flush completo: %u paquetes enviados\n", sent);
  }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n====== TRACKER RX v3 (firmware %s) ======\n", FIRMWARE_VERSION);
  memset(veh_states, 0, sizeof(veh_states));
  pinMode(BUTTON_PIN, INPUT);
  initPMU();
  initLoRa();
  initStorage();

  if (loadCredentials()) {
    mqtt.setServer(creds.broker, creds.port);
    mqtt.setBufferSize(1024);  // un poco más grande por el payload OTA
    mqtt.setCallback(mqttCallback);
    connectWiFi();
  } else {
    startConfigPortal();
  }
}

void loop() {
  if (configMode) {
    configServer.handleClient();
    return;  // no hacer nada más mientras se configura
  }

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
    if (millis() - lastFlush > 30000) {
      lastFlush = millis();
      flushOfflineQueue();
    }
  }

  // ─── Heartbeat periódico (independiente de LoRa) ─────────
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    publishHeartbeat();
  }

  // ─── Recepción LoRa ──────────────────────────────────────
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  if (!LoRa.available()) return;

  uint8_t pkt_type_peek = LoRa.peek();

  if (pkt_type_peek == PKT_DATA && packetSize == sizeof(data_pkt_t)) {
    data_pkt_t pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
    handleDataPacket(pkt, LoRa.packetRssi(), LoRa.packetSnr());
  } else {
    Serial.printf("[LoRa] Ignorado: type=0x%02X size=%d\n", pkt_type_peek, packetSize);
    while (LoRa.available()) LoRa.read();
    totalErr++;
  }
}

void publishHeartbeat() {
  if (!mqtt.connected()) return;

  char topic[128];
  snprintf(topic, sizeof(topic), "tracker/status/%s", creds.stationId);

  bool usbPresent = PMU.isVbusIn();
  bool charging = PMU.isCharging();
  uint8_t battPercent = PMU.getBatteryPercent();

  char buf[256];
  snprintf(buf, sizeof(buf),
      "{"
      "\"status\":\"online\","
      "\"firmware\":\"%s\","
      "\"power\":\"%s\","
      "\"charging\":%s,"
      "\"battery_percent\":%u,"
      "\"wifi_rssi\":%d,"
      "\"rx\":%lu,"
      "\"err\":%lu"
      "}",
      FIRMWARE_VERSION,
      usbPresent ? "usb" : "battery",
      charging ? "true" : "false",
      battPercent,
      WiFi.RSSI(),
      totalRx,
      totalErr);

  mqtt.publish(topic, buf, true);

  Serial.println("[HB] Heartbeat enviado:");
  Serial.println(buf);
}

// ============================================================
//  Manejo de un paquete DATA recibido
// ============================================================
void handleDataPacket(const data_pkt_t& pkt, int rssi, float snr) {
  totalRx++;

  if (pkt.device_id == 0 || pkt.device_id > 9999) { totalErr++; return; }
  if (pkt.lat < -900000000 || pkt.lat > 900000000) { totalErr++; return; }
  if (pkt.lon < -1800000000 || pkt.lon > 1800000000) { totalErr++; return; }
  if (pkt.timestamp[0] == '\0') { totalErr++; return; }

  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  if (!v) {
    Serial.println("[RX] MAX_VEHICLES alcanzado");
    return;
  }

  bool hasGap = false;

  if (!v->initialized) {
    v->last_seq = pkt.seq;
    v->initialized = true;
    Serial.printf("[RX] Vehiculo #%u visto por primera vez. seq=%u\n",
                  pkt.device_id, pkt.seq);
  } else {
    uint16_t expected = (uint16_t)(v->last_seq + 1);

    if (pkt.seq == expected) {
      v->last_seq = pkt.seq;
    } else if ((uint16_t)(pkt.seq - v->last_seq) > 1) {
      uint16_t from = (uint16_t)(v->last_seq + 1);
      uint16_t to = (uint16_t)(pkt.seq - 1);
      uint16_t missing = (uint16_t)(to - from + 1);

      Serial.printf("[GAP] Bus #%u: faltan seq %u..%u (%u paquetes)\n",
                    pkt.device_id, from, to, missing);
      v->gap_count++;
      hasGap = true;

      delay(random(20, 80));
      sendReq(pkt.device_id, from, to);

      v->last_seq = pkt.seq;
    } else {
      return;
    }
  }

  v->rx_count++;
  printPacket(pkt, rssi, snr, hasGap);
  publishPacket(pkt, rssi, snr);
}

// ============================================================
//  Enviar paquete REQ al TX (RX -> TX, half-duplex)
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

  Serial.printf("[REQ->] Bus #%u: pidiendo seq %u..%u\n",
                device_id, from_seq, to_seq);
}

// ============================================================
//  Publicar paquete a MQTT como JSON
// ============================================================
void publishPacket(const data_pkt_t& pkt, int rssi, float snr) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);

  if (v && v->global_seq == pkt.seq) {
    Serial.printf("[DEDUP] Seq %u bus #%u ya publicado, skip\n",
                  pkt.seq, pkt.device_id);
    return;
  }

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
    Serial.printf("[MQTT] Publicado -> %s\n", topic);
  } else {
    totalErr++;
    saveOffline(pkt, rssi, snr);
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
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (!veh_states[i].initialized) {
      veh_states[i].device_id = device_id;
      return &veh_states[i];
    }
  }
  return nullptr;
}

// ============================================================
void printPacket(const data_pkt_t& pkt, int rssi, float snr, bool gap) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  Serial.printf("%s[PKT] Bus#%-4u seq=%-5u %s fix=%c | %+.5f,%+.5f | "
                "%.1f km/h | RSSI=%d SNR=%.1f | rx=%lu gaps=%lu\n",
                gap ? "[!] " : "",
                pkt.device_id, pkt.seq,
                pkt.timestamp,
                (pkt.flags & 0x01) ? 'Y' : 'N',
                pkt.lat / 1e7f, pkt.lon / 1e7f,
                pkt.speed / 10.0f,
                rssi, snr,
                v ? v->rx_count : 0UL,
                v ? v->gap_count : 0UL);
}