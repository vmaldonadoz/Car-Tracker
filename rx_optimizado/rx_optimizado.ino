// ============================================================
//  MQTT-RX.ino  —  v4  (seguridad MQTT + OTA)
//  Store-and-Forward + Portal WiFi + OTA MQTT + Auth
//  Coordinacion de REQ entre multiples RX via tracker/req_lock
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

// ─── Version de firmware ─────────────────────────────────────
#define FIRMWARE_VERSION "1.5"

// ─── Seguridad MQTT ──────────────────────────────────────────
// MQTT_USE_TLS=1: cifra la conexion TCP al broker (necesita TLS en el broker).
// Con 0 usa TCP plano; la autenticacion usuario/contrasena sigue activa.
// El token OTA es SIEMPRE requerido independientemente de TLS.
#define MQTT_USE_TLS 0

// ─── LoRa Pins ───────────────────────────────────────────────
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// ─── Boton ───────────────────────────────────────────────────
#define BUTTON_PIN 38
#define LONG_PRESS_MS 3000
#define DEBOUNCE_MS 50

// ─── Reconexion ──────────────────────────────────────────────
#define WIFI_RETRY_MS 15000
#define MQTT_RETRY_MS 5000

// ─── Vehiculos ───────────────────────────────────────────────
#define MAX_VEHICLES 16

// ─── Tipos de paquete ────────────────────────────────────────
#define PKT_DATA 0x01
#define PKT_REQ 0x02

#define TOPIC_GLOBAL "tracker/global"
#define TOPIC_REQ_LOCK "tracker/req_lock"  // coordinacion de REQ entre RX

// ─── Cola offline ────────────────────────────────────────────
#define OFFLINE_DATA_FILE "/ofq.bin"
#define OFFLINE_MAX_RECORDS 400

// ─── Coordinacion de REQ entre multiples RX ──────────────────
// Si este RX o cualquier otro ya pidio el mismo rango
// no volver a pedir hasta que termine el burst estimado.
// Con SF12 cada paquete del burst tarda ~2.9s (ToA + gap).
// Lock minimo: 30s. Lock dinamico: N_faltantes * 3s + 30s de margen.
#define REQ_LOCK_MIN_MS 30000  // minimo 30 segundos
#define REQ_LOCK_PER_PKT 3000  // ~3s por paquete del burst (SF12)
#define MAX_REQ_LOCKS 8        // locks por device_id a recordar

// ─── Portal WiFi ─────────────────────────────────────────────
#define CONFIG_AP_SSID_PREFIX "LoRa-Config-"
#define CONFIG_AP_PASS ""

// ─── Estructuras ─────────────────────────────────────────────
typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char timestamp[20];
  int32_t lat;
  int32_t lon;
  uint16_t speed;
  uint8_t flags;
} data_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;

struct __attribute__((packed)) OfflineRecord {
  data_pkt_t pkt;
  int16_t rssi;
  float snr;
  char stationId[32];
};

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t) == 7, "req_pkt_t size mismatch");

// ─── Lock de REQ por vehiculo ────────────────────────────────
struct ReqLock {
  uint16_t from_seq;
  uint16_t to_seq;
  unsigned long locked_at;      // millis() cuando se registro el lock
  unsigned long lock_duration;  // cuanto tiempo dura el lock (dinamico)
};


// ─── Estado por vehiculo ─────────────────────────────────────
struct VehicleState {
  uint16_t device_id;
  uint16_t last_seq;
  uint16_t global_seq;
  bool initialized;
  uint32_t rx_count;
  uint32_t gap_count;
  ReqLock req_locks[MAX_REQ_LOCKS];
  uint8_t req_lock_idx;

  // ── Gap pendiente (no resuelto) ─────────────────────────────
  bool pending_active;  // hay un hueco que aun no se confirmo resuelto
  uint16_t pending_from;
  uint16_t pending_to;
  unsigned long pending_expiry;  // cuando vence el lock que justifico no pedir
  uint8_t pending_retries;       // cuantas veces ya reintentamos
};
VehicleState veh_states[MAX_VEHICLES];

#define MAX_PENDING_RETRIES 3  // evita loop infinito si el TX no responde nunca

// ─── Credenciales ────────────────────────────────────────────
struct Credentials {
  char ssid[64];
  char pass[64];
  char broker[64];
  int port;
  char topicBase[96];
  char stationId[32];
  // ── Seguridad ─────────────────────────────────────
  char mqttUser[32];  // usuario MQTT (vacio = sin auth)
  char mqttPass[64];  // contrasena MQTT
  char otaToken[64];  // token secreto — REQUERIDO para aceptar OTA
};

// ─── Globals ─────────────────────────────────────────────────
Preferences prefs;
Credentials creds;
// WiFiClientSecure si MQTT_USE_TLS=1; WiFiClient si =0
#if MQTT_USE_TLS
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
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

// ─── Contadores de error por razon ───────────────────────────
// Permite diagnosticar exactamente que esta fallando
enum ErrReason {
  ERR_LORA_BAD_SIZE = 0,  // paquete LoRa con tamano incorrecto
  ERR_LORA_BAD_TYPE = 1,  // tipo de paquete desconocido
  ERR_BAD_DEVICE_ID = 2,  // device_id == 0 o > 9999
  ERR_BAD_LAT = 3,        // latitud fuera de rango
  ERR_BAD_LON = 4,        // longitud fuera de rango
  ERR_BAD_TIMESTAMP = 5,  // timestamp vacio
  ERR_MQTT_PUBLISH = 6,   // mqtt.publish() fallo
  ERR_MQTT_CONNECT = 7,   // no se pudo conectar al broker
  ERR_REASON_COUNT = 8
};
const char* ERR_NAMES[ERR_REASON_COUNT] = {
  "lora_bad_size", "lora_bad_type",
  "bad_device_id", "bad_lat", "bad_lon", "bad_timestamp",
  "mqtt_publish", "mqtt_connect"
};
uint32_t errCount[ERR_REASON_COUNT] = { 0 };

void countErr(ErrReason r) {
  totalErr++;
  errCount[r]++;
  Serial.printf("[ERR] %s (total_err=%lu)\n", ERR_NAMES[r], totalErr);
}

// ─── OTA topics ──────────────────────────────────────────────
#define OTA_TOPIC_PREFIX "tracker/cmd/"
#define OTA_TOPIC_SUFFIX "/ota"
#define OTA_STATUS_PREFIX "tracker/status/"

// ─── Forward declarations ────────────────────────────────────
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
bool isReqLocked(VehicleState* v, uint16_t from_seq, uint16_t to_seq);
bool isReqFullyLocked(VehicleState* v, uint16_t from, uint16_t to);
void lockReq(VehicleState* v, uint16_t from_seq, uint16_t to_seq);
void publishReqLock(uint16_t device_id, uint16_t from_seq, uint16_t to_seq);
void checkPendingGaps();

// ============================================================
//  Coordinacion REQ: lock local + publicacion MQTT
// ============================================================

// Verifica si ya hay un lock activo para este rango

bool isReqLocked(VehicleState* v, uint16_t from_seq, uint16_t to_seq) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_REQ_LOCKS; i++) {
    if (v->req_locks[i].locked_at == 0) continue;
    if (now - v->req_locks[i].locked_at > v->req_locks[i].lock_duration) continue;
    if (from_seq <= v->req_locks[i].to_seq &&
        to_seq   >= v->req_locks[i].from_seq) {
      return true;
    }
  }
  return false;
}

// NUEVA: solo devuelve true si el lock cubre el rango COMPLETO
bool isReqFullyLocked(VehicleState* v, uint16_t from, uint16_t to) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_REQ_LOCKS; i++) {
    if (v->req_locks[i].locked_at == 0) continue;
    if (now - v->req_locks[i].locked_at > v->req_locks[i].lock_duration) continue;
    if (v->req_locks[i].from_seq <= from && v->req_locks[i].to_seq >= to) {
      return true;
    }
  }
  return false;
}

// Registra el lock localmente con duracion dinamica
void lockReq(VehicleState* v, uint16_t from_seq, uint16_t to_seq, unsigned long duration_ms) {
  v->req_locks[v->req_lock_idx] = { from_seq, to_seq, millis(), duration_ms };
  v->req_lock_idx = (v->req_lock_idx + 1) % MAX_REQ_LOCKS;
}

// Publica el lock en MQTT para que otros RX lo vean
// topic: tracker/req_lock/{device_id}
// payload: {"from":50,"to":52,"rx":"parada1"}
// retained: false (es efimero, solo para RX activos en ese momento)
void publishReqLock(uint16_t device_id, uint16_t from_seq, uint16_t to_seq) {
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%u", TOPIC_REQ_LOCK, device_id);

  char payload[80];
  snprintf(payload, sizeof(payload),
           "{\"from\":%u,\"to\":%u,\"rx\":\"%s\"}",
           from_seq, to_seq, creds.stationId);

  mqtt.publish(topic, payload, false);
}


// ============================================================
//  Portal WiFi de configuracion
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
    button { width: 100%; padding: 12px; margin-top: 20px; background: #2563eb;
             color: white; border: none; border-radius: 6px; font-size: 16px; }
    hr { margin-top: 20px; border: none; border-top: 1px solid #ccc; }
    .sec { background:#fef9ec; border-left:4px solid #f59e0b; padding:8px 12px;
           margin-top:14px; border-radius:4px; font-size:13px; }
  </style>
</head>
<body>
  <h2>Configurar Nodo RX</h2>
  <form action="/save" method="POST">
    <label>WiFi SSID</label><input name="ssid" required>
    <label>WiFi Password</label><input name="pass" type="password">
    <label>Broker MQTT (IP o dominio)</label><input name="broker" required>
    <label>Puerto MQTT</label><input name="port" type="number" value="1883">
    <label>Topic base (ej: gps/vehiculos)</label><input name="topic" value="gps/vehiculos">
    <label>ID de esta estacion</label><input name="station" required>
    <hr>
    <div class="sec">&#128274; Seguridad MQTT &amp; OTA</div>
    <label>Usuario MQTT <small>(opcional)</small></label>
    <input name="mqttuser">
    <label>Contrase&ntilde;a MQTT <small>(opcional)</small></label>
    <input name="mqttpass" type="password">
    <label>Token secreto OTA <span style="color:red">*requerido para actualizar*</span></label>
    <input name="otatoken" type="password" placeholder="Minimo 8 caracteres">
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
  strlcpy(creds.mqttUser, configServer.arg("mqttuser").c_str(), sizeof(creds.mqttUser));
  strlcpy(creds.mqttPass, configServer.arg("mqttpass").c_str(), sizeof(creds.mqttPass));
  strlcpy(creds.otaToken, configServer.arg("otatoken").c_str(), sizeof(creds.otaToken));

  saveCredentials();

  String warn = (strlen(creds.otaToken) < 8)
                  ? "<p style='color:red'>&#9888; Token OTA vacio o muy corto &mdash; OTA bloqueado.</p>"
                  : "<p style='color:green'>&#10003; Token OTA configurado correctamente.</p>";

  configServer.send(200, "text/html",
                    "<html><body style='font-family:sans-serif;text-align:center;margin-top:50px'>"
                    "<h2>Guardado correctamente</h2>"
                      + warn + "<p>Reiniciando...</p></body></html>");

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
  Serial.printf("[CONFIG] AP: %s  IP: %s\n",
                apName.c_str(), WiFi.softAPIP().toString().c_str());
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
    strlcpy(creds.mqttUser, prefs.getString("muser", "").c_str(), sizeof(creds.mqttUser));
    strlcpy(creds.mqttPass, prefs.getString("mpass", "").c_str(), sizeof(creds.mqttPass));
    strlcpy(creds.otaToken, prefs.getString("otatk", "").c_str(), sizeof(creds.otaToken));
  }
  prefs.end();
  bool valid = exists && strlen(creds.ssid) > 0 && strlen(creds.broker) > 0;
  Serial.printf("[NVS] %s\n", valid ? "OK" : "Sin credenciales -> portal config");
  if (valid && strlen(creds.otaToken) < 8)
    Serial.println("[SEC] *** ADVERTENCIA: Token OTA no configurado -- OTA bloqueado ***");
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
  prefs.putString("muser", creds.mqttUser);
  prefs.putString("mpass", creds.mqttPass);
  prefs.putString("otatk", creds.otaToken);
  prefs.end();
}


// ============================================================
//  Boton -> portal de configuracion
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
  String payloadStr;
  for (unsigned int i = 0; i < length; i++) payloadStr += (char)payload[i];

  // ── OTA ──────────────────────────────────────────────
  String otaTopic = String(OTA_TOPIC_PREFIX) + creds.stationId + OTA_TOPIC_SUFFIX;
  if (topicStr == otaTopic) {
    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, payloadStr) != DeserializationError::Ok) return;

    // ── Capa 1: token secreto OTA obligatorio ───────────────────
    if (strlen(creds.otaToken) < 8) {
      Serial.println("[SEC] OTA rechazado: token no configurado en este nodo");
      publishOtaStatus("{\"error\":\"ota_sin_token_configurado\"}");
      return;
    }
    const char* token = doc["token"];
    if (!token || strcmp(token, creds.otaToken) != 0) {
      Serial.println("[SEC] OTA rechazado: token invalido");
      publishOtaStatus("{\"error\":\"token_invalido\"}");
      return;
    }
    // ── Capa 2: la URL debe ser HTTPS ──────────────────────────
    const char* ver = doc["version"];
    const char* url = doc["url"];
    if (!ver || !url) return;
    if (!String(url).startsWith("https://")) {
      Serial.println("[SEC] OTA rechazado: la URL debe ser HTTPS");
      publishOtaStatus("{\"error\":\"url_no_https\"}");
      return;
    }
    doOTA(String(ver), String(url));
    return;
  }

  // ── tracker/req_lock/{device_id} — otro RX ya pidio este rango ──
  if (topicStr.startsWith(String(TOPIC_REQ_LOCK) + "/")) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payloadStr) != DeserializationError::Ok) return;

    // Ignorar si el lock lo publico este mismo RX
    const char* rxId = doc["rx"];
    if (rxId && String(rxId) == String(creds.stationId)) return;

    uint16_t device_id = (uint16_t)topicStr.substring(topicStr.lastIndexOf('/') + 1).toInt();
    uint16_t from_seq = doc["from"] | 0;
    uint16_t to_seq = doc["to"] | 0;
    uint16_t missing = (uint16_t)(to_seq - from_seq + 1);

    VehicleState* v = findOrCreateVehicle(device_id);
    if (!v) return;

    // Lock con la misma duracion dinamica que usaria este RX
    unsigned long lockMs = (unsigned long)missing * REQ_LOCK_PER_PKT + REQ_LOCK_MIN_MS;
    lockReq(v, from_seq, to_seq, lockMs);

    Serial.printf("[REQ_LOCK] Bus #%u seq %u..%u ya pedido por %s (lock %lu s)\n",
                  device_id, from_seq, to_seq, rxId ? rxId : "?", lockMs / 1000);
    return;
  }

  // ── tracker/global/{device_id} — sync de secuencia ────────
  if (topicStr.startsWith(String(TOPIC_GLOBAL) + "/")) {
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
    return;
  }
}

void connectMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;

    // ── Capa 3: TLS (cifrado de transporte, opcional) ────────────
    // Requiere que el broker escuche en puerto TLS (tipicamente 8883).
    // setInsecure() cifra sin validar el certificado CA del broker.
    // Para validacion completa: usar espClient.setCACert("...").
#if MQTT_USE_TLS
  espClient.setInsecure();
#endif

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String clientId = String("RX_") + creds.stationId + "_" + mac.substring(6);

  char lwtTopic[128];
  snprintf(lwtTopic, sizeof(lwtTopic), "tracker/status/%s", creds.stationId);

  // ── Capa 4: autenticacion usuario/contrasena MQTT ────────────
  const char* mqttUser = strlen(creds.mqttUser) > 0 ? creds.mqttUser : nullptr;
  const char* mqttPass = strlen(creds.mqttPass) > 0 ? creds.mqttPass : nullptr;
  if (mqttUser) {
    Serial.printf("[MQTT] Conectando con usuario '%s'...\n", mqttUser);
  } else {
    Serial.println("[SEC] Advertencia: MQTT sin autenticacion de usuario");
  }

  mqtt.setSocketTimeout(5);
  if (mqtt.connect(clientId.c_str(), mqttUser, mqttPass, lwtTopic, 1, true, "offline")) {
    mqtt.publish(lwtTopic, "online", true);
    Serial.printf("[MQTT] Conectado como '%s'\n", clientId.c_str());

    // tracker/global/+
    char sub1[64];
    snprintf(sub1, sizeof(sub1), "%s/+", TOPIC_GLOBAL);
    mqtt.subscribe(sub1);

    // tracker/req_lock/+ — para coordinar REQ entre multiples RX
    char sub2[64];
    snprintf(sub2, sizeof(sub2), "%s/+", TOPIC_REQ_LOCK);
    mqtt.subscribe(sub2);

    // OTA
    char otaTopic[96];
    snprintf(otaTopic, sizeof(otaTopic), "%s%s%s",
             OTA_TOPIC_PREFIX, creds.stationId, OTA_TOPIC_SUFFIX);
    mqtt.subscribe(otaTopic);

    Serial.printf("[MQTT] Suscrito a %s, %s, %s\n", sub1, sub2, otaTopic);
    Serial.printf("[MQTT] Firmware v%s\n", FIRMWARE_VERSION);
  } else {
    // rc codes: -4=TIMEOUT -3=LOST -2=FAILED -1=DISCONNECTED 1=BAD_PROTO
    //            2=BAD_CLIENTID 3=UNAVAILABLE 4=BAD_CREDS 5=UNAUTHORIZED
    Serial.printf("[MQTT] Fallo rc=%d (%s)\n", mqtt.state(),
                  mqtt.state() == -4 ? "TIMEOUT" : mqtt.state() == -3 ? "LOST"
                                                 : mqtt.state() == -2 ? "FAILED"
                                                 : mqtt.state() == 4  ? "BAD_CREDS"
                                                 : mqtt.state() == 5  ? "UNAUTHORIZED"
                                                                      : "otro");
    countErr(ERR_MQTT_CONNECT);
  }
}


// ============================================================
//  OTA
// ============================================================
void publishOtaStatus(const String& json) {
  char topic[128];
  snprintf(topic, sizeof(topic), "%s%s", OTA_STATUS_PREFIX, creds.stationId);
  mqtt.publish(topic, json.c_str(), false);
  Serial.printf("[OTA] %s\n", json.c_str());
}

void doOTA(const String& version, const String& url) {
  Serial.println("\n[OTA] ========== INICIANDO OTA ==========");
  Serial.printf("[OTA] Actual: %s  Nueva: %s\n", FIRMWARE_VERSION, version.c_str());

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
  Serial.printf("[OTA] HTTP: %d\n", httpCode);

  if (httpCode != 200) {
    publishOtaStatus("{\"error\":\"http_" + String(httpCode) + "\"}");
    http.end();
    return;
  }

  int totalSize = http.getSize();
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
      int bytesRead = stream->readBytes(buf, min(available, (int)sizeof(buf)));
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
    Serial.println("[OTA] OK. Reiniciando...");
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
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.enableCrc();
  Serial.println("[LoRa] Receptor listo (SF12 BW125 CR4/6)");
}


// ============================================================
//  Almacenamiento offline
// ============================================================
void initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Error montando LittleFS");
  } else {
    Serial.printf("[FS] LittleFS OK — %u / %u KB\n",
                  LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);
  }
}

void saveOffline(const data_pkt_t& pkt, int rssi, float snr) {
  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  uint32_t count = f ? (f.size() / sizeof(OfflineRecord)) : 0;
  if (f) f.close();

  if (count >= OFFLINE_MAX_RECORDS) {
    Serial.println("[FS] Cola llena — descartado");
    return;
  }

  File fw = LittleFS.open(OFFLINE_DATA_FILE, "a");
  if (!fw) return;

  OfflineRecord rec;
  memcpy(&rec.pkt, &pkt, sizeof(data_pkt_t));
  rec.rssi = (int16_t)rssi;
  rec.snr = snr;
  strlcpy(rec.stationId, creds.stationId, sizeof(rec.stationId));
  fw.write((uint8_t*)&rec, sizeof(OfflineRecord));
  fw.close();

  Serial.printf("[FS] Offline seq=%u bus#%u (pendientes=%u)\n",
                pkt.seq, pkt.device_id, count + 1);
}

void flushOfflineQueue() {
  if (!mqtt.connected() || !LittleFS.exists(OFFLINE_DATA_FILE)) return;

  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  if (!f) return;

  uint32_t total = f.size() / sizeof(OfflineRecord);
  if (total == 0) {
    f.close();
    LittleFS.remove(OFFLINE_DATA_FILE);
    return;
  }

  Serial.printf("[FS] Flushing %u paquetes...\n", total);

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
               rec.pkt.device_id, rec.pkt.timestamp,
               rec.pkt.lat / 1e7f, rec.pkt.lon / 1e7f,
               rec.pkt.speed / 10.0f, rec.pkt.flags,
               rec.rssi, rec.snr, rec.stationId, rec.pkt.seq);
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
    Serial.printf("[FS] Flush completo: %u paquetes\n", sent);
  }
}


// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n====== TRACKER RX v4 (firmware %s) ======\n", FIRMWARE_VERSION);

  memset(veh_states, 0, sizeof(veh_states));
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  initPMU();
  initLoRa();
  initStorage();

  if (loadCredentials()) {
    mqtt.setServer(creds.broker, creds.port);
    mqtt.setBufferSize(1024);
    mqtt.setCallback(mqttCallback);
    connectWiFi();
  } else {
    startConfigPortal();
  }
}

void loop() {
  if (configMode) {
    configServer.handleClient();
    return;
  }

  checkButton();

  // ─── Reintento de gaps pendientes ────────────────────────
  static unsigned long lastGapCheck = 0;
  if (millis() - lastGapCheck > 10000) {
    lastGapCheck = millis();
    checkPendingGaps();
  }

  // ─── Heartbeat independiente de LoRa ─────────────────────
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    publishHeartbeat();
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiRetry > WIFI_RETRY_MS) {
      lastWifiRetry = millis();
      connectWiFi();
    }
    return;
  }

  static bool wifiLogged = false;
  if (!wifiLogged) {
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
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

  // ─── Recepcion LoRa ──────────────────────────────────────
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  if (!LoRa.available()) return;

  uint8_t pkt_type_peek = LoRa.peek();

  if (pkt_type_peek == PKT_DATA && packetSize == sizeof(data_pkt_t)) {
    data_pkt_t pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
    handleDataPacket(pkt, LoRa.packetRssi(), LoRa.packetSnr());
  } else {
    // Log detallado para diagnostico
    if (pkt_type_peek == PKT_DATA && packetSize != (int)sizeof(data_pkt_t)) {
      Serial.printf("[LoRa] BAD SIZE: type=PKT_DATA esperado=%d recibido=%d RSSI=%d\n",
                    (int)sizeof(data_pkt_t), packetSize, LoRa.packetRssi());
      countErr(ERR_LORA_BAD_SIZE);
    } else if (pkt_type_peek == PKT_REQ && packetSize == sizeof(req_pkt_t)) {
      // Es un REQ que llego al RX (rebote) — descartar silenciosamente
      Serial.printf("[LoRa] REQ propio recibido (rebote), descartando\n");
      // No contar como error
    } else {
      Serial.printf("[LoRa] TIPO DESCONOCIDO: type=0x%02X size=%d RSSI=%d\n",
                    pkt_type_peek, packetSize, LoRa.packetRssi());
      countErr(ERR_LORA_BAD_TYPE);
    }
    while (LoRa.available()) LoRa.read();
  }
}

void publishHeartbeat() {
  if (!mqtt.connected()) return;

  char topic[128];
  snprintf(topic, sizeof(topic), "tracker/status/%s", creds.stationId);

  // Construir detalle de errores por razon
  char errDetail[220];
  int pos = 0;
  pos += snprintf(errDetail + pos, sizeof(errDetail) - pos, ",\"err_detail\":{");
  for (int i = 0; i < ERR_REASON_COUNT; i++) {
    pos += snprintf(errDetail + pos, sizeof(errDetail) - pos,
                    "\"%s\":%lu%s", ERR_NAMES[i], errCount[i],
                    i < ERR_REASON_COUNT - 1 ? "," : "");
  }
  pos += snprintf(errDetail + pos, sizeof(errDetail) - pos, "}");

  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"status\":\"online\",\"firmware\":\"%s\","
           "\"power\":\"%s\",\"charging\":%s,\"battery_percent\":%u,"
           "\"wifi_rssi\":%d,\"rx\":%lu,\"pub\":%lu,\"err\":%lu%s}",
           FIRMWARE_VERSION,
           PMU.isVbusIn() ? "usb" : "battery",
           PMU.isCharging() ? "true" : "false",
           PMU.getBatteryPercent(),
           WiFi.RSSI(),
           totalRx, totalPub, totalErr, errDetail);

  mqtt.publish(topic, buf, true);
  Serial.printf("[HB] %s\n", buf);
}


// ============================================================
//  Manejo de paquete DATA
// ============================================================
void handleDataPacket(const data_pkt_t& pkt, int rssi, float snr) {
  totalRx++;

  if (pkt.device_id == 0 || pkt.device_id > 9999) {
    countErr(ERR_BAD_DEVICE_ID);
    Serial.printf("[ERR] device_id invalido: %u\n", pkt.device_id);
    return;
  }
  if (pkt.lat < -900000000 || pkt.lat > 900000000) {
    countErr(ERR_BAD_LAT);
    Serial.printf("[ERR] lat fuera de rango: %ld\n", (long)pkt.lat);
    return;
  }
  if (pkt.lon < -1800000000 || pkt.lon > 1800000000) {
    countErr(ERR_BAD_LON);
    Serial.printf("[ERR] lon fuera de rango: %ld\n", (long)pkt.lon);
    return;
  }
  if (pkt.timestamp[0] == '\0') {
    countErr(ERR_BAD_TIMESTAMP);
    Serial.println("[ERR] timestamp vacio");
    return;
  }

  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  if (!v) { Serial.println("[RX] MAX_VEHICLES alcanzado"); return; }

  bool hasGap = false;

  if (!v->initialized) {
    v->last_seq    = pkt.seq;
    v->initialized = true;
    Serial.printf("[RX] Vehiculo #%u primera vez. seq=%u\n", pkt.device_id, pkt.seq);

  } else {
    uint16_t expected = (uint16_t)(v->last_seq + 1);

    if (pkt.seq == expected) {
      // Secuencia correcta
      v->last_seq = pkt.seq;

    } else if ((int16_t)(pkt.seq - v->last_seq) > 1) {
      // GAP detectado (pkt.seq esta adelante de last_seq por mas de 1)
      uint16_t from    = (uint16_t)(v->last_seq + 1);
      uint16_t to      = (uint16_t)(pkt.seq - 1);
      uint16_t missing = (uint16_t)(to - from + 1);

      // Sanidad extra: el rango debe ser coherente (from <= to en espacio uint16)
      if (from > to && (uint32_t)to + 1 != (uint32_t)from) {
        Serial.printf("[GAP] Rango anomalo ignorado: from=%u to=%u\n", from, to);
        v->last_seq = pkt.seq;
      } else {
        Serial.printf("[GAP] Bus #%u: faltan seq %u..%u (%u paquetes)\n",
                      pkt.device_id, from, to, missing);
        v->gap_count++;
        hasGap = true;

        // ── Coordinacion multi-RX ─────────────────────────────
        if (!isReqFullyLocked(v, from, to)) {
          // Nadie cubre el rango completo todavia -> lo pedimos nosotros
          unsigned long lockMs = (unsigned long)missing * REQ_LOCK_PER_PKT + REQ_LOCK_MIN_MS;
          lockReq(v, from, to, lockMs);                 // bloquear localmente
          publishReqLock(pkt.device_id, from, to);      // avisar a otros RX
          Serial.printf("[REQ] Lock por %lu s para %u paquetes\n", lockMs/1000, missing);
          delay(random(20, 80));                        // evitar colision
          sendReq(pkt.device_id, from, to);              // pedir al TX

          v->pending_active  = true;
          v->pending_from    = from;
          v->pending_to      = to;
          v->pending_expiry  = millis() + lockMs;
          v->pending_retries = 0;
        } else {
          Serial.printf("[REQ_LOCK] Bus #%u seq %u..%u cubierto por lock existente, esperando retransmision\n",
                        pkt.device_id, from, to);

          // Igual lo marcamos como pendiente, por si el que lo pidio
          // nunca lo termina de resolver
          v->pending_active  = true;
          v->pending_from    = from;
          v->pending_to      = to;
          v->pending_expiry  = millis() + REQ_LOCK_MIN_MS;
          v->pending_retries = 0;
        }
        // ─────────────────────────────────────────────────────

        v->last_seq = pkt.seq;
      }

    } else if ((int16_t)(pkt.seq - v->last_seq) == 0) {
      // Duplicado exacto — ignorar silenciosamente
      return;

    } else {
      // pkt.seq < v->last_seq: retransmision de un paquete antiguo
      // Publicar igual (rellena el hueco en el backend)
      Serial.printf("[RETX] Bus #%u seq=%u (last_seq=%u) — publicando retransmision\n",
                    pkt.device_id, pkt.seq, v->last_seq);
      v->rx_count++;
      printPacket(pkt, rssi, snr, false);
      publishPacket(pkt, rssi, snr);

      // Si esta retransmision empieza a llenar el hueco pendiente,
      // achicar el rango pendiente desde el frente
      if (v->pending_active && pkt.seq >= v->pending_from && pkt.seq <= v->pending_to) {
        if (pkt.seq == v->pending_from) {
          v->pending_from++;
          if (v->pending_from > v->pending_to) {
            v->pending_active = false;
            Serial.printf("[GAP-RESUELTO] Bus #%u: hueco llenado por retransmision\n",
                          pkt.device_id);
          }
        }
      }
      return;  // return directo, sin tocar last_seq
    }
  }

  v->rx_count++;
  printPacket(pkt, rssi, snr, hasGap);
  publishPacket(pkt, rssi, snr);
}

// ============================================================
//  Enviar REQ al TX
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


void checkPendingGaps() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_VEHICLES; i++) {
    VehicleState* v = &veh_states[i];
    if (!v->initialized || !v->pending_active) continue;
    if (now < v->pending_expiry) continue;

    if (v->pending_retries >= MAX_PENDING_RETRIES) {
      Serial.printf("[GAP-GIVEUP] Bus #%u seq %u..%u: agotados los reintentos\n",
                    v->device_id, v->pending_from, v->pending_to);
      v->pending_active = false;
      continue;
    }

    uint16_t from = v->pending_from;
    uint16_t to   = v->pending_to;
    uint16_t missing = (uint16_t)(to - from + 1);
    unsigned long lockMs = (unsigned long)missing * REQ_LOCK_PER_PKT + REQ_LOCK_MIN_MS;

    Serial.printf("[GAP-RETRY] Bus #%u seq %u..%u sin resolver, reintentando (intento %u)\n",
                  v->device_id, from, to, v->pending_retries + 1);

    lockReq(v, from, to, lockMs);
    publishReqLock(v->device_id, from, to);
    sendReq(v->device_id, from, to);

    v->pending_expiry = now + lockMs;
    v->pending_retries++;
  }
}


// ============================================================
//  Publicar paquete a MQTT
// ============================================================
void publishPacket(const data_pkt_t& pkt, int rssi, float snr) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);

  // Dedup: si global_seq ya tiene este seq (publicado por otro RX), skip
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

  if (mqtt.publish(topic, buf)) {
    totalPub++;
    // Actualizar global_seq para dedup
    char globalTopic[64];
    snprintf(globalTopic, sizeof(globalTopic), "tracker/global/%u", pkt.device_id);
    char seqStr[8];
    snprintf(seqStr, sizeof(seqStr), "%u", pkt.seq);
    mqtt.publish(globalTopic, seqStr, true);
    Serial.printf("[MQTT] -> %s seq=%u\n", topic, pkt.seq);
  } else {
    countErr(ERR_MQTT_PUBLISH);
    saveOffline(pkt, rssi, snr);
  }
}


// ============================================================
//  Helpers
// ============================================================
VehicleState* findOrCreateVehicle(uint16_t device_id) {
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (veh_states[i].initialized && veh_states[i].device_id == device_id)
      return &veh_states[i];
  }
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (!veh_states[i].initialized) {
      veh_states[i].device_id = device_id;
      memset(veh_states[i].req_locks, 0, sizeof(veh_states[i].req_locks));
      veh_states[i].req_lock_idx = 0;
      return &veh_states[i];
    }
  }
  return nullptr;
}

void printPacket(const data_pkt_t& pkt, int rssi, float snr, bool gap) {
  VehicleState* v = findOrCreateVehicle(pkt.device_id);
  Serial.printf("%s[PKT] Bus#%-4u seq=%-5u %s fix=%c | %+.5f,%+.5f | "
                "%.1f km/h | RSSI=%d SNR=%.1f | rx=%lu gaps=%lu\n",
                gap ? "[!] " : "",
                pkt.device_id, pkt.seq, pkt.timestamp,
                (pkt.flags & 0x01) ? 'Y' : 'N',
                pkt.lat / 1e7f, pkt.lon / 1e7f,
                pkt.speed / 10.0f, rssi, snr,
                v ? v->rx_count : 0UL,
                v ? v->gap_count : 0UL);
}
