/*
  GPS-MQTT.ino  —  Tracker GPS L76K + MQTT Nativo (A7670G)  v2 (OTA)
  ──────────────────────────────────────────────────────────────────
  Lee posición del GPS Quectel L76K via TinyGPS++,
  construye el mismo JSON que publica el rx.ino y lo envía
  por MQTT nativo del módem A7670G (AT+CMQTT*).

  ── OTA (Over-The-Air) ──────────────────────────────────────────
  Escucha el topic MQTT:
    tracker/cmd/ota/<DEVICE_ID>
  Payload JSON:
    {"token":"<secret>","version":"1.1","url":"https://github.com/..."}

  Seguridad:
    1. Token secreto almacenado en NVS (Preferences) — REQUERIDO.
    2. URL debe comenzar con "https://".
    3. TLS verificado por el módem A7670G.

  Estado OTA publicado en:
    tracker/status/ota/<DEVICE_ID>

  JSON publicado (igual que publishPacket() en rx.ino):
    {
      "device_id": <uint>,
      "ts":        "<YYYY-MM-DDTHH:MM:SS>",
      "lat":       <float 7 dec>,
      "lon":       <float 7 dec>,
      "speed":     <float 1 dec>,
      "flags":     <uint>,
      "seq":       <uint>
    }

  Topics (mismo esquema que rx.ino):
    <TOPIC_BASE>/<DEVICE_ID>          ← datos JSON   (QoS 1)
    tracker/global/<DEVICE_ID>        ← seq retained (QoS 1)

  Plataforma: LilyGO T-A7670G  (ESP32 + módem A7670G)
  GPS:        Quectel L76K (solo RX, pin GPIO 22)
*/

#include <Preferences.h>
#include <Update.h>

// ─── Versión de firmware ────────────────────────────────────────
#define FIRMWARE_VERSION "2.2"

// ─── OTA automático al arrancar ─────────────────────────────────
// URL de un JSON estático con la versión disponible. Formato:
//   {"version":"2.1","url":"https://example.com/firmware.bin"}
// Dejar vacío ("") para deshabilitar la verificación automática.
#define OTA_VERSION_URL \
  "https://raw.githubusercontent.com/vmaldonadoz/Car-Tracker/main/" \
  "version.json"

// ─── Identificación del dispositivo ────────────────────────────
#define DEVICE_ID 4

// ─── SIM y APN ─────────────────────────────────────────────────
#define SIM_PIN ""  // dejar vacío si la SIM no tiene PIN
#define NETWORK_APN "internet.movistar.ve"

// ─── Pines LilyGO T-A7670G ────────────────────────────────────
#define MODEM_BAUDRATE 115200
#define MODEM_TX_PIN 26
#define MODEM_RX_PIN 27
#define BOARD_PWRKEY_PIN 4
#define BOARD_POWERON_PIN 12  // HIGH = alimenta el módulo A7670G
#define MODEM_RESET_PIN 5
#define MODEM_RESET_LEVEL HIGH
#define SerialAT Serial1

// ─── Pines GPS L76K ────────────────────────────────────────────
// Solo recibimos NMEA del GPS; TX del ESP no está conectado.
#define GPS_RX_PIN 22  // GPIO que recibe el TX del L76K
#define GPS_BAUD 9600
HardwareSerial GPS(2);  // UART2 del ESP32

// ─── MQTT broker ───────────────────────────────────────────────
const char *broker = "200.44.171.179";
const int port = 4033;

// ─── Topics (mismo esquema que rx.ino) ─────────────────────────
#define TOPIC_BASE "gps/vehiculos"  // <topicBase>/<device_id>
#define TOPIC_GLOBAL "gps/global"   // retained con el seq

// ─── ID de esta estación (equivale a stationId del rx) ────────
#define STATION_ID "hoatzin"

// ─── Topics OTA ────────────────────────────────────────────────
// Escucha:  tracker/cmd/ota/<DEVICE_ID>
//   payload: {"token":"...","version":"1.1","url":"https://..."}
// Publica:  tracker/status/ota/<DEVICE_ID>
// Versión:  tracker/status/version/<DEVICE_ID>  (retained)
#define OTA_CMD_TOPIC_PREFIX "tracker/cmd/ota/"
#define OTA_STATUS_TOPIC_PREFIX "tracker/status/ota/"
#define VERSION_TOPIC_PREFIX "tracker/status/version/"

// ─── Token OTA por defecto (NVS sobreescribe si existe) ────────
// Mínimo 8 caracteres. Dejar vacío fuerza configuración via NVS.
#define OTA_TOKEN_DEFAULT ""

// ─── Umbrales de movimiento real ─────────────────────────────────────
#define DISTANCE_MIN_M 15.0f  // metros mínimos entre publicaciones en movimiento
#define MIN_SPEED_KMPH 2.0f   // km/h mínimo para considerar movimiento real
#define HDOP_MAX 1.5f         // HDOP máximo aceptable
#define SAT_MIN 5             // satélites mínimos requeridos

// EMA adaptativo
#define EMA_ALPHA_MOVING 0.4f  // cuando está en movimiento
#define EMA_ALPHA_STILL 0.05f  // cuando está detenido — filtro muy agresivo

// Hysteresis de velocidad: cuántas lecturas consecutivas por encima
// del umbral antes de declarar "en movimiento"
#define MOVING_CONFIRM_COUNT 2

// Cooldown mínimo entre publicaciones consecutivas en movimiento (ms)
#define PUB_COOLDOWN_MS 5000UL

// Heartbeat estacionario: publicar aunque no haya movimiento
// para confirmar que el tracker está activo y reportar última posición.
#define STATIONARY_HEARTBEAT_MS 300000UL  // cada 5 minutos

// ─── Tamaño máximo del JSON de payload ─────────────────────────
//  device_id(5)+ts(19)+lat(12)+lon(12)+flags(1)+seq(5) + claves ≈ 95
#define JSON_BUF_SIZE 160

// ─── Variables watchdog ───────────────────────────────────────────
uint32_t lastModemCheck = 0;
#define MODEM_CHECK_INTERVAL 120000UL  // verificar cada 2 minutos

// ─── Log periódico del GPS ───────────────────────────────────────
uint32_t lastGpsLog = 0;
#define GPS_LOG_INTERVAL 30000UL  // imprimir estado cada 30 s

// ─── Heartbeat estacionario ─────────────────────────────────────
uint32_t lastHeartbeat = 0;  // última publicación de posición estacionaria

// ─── Almacenamiento offline (store-and-forward) ─────────────────
#define OFFLINE_DATA_FILE "/ofq.bin"
#define OFFLINE_MAX_RECORDS 5000

// ═══════════════════════════════════════════════════════════════
//  ESTRUCTURA DEL PAQUETE (packed, 36 bytes) — misma que tx/rx
// ═══════════════════════════════════════════════════════════════
#define PKT_DATA 0x01

// ─── Verificación OTA pendiente (reintentable desde loop) ──────
bool otaCheckDone = false;  // true cuando el manifiesto se pudo leer con éxito
uint32_t lastOtaCheckAttempt = 0;
#define OTA_CHECK_RETRY_MS 60000UL  // reintentar cada 60 s si falló

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;    //  1 byte  – offset  0
  uint16_t device_id;  //  2 bytes – offset  1
  uint16_t seq;        //  2 bytes – offset  3
  char timestamp[20];  // 20 bytes – offset  5
  int32_t lat;         //  4 bytes – offset 25
  int32_t lon;         //  4 bytes – offset 29
  uint16_t speed;      //  2 bytes – offset 33
  uint8_t flags;       //  1 byte  – offset 35
} data_pkt_t;          // = 36 bytes total

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");

// ─── Estado global ─────────────────────────────────────────────
#include <LittleFS.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;
uint16_t gSeq = 0;
bool mqttConnected = false;
uint32_t lastReconnect = 0;
unsigned long lastFlush = 0;
// ─── Última posición publicada (para trigger por distancia) ───────
double lastLat = 0.0;
double lastLon = 0.0;
bool hasLastPos = false;
// ─── EMA suavizado de coordenadas ─────────────────────────────────
double smoothLat = 0.0;
double smoothLon = 0.0;
bool emaReady = false;

// ─── Hysteresis de movimiento ─────────────────────────────────────────
uint8_t movingConfirm = 0;   // contador de lecturas "rápidas"
bool isMovingState = false;  // estado filtrado: en movimiento o no
uint32_t lastPubMs = 0;      // timestamp de última publicación

// ─── Buffer de URCs asíncronos ────────────────────────────────
static String urcPending = "";  // URCs recibidos durante comandos AT

// ─── Token OTA (cargado desde NVS en setup) ───────────────────
static char otaToken[64] = OTA_TOKEN_DEFAULT;

// ─── Bandera OTA pendiente (procesada en loop, no dentro de ISR/AT) ──
static volatile bool otaPending = false;
static String otaVersion = "";
static String otaUrl = "";

// ═══════════════════════════════════════════════════════════════
//  NVS — carga / guarda token OTA
// ═══════════════════════════════════════════════════════════════

void loadOtaToken() {
  Preferences prefs;
  prefs.begin("gps_cfg", true);
  if (prefs.isKey("otatk")) {
    String tk = prefs.getString("otatk", "");
    strlcpy(otaToken, tk.c_str(), sizeof(otaToken));
  }
  prefs.end();
  if (strlen(otaToken) < 8) {
    Serial.println(
      "[SEC] *** ADVERTENCIA: Token OTA no configurado — OTA bloqueado ***");
    Serial.println(
      "[SEC]     Configura el token via: saveOtaToken(\"mi_token\")");
  } else {
    Serial.println("[SEC] Token OTA cargado desde NVS OK");
  }
}

// Llama esta función una vez para guardar el token en NVS.
// Ejemplo de uso desde setup() (solo la primera vez):
//   saveOtaToken("mi_token_secreto");
void saveOtaToken(const char *token) {
  Preferences prefs;
  prefs.begin("gps_cfg", false);
  prefs.putString("otatk", token);
  prefs.end();
  strlcpy(otaToken, token, sizeof(otaToken));
  Serial.printf("[SEC] Token OTA guardado en NVS (%u chars)\n", strlen(token));
}

// ═══════════════════════════════════════════════════════════════
//  HELPERS AT  (extraídos del MQTT.ino de referencia)
// ═══════════════════════════════════════════════════════════════

void flushModem() {
  delay(50);
  while (SerialAT.available())
    SerialAT.read();
}

bool sendAT(const String &cmd, const String &expected = "OK",
            uint32_t ms = 5000) {
  flushModem();
  SerialAT.println(cmd);
  String buf;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      buf += c;
      if (c == '\n') {
        if (buf.indexOf("+CGEV: NW PDN DEACT") != -1 || buf.indexOf("+CMQTTNONET") != -1 || buf.indexOf("+CMQTTCONNLOST") != -1) {
          urcPending += buf;
        }
        // ── Detectar mensaje MQTT entrante ───────────────────
        if (buf.indexOf("+CMQTTRXSTART") != -1) {
          urcPending += buf;
        }
      }
    }
    if (buf.indexOf(expected) != -1)
      return true;
    if (buf.indexOf("ERROR") != -1)
      return false;
  }
  return false;
}

String sendATStr(const String &cmd, uint32_t ms = 5000) {
  flushModem();
  SerialAT.println(cmd);
  String buf;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      buf += c;

      // Acumular líneas completas y detectar URCs
      if (c == '\n') {
        if (buf.indexOf("+CGEV: NW PDN DEACT") != -1 || buf.indexOf("+CMQTTNONET") != -1 || buf.indexOf("+CMQTTCONNLOST") != -1) {
          urcPending += buf;
        }
        // ── Detectar mensaje MQTT entrante ───────────────────
        if (buf.indexOf("+CMQTTRXSTART") != -1) {
          urcPending += buf;
        }
      }
    }
    if (buf.indexOf("OK") != -1 || buf.indexOf("ERROR") != -1)
      break;
  }
  return buf;
}

// Espera el prompt '>' y envía datos (texto o binario)
bool waitPromptAndSend(const uint8_t *data, int len, uint32_t ms = 5000) {
  String buf;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialAT.available())
      buf += (char)SerialAT.read();
    if (buf.indexOf(">") != -1) {
      SerialAT.write(data, len);
      delay(100);
      return true;
    }
    if (buf.indexOf("ERROR") != -1) {
      Serial.print("[AT] Prompt ERROR: ");
      Serial.println(buf);
      return false;
    }
  }
  Serial.print("[AT] Sin prompt '>': ");
  Serial.println(buf);
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  MQTT NATIVO — publicación de texto/JSON (AT+CMQTT*)
// ═══════════════════════════════════════════════════════════════

/*
  mqttPublishText:
    Publica un string de texto en el topic indicado.
    Opcionalmente con retain flag (último parámetro de AT+CMQTTPUB).

  Flujo AT (igual que MQTT.ino de referencia):
    1. AT+CMQTTTOPIC=0,<len>   → espera '>' → envía topic
    2. AT+CMQTTPAYLOAD=0,<len> → espera '>' → envía payload
    3. AT+CMQTTPUB=0,<qos>,60,<retain>
       → espera "+CMQTTPUB: 0,0" para confirmación
*/
bool mqttPublishText(const char *topic, const char *payload, int qos = 1,
                     int retain = 0) {
  int tLen = strlen(topic);
  int pLen = strlen(payload);

  // ── 1) Topic ──────────────────────────────────────────────────
  flushModem();
  SerialAT.println("AT+CMQTTTOPIC=0," + String(tLen));
  if (!waitPromptAndSend((const uint8_t *)topic, tLen)) {
    Serial.println("[MQTT] Fallo en TOPIC");
    return false;
  }
  delay(200);

  // ── 2) Payload ────────────────────────────────────────────────
  flushModem();
  SerialAT.println("AT+CMQTTPAYLOAD=0," + String(pLen));
  if (!waitPromptAndSend((const uint8_t *)payload, pLen)) {
    Serial.println("[MQTT] Fallo en PAYLOAD");
    return false;
  }
  delay(200);

  // ── 3) Publicar ──────────────────────────────────────────────
  // AT+CMQTTPUB=<index>,<qos>,<timeout_s>,<retained>
  flushModem();
  SerialAT.println("AT+CMQTTPUB=0," + String(qos) + ",60," + String(retain));

  String buf;
  uint32_t t0 = millis();
  while (millis() - t0 < 60000UL) {
    while (SerialAT.available())
      buf += (char)SerialAT.read();
    if (buf.indexOf("+CMQTTPUB: 0,0") != -1)
      return true;
    if (buf.indexOf("ERROR") != -1) {
      Serial.print("[MQTT] PUB ERROR: ");
      Serial.println(buf);
      return false;
    }
  }
  Serial.println("[MQTT] PUB timeout");
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  PUBLICAR PAQUETE — mismo esquema que publishPacket() del rx.ino
// ═══════════════════════════════════════════════════════════════

/*
  publishPacket:
    Reproduce exactamente el JSON que construye publishPacket() en rx.ino,
    usando AT+CMQTT* en lugar de mqtt.publish() de PubSubClient.

    Topic principal : TOPIC_BASE/device_id              (QoS 1, no retain)
    Topic global    : tracker/global/device_id (retain)  (QoS 1, retain=1)
*/
void publishPacket(const data_pkt_t &pkt) {

  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%u", TOPIC_BASE, pkt.device_id);

  char buf[JSON_BUF_SIZE];
  snprintf(buf, sizeof(buf),
           "{\"device_id\":%u,\"ts\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,"
           "\"speed\":%.1f,"
           "\"flags\":%u,\"seq\":%u}",
           pkt.device_id, pkt.timestamp, pkt.lat / 1e7f, pkt.lon / 1e7f,
           pkt.speed / 10.0f, pkt.flags, pkt.seq);

  Serial.printf("[MQTT] → %s\n%s\n", topic, buf);

  bool ok = mqttPublishText(topic, buf, /*qos=*/0, /*retain=*/0);
  if (ok) {
    Serial.printf("[MQTT] PUB OK  seq=%u\n", pkt.seq);
  } else {
    Serial.printf("[MQTT] PUB FAIL  seq=%u\n", pkt.seq);
    mqttConnected = false;
  }
}

// ─── Configura DNS público en el módem ──────────────────────────────────
// Las SIMs Movistar en el pool 10.173.x.x no reciben DNS interno.
// Forzar Google Public DNS resuelve el fallo +CDNSGIP: 0,10.
void configureDNS() {
  Serial.println("[NET] Configurando DNS públicos (8.8.8.8 / 8.8.4.4)...");
  String r = sendATStr("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"", 5000);
  Serial.print("[NET] DNS cfg: ");
  Serial.println(r);
}

// ─── Detecta sesión LTE "fantasma" (MCC/MNC=000-00, banda=0) ─────────────
// Cuando la SIM obtiene registro pero el contexto de datos no se ancló
// a una celda real, AT+CPSI devuelve 000-00 y DNS falla aunque haya IP.
bool isPDPSessionValid() {
  String cpsi = sendATStr("AT+CPSI?", 5000);
  Serial.print("[NET] CPSI: ");
  Serial.println(cpsi);

  // Sesión inválida: MCC=000 o BAND=0 → módem no enganchado a celda real
  if (cpsi.indexOf("000-00") != -1 || cpsi.indexOf("EUTRAN-BAND0") != -1) {
    Serial.println("[NET] ⚠ Sesión LTE fantasma detectada (celda 000-00)");
    return false;
  }
  // Si no hay respuesta CPSI útil tampoco es válida
  if (cpsi.indexOf("+CPSI:") == -1) {
    Serial.println("[NET] ⚠ Sin respuesta CPSI");
    return false;
  }
  return true;
}

// ─── Fuerza re-enganche completo a la red ────────────────────────────────
// Útil cuando la SIM está en estado "fantasma": tiene IP pero sin celda real.
bool forceReattach() {
  Serial.println("[NET] Forzando re-enganche (COPS deregister)...");

  // Desactivar PDP primero
  sendAT("AT+CGACT=0,1", "OK", 10000);
  delay(500);

  // Deregistrar de la red y volver a modo automático
  sendAT("AT+COPS=2", "OK", 15000);  // desconectar de operador
  delay(3000);
  sendAT("AT+COPS=0", "OK", 30000);  // reconectar automático
  delay(2000);

  // Esperar registro real (con celda válida)
  Serial.print("[NET] Esperando re-registro");
  for (int i = 0; i < 30; i++) {
    String r = sendATStr("AT+CREG?", 3000);
    if (r.indexOf(",1") != -1 || r.indexOf(",5") != -1) {
      // Verificar que la celda sea real antes de continuar
      if (isPDPSessionValid() || i > 20) {  // tras 20 s aceptar igual
        Serial.println(" OK");
        return true;
      }
    }
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n[NET] Re-enganche falló");
  return false;
}

bool reactivatePDP() {
  Serial.println("[NET] Reactivando contexto PDP...");

  // Desactivar y volver a activar
  sendAT("AT+CGACT=0,1", "OK", 10000);
  delay(1000);

  for (int i = 0; i < 3; i++) {
    if (sendAT("AT+CGACT=1,1", "OK", 30000)) {
      String ip = sendATStr("AT+CGPADDR=1", 5000);
      Serial.print("[NET] IP tras reactivar: ");
      Serial.println(ip);
      bool hasIp = ip.indexOf("186.") != -1 || ip.indexOf("10.") != -1 || ip.indexOf("172.") != -1 || ip.indexOf("192.") != -1;
      if (!hasIp) {
        delay(3000);
        continue;
      }

      // Verificar que no sea una sesión fantasma
      if (!isPDPSessionValid()) {
        Serial.println(
          "[NET] Sesión inválida tras reactivar — forzando re-enganche");
        forceReattach();
        // Reactivar PDP después del re-enganche
        sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);
        sendAT("AT+CGACT=1,1", "OK", 30000);
        ip = sendATStr("AT+CGPADDR=1", 5000);
        Serial.print("[NET] IP tras re-enganche: ");
        Serial.println(ip);
      }
      // Siempre (re)configurar DNS después de activar PDP
      configureDNS();
      return true;
    }
    delay(3000);
  }
  Serial.println("[NET] Fallo reactivando PDP");
  return false;
}

void diagNetwork() {
  Serial.println("\n─── DIAGNÓSTICO DE RED ───────────────────");

  // ── Test básico de comunicación AT ───────────────────────────
  Serial.println("[AT] Enviando AT...");
  flushModem();
  SerialAT.println("AT");
  delay(1000);
  String raw = "";
  while (SerialAT.available())
    raw += (char)SerialAT.read();
  Serial.print("[AT] Respuesta raw: '");
  Serial.print(raw);
  Serial.println("'");

  if (raw.length() == 0) {
    Serial.println("[AT] ⚠ SIN RESPUESTA — UART o módem caído");
    return;
  }

  // Estado de registro
  String creg = sendATStr("AT+CREG?", 3000);
  String cgreg = sendATStr("AT+CGREG?", 3000);
  String cereg = sendATStr("AT+CEREG?", 3000);
  Serial.print("[NET] CREG:  ");
  Serial.println(creg);
  Serial.print("[NET] CGREG: ");
  Serial.println(cgreg);
  Serial.print("[NET] CEREG: ");
  Serial.println(cereg);

  // Calidad de señal
  String csq = sendATStr("AT+CSQ", 3000);
  Serial.print("[NET] CSQ:   ");
  Serial.println(csq);

  // Operador actual
  String cops = sendATStr("AT+COPS?", 5000);
  Serial.print("[NET] COPS:  ");
  Serial.println(cops);

  // Estado PDP
  String cgact = sendATStr("AT+CGACT?", 3000);
  Serial.print("[NET] CGACT: ");
  Serial.println(cgact);

  Serial.println("──────────────────────────────────────────\n");
}

// ─── Watchdog del módem ───────────────────────────────────────
// Retorna true si el módem responde a AT, false si está colgado
bool modemIsAlive() {
  flushModem();
  SerialAT.println("AT");
  String buf;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    while (SerialAT.available())
      buf += (char)SerialAT.read();
    if (buf.indexOf("OK") != -1)
      return true;
  }
  return false;
}

// Reset hardware completo del A7670G via PWRKEY
// Igual que en setup() pero sin reiniciar el ESP32
void hardResetModem() {
  Serial.println("[MDM] ⚠ Módem no responde — reset hardware...");

  // Apagar: pulso largo en PWRKEY (>1.2 s apaga el módulo)
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1500);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(2000);  // esperar que apague completamente

  // Encender: pulso corto en PWRKEY
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  // Esperar que el módulo arranque y responda AT (~8-12 s)
  Serial.print("[MDM] Esperando arranque");
  int retry = 0;
  while (!modemIsAlive()) {
    Serial.print(".");
    delay(1000);
    if (retry++ > 20) {
      Serial.println("\n[MDM] ⚠ Módem no arranca — reiniciando ESP32");
      ESP.restart();  // último recurso
    }
  }
  Serial.println(" OK");

  // Restaurar configuración básica
  sendAT("ATE0", "OK");
  sendAT("AT+CMEE=2", "OK");

  // Esperar registro de red
  Serial.print("[MDM] Esperando red");
  for (int i = 0; i < 30; i++) {
    if (isNetworkUp()) {
      Serial.println(" OK");
      return;
    }
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n[MDM] ⚠ Sin red tras reset");
}

// ═══════════════════════════════════════════════════════════════
//  CONEXIÓN MQTT
// ═══════════════════════════════════════════════════════════════

bool mqttConnect() {
  Serial.printf("[MQTT] Conectando a %s:%d\n", broker, port);

  // ── Verificar que el módem esté vivo ─────────────────────────
  if (!modemIsAlive()) {
    hardResetModem();

    // Reactivar PDP tras el reset
    sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);
    for (int i = 0; i < 3; i++) {
      if (sendAT("AT+CGACT=1,1", "OK", 30000))
        break;
      delay(3000);
    }
  }

  // ── Verificar red antes de continuar ─────────────────────────
  if (!isNetworkUp()) {
    Serial.println("[MQTT] Sin registro de red");
    return false;
  }

  // ── Verificar/reactivar PDP ───────────────────────────────────
  String pdpCheck = sendATStr("AT+CGACT?", 5000);
  if (pdpCheck.indexOf("+CGACT: 1,1") == -1) {
    Serial.println("[MQTT] PDP caído — reactivando...");
    if (!reactivatePDP()) {
      Serial.println("[MQTT] No se pudo reactivar PDP");
      return false;
    }
  }

  // ── Limpieza agresiva: intentar hasta 3 veces ────────────────
  bool accqOk = false;
  for (int attempt = 0; attempt < 3 && !accqOk; attempt++) {
    if (attempt > 0) {
      Serial.printf("[MQTT] Reintento limpieza #%d\n", attempt);
      delay(3000);
    }

    // Secuencia completa de limpieza — ignorar resultados
    sendAT("AT+CMQTTDISC=0,10", "OK", 12000);
    delay(800);
    sendAT("AT+CMQTTREL=0", "OK", 3000);
    delay(800);
    sendAT("AT+CMQTTSTOP", "OK", 8000);
    delay(2500);  // pausa crítica para cerrar socket TCP interno

    // Arrancar stack limpio
    if (!sendAT("AT+CMQTTSTART", "OK", 10000)) {
      Serial.println("[MQTT] CMQTTSTART falló — reintentando");
      continue;
    }
    delay(500);

    // Verificar PDP activo antes de continuar (CMQTTSTOP puede haberlo bajado)
    {
      String pdp = sendATStr("AT+CGACT?", 3000);
      if (pdp.indexOf("+CGACT: 1,1") == -1) {
        Serial.println("[MQTT] PDP caído tras CMQTTSTOP — reactivando");
        sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);
        for (int p = 0; p < 3; p++) {
          if (sendAT("AT+CGACT=1,1", "OK", 30000))
            break;
          delay(3000);
        }
        configureDNS();
      }
    }

    // Adquirir cliente 0 — clientId FIJO para identificación estable
    // en el broker (el broker desconecta la sesión anterior automáticamente).
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "tracker_%u", DEVICE_ID);
    String accq = "AT+CMQTTACCQ=0,\"";
    accq += clientId;
    accq += "\",0";
    Serial.printf("[MQTT] ClientID: %s\n", clientId);

    if (sendAT(accq, "OK", 5000)) {
      accqOk = true;
    } else {
      Serial.println("[MQTT] CMQTTACCQ falló — limpiando de nuevo");
      delay(2000);
    }
  }

  if (!accqOk) {
    Serial.println("[MQTT] No se pudo adquirir cliente MQTT tras 3 intentos");
    return false;
  }

  // ── Conectar al broker ────────────────────────────────────────
  String cmd = "AT+CMQTTCONNECT=0,\"tcp://";
  cmd += broker;
  cmd += ":";
  cmd += port;
  cmd += "\",60,1";

  flushModem();
  SerialAT.println(cmd);

  String buf;
  uint32_t t0 = millis();
  bool gotConnAck = false;
  while (millis() - t0 < 20000) {
    while (SerialAT.available())
      buf += (char)SerialAT.read();
    if (buf.indexOf("+CMQTTCONNECT: 0,0") != -1)
      gotConnAck = true;
    if (gotConnAck && buf.indexOf("OK") != -1)
      break;
    if (buf.indexOf("+CMQTTCONNLOST") != -1)
      break;
    if (buf.indexOf("ERROR") != -1)
      break;
  }

  if (!gotConnAck) {
    Serial.print("[MQTT] fail (sin CONNACK): ");
    Serial.println(buf);
    return false;
  }
  if (buf.indexOf("+CMQTTCONNLOST") != -1) {
    Serial.println("[MQTT] fail (CONNLOST inmediato)");
    return false;
  }

  Serial.println("[MQTT] Conectado OK");
  delay(500);

  // ── Suscribir al topic OTA ─────────────────────────────────
  char otaTopic[64];
  snprintf(otaTopic, sizeof(otaTopic), "%s%u", OTA_CMD_TOPIC_PREFIX, DEVICE_ID);

  // AT+CMQTTSUBTOPIC=<index>,<len>,<qos>
  flushModem();
  SerialAT.println("AT+CMQTTSUBTOPIC=0," + String(strlen(otaTopic)) + ",1");
  if (waitPromptAndSend((const uint8_t *)otaTopic, strlen(otaTopic))) {
    delay(200);
    // AT+CMQTTSUB=<index>,<timeout>
    String subResult = sendATStr("AT+CMQTTSUB=0,10", 12000);
    if (subResult.indexOf("+CMQTTSUB: 0,0") != -1) {
      Serial.printf("[MQTT] Suscrito a OTA topic: %s\n", otaTopic);
    } else {
      Serial.printf("[MQTT] Fallo subscribe OTA: %s\n", subResult.c_str());
    }
  } else {
    Serial.println("[MQTT] Fallo enviando OTA topic para subscribe");
  }

  Serial.printf("[MQTT] Firmware v%s listo\n", FIRMWARE_VERSION);
  delay(500);
  return true;
}

// Publica el estado OTA en tracker/status/ota/<DEVICE_ID>
void publishOtaStatus(const char *json) {
  char topic[64];
  snprintf(topic, sizeof(topic), "%s%u", OTA_STATUS_TOPIC_PREFIX, DEVICE_ID);
  mqttPublishText(topic, json, /*qos=*/0, /*retain=*/0);
  Serial.printf("[OTA] Status: %s\n", json);
}

// ═══════════════════════════════════════════════════════════════
//  OTA — descarga via AT+HTTP* del módem A7670G
// ═══════════════════════════════════════════════════════════════
//  Comandos usados (sección 16 del manual A7670G):
//    AT+HTTPINIT    — inicia el servicio HTTP
//    AT+HTTPPARA    — configura parámetros (URL, etc.)
//    AT+HTTPACTION  — ejecuta GET (0) / POST (1) / HEAD (2)
//    AT+HTTPREAD    — lee el cuerpo de la respuesta en bloques
//    AT+HTTPTERM    — detiene el servicio HTTP

/*
  httpGetInit:
    Inicia una sesión HTTP GET hacia 'url' y espera la respuesta.
    Retorna el Content-Length anunciado por el servidor (> 0) o -1 si falla.
    La respuesta queda en el buffer interno del módem — léela con
    AT+HTTPREAD antes de llamar AT+HTTPTERM.

  Flujo AT:
    1. AT+HTTPTERM          → limpia sesión previa (ignorar resultado)
    2. AT+HTTPINIT          → inicia servicio HTTP
    3. AT+HTTPPARA="URL","…" → fija la URL (HTTP o HTTPS)
    4. AT+HTTPACTION=0      → lanza el GET
    5. Esperar URC: +HTTPACTION: 0,<code>,<len>
*/
int32_t httpGetInit(const String &url) {
  // Limpieza preventiva de sesión anterior
  sendAT("AT+HTTPTERM", "OK", 3000);
  delay(300);

  if (!sendAT("AT+HTTPINIT", "OK", 5000)) {
    Serial.println("[HTTP] HTTPINIT falló");
    return -1;
  }
  delay(100);

  if (!sendAT("AT+HTTPPARA=\"URL\",\"" + url + "\"", "OK", 5000)) {
    Serial.println("[HTTP] HTTPPARA URL falló");
    sendAT("AT+HTTPTERM", "OK", 3000);
    return -1;
  }

  // Lanzar GET — el módem responde OK y luego emite el URC +HTTPACTION
  flushModem();
  SerialAT.println("AT+HTTPACTION=0");

  String buf = "";
  uint32_t t0 = millis();
  while (millis() - t0 < 60000UL) {
    while (SerialAT.available())
      buf += (char)SerialAT.read();

    int idx = buf.indexOf("+HTTPACTION:");
    if (idx != -1) {
      int eol = buf.indexOf('\n', idx);
      if (eol != -1) {
        // Formato: "+HTTPACTION: 0,<code>,<len>"
        String line = buf.substring(idx, eol);
        int c1 = line.indexOf(',');
        int c2 = (c1 != -1) ? line.indexOf(',', c1 + 1) : -1;
        if (c1 != -1 && c2 != -1) {
          int code = line.substring(c1 + 1, c2).toInt();
          int32_t len = line.substring(c2 + 1).toInt();
          Serial.printf("[HTTP] +HTTPACTION: code=%d len=%d\n", code, len);
          if (code >= 200 && code < 300)
            return len;
          Serial.printf("[HTTP] Error HTTP %d\n", code);
          sendAT("AT+HTTPTERM", "OK", 3000);
          return -1;
        }
      }
    }

    // ERROR sin +HTTPACTION → fallo real
    if (buf.indexOf("ERROR") != -1 && buf.indexOf("+HTTPACTION:") == -1) {
      Serial.printf("[HTTP] Error en HTTPACTION: %s\n", buf.c_str());
      sendAT("AT+HTTPTERM", "OK", 3000);
      return -1;
    }
  }

  Serial.println("[HTTP] Timeout esperando +HTTPACTION");
  sendAT("AT+HTTPTERM", "OK", 3000);
  return -1;
}

/*
  httpReadChunk:
    Lee 'size' bytes desde 'offset' de la respuesta HTTP ya buffereada.
    Escribe los bytes en 'buf' (debe tener al menos 'size' bytes).
    Retorna bytes leídos (0 = fin de datos, -1 = error).

  Comportamiento de lectura binaria:
    Tras el header "+HTTPREAD: <n>\r\n" llegan exactamente <n> bytes binarios.
    El timeout se resetea en cada byte recibido (inter-byte timeout = 3 s),
    lo que elimina el fallo "bloque incompleto" de implementaciones anteriores.
*/
int32_t httpReadChunk(int32_t offset, int32_t size, uint8_t *buf) {
  flushModem();
  SerialAT.println("AT+HTTPREAD=" + String(offset) + "," + String(size));

  // ── Fase 1: leer la cabecera ASCII "+HTTPREAD: <n>" ──────────
  String hdr = "";
  int32_t dataLen = -1;
  uint32_t t0 = millis();

  while (millis() - t0 < 10000 && dataLen == -1) {
    while (SerialAT.available() && dataLen == -1) {
      char c = SerialAT.read();
      hdr += c;

      if (c == '\n') {
        if (hdr.indexOf("+HTTPREAD:") != -1) {
          int idx = hdr.lastIndexOf("+HTTPREAD:");
          String lenStr = hdr.substring(idx + 10);
          lenStr.trim();
          dataLen = lenStr.toInt();
        } else if (hdr.indexOf("ERROR") != -1) {
          Serial.printf("[HTTP] Error en HTTPREAD: %s\n", hdr.c_str());
          return -1;
        }
      }
    }
  }

  if (dataLen < 0) {
    Serial.println("[HTTP] Timeout esperando +HTTPREAD header");
    return -1;
  }
  if (dataLen == 0)
    return 0;

  // ── Fase 2: leer exactamente dataLen bytes binarios ───────────
  // Timeout inter-byte: si pasan 3 s sin recibir ningún byte → timeout.
  // A 115200 baud (~11 KB/s) un bloque de 2 KB llega en ~180 ms;
  // 3 s de margen es más que suficiente para cualquier latencia real.
  int32_t got = 0;
  uint32_t tLast = millis();
  const uint32_t INTER_BYTE_TO = 3000;

  while (got < dataLen && millis() - tLast < INTER_BYTE_TO) {
    if (SerialAT.available()) {
      buf[got++] = SerialAT.read();
      tLast = millis();  // resetear en cada byte recibido
    }
  }

  if (got < dataLen) {
    Serial.printf("[HTTP] Chunk parcial: esperados=%d recibidos=%d\n", dataLen,
                  got);
  }
  return got;
}

/*
  doOTA:
    Descarga el firmware desde 'url' (debe ser https://) usando AT+HTTP*
    y lo escribe en la partición OTA del ESP32 con la API Update.h.

  Flujo:
    1. httpGetInit(url)          → GET, espera Content-Length
    2. Update.begin(totalSize)   → prepara la partición OTA
    3. Loop: httpReadChunk()     → lee en bloques de 2 KB
             Update.write()     → escribe en flash
    4. AT+HTTPTERM               → cierra sesión HTTP
    5. Update.end()              → valida y activa la partición
    6. ESP.restart()             → aplica el nuevo firmware
*/
void doOTA(const String &version, const String &url) {
  Serial.println("\n[OTA] ======== INICIANDO DESCARGA OTA ========");
  Serial.printf("[OTA] URL   : %s\n", url.c_str());
  Serial.printf("[OTA] Versión objetivo: %s\n", version.c_str());

  publishOtaStatus(
    ("{\"status\":\"descargando\",\"target\":\"" + version + "\"}").c_str());
  delay(300);

  int32_t totalSize = httpGetInit(url);
  if (totalSize <= 0) {
    Serial.println("[OTA] Fallo en la petición HTTP al firmware");
    publishOtaStatus("{\"error\":\"http_fail\"}");
    return;
  }

  Serial.printf("[OTA] Tamaño del firmware: %d bytes\n", totalSize);

  if (!Update.begin(totalSize)) {
    Update.printError(Serial);
    publishOtaStatus("{\"error\":\"no_space\"}");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }
  Serial.println("[OTA] Update.begin() OK — descargando...");

  const int32_t CHUNK_SIZE = 1024;
  static uint8_t chunkBuf[CHUNK_SIZE];  // static → no consume stack
  int32_t offset = 0;
  int32_t written = 0;
  bool failed = false;
  uint32_t tStart = millis();

  while (offset < totalSize && !failed) {
    int32_t toRead = min((int32_t)CHUNK_SIZE, totalSize - offset);
    int32_t got = httpReadChunk(offset, toRead, chunkBuf);

    if (got < 0) {
      Serial.printf("[OTA] Error leyendo chunk en offset=%d\n", offset);
      failed = true;
      break;
    }
    if (got == 0) {
      if (offset < totalSize) {
        Serial.printf("[OTA] Fin prematuro en offset=%d (esperado=%d)\n",
                      offset, totalSize);
        failed = true;
      }
      break;
    }

    int32_t w = Update.write(chunkBuf, got);
    if (w != got) {
      Serial.printf("[OTA] Error flash: write=%d got=%d\n", w, got);
      failed = true;
      break;
    }

    written += w;
    offset += got;

    // Progreso cada ~32 KB o al finalizar
    if ((offset % (CHUNK_SIZE * 16) < CHUNK_SIZE) || offset >= totalSize) {
      float elapsed = (millis() - tStart) / 1000.0f + 0.001f;
      Serial.printf("[OTA] %d / %d bytes (%.0f%%) — %.1f KB/s\r", written,
                    totalSize, written * 100.0f / totalSize,
                    written / 1024.0f / elapsed);
    }
  }

  Serial.println();
  sendAT("AT+HTTPTERM", "OK", 5000);
  Serial.printf("[OTA] Descargados: %d / %d bytes\n", written, totalSize);

  if (failed) {
    publishOtaStatus("{\"error\":\"descarga_fallida\"}");
    Update.abort();
    return;
  }

  if (Update.end() && Update.isFinished()) {
    char okBuf[64];
    snprintf(okBuf, sizeof(okBuf), "{\"status\":\"ok\",\"version\":\"%s\"}",
             version.c_str());
    publishOtaStatus(okBuf);
    Serial.println("[OTA] ✓ OK — reiniciando en 2 s...");
    delay(2000);
    ESP.restart();
  } else {
    publishOtaStatus("{\"error\":\"update_incompleto\"}");
    Update.printError(Serial);
  }
}

// ═══════════════════════════════════════════════════════════════
//  OTA AUTOMÁTICO AL ARRANCAR — verifica servidor de versiones
// ═══════════════════════════════════════════════════════════════

/*
  checkOtaOnBoot:
    Al inicio, descarga un JSON estático desde OTA_VERSION_URL y
    compara el campo "version" con FIRMWARE_VERSION.
    Si son distintos, llama a doOTA() con la URL del firmware.

    Formato esperado del JSON en el servidor:
      {"version":"2.1","url":"https://example.com/firmware.bin"}

    Nota: AT+HTTPREAD devuelve solo el cuerpo (el módem quita los
    headers HTTP), así que no hay que buscar \r\n\r\n.

    Llama esta función desde setup() DESPUÉS de mqttConnect().
*/
bool checkOtaOnBoot() {
  if (strlen(OTA_VERSION_URL) == 0) {
    Serial.println("[OTA-BOOT] Sin URL de versión configurada — omitiendo");
    return true;  // no hay nada que verificar, no es un fallo
  }

  Serial.println("[OTA-BOOT] ── Verificando versión de firmware ──");
  Serial.printf("[OTA-BOOT] Versión actual : %s\n", FIRMWARE_VERSION);
  Serial.printf("[OTA-BOOT] URL manifiesto : %s\n", OTA_VERSION_URL);

  int32_t manifestLen = httpGetInit(String(OTA_VERSION_URL));
  if (manifestLen <= 0) {
    Serial.println("[OTA-BOOT] No se pudo obtener el manifiesto");
    return false;  // ← señal para reintentar más tarde
  }

  const int32_t MAX_MANIFEST = 512;
  static uint8_t manifestBuf[MAX_MANIFEST + 1];
  int32_t got = httpReadChunk(0, min(manifestLen, MAX_MANIFEST), manifestBuf);
  sendAT("AT+HTTPTERM", "OK", 5000);

  if (got <= 0) {
    Serial.println("[OTA-BOOT] No se pudo leer el manifiesto");
    return false;  // ← también reintentar
  }
  manifestBuf[got] = '\0';

  String jsonBody = String((char *)manifestBuf);
  jsonBody.trim();
  Serial.printf("[OTA-BOOT] Manifiesto: %s\n", jsonBody.c_str());

  auto extractField = [](const String &json, const String &key) -> String {
    String search = "\"" + key + "\":\"";
    int idx = json.indexOf(search);
    if (idx == -1) return "";
    idx += search.length();
    int end = json.indexOf("\"", idx);
    if (end == -1) return "";
    return json.substring(idx, end);
  };

  String remoteVersion = extractField(jsonBody, "version");
  String remoteUrl = extractField(jsonBody, "url");

  if (remoteVersion.length() == 0 || remoteUrl.length() == 0) {
    Serial.println("[OTA-BOOT] Manifiesto inválido (faltan version o url)");
    return true;  // el manifiesto respondió pero está mal formado — no insistir en loop
  }

  Serial.printf("[OTA-BOOT] Versión remota : %s\n", remoteVersion.c_str());

  if (remoteVersion == FIRMWARE_VERSION) {
    Serial.println("[OTA-BOOT] Firmware al día — sin actualización.");
    return true;
  }

  if (!remoteUrl.startsWith("https://")) {
    Serial.println("[OTA-BOOT] URL del firmware no es https:// — abortando");
    return true;  // no es un fallo de red, es config inválida — no insistir
  }

  Serial.printf("[OTA-BOOT] *** Nueva versión disponible: %s → %s ***\n",
                FIRMWARE_VERSION, remoteVersion.c_str());

  if (mqttConnected) {
    String statusMsg = "{\"status\":\"auto_ota_inicio\",\"desde\":\"" + String(FIRMWARE_VERSION) + "\",\"hacia\":\"" + remoteVersion + "\"}";
    publishOtaStatus(statusMsg.c_str());
    delay(300);
  }

  doOTA(remoteVersion, remoteUrl);  // si tiene éxito, reinicia el ESP32 y no vuelve aquí

  return true;  // si doOTA falló sin reiniciar, igual no insistas en loop — ya quedó loggeado
}

/*
  handleOtaCommand:
    Parsea el payload JSON del mensaje MQTT OTA (sin librería JSON).
    Formato esperado: {"token":"...","version":"...","url":"https://..."}

    Seguridad:
      1. Token secreto — verifica contra NVS.
      2. URL debe comenzar en https://.
*/
void handleOtaCommand(const String &payload) {
  Serial.printf("[OTA] Comando recibido: %s\n", payload.c_str());

  // ── Capa 1: token obligatorio ────────────────────────────────
  if (strlen(otaToken) < 8) {
    Serial.println("[SEC] OTA rechazado: token no configurado");
    publishOtaStatus("{\"error\":\"token_no_configurado\"}");
    return;
  }

  // Extraer campo "token" del JSON a mano
  // Buscamos: "token":"<valor>"
  auto extractField = [](const String &json, const String &key) -> String {
    String search = "\"" + key + "\":\"";
    int idx = json.indexOf(search);
    if (idx == -1)
      return "";
    idx += search.length();
    int end = json.indexOf("\"", idx);
    if (end == -1)
      return "";
    return json.substring(idx, end);
  };

  String token = extractField(payload, "token");
  String version = extractField(payload, "version");
  String url = extractField(payload, "url");

  if (token != String(otaToken)) {
    Serial.println("[SEC] OTA rechazado: token inválido");
    publishOtaStatus("{\"error\":\"token_invalido\"}");
    return;
  }

  if (version.length() == 0 || url.length() == 0) {
    Serial.println("[OTA] Payload incompleto (version o url faltante)");
    publishOtaStatus("{\"error\":\"payload_incompleto\"}");
    return;
  }

  // ── Capa 2: URL debe ser HTTPS ───────────────────────────────
  if (!url.startsWith("https://")) {
    Serial.println("[SEC] OTA rechazado: URL no es HTTPS");
    publishOtaStatus("{\"error\":\"url_no_https\"}");
    return;
  }

  Serial.printf("[OTA] Token OK. Versión objetivo: %s\n", version.c_str());
  doOTA(version, url);
}

// ═══════════════════════════════════════════════════════════════
//  MANEJO DE MENSAJES MQTT ENTRANTES (URC +CMQTTRXSTART)
// ═══════════════════════════════════════════════════════════════

/*
  processIncomingMqtt:
    Llamada desde loop() cuando se detecta el URC "+CMQTTRXSTART".
    Lee el topic y payload del mensaje usando AT+CMQTTRXTOPIC y
    AT+CMQTTRXPAYLOAD, luego despacha al handler correspondiente.

  Flujo AT para recibir un mensaje MQTT:
    Cuando llega un mensaje, el módem emite una secuencia de URCs:
      +CMQTTRXSTART: <index>,<topicLen>,<payloadLen>
      +CMQTTRXTOPIC: <index>,<topicLen>
      <topic>
      +CMQTTRXPAYLOAD: <index>,<payloadLen>
      <payload>
      +CMQTTRXEND: <index>
*/
void processIncomingMqtt() {
  // Leer todo lo que haya en el buffer de la UART (el URC puede estar
  // parcialmente en urcPending; leer el resto directamente)
  String fullBuf = urcPending;

  // Leer bytes adicionales con timeout corto
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    while (SerialAT.available()) {
      fullBuf += (char)SerialAT.read();
      t0 = millis();  // resetear timeout mientras llegan datos
    }
    if (fullBuf.indexOf("+CMQTTRXEND") != -1)
      break;
    delay(20);
  }

  Serial.println("[MQTT-RX] URC completo recibido");

  // ── Extraer topic ─────────────────────────────────────────────
  // El topic viene después de "+CMQTTRXTOPIC: <index>,<len>\r\n"
  String topic = "";
  int topicUrcIdx = fullBuf.indexOf("+CMQTTRXTOPIC:");
  if (topicUrcIdx != -1) {
    int topicLineEnd = fullBuf.indexOf('\n', topicUrcIdx);
    if (topicLineEnd != -1) {
      // El topic raw está en la siguiente línea
      int topicStart = topicLineEnd + 1;
      int topicEnd = fullBuf.indexOf('\n', topicStart);
      if (topicEnd == -1)
        topicEnd = fullBuf.length();
      topic = fullBuf.substring(topicStart, topicEnd);
      topic.trim();
    }
  }

  // ── Extraer payload ───────────────────────────────────────────
  String payload = "";
  int payloadUrcIdx = fullBuf.indexOf("+CMQTTRXPAYLOAD:");
  if (payloadUrcIdx != -1) {
    int payloadLineEnd = fullBuf.indexOf('\n', payloadUrcIdx);
    if (payloadLineEnd != -1) {
      int payloadStart = payloadLineEnd + 1;
      int payloadEnd = fullBuf.indexOf("+CMQTTRXEND", payloadStart);
      if (payloadEnd == -1)
        payloadEnd = fullBuf.length();
      payload = fullBuf.substring(payloadStart, payloadEnd);
      payload.trim();
    }
  }

  if (topic.length() == 0) {
    Serial.println("[MQTT-RX] Topic vacío — descartado");
    return;
  }

  Serial.printf("[MQTT-RX] Topic: %s\n", topic.c_str());
  Serial.printf("[MQTT-RX] Payload: %s\n", payload.c_str());

  // ── Dispatch ──────────────────────────────────────────────────
  char otaTopic[64];
  snprintf(otaTopic, sizeof(otaTopic), "%s%u", OTA_CMD_TOPIC_PREFIX, DEVICE_ID);

  if (topic == String(otaTopic)) {
    handleOtaCommand(payload);
    return;
  }

  Serial.printf("[MQTT-RX] Topic desconocido: %s\n", topic.c_str());
}

// ═══════════════════════════════════════════════════════════════
//  CONSTRUCCIÓN DEL PAQUETE GPS
// ═══════════════════════════════════════════════════════════════

/*  Convierte fecha/hora UTC del GPS a hora Venezuela (UTC-4)
    y formatea como "YYYY-MM-DDTHH:MM:SS" en 'out' (20 bytes).   */
void gpsToISO_VET(int year, int month, int day, int hour, int minute,
                  int second, char *out) {
  static const uint8_t dim[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  // Workaround GPS week rollover
  if (year < 2019) {
    day += 7168;
    while (true) {
      bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
      int d = (month == 2 && leap) ? 29 : dim[month - 1];
      if (day <= d)
        break;
      day -= d;
      if (++month > 12) {
        month = 1;
        year++;
      }
    }
  }

  // UTC → VET (UTC-4)
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

  snprintf(out, 20, "%04d-%02d-%02dT%02d:%02d:%02d", year, month, day, hour,
           minute, second);
}

// ═══════════════════════════════════════════════════════════════
//  RED MÓVIL — verificación rápida de registro
// ═══════════════════════════════════════════════════════════════

bool isNetworkUp() {
  // Cualquier registro (GSM, GPRS o LTE) es suficiente.
  // No requerir todos simultáneos: tras CMQTTSTOP el modem puede
  // emitir +CGEV que baja CGREG temporalmente aunque siga en LTE.
  String r = sendATStr("AT+CREG?", 3000);
  if (r.indexOf(",1") != -1 || r.indexOf(",5") != -1)
    return true;
  String rg = sendATStr("AT+CGREG?", 3000);
  if (rg.indexOf(",1") != -1 || rg.indexOf(",5") != -1)
    return true;
  String re = sendATStr("AT+CEREG?", 3000);
  return re.indexOf(",1") != -1 || re.indexOf(",5") != -1;
}

// ═══════════════════════════════════════════════════════════════
//  ALMACENAMIENTO OFFLINE  (store-and-forward)
// ═══════════════════════════════════════════════════════════════

void initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Error montando LittleFS");
  } else {
    Serial.printf("[FS] LittleFS OK — %u / %u KB\n",
                  LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);
  }
}

void saveOffline(const data_pkt_t &pkt) {
  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  uint32_t count = f ? (f.size() / sizeof(data_pkt_t)) : 0;
  if (f)
    f.close();

  if (count < OFFLINE_MAX_RECORDS) {
    // Cola no llena: agregar al final directamente
    File fw = LittleFS.open(OFFLINE_DATA_FILE, "a");
    if (!fw) {
      Serial.println("[FS] Error abriendo archivo");
      return;
    }
    fw.write((uint8_t *)&pkt, sizeof(data_pkt_t));
    fw.close();
    Serial.printf("[FS] Guardado offline seq=%u bus#%u (total=%u)\n", pkt.seq,
                  pkt.device_id, count + 1);
  } else {
    // Cola llena: descartar el más antiguo, agregar el nuevo al final
    File src = LittleFS.open(OFFLINE_DATA_FILE, "r");
    File tmp = LittleFS.open("/ofq_tmp.bin", "w");
    if (!src || !tmp) {
      if (src)
        src.close();
      if (tmp)
        tmp.close();
      Serial.println("[FS] Error en rotación circular");
      return;
    }

    // Saltar el primer registro (el más antiguo)
    src.seek(sizeof(data_pkt_t));

    // Copiar los restantes al archivo temporal
    uint8_t cbuf[sizeof(data_pkt_t)];
    while (src.readBytes((char *)cbuf, sizeof(data_pkt_t)) == sizeof(data_pkt_t))
      tmp.write(cbuf, sizeof(data_pkt_t));

    // Agregar el nuevo paquete al final
    tmp.write((uint8_t *)&pkt, sizeof(data_pkt_t));

    src.close();
    tmp.close();

    LittleFS.remove(OFFLINE_DATA_FILE);
    LittleFS.rename("/ofq_tmp.bin", OFFLINE_DATA_FILE);

    Serial.printf(
      "[FS] Buffer circular: seq=%u guardado (antiguo descartado)\n",
      pkt.seq);
  }
}

void flushOfflineQueue() {
  if (!mqttConnected || !LittleFS.exists(OFFLINE_DATA_FILE))
    return;

  File f = LittleFS.open(OFFLINE_DATA_FILE, "r");
  if (!f)
    return;

  uint32_t total = f.size() / sizeof(data_pkt_t);
  if (total == 0) {
    f.close();
    LittleFS.remove(OFFLINE_DATA_FILE);
    return;
  }

  Serial.printf("[FS] Enviando %u paquetes almacenados...\n", total);

  File tmp = LittleFS.open("/ofq_tmp.bin", "w");
  if (!tmp) {
    f.close();
    return;
  }

  uint32_t sent = 0, kept = 0;
  bool mqttFailed = false;

  while (f.available() >= (int)sizeof(data_pkt_t)) {
    data_pkt_t pkt;
    f.readBytes((char *)&pkt, sizeof(data_pkt_t));

    if (!mqttFailed && mqttConnected) {
      char topic[128];
      snprintf(topic, sizeof(topic), "%s/%u", TOPIC_BASE, pkt.device_id);

      char buf[JSON_BUF_SIZE];
      snprintf(buf, sizeof(buf),
               "{\"device_id\":%u,\"ts\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,"
               "\"speed\":%.1f,\"flags\":%u,\"seq\":%u,\"stored\":true}",
               pkt.device_id, pkt.timestamp, pkt.lat / 1e7f, pkt.lon / 1e7f,
               pkt.speed / 10.0f, pkt.flags, pkt.seq);

      bool ok = mqttPublishText(topic, buf, /*qos=*/0, /*retain=*/0);
      if (ok) {
        sent++;
      } else {
        mqttFailed = true;
        mqttConnected = false;
        tmp.write((uint8_t *)&pkt, sizeof(data_pkt_t));
        kept++;
      }
    } else {
      tmp.write((uint8_t *)&pkt, sizeof(data_pkt_t));
      kept++;
    }
  }

  f.close();
  tmp.close();
  LittleFS.remove(OFFLINE_DATA_FILE);

  if (kept > 0) {
    LittleFS.rename("/ofq_tmp.bin", OFFLINE_DATA_FILE);
    Serial.printf("[FS] Flush parcial: %u enviados, %u pendientes\n", sent,
                  kept);
  } else {
    LittleFS.remove("/ofq_tmp.bin");
    Serial.printf("[FS] Flush completo: %u paquetes enviados\n", sent);
  }
}

/*  buildAndPublish:
    Lee el GPS, arma el pkt y lo publica como JSON via AT+CMQTT*
    siguiendo el mismo formato que publishPacket() del rx.ino.     */
bool buildAndPublish() {
  bool hasFix =
    gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  bool hdopOk = hasFix && gps.hdop.isValid() && (gps.hdop.hdop() < HDOP_MAX);
  bool moving = hasFix && gps.speed.isValid() && (gps.speed.kmph() >= 2.0f);

  data_pkt_t pkt;
  pkt.pkt_type = PKT_DATA;
  pkt.device_id = DEVICE_ID;
  pkt.seq = gSeq++;
  pkt.flags = 0;

  if (hasFix) {
    bool hasSats =
      !gps.satellites.isValid() || (gps.satellites.value() >= SAT_MIN);
    if (!hasSats) {
      Serial.printf("[GPS] Satélites insuficientes (%u < %u) — cancelado\n",
                    gps.satellites.value(), SAT_MIN);
      return false;
    }
    // Usar coordenadas suavizadas por EMA (definidas en loop)
    pkt.lat = (int32_t)(smoothLat * 1e7);
    pkt.lon = (int32_t)(smoothLon * 1e7);
    pkt.speed = (uint16_t)(gps.speed.kmph() * 10.0f);
    gpsToISO_VET(gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second(),
                 pkt.timestamp);
    pkt.flags |= 0x01;  // bit0 = fix válido
    if (hdopOk)
      pkt.flags |= 0x02;  // bit1 = HDOP OK
    if (moving)
      pkt.flags |= 0x04;  // bit2 = en movimiento
  } else {
    Serial.println("[GPS] Sin fix — publicación cancelada");
    return false;
  }

  // Debug por Serial (mismo formato que printPacket() del rx.ino)
  Serial.printf("[PKT] Bus#%-4u seq=%-5u %s fix=%c | %+.5f,%+.5f | %.1f km/h\n",
                pkt.device_id, pkt.seq, pkt.timestamp,
                (pkt.flags & 0x01) ? 'Y' : 'N', pkt.lat / 1e7f, pkt.lon / 1e7f,
                pkt.speed / 10.0f);

  // ── Verificar red móvil antes de publicar ────────────────────
  if (!isNetworkUp()) {
    Serial.println("[NET] Sin red móvil — guardando offline");
    saveOffline(pkt);
    mqttConnected = false;  // forzar reconexión cuando vuelva la red
    return false;
  }

  if (!mqttConnected) {
    Serial.println("[MQTT] Sin conexión MQTT — guardando offline");
    saveOffline(pkt);
    return false;
  }

  publishPacket(pkt);
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] GPS-MQTT Tracker v2 (OTA) — iniciando...");
  Serial.printf("[BOOT] Firmware: v%s\n", FIRMWARE_VERSION);
  Serial.printf("[BOOT] Topic datos:  %s/%u\n", TOPIC_BASE, DEVICE_ID);
  Serial.printf("[BOOT] Topic global: %s/%u\n", TOPIC_GLOBAL, DEVICE_ID);
  Serial.printf("[BOOT] OTA topic:    %s%u\n", OTA_CMD_TOPIC_PREFIX, DEVICE_ID);
  Serial.printf("[BOOT] Struct size:  %u bytes\n",
                (unsigned)sizeof(data_pkt_t));

  // ── Cargar token OTA desde NVS ──────────────────────────────
  loadOtaToken();

  // ── CONFIGURA EL TOKEN OTA AQUÍ LA PRIMERA VEZ ─────────────
  // Descomenta y ajusta la siguiente línea para guardar el token:
  // saveOtaToken("12345678");

  // ── Iniciar SerialAT ─────────────────────────────────────────
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  // ── Alimentar el módulo A7670G ───────────────────────────────
  // GPIO 12 habilita el regulador; DEBE estar en HIGH antes que todo
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
  delay(100);

  // ── Reset hardware del módulo ────────────────────────────────
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  // ── Pulso en PWRKEY para encender ───────────────────────────
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  // ── Esperar que el módem responda a AT ──────────────────────
  Serial.print("[BOOT] Esperando módem");
  int retry = 0;
  while (!sendAT("AT", "OK", 1000)) {
    Serial.print(".");
    if (retry++ > 10) {
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
    }
  }
  Serial.println(" OK");

  sendAT("ATE0", "OK");       // desactivar eco
  sendAT("AT+CMEE=2", "OK");  // errores legibles

  delay(1000);

  diagSIMNetwork();

  // ── Verificar SIM ────────────────────────────────────────────
  while (true) {
    String r = sendATStr("AT+CPIN?", 3000);
    if (r.indexOf("READY") != -1) {
      Serial.println("[SIM] SIM lista");
      break;
    }
    if (r.indexOf("SIM PIN") != -1) {
      Serial.println("[SIM] SIM bloqueada — introduciendo PIN");
      if (strlen(SIM_PIN) > 0)
        sendAT("AT+CPIN=\"" SIM_PIN "\"", "OK", 5000);
    }
    delay(1000);
  }

  // ── Modo de red y APN ────────────────────────────────────────
  sendAT("AT+CNMP=2", "OK", 5000);  // modo automático (LTE/3G/2G)
  sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);

  // ── Esperar registro en red ──────────────────────────────────
  // CREG=3 (rechazado) durante sesión fantasma es TEMPORAL:
  // no abandonar — intentar re-enganche y seguir esperando.
  Serial.print("[NET] Esperando registro");
  int rejectCount = 0;
  while (true) {
    String r = sendATStr("AT+CREG?", 3000);
    if (r.indexOf(",1") != -1 || r.indexOf(",5") != -1) {
      Serial.println(" — Registrado");
      break;
    }
    if (r.indexOf(",3") != -1) {
      rejectCount++;
      Serial.printf("\n[NET] Registro rechazado (%d) — forzando re-enganche\n",
                    rejectCount);
      // Reintentar re-enganche (máximo 3 veces antes de reiniciar ESP)
      if (rejectCount >= 3) {
        Serial.println("[NET] ⚠ Demasiados rechazos — reiniciando ESP32");
        delay(2000);
        ESP.restart();
      }
      // Desregistrar y re-registrar automáticamente
      sendAT("AT+COPS=2", "OK", 15000);
      delay(2000);
      sendAT("AT+COPS=0", "OK", 30000);
      delay(3000);
      Serial.print("[NET] Esperando registro");
      continue;
    }
    Serial.print(".");
    delay(1000);
  }

  // ── Activar contexto PDP ─────────────────────────────────────
  for (int i = 0; i < 3; i++) {
    if (sendAT("AT+CGACT=1,1", "OK", 30000))
      break;
    sendAT("AT+CGACT=0,1", "OK", 5000);
    delay(2000);
  }
  String ip = sendATStr("AT+CGPADDR=1", 5000);
  Serial.print("[NET] IP: ");
  Serial.println(ip);

  // ── Verificar sesión LTE (detecta SIMs con sesión "fantasma") ────────
  // Algunas SIMs obtienen registro y una IP interna pero sin ancla a
  // celda real (CPSI muestra 000-00). En ese caso el DNS falla y MQTT
  // nunca conecta. Se fuerza re-enganche antes de intentar MQTT.
  if (!isPDPSessionValid()) {
    Serial.println("[NET] ⚠ Sesión inicial inválida — forzando re-enganche");
    if (forceReattach()) {
      sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);
      for (int i = 0; i < 3; i++) {
        if (sendAT("AT+CGACT=1,1", "OK", 30000))
          break;
        delay(3000);
      }
      ip = sendATStr("AT+CGPADDR=1", 5000);
      Serial.print("[NET] IP tras re-enganche: ");
      Serial.println(ip);
    } else {
      Serial.println("[NET] ⚠ Re-enganche falló — continuando de todas formas");
    }
  }

  // ── Configurar DNS públicos (independiente del resultado de CPSI) ────
  configureDNS();

  // ── Almacenamiento LittleFS ──────────────────────────────────
  initStorage();

  // ── GPS L76K: solo RX, UART2 ────────────────────────────────
  GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
  Serial.println("[GPS] L76K iniciado — esperando señal...");

  // ── Primera conexión MQTT ───────────────────────────────────
  if (mqttConnect()) {
    mqttConnected = true;

    // ── Publicar versión actual (retained) ─────────────────────
    // Queda en el broker para consultarla en cualquier momento.
    // Topic: tracker/status/version/<DEVICE_ID>
    char verTopic[64];
    char verPayload[96];
    snprintf(verTopic, sizeof(verTopic), "%s%u", VERSION_TOPIC_PREFIX,
             DEVICE_ID);
    snprintf(verPayload, sizeof(verPayload),
             "{\"device_id\":%u,\"version\":\"%s\"}", DEVICE_ID,
             FIRMWARE_VERSION);
    mqttPublishText(verTopic, verPayload, /*qos=*/1, /*retain=*/1);
    Serial.printf("[BOOT] Versión publicada en %s\n", verTopic);

  } else {
    Serial.println("[MQTT] Conexión inicial fallida — se reintentará en loop");
  }

  // ── Verificación automática de OTA al arrancar ───────────────
  // Se ejecuta aunque MQTT haya fallado (solo necesita datos móviles).
  // Si hay nueva versión, doOTA() descarga e instala, luego reinicia.
  checkOtaOnBoot();

  Serial.println("[BOOT] Sistema listo");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  // Alimentar el parser GPS con todo byte disponible
  while (GPS.available())
    gps.encode(GPS.read());

  // ── Log periódico de estado GPS ──────────────────────────────────
  if (millis() - lastGpsLog >= GPS_LOG_INTERVAL) {
    lastGpsLog = millis();
    uint32_t chars = gps.charsProcessed();
    if (chars == 0) {
      Serial.println("[GPS] ⚠ 0 bytes NMEA recibidos — revisar cableado GPIO22 "
                     "/ alimentación L76K");
    } else {
      bool fix =
        gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
      uint8_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f;
      float speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
      Serial.printf("[GPS] chars=%lu fix=%s sats=%u HDOP=%.1f spd=%.1f km/h",
                    chars, fix ? "SI" : "NO", sats, hdop, speed);
      if (fix)
        Serial.printf(" lat=%.6f lon=%.6f", gps.location.lat(),
                      gps.location.lng());
      Serial.println();
    }
  }

  // ── Procesar URCs acumulados ──────────────────────────────────
  if (urcPending.length() > 0) {
    bool hasMqttDisconnect = urcPending.indexOf("+CGEV: NW PDN DEACT") != -1 || urcPending.indexOf("+CMQTTNONET") != -1 || urcPending.indexOf("+CMQTTCONNLOST") != -1;

    bool hasMqttMsg = urcPending.indexOf("+CMQTTRXSTART") != -1;

    if (hasMqttDisconnect) {
      Serial.print("[NET] URC: ");
      Serial.print(urcPending);
      mqttConnected = false;
    }

    // ── Procesar mensaje MQTT entrante ───────────────────────
    if (hasMqttMsg) {
      Serial.println("[MQTT-RX] Mensaje entrante detectado");
      processIncomingMqtt();
    }

    urcPending = "";
  }

  // ── Leer URCs cuando el loop corre libre (sin comandos AT) ───
  // Solo cuando no hay comandos AT en vuelo — es decir, aquí en
  // el loop principal, no dentro de funciones que usen SerialAT
  static String urcBuf;
  while (SerialAT.available()) {
    char c = SerialAT.read();
    urcBuf += c;
    if (c == '\n') {
      bool isDisconnect = urcBuf.indexOf("+CGEV: NW PDN DEACT") != -1 || urcBuf.indexOf("+CMQTTNONET") != -1 || urcBuf.indexOf("+CMQTTCONNLOST") != -1;

      if (isDisconnect) {
        Serial.print("[NET] URC (libre): ");
        Serial.print(urcBuf);
        mqttConnected = false;
      }

      // ── Detectar mensaje MQTT entrante ─────────────────────
      if (urcBuf.indexOf("+CMQTTRXSTART") != -1) {
        Serial.println("[MQTT-RX] Mensaje detectado en loop libre");
        urcPending += urcBuf;
        // Procesarlo en la próxima iteración del loop
      }

      urcBuf = "";
    }
    if (urcBuf.length() > 256)
      urcBuf = "";
  }

  // ── Watchdog del módem ────────────────────────────────────────
  if (millis() - lastModemCheck > MODEM_CHECK_INTERVAL) {
    lastModemCheck = millis();
    if (!modemIsAlive()) {
      Serial.println("[WDT] Módem no responde en loop — reset");
      mqttConnected = false;
      hardResetModem();
      sendAT("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"", "OK", 5000);
      for (int i = 0; i < 3; i++) {
        if (sendAT("AT+CGACT=1,1", "OK", 30000))
          break;
        delay(3000);
      }
    }
  }

  // ── Reconexión automática ────────────────────────────────────
  if (!mqttConnected) {
    if (millis() - lastReconnect > 45000UL) {
      lastReconnect = millis();
      Serial.println("[MQTT] Reconectando...");
      if (mqttConnect()) {
        mqttConnected = true;
        lastReconnect = 0;
        flushOfflineQueue();  // enviar lo acumulado al reconectar
      }
    }
    return;  // no publicar hasta tener conexión
  }

  // ── Reintentar verificación OTA si falló por falta de red ───────
  if (!otaCheckDone && (millis() - lastOtaCheckAttempt > OTA_CHECK_RETRY_MS)) {
    lastOtaCheckAttempt = millis();
    if (isNetworkUp()) {
      Serial.println("[OTA-BOOT] Reintentando verificación de firmware...");
      otaCheckDone = checkOtaOnBoot();
    } else {
      Serial.println("[OTA-BOOT] Red aún no disponible — se reintentará");
    }
  }

  // ── Flush periódico por si quedaron pendientes ───────────────
  if (millis() - lastFlush > 60000UL) {
    lastFlush = millis();
    flushOfflineQueue();
  }

  // ── Publicar si el GPS tiene fix y se movió ≥ DISTANCE_MIN_M ───────
  if (!gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
    emaReady = false;  // resetear EMA si se pierde el fix
    return;
  }

  double curLat = gps.location.lat();
  double curLon = gps.location.lng();

  // ── Actualizar EMA adaptativo ──────────────────────────────────────
  // Alpha pequeño cuando estamos quietos = filtro muy agresivo
  float gpsSpeed = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
  float alpha =
    (gpsSpeed >= MIN_SPEED_KMPH) ? EMA_ALPHA_MOVING : EMA_ALPHA_STILL;

  if (!emaReady) {
    smoothLat = curLat;
    smoothLon = curLon;
    emaReady = true;
  } else {
    smoothLat = alpha * curLat + (1.0f - alpha) * smoothLat;
    smoothLon = alpha * curLon + (1.0f - alpha) * smoothLon;
  }

  // ── Hysteresis de velocidad ────────────────────────────────────────
  // El vehículo entra en estado "moving" solo si mantiene velocidad
  // suficiente durante MOVING_CONFIRM_COUNT lecturas consecutivas.
  // Sale del estado al primer reporte de velocidad baja.
  if (gpsSpeed >= MIN_SPEED_KMPH) {
    if (movingConfirm < MOVING_CONFIRM_COUNT)
      movingConfirm++;
    if (movingConfirm >= MOVING_CONFIRM_COUNT)
      isMovingState = true;
  } else {
    movingConfirm = 0;
    isMovingState = false;  // sale inmediatamente al detenerse
  }

  // ── Distancia desde última publicación ────────────────────────────
  double dist = hasLastPos ? TinyGPSPlus::distanceBetween(lastLat, lastLon,
                                                          smoothLat, smoothLon)
                           : DISTANCE_MIN_M;

  // ── Cooldown entre publicaciones ──────────────────────────────────
  bool cooldownOk = (millis() - lastPubMs) >= PUB_COOLDOWN_MS;

  // ── Condición 1: en movimiento (velocidad + distancia + cooldown) ──
  if (isMovingState && dist >= DISTANCE_MIN_M && cooldownOk) {
    if (buildAndPublish()) {
      lastLat = smoothLat;
      lastLon = smoothLon;
      hasLastPos = true;
      lastPubMs = millis();
      lastHeartbeat = millis();  // reiniciar heartbeat también
    }
    return;
  }

  // ── Condición 2: heartbeat estacionario (fix válido + tiempo) ─────
  // Publica aunque el vehículo esté detenido para confirmar que el
  // tracker sigue activo y reportar su última posición conocida.
  if ((millis() - lastHeartbeat) >= STATIONARY_HEARTBEAT_MS) {
    Serial.println("[GPS] Heartbeat estacionario");
    if (buildAndPublish()) {
      lastHeartbeat = millis();
      lastPubMs = millis();
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  INFORMACIÓN SIM + RED CELULAR
// ═══════════════════════════════════════════════════════════════

void diagSIMNetwork() {

  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.println("      DIAGNÓSTICO SIM / RED LTE");
  Serial.println("══════════════════════════════════════");

  struct ATQuery {
    const char *name;
    const char *cmd;
  };

  ATQuery queries[] = {

    { "ICCID SIM", "AT+CICCID" }, { "IMSI SIM", "AT+CIMI" }, { "OPERADOR", "AT+COPS?" }, { "ESTADO SIM", "AT+CPIN?" }, { "CALIDAD RSSI", "AT+CSQ" }, { "CALIDAD EXTENDIDA", "AT+CESQ" }, { "REGISTRO GSM", "AT+CREG?" }, { "REGISTRO GPRS", "AT+CGREG?" }, { "REGISTRO LTE", "AT+CEREG?" }, { "APN", "AT+CGDCONT?" }, { "IP", "AT+CGPADDR=1" }, { "PDP", "AT+CGACT?" }, { "RED", "AT+CPSI?" }, { "PING", "AT+CDNSGIP=\"google.com\"" }, { "IPv", "AT+CGPADDR=1" }, { "RESTRICC", "AT+COPS?" }, { "RESTRIC2", "AT+CPSI?" }

  };

  for (auto &q : queries) {

    Serial.println();
    Serial.print("[");
    Serial.print(q.name);
    Serial.println("]");

    String response = sendATStr(q.cmd, 5000);

    Serial.println(response);

    delay(300);
  }

  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.println(" FIN DIAGNÓSTICO SIM / RED");
  Serial.println("══════════════════════════════════════");
}
