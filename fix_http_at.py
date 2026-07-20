#!/usr/bin/env python3
"""
Reemplaza las funciones doOTA y checkOtaOnBoot en GPS-MQTT.ino
para usar los comandos AT+HTTP* correctos del A7670G.
"""

import re

FILE = "GPS-MQTT/GPS-MQTT.ino"

with open(FILE, "r", encoding="utf-8") as f:
    content = f.read()

# ─────────────────────────────────────────────────────────────────
# 1. Reemplazar la función doOTA completa (del comentario al cierre })
# ─────────────────────────────────────────────────────────────────
OLD_DOOTA_COMMENT_START = "/*\n  doOTA:\n    Descarga el firmware desde 'url' (debe ser https://) usando los\n    comandos AT+CHTTPS* del módem A7670G"
NEW_DOOTA = r'''/*
  doOTA:
    Descarga el firmware desde 'url' (debe ser https://) usando los
    comandos AT+HTTP* del módem A7670G y lo escribe en la partición
    OTA del ESP32 con la API Update.h.

  Flujo AT:
    1. AT+HTTPTERM / AT+HTTPINIT        → limpiar e iniciar stack HTTP
    2. AT+HTTPPARA="CID",1             → contexto PDP
    3. AT+HTTPPARA="URL","<url>"       → URL completa
    4. AT+HTTPACTION=0                  → GET (espera +HTTPACTION: 0,200,<size>)
    5. AT+HTTPREAD=<start>,<len>        → leer en bloques
    6. AT+HTTPTERM                      → liberar stack HTTP
*/
void doOTA(const String &version, const String &url) {
  Serial.println("\n[OTA] ========== INICIANDO OTA ==========");
  Serial.printf("[OTA] Actual: %s  Nueva: %s\n", FIRMWARE_VERSION,
                version.c_str());
  Serial.printf("[OTA] URL: %s\n", url.c_str());

  if (version == FIRMWARE_VERSION) {
    publishOtaStatus("{\"info\":\"ya_tengo_esta_version\"}");
    return;
  }

  if (!url.startsWith("https://") && !url.startsWith("http://")) {
    publishOtaStatus("{\"error\":\"url_invalida\"}");
    return;
  }

  publishOtaStatus(
      ("{\"status\":\"descargando\",\"target\":\"" + version + "\"}").c_str());
  delay(300);

  // ── 1. Suspender MQTT si está activo (no pueden coexistir) ───
  if (mqttConnected) {
    Serial.println("[OTA] Suspendiendo MQTT antes de usar stack HTTP...");
    sendAT("AT+CMQTTDISC=0,10", "OK", 12000);
    delay(500);
    sendAT("AT+CMQTTREL=0", "OK", 3000);
    delay(500);
    sendAT("AT+CMQTTSTOP", "OK", 8000);
    delay(2000);
    mqttConnected = false;
  }

  // ── 2. Limpiar stack HTTP previo + iniciar stack nuevo ───────────────
  sendAT("AT+HTTPTERM", "OK", 3000);
  delay(300);
  if (!sendAT("AT+HTTPINIT", "OK", 5000)) {
    Serial.println("[OTA] Fallo iniciando stack HTTP");
    publishOtaStatus("{\"error\":\"httpinit_fail\"}");
    return;
  }
  delay(500);

  // ── 3. Configurar parámetros HTTP ──────────────────────────────────
  // AT+HTTPPARA="CID",1         → contexto PDP 1
  // AT+HTTPPARA="URL","<url>"  → URL completa (incluye https://)
  sendAT("AT+HTTPPARA=\"CID\",1", "OK", 3000);
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  if (!sendAT(urlCmd, "OK", 5000)) {
    Serial.println("[OTA] Fallo configurando URL HTTP");
    publishOtaStatus("{\"error\":\"httppara_fail\"}");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }
  delay(300);

  // ── 4. Ejecutar GET ──────────────────────────────────────────────────
  // AT+HTTPACTION=0  (0=GET, 1=POST, 2=HEAD)
  // Respuesta asíncrona: +HTTPACTION: <method>,<http_code>,<content_length>
  flushModem();
  SerialAT.println("AT+HTTPACTION=0");

  int32_t totalSize = -1;
  int httpCode = 0;
  {
    String buf;
    uint32_t t0 = millis();
    while (millis() - t0 < 60000UL) {
      while (SerialAT.available())
        buf += (char)SerialAT.read();
      int idx = buf.indexOf("+HTTPACTION:");
      if (idx != -1) {
        int eol = buf.indexOf('\n', idx);
        if (eol != -1) {
          String line = buf.substring(idx + 13, eol);
          line.trim();
          // formato: "0,200,123456"
          int c1 = line.indexOf(',');
          int c2 = line.indexOf(',', c1 + 1);
          if (c1 != -1) {
            httpCode = line.substring(c1 + 1, c2 != -1 ? c2 : line.length()).toInt();
            if (c2 != -1)
              totalSize = line.substring(c2 + 1).toInt();
          }
          break;
        }
      }
      if (buf.indexOf("ERROR") != -1)
        break;
    }
  }

  Serial.printf("[OTA] HTTP %d  Content-Length: %d bytes\n", httpCode, totalSize);

  if (httpCode == 301 || httpCode == 302) {
    Serial.println("[OTA] Redirect — no soportado directamente");
    publishOtaStatus("{\"error\":\"redirect_no_soportado\"}");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }

  if (httpCode != 200) {
    char errBuf[48];
    snprintf(errBuf, sizeof(errBuf), "{\"error\":\"http_%d\"}", httpCode);
    publishOtaStatus(errBuf);
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }

  if (totalSize <= 0) {
    Serial.println("[OTA] Content-Length desconocido — abortando");
    publishOtaStatus("{\"error\":\"content_length_desconocido\"}");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }

  // ── 5. Iniciar partición OTA ─────────────────────────────────────────
  if (!Update.begin(totalSize)) {
    Update.printError(Serial);
    publishOtaStatus("{\"error\":\"no_space\"}");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }
  Serial.println("[OTA] Update.begin() OK — descargando firmware...");

  // ── 6. Leer datos en bloques con AT+HTTPREAD ─────────────────────────
  // AT+HTTPREAD=<start>,<size>  →  +HTTPREAD: <len>\r\n<datos binarios>\r\nOK
  const int RECV_BLOCK = 512;
  int32_t written = 0;
  uint32_t otaTimeout = millis();
  const uint32_t OTA_MAX_MS = 300000UL; // 5 minutos máximo

  while (written < totalSize && millis() - otaTimeout < OTA_MAX_MS) {
    int toRead = min((int32_t)RECV_BLOCK, totalSize - written);

    flushModem();
    SerialAT.println("AT+HTTPREAD=" + String(written) + "," + String(toRead));

    String recvHdr = "";
    uint32_t t0 = millis();
    bool gotData = false;

    while (millis() - t0 < 15000) {
      while (SerialAT.available()) {
        char c = SerialAT.read();
        recvHdr += c;

        if (!gotData && recvHdr.indexOf("+HTTPREAD:") != -1) {
          int idx = recvHdr.indexOf("+HTTPREAD:");
          int eol = recvHdr.indexOf('\n', idx);
          if (eol == -1)
            continue;
          String lenStr = recvHdr.substring(idx + 10, eol);
          lenStr.trim();
          int dataLen = lenStr.toInt();
          if (dataLen <= 0) {
            goto ota_done;
          }

          gotData = true;

          uint8_t rawBuf[RECV_BLOCK + 32];
          int received = 0;
          uint32_t t1 = millis();
          while (received < dataLen && millis() - t1 < 15000) {
            if (SerialAT.available())
              rawBuf[received++] = SerialAT.read();
          }

          if (received > 0) {
            written += Update.write(rawBuf, received);
            Serial.printf("[OTA] %d / %d bytes (%.0f%%)\r",
                          written, totalSize,
                          (written * 100.0f / totalSize));
          }
          break;
        }

        if (recvHdr.indexOf("ERROR") != -1) {
          Serial.println("\n[OTA] Error en AT+HTTPREAD");
          goto ota_done;
        }
      }
      if (gotData)
        break;
    }

    if (!gotData) {
      Serial.println("\n[OTA] Timeout leyendo bloque");
      break;
    }
    delay(20);
  }

ota_done:
  Serial.println();
  Serial.printf("[OTA] Descargados: %d / %d bytes\n", written, totalSize);

  // ── 7. Cerrar stack HTTP ────────────────────────────────────────────
  sendAT("AT+HTTPTERM", "OK", 5000);

  if (written == 0) {
    publishOtaStatus("{\"error\":\"update_no_iniciado\"}");
    return;
  }

  // ── 8. Finalizar y reiniciar ────────────────────────────────────────
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
}'''

# ─────────────────────────────────────────────────────────────────
# 2. Reemplazar el bloque HTTPS de checkOtaOnBoot
# ─────────────────────────────────────────────────────────────────
OLD_BOOT_HTTP = '''  // ── Iniciar stack HTTPS ────────────────────────────────────────
  sendAT("AT+CHTTPSINIT", "OK", 5000);
  delay(300);

  String openCmd = "AT+CHTTPSOPSE=\\"" + host + "\\",443,1";
  if (!sendAT(openCmd, "+CHTTPSOPSE: 0", 30000)) {
    Serial.println("[OTA-BOOT] No se pudo conectar al servidor de versiones");
    sendAT("AT+CHTTPSNULL", "OK", 5000);
    return;
  }
  delay(300);

  // ── Petición GET al manifiesto ─────────────────────────────────
  String httpReq = "GET " + path + " HTTP/1.1\\r\\n";
  httpReq += "Host: " + host + "\\r\\n";
  httpReq += "User-Agent: ESP32-OTA/2.0\\r\\n";
  httpReq += "Connection: close\\r\\n";
  httpReq += "\\r\\n";

  flushModem();
  SerialAT.println("AT+CHTTPSHEAD=0," + String(httpReq.length()));
  if (!waitPromptAndSend((const uint8_t *)httpReq.c_str(), httpReq.length(), 10000)) {
    Serial.println("[OTA-BOOT] Fallo enviando petición GET");
    sendAT("AT+CHTTPSCLSE=0", "OK", 5000);
    sendAT("AT+CHTTPSNULL", "OK", 5000);
    return;
  }
  delay(500);

  // ── Leer respuesta (máx 2 KB — el manifiesto es pequeño) ───────
  String responseBuf = "";
  const int MANIFEST_BLOCK = 512;
  uint32_t t0 = millis();
  bool done = false;

  while (!done && millis() - t0 < 20000) {
    flushModem();
    SerialAT.println("AT+CHTTPSRECV=0," + String(MANIFEST_BLOCK));

    String recvHdr = "";
    uint32_t t1 = millis();
    bool gotBlock = false;

    while (millis() - t1 < 8000) {
      while (SerialAT.available()) {
        char c = SerialAT.read();
        recvHdr += c;

        if (!gotBlock && recvHdr.indexOf("+CHTTPSRECV: DATA,") != -1) {
          int idx = recvHdr.indexOf("+CHTTPSRECV: DATA,");
          int eol = recvHdr.indexOf('\\n', idx);
          if (eol == -1) continue;
          int dataLen = recvHdr.substring(idx + 18, eol).toInt();
          if (dataLen <= 0) { done = true; break; }

          gotBlock = true;
          uint8_t tmp[MANIFEST_BLOCK + 32];
          int rx = 0;
          uint32_t t2 = millis();
          while (rx < dataLen && millis() - t2 < 8000) {
            if (SerialAT.available())
              tmp[rx++] = SerialAT.read();
          }
          responseBuf += String((char *)tmp).substring(0, rx);

          // Dejar de leer si ya tenemos suficiente (2 KB máx)
          if (responseBuf.length() >= 2048) done = true;
          break;
        }

        if (recvHdr.indexOf("+CHTTPSRECV: 0") != -1 ||
            recvHdr.indexOf("+CHTTPSCLSE") != -1) {
          done = true;
          break;
        }
      }
      if (gotBlock || done) break;
    }
    if (!gotBlock) done = true;
    delay(50);
  }

  sendAT("AT+CHTTPSCLSE=0", "OK", 5000);
  sendAT("AT+CHTTPSNULL", "OK", 5000);'''

NEW_BOOT_HTTP = '''  // ── Iniciar stack HTTP ────────────────────────────────────────
  // AT+HTTPTERM libera cualquier sesión residual.
  // AT+HTTPINIT arranca el stack nuevo.
  sendAT("AT+HTTPTERM", "OK", 3000);
  delay(300);
  if (!sendAT("AT+HTTPINIT", "OK", 5000)) {
    Serial.println("[OTA-BOOT] Fallo iniciando stack HTTP");
    return;
  }
  delay(500);

  // ── Configurar URL y ejecutar GET al manifiesto ────────────────
  // AT+HTTPPARA="CID",1     → contexto PDP 1
  // AT+HTTPPARA="URL","...  → URL completa del manifiesto
  // AT+HTTPACTION=0          → GET asíncrono
  sendAT("AT+HTTPPARA=\\"CID\\",1", "OK", 3000);
  String urlCmd = "AT+HTTPPARA=\\"URL\\",\\"" + manifestUrl + "\\"";
  if (!sendAT(urlCmd, "OK", 5000)) {
    Serial.println("[OTA-BOOT] Fallo configurando URL del manifiesto");
    sendAT("AT+HTTPTERM", "OK", 5000);
    return;
  }
  delay(300);

  flushModem();
  SerialAT.println("AT+HTTPACTION=0");

  // Esperar +HTTPACTION: 0,<code>,<size>
  int32_t manifestSize = 0;
  {
    String buf;
    uint32_t t0 = millis();
    bool actionDone = false;
    while (millis() - t0 < 30000UL) {
      while (SerialAT.available())
        buf += (char)SerialAT.read();
      int idx = buf.indexOf("+HTTPACTION:");
      if (idx != -1) {
        int eol = buf.indexOf('\\n', idx);
        if (eol != -1) {
          String line = buf.substring(idx + 13, eol);
          line.trim();
          int c1 = line.indexOf(',');
          int c2 = line.indexOf(',', c1 + 1);
          int code = line.substring(c1 + 1, c2 != -1 ? c2 : line.length()).toInt();
          if (c2 != -1)
            manifestSize = line.substring(c2 + 1).toInt();
          Serial.printf("[OTA-BOOT] HTTP %d  size=%d\\n", code, manifestSize);
          if (code != 200) {
            Serial.println("[OTA-BOOT] Error HTTP al descargar manifiesto");
            sendAT("AT+HTTPTERM", "OK", 5000);
            return;
          }
          actionDone = true;
          break;
        }
      }
      if (buf.indexOf("ERROR") != -1)
        break;
    }
    if (!actionDone) {
      Serial.println("[OTA-BOOT] Timeout esperando respuesta HTTP");
      sendAT("AT+HTTPTERM", "OK", 5000);
      return;
    }
  }

  // ── Leer respuesta (el manifiesto JSON) ───────────────────────
  // AT+HTTPREAD=<start>,<size>  →  +HTTPREAD: <len>\\r\\n<datos>
  String responseBuf = "";
  {
    int toRead = (manifestSize > 0 && manifestSize < 2048) ? manifestSize : 2048;
    flushModem();
    SerialAT.println("AT+HTTPREAD=0," + String(toRead));

    String recvHdr = "";
    uint32_t t0 = millis();
    while (millis() - t0 < 10000) {
      while (SerialAT.available()) {
        char c = SerialAT.read();
        recvHdr += c;

        if (recvHdr.indexOf("+HTTPREAD:") != -1) {
          int idx = recvHdr.indexOf("+HTTPREAD:");
          int eol = recvHdr.indexOf('\\n', idx);
          if (eol == -1)
            continue;
          int dataLen = recvHdr.substring(idx + 10, eol).toInt();
          if (dataLen > 0) {
            uint8_t tmp[2048 + 32];
            int rx = 0;
            uint32_t t1 = millis();
            while (rx < dataLen && millis() - t1 < 8000) {
              if (SerialAT.available())
                tmp[rx++] = SerialAT.read();
            }
            responseBuf = String((char *)tmp).substring(0, rx);
          }
          break;
        }
        if (recvHdr.indexOf("ERROR") != -1)
          break;
      }
      if (responseBuf.length() > 0)
        break;
    }
  }

  sendAT("AT+HTTPTERM", "OK", 5000);'''

# Verificar que encontramos los textos a reemplazar
if OLD_DOOTA_COMMENT_START not in content:
    print("ERROR: No se encontró el inicio del comentario doOTA")
    exit(1)

if OLD_BOOT_HTTP not in content:
    print("ERROR: No se encontró el bloque HTTPS de checkOtaOnBoot")
    exit(1)

# Encontrar y reemplazar la función doOTA completa
# Buscar desde el inicio del comentario hasta el "}" de cierre de la función
start_idx = content.find(OLD_DOOTA_COMMENT_START)
# Buscar el cierre de la función doOTA: la línea "}" que viene después de
# Update.printError(Serial); (última línea de la función)
end_marker = "  Update.printError(Serial);\n}"
end_idx = content.find(end_marker, start_idx)
if end_idx == -1:
    print("ERROR: No se encontró el final de doOTA")
    exit(1)
end_idx += len(end_marker)

content = content[:start_idx] + NEW_DOOTA + content[end_idx:]

# Reemplazar el bloque HTTP de checkOtaOnBoot
if OLD_BOOT_HTTP not in content:
    print("ERROR: Bloque checkOtaOnBoot no encontrado tras editar doOTA")
    exit(1)

content = content.replace(OLD_BOOT_HTTP, NEW_BOOT_HTTP, 1)

# Escribir resultado
with open(FILE, "w", encoding="utf-8") as f:
    f.write(content)

print("✓ GPS-MQTT.ino actualizado correctamente")
print("  - doOTA: AT+CHTTPS* → AT+HTTP*")
print("  - checkOtaOnBoot: AT+CHTTPS* → AT+HTTP*")
