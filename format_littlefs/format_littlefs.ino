// ============================================================
//  format_littlefs.ino
//  Sketch de un solo uso para formatear la partición LittleFS.
//
//  INSTRUCCIONES:
//  1. Selecciona: Tools → Partition Scheme → 8M with spiffs
//  2. Sube ESTE sketch al ESP32
//  3. Abre Serial Monitor (115200 baud)
//  4. Espera a ver "Formato OK" o "Formato FALLO"
//  5. Vuelve a subir el sketch principal (TX)
// ============================================================
#include <LittleFS.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[FORMAT] Formateando particion LittleFS...");

  if (LittleFS.format()) {
    Serial.println("[FORMAT] Formato OK — ya puedes subir el sketch principal");
  } else {
    Serial.println("[FORMAT] Formato FALLO — revisa Partition Scheme");
  }
}

void loop() {}
