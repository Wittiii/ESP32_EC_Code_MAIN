#include "WiFi.h"
#include "PubSubClient.h"
#include "header.h"
WiFiClient ESP32Client_Hydroponik;
PubSubClient client(ESP32Client_Hydroponik);

// MQTT Broker Einstellungen
const char* mqtt_server = "192.168.178.27"; 
const int mqtt_port = 1883;

const char* mqtt_user = "pwi";
const char* mqtt_password = "1234";

// Topics definieren
const char* topic_publish = "esp32_Hydroponic/ec";
const char* topic_subscribe = "esp32_Hydroponic/ec";

long lastMsg = 0;
char msg[50];
int value = 0;

// Neue Variablen für nicht-blockierendes Reconnect
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectIntervalMs = 5000; // alle 5s versuchen

void reconnect() {
  unsigned long now = millis();

  // Warten bis zum nächsten Versuch, ohne zu blockieren
  if (now - lastReconnectAttempt < reconnectIntervalMs) return;
  lastReconnectAttempt = now;

  // Einzelner Versuch
  if (client.connect("ESP32Client_Hydroponik_ECSensor", mqtt_user, mqtt_password)) {
    client.subscribe(topic_subscribe);
    lastReconnectAttempt = 0; // Reset, damit sofort wieder senden darf
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Hier beginnt die Logik zur Verarbeitung der empfangenen Nachricht.
  
  // 1. Prüfen, von welchem Topic die Nachricht stammt:
  if (strcmp(topic, topic_subscribe) == 0) {
        // 2. Den Payload (Daten) in ein lesbares Format umwandeln:
        String message;
        for (int i = 0; i < length; i++) {
        message += (char)payload[i];
        }
    }

}

void mqttInit(){
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
};

void network_publish_measurement(ECraw measurement) {
    if (!client.connected()) {
        reconnect();
        return;
    }
    client.loop();

    // Nachricht vorbereiten
    String payload = "Vcell=" + String(measurement.Vcell, 2) + "V, " +
                     "rho=" + String(measurement.rho, 2) + ", " +
                     "Rcell=" + String(measurement.Rcell, 2) + "Ohm, " +
                     "EC_raw=" + String(measurement.EC_raw_mS, 5) + "mS, " +
                     "Temp=" + String(measurement.temperature, 2) + "C, " +
                     "EC_comp=" + String(measurement.EC_comp_mS, 5) + "mS";
    
    payload.toCharArray(msg, sizeof(msg));

    // Veröffentlichen
    client.publish(topic_publish, msg);
};


void mqttLoop() {
    if (!client.connected()) {
    reconnect();
    return;
    }
    client.loop();

    long now = millis();
    if (now - lastMsg > 10000) {
    lastMsg = now;
    value++;
    
    // Nachricht vorbereiten
    String status = "Status-Update: " + String(value);
    status.toCharArray(msg, 50);

    // Veröffentlichen
    client.publish(topic_publish, msg);
    
      
  }
}

