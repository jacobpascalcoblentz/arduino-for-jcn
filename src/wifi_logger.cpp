/**
 * @file wifi_logger.cpp
 * @brief WiFi data logging implementation for ESP32
 */

#include "wifi_logger.h"
#include <string.h>

#ifndef ARDUINO
#include "mock_arduino.h"
#endif

// ============================================================================
// WIFI LOGGER IMPLEMENTATION
// ============================================================================

WiFiLogger::WiFiLogger()
    : sensors_(nullptr)
    , state_(ConnectionState::DISCONNECTED)
    , startTime_(0)
    , lastPublishTime_(0)
    , requestCount_(0)
    , reconnectAttempts_(0) {

    // Default configuration
    memset(&config_, 0, sizeof(config_));
    strncpy(config_.ssid, "HydroponicsAP", sizeof(config_.ssid) - 1);
    strncpy(config_.hostname, "hydroponics", sizeof(config_.hostname) - 1);
    config_.webPort = 80;
    config_.enableMqtt = false;
    config_.publishIntervalMs = 5000;

    memset(ipAddressStr_, 0, sizeof(ipAddressStr_));
    strncpy(ipAddressStr_, "0.0.0.0", sizeof(ipAddressStr_) - 1);

#ifdef ESP32
    server_ = nullptr;
#endif
}

void WiFiLogger::setConfig(const WiFiConfig& config) {
    config_ = config;
}

bool WiFiLogger::begin() {
#ifdef ESP32
    state_ = ConnectionState::CONNECTING;
    startTime_ = millis();

    // Set hostname
    WiFi.setHostname(config_.hostname);

    // Try to connect to WiFi
    if (strlen(config_.ssid) > 0 && strlen(config_.password) > 0) {
        Serial.print("Connecting to WiFi: ");
        Serial.println(config_.ssid);

        WiFi.mode(WIFI_STA);
        WiFi.begin(config_.ssid, config_.password);

        // Wait for connection with timeout
        uint32_t startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            state_ = ConnectionState::CONNECTED;
            IPAddress ip = WiFi.localIP();
            snprintf(ipAddressStr_, sizeof(ipAddressStr_), "%d.%d.%d.%d",
                     ip[0], ip[1], ip[2], ip[3]);

            Serial.println();
            Serial.print("Connected! IP: ");
            Serial.println(ipAddressStr_);

            return setupWebServer();
        }
    }

    // Fall back to AP mode if connection fails
    Serial.println("\nStarting Access Point mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(config_.hostname, "hydroponics123");

    IPAddress ip = WiFi.softAPIP();
    snprintf(ipAddressStr_, sizeof(ipAddressStr_), "%d.%d.%d.%d",
             ip[0], ip[1], ip[2], ip[3]);

    state_ = ConnectionState::AP_MODE;
    Serial.print("AP Mode - Connect to '");
    Serial.print(config_.hostname);
    Serial.print("' and browse to http://");
    Serial.println(ipAddressStr_);

    return setupWebServer();
#else
    return false;
#endif
}

void WiFiLogger::end() {
#ifdef ESP32
    if (server_) {
        server_->stop();
        delete server_;
        server_ = nullptr;
    }
    WiFi.disconnect();
    state_ = ConnectionState::DISCONNECTED;
#endif
}

bool WiFiLogger::isConnected() const {
#ifdef ESP32
    return WiFi.status() == WL_CONNECTED || state_ == ConnectionState::AP_MODE;
#else
    return false;
#endif
}

const char* WiFiLogger::getIpAddress() const {
    return ipAddressStr_;
}

uint32_t WiFiLogger::getUptime() const {
    return (millis() - startTime_) / 1000;
}

void WiFiLogger::update() {
#ifdef ESP32
    // Handle web server requests
    if (server_) {
        server_->handleClient();
    }

    // Check WiFi connection
    if (state_ == ConnectionState::CONNECTED && WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost, reconnecting...");
        state_ = ConnectionState::CONNECTING;
        reconnectAttempts_++;

        if (reconnectAttempts_ < MAX_RECONNECT_ATTEMPTS) {
            WiFi.reconnect();
        } else {
            // Switch to AP mode after too many failures
            Serial.println("Too many reconnect attempts, switching to AP mode");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(config_.hostname, "hydroponics123");
            state_ = ConnectionState::AP_MODE;
        }
    }

    // MQTT publishing
    if (config_.enableMqtt && isConnected()) {
        uint32_t now = millis();
        if (now - lastPublishTime_ >= config_.publishIntervalMs) {
            publishMqtt();
            lastPublishTime_ = now;
        }
    }
#endif
}

#ifdef ESP32
bool WiFiLogger::setupWebServer() {
    if (server_) {
        delete server_;
    }

    server_ = new WebServer(config_.webPort);

    // Route handlers - using lambdas to call member functions
    server_->on("/", HTTP_GET, [this]() { handleRoot(); });
    server_->on("/api/sensors", HTTP_GET, [this]() { handleApiSensors(); });
    server_->on("/api/config", HTTP_GET, [this]() { handleApiConfig(); });
    server_->onNotFound([this]() { handleNotFound(); });

    server_->begin();
    Serial.print("Web server started on port ");
    Serial.println(config_.webPort);

    return true;
}

void WiFiLogger::handleRoot() {
    requestCount_++;
    server_->send(200, "text/html", DASHBOARD_HTML);
}

void WiFiLogger::handleApiSensors() {
    requestCount_++;
    server_->send(200, "application/json", generateSensorJson());
}

void WiFiLogger::handleApiConfig() {
    requestCount_++;

    StaticJsonDocument<512> doc;
    doc["hostname"] = config_.hostname;
    doc["ssid"] = config_.ssid;
    doc["webPort"] = config_.webPort;
    doc["mqttEnabled"] = config_.enableMqtt;
    doc["mqttServer"] = config_.mqttServer;
    doc["mqttTopic"] = config_.mqttTopic;
    doc["uptime"] = getUptime();
    doc["requests"] = requestCount_;
    doc["freeHeap"] = ESP.getFreeHeap();

    String response;
    serializeJson(doc, response);
    server_->send(200, "application/json", response);
}

void WiFiLogger::handleNotFound() {
    server_->send(404, "text/plain", "Not Found");
}

String WiFiLogger::generateSensorJson() {
    StaticJsonDocument<256> doc;

    if (sensors_) {
        SensorReadings readings = sensors_->getReadings();
        doc["ph"] = readings.ph;
        doc["temperature"] = readings.temperatureC;
        doc["tds"] = readings.tdsPpm;
        doc["level"] = readings.waterLevelPercent;
        doc["timestamp"] = readings.timestamp;
        doc["valid"] = readings.allValid;
    } else {
        doc["ph"] = 0;
        doc["temperature"] = 0;
        doc["tds"] = 0;
        doc["level"] = 0;
        doc["timestamp"] = 0;
        doc["valid"] = false;
    }

    doc["uptime"] = getUptime();

    String response;
    serializeJson(doc, response);
    return response;
}

void WiFiLogger::publishMqtt() {
    // MQTT publishing would go here
    // For now, just a placeholder - would need PubSubClient library
    // This is left as a TODO for integration with home automation systems
}
#endif
