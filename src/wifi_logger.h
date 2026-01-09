/**
 * @file wifi_logger.h
 * @brief WiFi data logging for ESP32 builds
 *
 * Provides remote monitoring via WiFi:
 * - Web dashboard for real-time sensor data
 * - JSON API endpoint for data retrieval
 * - Optional MQTT publishing for home automation integration
 */

#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

#ifdef ESP32
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#endif

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================

struct WiFiConfig {
    char ssid[32];
    char password[64];
    char hostname[32];
    uint16_t webPort;
    bool enableMqtt;
    char mqttServer[64];
    uint16_t mqttPort;
    char mqttTopic[64];
    uint32_t publishIntervalMs;
};

// ============================================================================
// WIFI LOGGER CLASS
// ============================================================================

class WiFiLogger {
public:
    WiFiLogger();

    // Configuration
    void setConfig(const WiFiConfig& config);
    WiFiConfig getConfig() const { return config_; }

    // Connection management
    bool begin();
    void end();
    bool isConnected() const;
    const char* getIpAddress() const;

    // Main loop - call frequently
    void update();

    // Set sensor data source
    void setSensorManager(SensorManager* sensors) { sensors_ = sensors; }

    // Get connection status
    enum class ConnectionState : uint8_t {
        DISCONNECTED = 0,
        CONNECTING,
        CONNECTED,
        AP_MODE,
        ERROR
    };
    ConnectionState getState() const { return state_; }

    // Get stats
    uint32_t getRequestCount() const { return requestCount_; }
    uint32_t getUptime() const;

#ifdef ESP32
    // Web server handlers (public for callback registration)
    void handleRoot();
    void handleApi();
    void handleApiSensors();
    void handleApiConfig();
    void handleNotFound();
#endif

private:
    WiFiConfig config_;
    SensorManager* sensors_;
    ConnectionState state_;
    uint32_t startTime_;
    uint32_t lastPublishTime_;
    uint32_t requestCount_;
    uint32_t reconnectAttempts_;
    static const uint32_t MAX_RECONNECT_ATTEMPTS = 10;
    static const uint32_t RECONNECT_DELAY_MS = 5000;

#ifdef ESP32
    WebServer* server_;
    bool setupWebServer();
    void handleClient();
    String generateDashboardHtml();
    String generateSensorJson();
    void publishMqtt();
#endif

    char ipAddressStr_[16];
};

// ============================================================================
// HTML DASHBOARD TEMPLATE
// ============================================================================

#ifdef ESP32
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Hydroponics Monitor</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #eee;
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 30px;
            font-size: 2em;
            text-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
        }
        .card {
            background: rgba(255,255,255,0.1);
            border-radius: 16px;
            padding: 24px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.1);
            transition: transform 0.2s;
        }
        .card:hover { transform: translateY(-5px); }
        .card-title {
            font-size: 0.9em;
            color: #aaa;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 8px;
        }
        .card-value {
            font-size: 3em;
            font-weight: 700;
            line-height: 1;
        }
        .card-unit {
            font-size: 1.2em;
            color: #888;
            margin-left: 5px;
        }
        .status {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 0.8em;
            margin-top: 10px;
        }
        .status-ok { background: #00c853; color: #000; }
        .status-warn { background: #ffc107; color: #000; }
        .status-error { background: #f44336; color: #fff; }
        .ph { color: #7c4dff; }
        .temp { color: #ff5722; }
        .tds { color: #00bcd4; }
        .level { color: #4caf50; }
        .footer {
            text-align: center;
            margin-top: 40px;
            color: #666;
            font-size: 0.9em;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        .updating { animation: pulse 1s infinite; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🌱 Hydroponics Monitor</h1>
        <div class="grid">
            <div class="card">
                <div class="card-title">pH Level</div>
                <div class="card-value ph"><span id="ph">--</span></div>
                <span class="status status-ok" id="ph-status">OK</span>
            </div>
            <div class="card">
                <div class="card-title">Temperature</div>
                <div class="card-value temp"><span id="temp">--</span><span class="card-unit">°C</span></div>
                <span class="status status-ok" id="temp-status">OK</span>
            </div>
            <div class="card">
                <div class="card-title">TDS / Nutrients</div>
                <div class="card-value tds"><span id="tds">--</span><span class="card-unit">ppm</span></div>
                <span class="status status-ok" id="tds-status">OK</span>
            </div>
            <div class="card">
                <div class="card-title">Water Level</div>
                <div class="card-value level"><span id="level">--</span><span class="card-unit">%</span></div>
                <span class="status status-ok" id="level-status">OK</span>
            </div>
        </div>
        <div class="footer">
            <p>Uptime: <span id="uptime">--</span> | Last update: <span id="lastUpdate">--</span></p>
            <p>Hydroponics Controller v1.0</p>
        </div>
    </div>
    <script>
        function updateData() {
            fetch('/api/sensors')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('ph').textContent = data.ph.toFixed(2);
                    document.getElementById('temp').textContent = data.temperature.toFixed(1);
                    document.getElementById('tds').textContent = Math.round(data.tds);
                    document.getElementById('level').textContent = Math.round(data.level);
                    document.getElementById('uptime').textContent = formatUptime(data.uptime);
                    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();

                    updateStatus('ph', data.ph, 5.5, 7.0);
                    updateStatus('temp', data.temperature, 18, 28);
                    updateStatus('tds', data.tds, 400, 1200);
                    updateStatus('level', data.level, 20, 90);
                })
                .catch(e => console.error('Update failed:', e));
        }

        function updateStatus(id, value, min, max) {
            const el = document.getElementById(id + '-status');
            if (value < min || value > max) {
                el.className = 'status status-error';
                el.textContent = value < min ? 'LOW' : 'HIGH';
            } else if (value < min * 1.1 || value > max * 0.9) {
                el.className = 'status status-warn';
                el.textContent = 'WARN';
            } else {
                el.className = 'status status-ok';
                el.textContent = 'OK';
            }
        }

        function formatUptime(seconds) {
            const h = Math.floor(seconds / 3600);
            const m = Math.floor((seconds % 3600) / 60);
            return h + 'h ' + m + 'm';
        }

        updateData();
        setInterval(updateData, 2000);
    </script>
</body>
</html>
)rawliteral";
#endif

#endif // WIFI_LOGGER_H
