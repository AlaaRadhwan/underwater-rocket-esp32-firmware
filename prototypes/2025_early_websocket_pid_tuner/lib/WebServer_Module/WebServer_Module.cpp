#include "WebServer_Module.h"

WebServer_Module::WebServer_Module(const char *ssid, const char *password)
    : ssid(ssid), password(password), server(80), ws("/ws") {}

void WebServer_Module::handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                            AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("🌐 Client %u connected\n", client->id());
        client->text("🛠 Connected to ESP32!");
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("🌐 Client %u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA)
    {
        String msg = "";
        for (size_t i = 0; i < len; i++)
        {
            msg += (char)data[i];
        }
        Serial.printf("📥 From Web: %s\n", msg.c_str());

        // Call user-defined callback if set
        if (onMessageCallback)
        {
            onMessageCallback(msg);
        }

        client->text("✅ Received: " + msg);
    }
}

void WebServer_Module::begin()
{
    // If AP already running, don't start it again
    wifi_mode_t mode = WiFi.getMode();
    bool apRunning = (mode == WIFI_MODE_AP) || (mode == WIFI_MODE_APSTA);

    // Serial.println("🚀 Connecting to WiFi...");
    // WiFi.begin(ssid, password);

    if (!apRunning) {
        Serial.println("Starting Access Point....");
        IPAddress local_ip(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet);
        WiFi.softAP(ssid, password);
        delay(100);
    } else {
        Serial.println("AP already running, skipping softAP() in WebServer module.");
    }


    Serial.print("AP Started! ESP32 IP: ");
    Serial.println(WiFi.softAPIP());

    Serial.println("🌐 WebSocket available at ws://192.168.4.1/ws");

    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     Serial.print(".");
    // }

    // Serial.print("\n✅ Connected! ESP32 IP: ");
    // Serial.println(WiFi.localIP());

    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len)
               { this->handleWebSocketEvent(server, client, type, arg, data, len); });

    server.addHandler(&ws);
    server.begin();
    Serial.println("🌐 WebSocket server started");
}

void WebServer_Module::sendToAll(const String &message)
{
    ws.textAll(message);
}

void WebServer_Module::onMessage(WebSocketCallback callback)
{
    onMessageCallback = callback;
}
