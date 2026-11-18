#ifndef WEBSERVER_MODULE_H
#define WEBSERVER_MODULE_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

typedef std::function<void(const String &)> WebSocketCallback;

class WebServer_Module
{
private:
    const char *ssid;
    const char *password;
    AsyncWebServer server;
    AsyncWebSocket ws;
    WebSocketCallback onMessageCallback;

    void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                              AwsEventType type, void *arg, uint8_t *data, size_t len);

public:
    WebServer_Module(const char *ssid, const char *password);
    void begin();
    void sendToAll(const String &message);
    void onMessage(WebSocketCallback callback);
};

#endif
