#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <LittleFS.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class WebServer {
public:
  using WsMessageHandler = std::function<void(const String& msg)>;

  WebServer();

  // Start LittleFS + DNS captive portal + HTTP + WebSocket
  // apIp should usually be WiFi.softAPIP() (often 192.168.4.1)
  bool begin(const IPAddress& apIp, const char* hostname = "esp32");

  // Must be called frequently in loop() for DNS captive portal handling
  void loop();

  // Push status/events to all connected websocket clients
  void broadcastText(const char* msg);

  // User callback: called when websocket text message arrives
  void onWsMessage(WsMessageHandler cb);

private:
  bool _started = false;
  IPAddress _apIp;

  DNSServer _dns;
  AsyncWebServer _server;
  AsyncWebSocket _ws;

  WsMessageHandler _wsCb;

  void _setupRoutes();
  void _setupWebSocket();
  static bool _isCaptivePortalRequest(AsyncWebServerRequest* request);
};