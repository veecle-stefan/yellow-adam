#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <unordered_map>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "drivetrain.h"
#include "JSONrw.h"

class WebServer {
public:
  static constexpr uint32_t WSIdleTimeout = 10000; // after 10s

  using WsMessageHandler = std::function<void(const String& msg)>;

  WebServer();

  // Start LittleFS + DNS captive portal + HTTP + WebSocket
  // apIp should usually be WiFi.softAPIP() (often 192.168.4.1)
  bool begin(const IPAddress& apIp, const char* hostname, QueueHandle_t statusQueue, DriveTrain* drive, uint32_t updateInterval);

  // Must be called frequently in loop() for DNS captive portal handling
  void tidy();

  // Push status/events to all connected websocket clients
  void broadcastText(const char* msg);

  // User callback: called when websocket text message arrives
  void onWsMessage(WsMessageHandler cb);

protected:
  bool _started = false;
  IPAddress _apIp;

  DNSServer _dns;
  AsyncWebServer _server;
  AsyncWebSocket _ws;

  WsMessageHandler _wsCb;
  std::unordered_map<uint32_t, uint32_t> _wsLastSeen;
  JSONInteraction _json;
  QueueHandle_t _statusQueue;
  uint32_t _updateInterval = 0;
  TaskHandle_t _bgTask = NULL;


  void _setupRoutes();
  void _setupWebSocket();
  static bool _isCaptivePortalRequest(AsyncWebServerRequest* request);
  void _backgroundUpdates();
};