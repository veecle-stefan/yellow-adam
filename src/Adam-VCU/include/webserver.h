#pragma once
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow" // wifi.h specific fix
#pragma GCC diagnostic ignored "-Wpedantic" // wifi.h specific fix
#pragma GCC diagnostic ignored "-Wconversion" // ESPAsyncWebServer.h specific fix
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#pragma GCC diagnostic pop
#include <DNSServer.h>
#include <LittleFS.h>
#include <unordered_map>
#include <AsyncTCP.h>
#include "drivetrain.h"
#include "JSONrw.h"

class WebServer {
public:
  static constexpr uint32_t WSIdleTimeout = 10000; // after 10s
  static constexpr uint32_t TidyEveryMs = 1000;   // tune: 250..1000ms
  static constexpr size_t WSTextBufferSize = 400;

  using WsMessageHandler = std::function<void(const String& msg)>;

  WebServer();

  // Start LittleFS + DNS captive portal + HTTP + WebSocket
  // apIp should usually be WiFi.softAPIP() (often 192.168.4.1)
  bool begin(const IPAddress& apIp, const char* hostname, QueueHandle_t statusQueue, DriveTrain* drive, uint32_t updateInterval);

  // Must be called frequently in loop() for DNS captive portal handling
  void tidy();

  void pump();

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
  QueueHandle_t _statusQueue;
  uint32_t _updateInterval = 0;
  TaskHandle_t _bgTask = NULL;
  // status TX double buffer (produced by bg task, consumed by pump())
  char*  _txBuf[2] = { nullptr, nullptr };
  volatile uint8_t _txFront = 0;
  volatile bool _txDirty = false;

  SemaphoreHandle_t _txMutex = nullptr;
  SemaphoreHandle_t _wsMutex = nullptr; // protects _wsLastSeen

  // pump/tidy timers
  uint32_t _lastTidyMs = 0;


  void _setupRoutes();
  void _setupWebSocket();
  static bool _isCaptivePortalRequest(AsyncWebServerRequest* request);
  void _backgroundUpdates();
};