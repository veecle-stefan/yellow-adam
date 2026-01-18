#include <Arduino.h>
#include "swconfig.h"
#include "webserver.h"

static constexpr uint16_t DNS_PORT = 53;
static constexpr uint16_t HTTP_PORT = 80;

WebServer::WebServer()
: _server(HTTP_PORT),
  _ws("/ws"),
  _json(400) { // buffer size 400 bytes
}

bool WebServer::begin(const IPAddress& apIp, const char* hostname, QueueHandle_t statusQueue, DriveTrain* drive, uint32_t updateInterval)
{
  if (_started) return false;

  _apIp = apIp;

  if (!LittleFS.begin(true)) {   // true = format on fail (good for first boot)
    Serial.println("[Web] LittleFS.begin() failed");
    return false;
  }

  // Captive portal: redirect all DNS queries to our AP IP
  /*
  _dns.setErrorReplyCode(DNSReplyCode::NoError);
  if (!_dns.start(DNS_PORT, "*", _apIp)) {
    Serial.println("[Web] DNS server start failed");
    return false;
  }
  */

  if (hostname && hostname[0]) {
    WiFi.setHostname(hostname);
  }

  _setupWebSocket();
  _setupRoutes();

  _server.begin();
  _started = true;

  this->_statusQueue = statusQueue;
  this->_updateInterval = updateInterval;
  xTaskCreatePinnedToCore(
    [](void* pvParameters) {
        static_cast<WebServer*>(pvParameters)->_backgroundUpdates();
        vTaskDelete(NULL);
    },
    "WS",
    SWConfig::Tasks::MinStakSize,
    this,
    SWConfig::Tasks::PrioMed,
    &this->_bgTask,
    SWConfig::CoreAffinity::CoreComms
);

  onWsMessage([&](const String& msg){
      _json.DispatchCommand(msg, drive);
    });

  return true;
}

void WebServer::_backgroundUpdates()
{
  TickType_t       lastWakeTime = xTaskGetTickCount();
  uint32_t timeAbs = 0;
  DriveTrain::DriveTrainStatus st;

  for(;;) {

    if (xQueuePeek(this->_statusQueue, &st, 0) == pdTRUE)
    {
      // there was (new) data
      size_t n = _json.EncodeStatusJson(st);
      if (n > 0) {
        broadcastText(_json.buffer);
      }
    }

    // clean up every second
    if (timeAbs % 1000 == 0) {
      tidy();
    }

    vTaskDelayUntil(&lastWakeTime, this->_updateInterval);
    timeAbs += this->_updateInterval;
  }
}

void WebServer::tidy() {
  if (!_started) return;

  _ws.cleanupClients();

  const uint32_t now = millis();

  // Close stale connected clients
  for (AsyncWebSocketClient& c : _ws.getClients()) {
    if (c.status() != WS_CONNECTED) continue;

    auto it = _wsLastSeen.find(c.id());
    const bool stale = (it == _wsLastSeen.end()) || ((now - it->second) > WSIdleTimeout);
    if (stale) {
      Serial.printf("[WS] #%u stale -> closing\n", c.id());
      c.close();
      _wsLastSeen.erase(c.id());
    }
  }

  // Optional: prune entries for clients that no longer exist (belt+braces)
  for (auto it = _wsLastSeen.begin(); it != _wsLastSeen.end(); ) {
    AsyncWebSocketClient* cptr = _ws.client(it->first); // if your build supports this
    if (!cptr || cptr->status() != WS_CONNECTED) it = _wsLastSeen.erase(it);
    else ++it;
  }
}

void WebServer::broadcastText(const char* msg)
{
  if (!_started) return;

  // Avoid queue explosion: only send to clients that can accept data.
  // (queueIsFull() exists on AsyncWebSocketClient in common builds.)
  for (AsyncWebSocketClient& client : _ws.getClients()) {
    if (client.status() != WS_CONNECTED) continue;

    // If client is slow, skip this tick rather than buffering infinitely.
    if (client.queueIsFull()) continue;

    client.text(msg);
  }
}

void WebServer::onWsMessage(WsMessageHandler cb)
{
  _wsCb = std::move(cb);
}

void WebServer::_setupWebSocket()
{
  _ws.onEvent([this](AsyncWebSocket* server,
                     AsyncWebSocketClient* client,
                     AwsEventType type,
                     void* arg,
                     uint8_t* data,
                     size_t len) {
    (void)server; (void)arg;

    const uint32_t now = millis();

    switch (type) {
      case WS_EVT_CONNECT: 
        // Newest wins: disconnect everyone else so you can always regain control.
        for (AsyncWebSocketClient& c : _ws.getClients()) {
          if (c.id() != client->id()) {
            c.close();
          }
        }

        // memorise last interaction
        _wsLastSeen[client->id()] = now;

        client->text("{\"type\":\"hello\",\"msg\":\"connected\"}");
        break;

      case WS_EVT_DISCONNECT:
        _wsLastSeen.erase(client->id());

        //Serial.printf("[WS] Client #%u disconnected\n", client->id());
        break;

      case WS_EVT_DATA: {
        // Any inbound traffic = alive. No need to detect "hb".
        _wsLastSeen[client->id()] = now;

        // Only dispatch complete TEXT frames to your app callback
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (!info || !info->final || info->index != 0) return;
        if (info->opcode != WS_TEXT) return;

        String msg;
        msg.reserve(len + 1);
        msg.concat((const char*)data, len);

        if (_wsCb) _wsCb(msg);
        break;
      }

      default:
        break;
    }
  });

  _ws.enable(true);
  _server.addHandler(&_ws);

}

bool WebServer::_isCaptivePortalRequest(AsyncWebServerRequest* request)
{
  // Captive portals often hit URLs like:
  // /generate_204, /hotspot-detect.html, /ncsi.txt, etc.
  // We'll just redirect any unknown path to "/".
  if (!request) return false;

  String url = request->url();
  if (url == "/") return false;

  // If the requested file exists in FS, do not treat it as captive
  if (LittleFS.exists(url)) return false;

  return true;
}

void WebServer::_setupRoutes()
{
  // Serve static files
  _server.serveStatic("/", LittleFS, "/")
         .setDefaultFile("index.html")
         .setCacheControl("no-cache");

  // Health check
  _server.on("/health", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "ok");
  });

  // Captive portal behavior:
  // Always serve the portal for unknown URLs
  _server.onNotFound([](AsyncWebServerRequest* req) {
    if (LittleFS.exists("/index.html")) {
      req->send(LittleFS, "/index.html", "text/html");
    } else {
      req->send(404, "text/plain", "Portal files missing");
    }
  });
}