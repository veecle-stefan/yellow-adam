#include <Arduino.h>
#include "captiveportal.h"

static constexpr uint16_t DNS_PORT = 53;
static constexpr uint16_t HTTP_PORT = 80;

CaptivePortalWeb::CaptivePortalWeb()
: _server(HTTP_PORT),
  _ws("/ws") {
}

bool CaptivePortalWeb::begin(const IPAddress& apIp, const char* hostname)
{
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

  Serial.printf("[Web] Captive portal up: http://%s/\n", _apIp.toString().c_str());
  return true;
}

void CaptivePortalWeb::loop()
{
  if (!_started) return;
  // _dns.processNextRequest();  // keep captive portal DNS responsive
}

void CaptivePortalWeb::broadcastText(const char* msg)
{
  if (!_started) return;
  _ws.textAll(msg);
}

void CaptivePortalWeb::onWsMessage(WsMessageHandler cb)
{
  _wsCb = std::move(cb);
}

void CaptivePortalWeb::_setupWebSocket()
{
  _ws.onEvent([this](AsyncWebSocket* server,
                     AsyncWebSocketClient* client,
                     AwsEventType type,
                     void* arg,
                     uint8_t* data,
                     size_t len) {
    (void)server; (void)arg;

    switch (type) {
      case WS_EVT_CONNECT:
        Serial.printf("[WS] Client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        // Optional: send initial state
        client->text("{\"type\":\"hello\",\"msg\":\"connected\"}");
        break;

      case WS_EVT_DISCONNECT:
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
        break;

      case WS_EVT_DATA: {
        auto* info = (AwsFrameInfo*)arg;
        if (!info || info->final == false || info->index != 0) return;
        if (info->opcode != WS_TEXT) return;

        String msg;
        msg.reserve(len + 1);
        for (size_t i = 0; i < len; i++) msg += (char)data[i];

        Serial.printf("[WS] <- %s\n", msg.c_str());
        if (_wsCb) _wsCb(msg);
        break;
      }

      default:
        break;
    }
  });

  _server.addHandler(&_ws);
}

bool CaptivePortalWeb::_isCaptivePortalRequest(AsyncWebServerRequest* request)
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

void CaptivePortalWeb::_setupRoutes()
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