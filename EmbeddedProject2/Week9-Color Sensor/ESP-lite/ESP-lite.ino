#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>

const char* ssid = "Titenet-IoT";
const char* password = "7kDtaphg";

// --- Data Variables ---
String lidarData = "0";
String compassData = "0";
String colorData = "#000000";

ESP8266WebServer server(80);

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin()) {
    Serial.println("Error while mounting SPIFFS");
    return;
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // Static Files
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/style.css", SPIFFS, "/style.css");
  server.serveStatic("/script.js", SPIFFS, "/script.js");
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png");

  // Existing Routes
  server.on("/forwards5", []() { handleMove(5); });
  server.on("/forwards20", []() { handleMove(20); });
  server.on("/backwards5", []() { handleMove(-5); });
  server.on("/backwards20", []() { handleMove(-20); });
  server.on("/compass", handleCompass);
  server.on("/ToNorth", handleNorth);
  server.on("/newspeed", handleSpeedUpdate);
  server.on("/todir", handleToDir);

  // --- NEW: Data Monitoring Routes ---
  server.on("/lidar", []() {
    server.send(200, "text/plain", lidarData);
  });
  server.on("/compassVal", []() {
    server.send(200, "text/plain", compassData);
  });

  server.on("/color", [](){
    server.send(200, "text/plain", colorData);
  });

  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  server.handleClient();

  // --- NEW: Serial Listener for Arduino Mega ---
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Removes glitchy hidden characters

    if (data.startsWith("LIDAR:")) {
      lidarData = data.substring(6);
    } else if (data.startsWith("Compass:")) {
      compassData = data.substring(8);
    } else if (data.startsWith("Color:")){
      colorData = data.substring(6);
    }
  }
}

// --- Handler Functions ---
void handleNotFound() { server.send(404, "text/plain", "404: Not Found"); }

void handleMove(int distance) {
  Serial.println("Move:" + String(distance));
  server.send(200);
}

void handleCompass() {
  if (server.hasArg("value")) {
    Serial.println("Turn:" + server.arg("value"));
  }
  server.send(200);
}

void handleSpeedUpdate() {
  if (server.hasArg("value")) {
    Serial.println("NewSpeed:" + server.arg("value"));
  }
  server.send(200);
}

void handleNorth() {
  Serial.println("ToNorth");
  server.send(200);
}

void handleToDir() {
  if (server.hasArg("value")) {
    Serial.println("Dir:" + server.arg("value"));
  }
  server.send(200);
}
