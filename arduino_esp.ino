#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Melani";
const char* password = "icdanetiegajle";

WebServer server(80);  

float oxygenData = 0.0;  

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17 (we only use RX)
  Serial.println("Setup Started");
  WiFi.begin(ssid, password);
  
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.begin();
}

void loop() {
  if (Serial1.available() >= 1) {
    Serial.println("Something received via communication with Arduino.");
  }
  if (Serial1.available() >= 4) {  
    Serial.println("Packet received");
    byte dataPacket[4];
    Serial1.readBytes(dataPacket, 4); 
  
    memcpy(&oxygenData, dataPacket, sizeof(oxygenData));
    Serial.print("Received Oxygen Data: ");
    Serial.println(oxygenData);
  }
  server.handleClient();
}


void handleRoot() {
  String html = "<html><body><h1>Oxygen Concentration</h1>";
  html += "<p>Oxygen Concentration: " + String(oxygenData) + "%</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);  
}