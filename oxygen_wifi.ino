#include <SPI.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiUdp.h>  
#include <HardwareSerial.h>

/* ---------- Sensor serial (UART1) ---------- */
HardwareSerial sensorSerial(1); // Use UART1 for oxygen sensor

/* ---------- Network settings ---------- */
const char* ssid = "Melani";
const char* password = "icdanetiegajle";

// Pinggy server – use the IPv4 you got from nslookup
IPAddress targetIp(172, 105, 83, 27);   // ← replace with your IP
const unsigned int targetPort = 23234;   // ← replace with your Pinggy UDP port
/* ---------- UDP ---------- */
WiFiUDP Udp;
const unsigned int localPort = 8888;

/* ---------- Sequence counter ---------- */
uint16_t seq_cnt = 0;

/* ------- Build 6-byte header (same rule as Python) ------- */
byte* buildHeader(uint16_t seq)
{
  static byte hdr[6];
  if (seq >= 16382) seq = 0;
  uint16_t value = 0xC000 + seq;
  hdr[0] = 0x00;
  hdr[1] = 0x67;
  hdr[2] = highByte(value);
  hdr[3] = lowByte(value);
  hdr[4] = 0x01;
  hdr[5] = 0x45; // this is the size of the packet where we sent
  Serial.print(F("ENV SEQ : "));
  Serial.println(seq);
  return hdr;
}

/* ---- Helper: convert float → 4 big-endian bytes ---- */
void floatToBytesBE(float f, byte *out)
{
  union { float f; uint32_t u32; } conv;
  conv.f = f;
  out[0] = (conv.u32 >> 24) & 0xFF;
  out[1] = (conv.u32 >> 16) & 0xFF;
  out[2] = (conv.u32 >>  8) & 0xFF;
  out[3] =  conv.u32        & 0xFF;
}

void setup()
{
  Serial.begin(115200);
  sensorSerial.begin(9600, SERIAL_8N1, 26, 25); // RX=26, TX=25
 

  // Change to Q&A mode (sensor-specific command)
  byte changeModeCommand[9] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
  sensorSerial.write(changeModeCommand, 9);
  delay(100);

  // Wi-Fi connect
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Restarting...");
    ESP.restart();
  }

  Udp.begin(localPort);
  Serial.println("UDP ready – sending every 5 s…");
}

void loop()
{
  float concentration = 0.0;             // oxygen concentration [%VOL]

  // Send command to read O2 concentration
  byte readValueCommand[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  sensorSerial.write(readValueCommand, 9);
  delay(100); // Give time for sensor to respond

  if (sensorSerial.available() >= 9) {
    byte response[9];
    sensorSerial.readBytes(response, 9);
    byte sum = 0;
    for (int i = 1; i < 8; i++) {
      sum += response[i];
    }
    byte calculatedChecksum = ~sum + 1;

    if (calculatedChecksum == response[8]) {
      int highByte = response[2];
      int lowByte = response[3];
      concentration = ((highByte << 8) + lowByte) * 0.1;
      Serial.println("concentration");
    } else {
      Serial.println("Checksum mismatch.");
    }
  } else {
    Serial.println("No response from sensor.");
  }

  /* ----- Build the 10-byte packet ----- */
  byte packet[10]; // 6 header + 4 oxygen
  memcpy(packet, buildHeader(seq_cnt), 6);
  floatToBytesBE(concentration, packet + 6);       // bytes 6–9  = oxygen

  /* ----- Send via UDP ----- */
  
  Serial.print(F(" %VOL → Pinggy … "));
  Serial.print(concentration, 1);

  if (Udp.beginPacket(targetIp, targetPort)) {
    Udp.write(packet, sizeof(packet));
    Udp.endPacket();
    Serial.println(F("✅ Sent"));
  } else {
    Serial.println(F("❌ Send failed"));
  }

  seq_cnt++;
  delay(1000); 
}
