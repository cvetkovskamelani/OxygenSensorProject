#include <SoftwareSerial.h>

SoftwareSerial sensorSerial(9, 10);
HardwareSerial MySerial(1);


void setup() {
  MySerial.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17
  sensorSerial.begin(115200);

  // Change to Q&A mode
  byte changeModeCommand[9] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
  sensorSerial.write(changeModeCommand, 9);
  delay(1000); 
}
void loop() {
  // Send command to read concentration value
  byte readValueCommand[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  sensorSerial.write(readValueCommand, 9);
  delay(1000);

  if (sensorSerial.available() >= 9) {
    byte response[9];
    sensorSerial.readBytes(response, 9);

    // Verify checksum
    byte sum = 0;
    for (int i = 1; i < 8; i++) {
      sum += response[i];
    }
    byte calculatedChecksum = ~sum + 1;


    if (calculatedChecksum == response[8]) {
      // Calculate O2 concentration
      int highByte = response[2];
      int lowByte = response[3];
      float concentration = ((highByte << 8) + lowByte) * 0.1;
      byte dataPacket[4];
      Serial.println("Packet created.");
      memcpy(dataPacket, &concentration, sizeof(concentration));
      Serial.write(dataPacket, 4);
      Serial.println("Packet sent.");
    } 
    else {
      Serial.println("Checksum mismatch.");
    }
  } else {
    Serial.println("No response from sensor.");
  }

  delay(1000); 
}