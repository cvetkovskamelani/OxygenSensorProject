#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2 // Replace with your actual data pin for DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
SoftwareSerial sensorSerial(9, 10); // RX, TX

void setup() {
  Serial.begin(9600);
  sensorSerial.begin(9600);
  tempSensor.begin();
  // Change to Q&A mode
  byte changeModeCommand[9] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
  sensorSerial.write(changeModeCommand, 9);
  delay(1000); // Wait for the sensor to process the command
}

void loop() {
  // Send command to read concentration value
  byte readValueCommand[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  sensorSerial.write(readValueCommand, 9);

  // Wait for response
  delay(1000);

  // Read response
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
      tempSensor.requestTemperatures();
      float temperatureC = tempSensor.getTempCByIndex(0);  // Read temperature

      // Send time (millis) and concentration in CSV format to serial
      Serial.print(millis()); // Time in ms
      Serial.print(",");
      Serial.print(concentration); // Concentration in %VOL
      Serial.print(",");
      Serial.println(temperatureC); 
    } else {
      Serial.println("Checksum mismatch.");
    }
  } else {
    Serial.println("No response from sensor.");
  }

  delay(1000); // Wait before next reading
}