/*
  Copyright (c) 2015 SODAQ. All rights reserved.

  This file is part of Sodaq_RN2483.

  Sodaq_RN2483 is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or(at your option) any later version.

  Sodaq_RN2483 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with Sodaq_RN2483.  If not, see
  <http://www.gnu.org/licenses/>.
*/

#include <Sodaq_RN2483.h>
#include "DHT.h"

// MBili / Tatu
//#define debugSerial Serial
// Autonomo
#define debugSerial SerialUSB
#define loraSerial Serial1

#define DHTPIN 10 // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11 // DHT 11
//#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE DHT21 // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLTPIN BAT_VOLT
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

// USE YOUR OWN KEYS!
const uint8_t devAddr[4] =
{
  0x00, 0x00, 0x19, 0x21  
};

// USE YOUR OWN KEYS!
const uint8_t appSKey[16] =
{
  0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x90
};

// USE YOUR OWN KEYS!
const uint8_t nwkSKey[16] =
{
  0x0A, 0x1B, 0x2C, 0x3D, 0x4E, 0x5F, 0x0A, 0x1B, 0x2C, 0x3D, 0x4E, 0x5F, 0x0A, 0x1B, 0x2C, 0x90
};

void setup()
{
  while ((!debugSerial) && (millis() < 10000));

  pinMode(13, OUTPUT);

  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  digitalWrite(BEE_VCC, HIGH);
  setupNetwork();

  dht.begin();
}

void loop()
{
  //debugSerial.println();
  digitalWrite(BEE_VCC, HIGH);

  debugSerial.println("Sending payload: Humidity, Temperature, Voltage");
  String reading = takeTHReading();
  reading += ", " + String(getRealBatteryVoltageMV());
  debugSerial.println(reading);

  switch (LoRaBee.sendReqAck(1, (uint8_t*)reading.c_str(), reading.length(),8 ))
  {
    case NoError:
      debugSerial.println("Successful transmission.");
      receiveData();
      break;
    case NoResponse:
      debugSerial.println("There was no response from the device.");
      break;
    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe program will now halt.");
      setupNetwork();
      break;
    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      setupNetwork();
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      setupNetwork();
      break;
    case NotConnected:
      debugSerial.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      setupNetwork();
      break;
    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      break;
    default:
      break;
  }

  delay(5000);
}

void setupNetwork() {
  LoRaBee.setDiag(debugSerial); // optional
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
  {
    debugSerial.println("Connection to the network was successful.");
  }
  else
  {
    debugSerial.println("Connection to the network failed!");
  }
}

void receiveData() {
  // After we have send some data, we can receive some data
  // First we make a buffer
  uint8_t payload[64];
  // Now we fill the buffer and
  // len = the size of the data
  uint16_t len = LoRaBee.receive(payload, 64);
  String HEXPayload = "";

  // When there is no payload the lorabee will return 131 (0x83)
  // I filter this out
  if (payload[0] != 131) {
    for (int i = 0; i < len; i++) {
      HEXPayload += String(payload[i], HEX);
    }
    debugSerial.println(HEXPayload);
    switchLED(HEXPayload);
  } else {
    debugSerial.println("no payload");
  }
}

void switchLED(String str) {
  // To switch on or off the LED send: 0x00 or 0x01
  //http://lrc1.thingpark.com:8807/sensor/?DevEUI=F03D291000000000&FPort=1&Payload=00 //off
  //http://lrc1.thingpark.com:8807/sensor/?DevEUI=F03D291000000000&FPort=1&Payload=01 //on
  if (str == "0") {
    digitalWrite(13, LOW);
    debugSerial.println("Led Off");
  } else if (str == "1") {
    digitalWrite(13, HIGH);
    debugSerial.println("Led On");
  }
}

String takeTHReading()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to A0 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  String data = String(h)  + ", ";
  data += String(t);

  return data;
}

float getRealBatteryVoltageMV()
{
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);
  return (ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage;
}
