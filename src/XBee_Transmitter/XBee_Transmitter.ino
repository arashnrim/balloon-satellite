 /**
   Copyright (c) 2009 Andrew Rapp. All rights reserved.
   This file is part of XBee-Arduino.
   XBee-Arduino is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   XBee-Arduino is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <XBee.h>
#include <Wire.h> // Used to establied serial communication on the I2C bus
#include <SparkFunTMP102.h> // Used to send and recieve specific information from our sensor
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;

const int ALERT_PIN = A3;
TMP102 sensor0;

/*
  This example is for Series 2 XBee
  Sends a ZB TX request with the value of analogRead(pin5) and checks the status response for success
*/

// create the XBee object
XBee xbee = XBee();

uint8_t payload[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x13A200, 0x4194502C);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
//ZBTxRequest zbTx = ZBTxRequest(0x1, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int pin5 = 0;
int humiditySensor = A0;

int statusLed = 13;
int errorLed = 13;

void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

union floatByte_t {
    float f;
    byte b[4];
};

void setup() {
  Wire.begin(); //Join I2C Bus
  pinMode(ALERT_PIN,INPUT); 
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  if(!sensor0.begin())
  {
    Serial.println("Cannot connect to TMP102.");
    Serial.println("Is the board connected? Is the device ID correct?");
    while(1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);

}

void loop() {
  // pack "ilovetocode" into the payload
  sensor0.wakeup();
  float temperature;
  int pressure;
  boolean alertPinState, alertRegisterState; 
  int i;
  int temp;
  int HIH4030_Value = analogRead(humiditySensor);
  payload[0] = HIH4030_Value / 100 % 10; 
  payload[1] = HIH4030_Value / 10 % 10;
  payload[2] = HIH4030_Value % 10;
  Serial.println(HIH4030_Value);
  
  temperature = sensor0.readTempC();
  Serial.println(temperature);
  temp = temperature * 100;
  payload[3] = temp / 1000 % 10; 
  payload[4] = temp / 100 % 10;
  payload[5] = temp / 10 % 10; 
  payload[6] = temp % 10; 
  
  pressure = ceil(ceil(bmp.readPressure())/1000);
  Serial.println(pressure);
  payload[7] = pressure / 100 % 10; 
  payload[8] = pressure / 10 % 10;
  payload[9] = pressure % 10;
  
   
  
  
  
  
 
  

  xbee.send(zbTx);

  // flash TX indicator
  flashLed(statusLed, 1, 100);

  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
    // got a response!

    // should be a znet tx status
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
        flashLed(statusLed, 5, 50);
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(errorLed, 3, 500);
      }
    }
  } else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");
    //nss.println(xbee.getResponse().getErrorCode());
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    flashLed(errorLed, 2, 50);
  }

  delay(1000);
}
