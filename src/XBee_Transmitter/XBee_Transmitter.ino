#include <XBee.h>
#include <Wire.h> // Used to establied serial communication on the I2C bus.
#include <SparkFunTMP102.h> // Used to send and recieve specific information from the temperature sensor (TMP-102).

const int ALERT_PIN = A3;
TMP102 sensor0;
int humiditySensor = A0;

XBee xbee = XBee();

// Initialises the array with payload data.
// The first three elements in payload[] are reserved for transmitting humidity information.
// The last four elements in payload[] are reserved for transmitting temperature information.
uint8_t payload[] = { 0, 0, 0, 0, 0, 0, 0 };

// Configures the sending XBee using the SH + SL Address of receiving XBee.
XBeeAddress64 addr64 = XBeeAddress64(0x13A200, 0x4194502C);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int pin5 = 0;

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
  Wire.begin(); // Joins the I2C Bus to read modules.
  pinMode(ALERT_PIN,INPUT); 
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  if(!sensor0.begin())
  {
    Serial.println("Cannot connect to TMP102.");
    Serial.println("Is the board connected? Is the device ID correct?");
    while(1);
  }
}

void loop() {
  sensor0.wakeup();
  boolean alertPinState, alertRegisterState; 

  // Reads the humidity data and stores it into the payload.
  int HIH4030_Value = analogRead(humiditySensor);
  payload[0] = HIH4030_Value / 100 % 10; 
  payload[1] = HIH4030_Value / 10 % 10;
  payload[2] = HIH4030_Value % 10;
  Serial.print("Humidity: ");
  Serial.print(HIH4030_Value);
  Serial.println("%");

  // Reads the temperature data and stores it into the payload.
  float temperature = sensor0.readTempC();
  Serial.println(temperature);
  int temp = temperature * 100;
  payload[3] = temp / 1000 % 10; 
  payload[4] = temp / 100 % 10;
  payload[5] = temp / 10 % 10; 
  payload[6] = temp % 10; 

  // Transmits the payload with the configuration set above.
  xbee.send(zbTx);

  // Flashes the TX indicator.
  flashLed(statusLed, 1, 100);

  // After sending a TX request, we expect a status response.
  if (xbee.readPacket(500)) {

    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      if (txStatus.getDeliveryStatus() == SUCCESS) {
        flashLed(statusLed, 5, 50);
      } else {
        // The receiving XBee did not receive our packet; errorLed is lit.
        flashLed(errorLed, 3, 500);
      }
    }
  } else if (xbee.getResponse().isError()) {
    nss.print("Error reading packet.  Error code: ");
    nss.println(xbee.getResponse().getErrorCode());
  } else {
    // The local XBee did not provide a timely TX Status Response; this is unexpected, so errorLed is lit.
    flashLed(errorLed, 2, 50);
  }
  delay(1000);
}
