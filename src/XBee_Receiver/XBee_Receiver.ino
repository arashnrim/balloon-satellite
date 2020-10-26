#include <XBee.h>
#include <Printers.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// Creates reusable response objects for responses we expect to handle.
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

int statusLed = 13;
int errorLed = 13;
int dataLed = 13;

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

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);

  // start serial
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  flashLed(statusLed, 3, 50);
}

// Continuously reads packets, looking for ZB Receive or Modem Status.
void loop() {

  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(rx);

      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        // The receiver successfully receives data and the sender receives an acknowledgement from the receiver.
        flashLed(statusLed, 10, 10);
      } else {
        // The receiver successfully receives data, but the sender does not receive an acknowledgement from the receiver.
        flashLed(errorLed, 2, 20);
      }
        // Unpacks the payload data coming from the sender.
        int first, second, third = rx.getData(0), rx.getData(1), rx.getData(2);
        float HIH4030_Value = (first * 100) + (second * 10) + third;
        float voltage = HIH4030_Value / 1023. * 5.0;
        float sensorRH = 161.0 * voltage / 5.0 - 25.8;
        // The temperature of the surroundings. Edit this to fit your surroundings.
        float temperature = 24;
        // Calculates the true relative humidity percentage based on the temperature.
        float trueRH = sensorRH / (1.0546 - 0.0026 * temperature);
        Serial.println(trueRH);

    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbee.getResponse().getModemStatusResponse(msr);
      // The local XBee sends this response on certain events, like during association/dissociation.

      if (msr.getStatus() == ASSOCIATED) {
        flashLed(statusLed, 10, 10);
      } else if (msr.getStatus() == DISASSOCIATED) {
        flashLed(errorLed, 10, 10);
      } else {
        flashLed(statusLed, 5, 10);
      }
    } else {
      // The receiver got something unexpected; the errorLed is lit.
      flashLed(errorLed, 1, 25);
    }
  } else if (xbee.getResponse().isError()) {
    // An error occurred when the receiver is trying to read a response from the sender.
    Serial.println("Error reading packet.  Error code: ");
    Serial.println(xbee.getResponse().getErrorCode());
  }
}
