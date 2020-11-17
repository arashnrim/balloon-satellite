#include <XBee.h>
#include <Printers.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
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
        int glb_count = 0;
      
        // Unpacks the payload data of the relative humidity coming from the sender.
        float rhValues[3];
        for (int int_count = 0; glb_count <= 2; int_count++) {
          rhValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float HIH4030_Value = (rhValues[0] * 100) + (rhValues[1] * 10) + rhValues[2];
        float voltage = HIH4030_Value / 1023 * 5.0;
        float sensorRH = 161.0 * voltage / 5.0 - 25.8;
        // The temperature of the surroundings. Edit this to fit your surroundings.
        float tempRH = 30;
        // Calculates the true relative humidity percentage based on the temperature.
        float trueRH = sensorRH / (1.0546 - 0.0026 * tempRH);
        Serial.print("Humidity: ");
        Serial.print(trueRH);
        Serial.println("%");

        // Unpacks the payload data of the temperature coming from the sender.
        float tempValues[4];
        for (int int_count = 0; glb_count <= 6; int_count++) {
          tempValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float temperature = tempValues[0] * 10 + tempValues[1] + tempValues[2] / 10 + tempValues[3] / 100;
        if (temperature == 0.00) {
          Serial.println("Warning: Temperature data not available");
        } else {
          Serial.print("Temperature: ");
          Serial.print(temperature);
          Serial.println("°C");
        }

        // Unpacks the payload data of the pressure coming from the sender.
        // We chose kPa over Pa due to Arduino's 16-bit restriction on integers.
        float pressureValues[3];
        for (int int_count = 0; glb_count <= 9; int_count++) {
          pressureValues[int_count] = rx.getData(glb_count);
          glb_count++;
        } 
        int pressure = pressureValues[0] * 100 + pressureValues[1] * 10 + pressureValues[2];
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println("kPa");

        // ============
        // =   9DOF   =
        // ============
        
        // Unpacks the payload data of the x-axis acceleration coming from the sender.
        char accel_xPolarity = rx.getData(10);
        glb_count++;
        float accel_xValues[4];
        for (int int_count = 0 ; glb_count <= 14; int_count++) {
          accel_xValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float accel_x;
        if (accel_xValues[0] == 0) {
          accel_x = accel_xValues[1] + accel_xValues[2] / 10 + accel_xValues[3] / 100;
        } else {
          accel_x = accel_xValues[0] * 10 + accel_xValues[1] + accel_xValues[2] / 10 + accel_xValues[3] / 100;
        }
        Serial.print("x-axis acceleration: ");
        Serial.print(accel_xPolarity);
        Serial.print(accel_x);
        Serial.println("m/s^2");

        // Unpacks the payload data of the y-axis acceleration coming from the sender.
        char accel_yPolarity = rx.getData(15);
        glb_count++;
        float accel_yValues[4];
        for (int int_count = 0 ; glb_count <= 19; int_count++) {
          accel_yValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float accel_y;
        if (accel_yValues[0] == 0) {
          accel_y = accel_yValues[1] + accel_yValues[2] / 10 + accel_yValues[3] / 100;
        } else {
          accel_y = accel_yValues[0] * 10 + accel_yValues[1] + accel_yValues[2] / 10 + accel_yValues[3] / 100;
        }
        Serial.print("y-axis acceleration: ");
        Serial.print(accel_yPolarity);
        Serial.print(accel_y);
        Serial.println("m/s^2");

        // Unpacks the payload data of the z-axis acceleration coming from the sender.
        char accel_zPolarity = rx.getData(20);
        glb_count++;
        float accel_zValues[4];
        for (int int_count = 0 ; glb_count <= 24; int_count++) {
          accel_zValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float accel_z;
        if (accel_zValues[0] == 0) {
          accel_z = accel_zValues[1] + accel_zValues[2] / 10 + accel_zValues[3] / 100;
        } else {
          accel_z = accel_zValues[0] * 10 + accel_zValues[1] + accel_zValues[2] / 10 + accel_zValues[3] / 100;
        }
        Serial.print("z-axis acceleration: ");
        Serial.print(accel_zPolarity);
        Serial.print(accel_z);
        Serial.println("m/s^2");

        // Unpacks the payload data of the x-axis magnetometer coming from the sender.
        char magnet_xPolarity = rx.getData(25);
        glb_count++;
        float magnet_xValues[4];
        for (int int_count = 0 ; glb_count <= 29; int_count++) {
          magnet_xValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float magnet_x;
        if (magnet_xValues[0] == 0) {
          magnet_x = magnet_xValues[1] + magnet_xValues[2] / 10 + magnet_xValues[3] / 100;
        } else {
          magnet_x = magnet_xValues[0] * 10 + magnet_xValues[1] + magnet_xValues[2] / 10 + magnet_xValues[3] / 100;
        }
        Serial.print("x-axis magnetometer: ");
        Serial.print(magnet_xPolarity);
        Serial.print(magnet_x);
        Serial.println("uT");

        // Unpacks the payload data of the y-axis magnetometer coming from the sender.
        char magnet_yPolarity = rx.getData(30);
        glb_count++;
        float magnet_yValues[4];
        for (int int_count = 0 ; glb_count <= 34; int_count++) {
          magnet_yValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float magnet_y;
        if (magnet_yValues[0] == 0) {
          magnet_y = magnet_yValues[1] + magnet_yValues[2] / 10 + magnet_yValues[3] / 100;
        } else {
          magnet_y = magnet_yValues[0] * 10 + magnet_yValues[1] + magnet_yValues[2] / 10 + magnet_yValues[3] / 100;
        }
        Serial.print("y-axis magnetometer: ");
        Serial.print(magnet_yPolarity);
        Serial.print(magnet_y);
        Serial.println("uT");

        // Unpacks the payload data of the z-axis magnetometer coming from the sender.
        char magnet_zPolarity = rx.getData(35);
        glb_count++;
        float magnet_zValues[4];
        for (int int_count = 0 ; glb_count <= 39; int_count++) {
          magnet_zValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float magnet_z;
        if (magnet_zValues[0] == 0) {
          magnet_z = magnet_zValues[1] + magnet_zValues[2] / 10 + magnet_zValues[3] / 100;
        } else {
          magnet_z = magnet_zValues[0] * 10 + magnet_zValues[1] + magnet_zValues[2] / 10 + magnet_zValues[3] / 100;
        }
        Serial.print("z-axis magnetometer: ");
        Serial.print(magnet_zPolarity);
        Serial.print(magnet_z);
        Serial.println("uT");

        // Unpacks the payload data of the x-axis gyroscope coming from the sender.
        char gyro_xPolarity = rx.getData(40);
        glb_count++;
        float gyro_xValues[4];
        for (int int_count = 0 ; glb_count <= 44; int_count++) {
          gyro_xValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float gyro_x;
        if (gyro_xValues[0] == 0) {
          gyro_x = gyro_xValues[1] + gyro_xValues[2] / 10 + gyro_xValues[3] / 100;
        } else {
          gyro_x = gyro_xValues[0] * 10 + gyro_xValues[1] + gyro_xValues[2] / 10 + gyro_xValues[3] / 100;
        }
        Serial.print("x-axis gryoscope: ");
        Serial.print(gyro_xPolarity);
        Serial.print(gyro_x);
        Serial.println("rad/s");

        // Unpacks the payload data of the y-axis gyroscope coming from the sender.
        char gyro_yPolarity = rx.getData(45);
        glb_count++;
        float gyro_yValues[4];
        for (int int_count = 0 ; glb_count <= 49; int_count++) {
          gyro_yValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float gyro_y;
        if (gyro_yValues[0] == 0) {
          gyro_y = gyro_yValues[1] + gyro_yValues[2] / 10 + gyro_yValues[3] / 100;
        } else {
          gyro_y = gyro_yValues[0] * 10 + gyro_yValues[1] + gyro_yValues[2] / 10 + gyro_yValues[3] / 100;
        }
        Serial.print("y-axis gyroscope: ");
        Serial.print(gyro_yPolarity);
        Serial.print(gyro_y);
        Serial.println("rad/s");

        // Unpacks the payload data of the z-axis gyroscope coming from the sender.
        char gyro_zPolarity = rx.getData(50);
        glb_count++;
        float gyro_zValues[4];
        for (int int_count = 0 ; glb_count <= 54; int_count++) {
          gyro_zValues[int_count] = rx.getData(glb_count);
          glb_count++;
        }
        float gyro_z;
        if (magnet_zValues[0] == 0) {
          gyro_z = magnet_zValues[1] + gyro_zValues[2] / 10 + gyro_zValues[3] / 100;
        } else {
          gyro_z = magnet_zValues[0] * 10 + gyro_zValues[1] + gyro_zValues[2] / 10 + gyro_zValues[3] / 100;
        }
        Serial.print("z-axis gyroscope: ");
        Serial.print(gyro_zPolarity);
        Serial.print(gyro_z);
        Serial.println("rad/s");

        Serial.println();

    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbee.getResponse().getModemStatusResponse(msr);

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
    Serial.println("Error reading packet. Error code: ");
    Serial.println(xbee.getResponse().getErrorCode());
  }
}
