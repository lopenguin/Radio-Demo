/*
Tests addressed transmission and receipt of integers over nrf69 radio. Code is
for Teensy 4.0 connected to Adafruit radio breakout board.
Based on the sample Radiohead library demo and Adafruit documentation.
Do not run for longer than 10 minutes.

Lorenzo Shaikewitz, 6/6/2021
*/

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
// distance sensor stuff:
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"

/*** Begin Radio Setup ***/
#define FREQ    906.0

#define MY_ADDRESS      1
#define DEST_ADDRESS    2

// Pins
#define RFM69_CS    10
#define RFM69_INT   6   // also called G0
#define RFM69_RST   5
#define STATUS_LED   13

RH_RF69 rf69(RFM69_CS, RFM69_INT);
/*** End Radio Setup ***/

// motor setup
#define LMOTOR_EN   0
#define LMOTOR_IN1  14
#define LMOTOR_IN2  15

#define RMOTOR_EN   1
#define RMOTOR_IN1  16
#define RMOTOR_IN2  17

unsigned long lastDistTime{0};
SFEVL53L1X distSen(Wire);

void setup() {
    Serial.begin(115200);
    // wait for serial monitor open
    // while (!Serial) {delay(10);}

    pinMode(STATUS_LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    // manual reset of radio
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if (!rf69.init()) {
        Serial.println("Radio init failed");
        while (1);
    }
    Serial.println("Radio init successful!");

    if (!rf69.setFrequency(FREQ)) {
        Serial.println("setFrequency() failed");
    }

    rf69.setTxPower(20, true);  // range is 14-20 for the power

    Serial.print("RFM69 radio @"); Serial.print((int)FREQ); Serial.println(" MHz");

    // transmit license without encryption
    rf69.waitPacketSent();
    char radiopacket[20] = "This is KN4FVI";
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();
    Serial.println("Transmitted license code: KN4FVI");

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    // distance sensor stuff
    Wire.begin();
    if (distSen.begin() != 0)  // Good init returns 0
    {
      Serial.println(F("Distance sensor failed to begin. Freezing..."));
      digitalWrite(STATUS_LED, HIGH);
      while(1)
        ;
    }
    // We only really need short range distance
    distSen.setTimingBudgetInMs(50);
    distSen.setIntermeasurementPeriod(50);
    Serial.println(F("Distance sensor online."));
    lastDistTime = millis();

    // motor setup
    pinMode(LMOTOR_EN, OUTPUT);
    digitalWrite(LMOTOR_EN, LOW);
    pinMode(LMOTOR_IN1, OUTPUT);
    pinMode(LMOTOR_IN2, OUTPUT);

    pinMode(RMOTOR_EN, OUTPUT);
    digitalWrite(RMOTOR_EN, LOW);
    pinMode(RMOTOR_IN1, OUTPUT);
    pinMode(RMOTOR_IN2, OUTPUT);
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
// uint8_t defaultData[] = "KN4FVI";

uint8_t distance[2] = {0};
int lostCount{ 0 };

void loop() {
    bool stableConnection = recieveAndReply(distance);
    if (!stableConnection) {
        Serial.println("Connection Lost!");
        ++lostCount;
        // turn off wheels if connection lost!
        if (lostCount > 10) {
            digitalWrite(LMOTOR_EN, LOW);
            digitalWrite(RMOTOR_EN, LOW);
        }
    }
    lostCount = 0;

    if (millis() - lastDistTime > 50) {
        lastDistTime = millis();
        distSen.startRanging();
    }
    if (distSen.checkForDataReady()) {
        int dist_16bit = distSen.getDistance();
        dist_16bit = constrain(dist_16bit, 0, 65535);
        distSen.clearInterrupt();
        distSen.stopRanging();
        distance[0] = (uint8_t)(dist_16bit & 255);
        distance[1] = (uint8_t)(dist_16bit >> 8);
        // Serial.print(distance[0]); Serial.print('\t'); Serial.print(distance[1]);
        // Serial.print('\t'); Serial.println(dist_16bit);
    }
}

bool recieveAndReply(uint8_t* data) {
    if (rf69.available()) {
        // Should be a message for us now
        uint8_t len = sizeof(buf);
        if (rf69.recv(buf, &len)) {
            if (!len) return true;
            buf[len] = 0;
            Serial.print("Received [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI: ");
            Serial.println(rf69.lastRssi(), DEC);

            if (buf[0] == 255) {
               // Send a reply!
               rf69.send(data, sizeof(data));
               rf69.waitPacketSent();
               Serial.println("Sent a reply");
               writeMotors(buf);
           }
        } else {
            Serial.println("Receive failed");
            return false;
        }
    }
    return true;
}

void writeMotors(uint8_t* data) {
    // assume: [+x, +y, -x, -y]
    // x: controls speed of motors
    // y: controls relative speed
    int lMotorSpeed{0};
    int rMotorSpeed{0};
    if (data[2] == 0) {
        // go in reverse
        lMotorSpeed = data[4];
        rMotorSpeed = data[4];

        digitalWrite(LMOTOR_IN1, HIGH);
        digitalWrite(LMOTOR_IN2, LOW);

        digitalWrite(RMOTOR_IN1, HIGH);
        digitalWrite(RMOTOR_IN2, LOW);
    } else {
        // go forward
        lMotorSpeed = data[2];
        rMotorSpeed = data[2];

        digitalWrite(LMOTOR_IN1, LOW);
        digitalWrite(LMOTOR_IN2, HIGH);

        digitalWrite(RMOTOR_IN1, LOW);
        digitalWrite(RMOTOR_IN2, HIGH);
    }

    if (data[3] == 0) {
        // turn left!
        lMotorSpeed -= data[1];
        rMotorSpeed += data[1];
        if (data[1] > 240)
            lMotorSpeed = 200;
    } else {
        // turn right!
        lMotorSpeed += data[3];
        rMotorSpeed -= data[3];
        if (data[3] > 240)
            rMotorSpeed = 200;
    }

    lMotorSpeed = constrain(lMotorSpeed, 0, 255);
    rMotorSpeed = constrain(rMotorSpeed, 0, 255);
    analogWrite(LMOTOR_EN, lMotorSpeed);
    analogWrite(RMOTOR_EN, rMotorSpeed);
}
