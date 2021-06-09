/*
Tests addressed transmission and receipt of integers over nrf69 radio. Code is
for Feather 32u4 radio board.
Based on the sample Radiohead library demo and Adafruit documentation.

Lorenzo Shaikewitz, 6/2/2021
*/

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/*** Begin Radio Setup ***/
#define FREQ    906.0

#define MY_ADDRESS      2
#define DEST_ADDRESS    1

// Pins
#define RFM69_CS    8
#define RFM69_INT   7   // also called G0
#define RFM69_RST   4
#define STATUS_LED  13
// #define RFM69_CS    10
// #define RFM69_INT   6   // also called G0
// #define RFM69_RST   5
// #define STATUS_LED   13

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission
/*** End Radio Setup ***/

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

    if (!rf69_manager.init()) {
        Serial.println("Radio init failed");
        while (1);
    }
    Serial.println("Radio init successful!");

    if (!rf69.setFrequency(FREQ)) {
        Serial.println("setFrequency() failed");
    }

    rf69.setTxPower(20, true);  // range is 14-20 for the power

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    Serial.print("RFM69 radio @"); Serial.print((int)FREQ); Serial.println(" MHz");
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "And hello back to you";

unsigned long startTime = 0;

void loop() {
    checkForMessage();
}

void checkForMessage() {
    if (rf69_manager.available())
    {
        startTime = millis();
        // Wait for a message addressed to us from the client
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (rf69_manager.recvfromAck(buf, &len, &from)) {
            buf[len] = 0; // zero out remaining string

            Serial.print("Got packet from #"); Serial.print(from);
            Serial.print(" [RSSI :");
            Serial.print(rf69.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            //Blink(STATUS_LED, 40, 3); //blink LED 3 times, 40ms between blinks

            // Send a reply back to the originator client
            if (!rf69_manager.sendtoWait(data, sizeof(data), from))
                Serial.println("Sending failed (no ack)");
        }
        Serial.println(millis() - startTime);
    }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
    for (byte i=0; i<loops; i++)  {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
    }
}
