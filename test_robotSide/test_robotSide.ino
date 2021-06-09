/*
Tests addressed transmission and receipt of integers over nrf69 radio. Code is
for Teensy 4.0 connected to Adafruit radio breakout board.
Based on the sample Radiohead library demo and Adafruit documentation.
Do not run for longer than 10 minutes.

Lorenzo Shaikewitz, 6/2/2021
*/

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/*** Begin Radio Setup ***/
#define FREQ    906.0

#define MY_ADDRESS      1
#define DEST_ADDRESS    2

// Pins
#define RFM69_CS    10
#define RFM69_INT   6   // also called G0
#define RFM69_RST   5
#define STATUS_LED   13
// #define RFM69_CS    8
// #define RFM69_INT   7   // also called G0
// #define RFM69_RST   4
// #define STATUS_LED  13

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
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
unsigned long startTime = 0;

void loop() {
    delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

    startTime = millis();
    char radiopacket[20] = "Hello World #";
    itoa(packetnum++, radiopacket+13, 10);
    Serial.print("Sending "); Serial.println(radiopacket);

    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
            buf[len] = 0; // zero out remaining string

            Serial.print("Got reply from #"); Serial.print(from);
            Serial.print(" [RSSI :");
            Serial.print(rf69.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            //Blink(STATUS_LED, 40, 3); //blink LED 3 times, 40ms between blinks
        } else {
            Serial.println("No reply, is anyone listening?");
        }
    } else {
        Serial.println("Sending failed (no ack)");
    }
    Serial.println(millis() - startTime);
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
    for (byte i=0; i<loops; i++)  {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
    }
}
