/*
Tests addressed transmission and receipt of integers over nrf69 radio. Code is
for Feather 32u4 radio board.
Based on the sample Radiohead library demo and Adafruit documentation.

Lorenzo Shaikewitz, 6/6/2021
*/

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
// For 7-segment display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

/*** Begin Radio Setup ***/
#define FREQ    906.0

#define MY_ADDRESS      2
#define DEST_ADDRESS    1

// Pins
#define RFM69_CS    8
#define RFM69_INT   7   // also called G0
#define RFM69_RST   4
#define STATUS_LED  13

RH_RF69 rf69(RFM69_CS, RFM69_INT);
/*** End Radio Setup ***/

// pins for joystick
#define JOYX_PIN    A0
#define JOYY_PIN    A1
// pins for RGB status LED
#define RGB_LED_R   9
#define RGB_LED_G   6
#define RGB_LED_B   5

unsigned long lastTime = 0;
Adafruit_7segment matrix = Adafruit_7segment();


void setup() {
    Serial.begin(115200);
    // wait for serial monitor open
    // while (!Serial) {delay(10);}
    matrix.begin(0x70);

    pinMode(RGB_LED_R, OUTPUT);
    pinMode(RGB_LED_G, OUTPUT);
    pinMode(RGB_LED_B, OUTPUT);
    digitalWrite(RGB_LED_R, LOW);
    digitalWrite(RGB_LED_G, LOW);
    digitalWrite(RGB_LED_B, LOW);

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

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    Serial.print("RFM69 radio @"); Serial.print((int)FREQ); Serial.println(" MHz");

    lastTime = millis();
}

const int radioDelayMS = 25;
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t data[] = "KN4FVI";

class ArrayUint8_t {
    int m_size;
    uint8_t* m_array;
public:
    ArrayUint8_t(int size) : m_size{size} {
        m_array = new uint8_t[m_size]{};
    }

    int size() {return m_size;}

    uint8_t& operator[](int idx) {
        if (idx < m_size)
            return m_array[idx];
    }

    String toString() const {
        String str = "[";
        for (int i = 0; i < m_size; ++i) {
            str += static_cast<String>(m_array[i]);
            str += ", ";
        }
        str = str.substring(0, str.length()-2);
        str += "]";
        return str;
    }

    operator uint8_t*() const {
        return m_array;
    }

    ~ArrayUint8_t() {
        delete[] m_array;
    }
};

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > radioDelayMS) {
        // send a packet with the joystick readings
        int joyX = analogRead(JOYX_PIN);
        int joyY = analogRead(JOYY_PIN);
        rescaleJoy(joyX, joyY);     // map from -255 -> 255

        // create the packet
        ArrayUint8_t packet = ArrayUint8_t(5);
        if (joyX < 0) {
            packet[3] = -joyX;
        } else {
            packet[1] = joyX;
        }
        if (joyY < 0) {
            packet[4] = -joyY;
        } else {
            packet[2] = joyY;
        }
        packet[0] = 255;    // this means "send reply"
        Serial.print("Sending "); Serial.println(packet.toString());    // need way to cast integer array to string at some point

        // Send!
        rf69.send(packet, packet.size());
        rf69.waitPacketSent();

        // Now wait for a reply
        if (rf69.waitAvailableTimeout(500))  {
            // Should be a reply message for us now
            if (rf69.recv(buf, &len)) {
                Serial.print("Got a reply: ");
                int distVal = distToInt(buf);
                Serial.println(distVal);
                Serial.print("RSSI: ");
                Serial.println(rf69.lastRssi(), DEC);
                rssiToLED(rf69.lastRssi());

                matrix.print(distVal, DEC);
                matrix.writeDisplay();
            } else {
                Serial.println("Receive failed");
                rssiToLED(-100);
            }
        } else {
            Serial.println("No reply, is another RFM69 listening?");
            rssiToLED(-100);
        }
    }
}

void rescaleJoy(int& joyX, int& joyY) {
    if (joyX < 461)
        joyX = map(joyX, 0, 511, -255, 0);
    else if (joyX > 561)
        joyX = map(joyX, 511, 1023, 0, 255);
    else
        joyX = 0;

    if (joyY < 461)
        joyY = map(joyY, 0, 511, -255, 0);
    else if (joyY > 561)
        joyY = map(joyY, 511, 1023, 0, 255);
    else
        joyY = 0;
}

void rssiToLED(int rssiVal) {
    static int lowerRSSI{ -20 };
    static int upperRSSI{ -100 };
    float gradVal = static_cast<float>(rssiVal - lowerRSSI) / (upperRSSI - lowerRSSI);
    // if gradVal >= 1, bad connection
    gradVal = constrain(gradVal, 0, 1);
    int redVal = map(gradVal, 0, 1, 0, 255);
    analogWrite(RGB_LED_R, redVal);
    analogWrite(RGB_LED_G, 255 - redVal);
}

int distToInt(uint8_t* data) {
    return (int)(data[0] | (data[1] << 8));
}
