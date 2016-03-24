// Standard libraries
#include <Arduino.h>
#include <printf.h>

// Servo controller
#include <Servo.h>

// Radio comms
#include <RF24_config.h>
#include <nRF24L01.h>
#include <RF24.h>

// Smart switch utils
#include "Switch.h"

#define ROLE_SENDER 0
#define ROLE_RECEIVER 1

// SET THE TARGET ROLE HERE -------------------------------
#define ROLE ROLE_SENDER

RF24 radio(7, 8);
byte addresses[][6] = { *(byte*)"1Node", *(byte*)"2Node" };

const byte triggerVal = 0b11111111;
const byte resetVal = 0b10101010;
const byte ackVal = 0b11110000;

#if ROLE == ROLE_SENDER
#define HEARTBEAT_THRESH_MILLIS 1000

uint8_t triggerButtonPin = 2;
uint8_t resetButtonPin = 3;

Switch triggerButton = Switch(triggerButtonPin);
Switch resetButton = Switch(resetButtonPin);

uint8_t heartbeatLEDPin = 13;

unsigned long lastHeartbeat = 0;
bool isCommHealthy = false;

#else
#define TRIGGER_SERVO_ANGLE 90
#define RESET_SERVO_ANGLE 0

// Should be a PWM pin
uint8_t servoPin = 5;
Servo actuationServo;
#endif

void setup()
{
    Serial.begin(115200);

    radio.begin();

    radio.enableAckPayload();
    radio.enableDynamicPayloads();

#if ROLE == ROLE_SENDER
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
#else
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
#endif

    radio.startListening();
    radio.writeAckPayload(1, &ackVal, 1);

#if ROLE == ROLE_SENDER
    pinMode(heartbeatLEDPin, OUTPUT);
#else
    actuationServo.attach(servoPin);
    actuationServo.write(RESET_SERVO_ANGLE);
#endif
}

#if ROLE == ROLE_SENDER
byte sendByte(byte val) {
    byte recvByte;

    radio.stopListening();
    if (radio.write(&val, 1)) {
        if (!radio.available()) {
            Serial.println("Empty recv!");
            return 0;
        }
        else {
            radio.read(&recvByte, 1);
            Serial.print("Received ack byte " + recvByte);
        }
    }
    else {
        Serial.println("Unknown failure");
        return 0;
    }

    return recvByte;
}

void validateAck(byte ackResponse) {
    if (ackResponse == ackVal)
        isCommHealthy = true;
    else {
        isCommHealthy = false;
        Serial.println("Bad ack received! This probably means that the connection is unstable.");
    }

    // We have validated that data was successfully sent; we can consider this a heartbeat.
    lastHeartbeat = millis();
}
#endif

void loop()
{
#if ROLE == ROLE_SENDER
    triggerButton.poll();
    resetButton.poll();

    // Require a long press to activate
    if (triggerButton.longPress()) {
        validateAck(sendByte(triggerVal));
    }
    else if (resetButton.pushed()) {
        validateAck(sendByte(resetVal));
    }
    else if (millis() - lastHeartbeat >= HEARTBEAT_THRESH_MILLIS) {
        // Re-use ack byte for heartbeat
        validateAck(sendByte(triggerVal));
    }

    digitalWrite(heartbeatLEDPin, isCommHealthy);
#else
    if (radio.available()) {
        byte recvByte;
        radio.read(&recvByte, 1);

        if (recvByte == triggerVal) {
            Serial.println("TRIGGER");
            actuationServo.write(TRIGGER_SERVO_ANGLE);
        }
        else if (recvByte == resetVal) {
            Serial.println("RESET");
            actuationServo.write(RESET_SERVO_ANGLE);
        }
        else if (recvByte == ackVal) {
            // Heartbeat received. Just no-op.
            Serial.println("Heartbeat byte received");
        }
        else {
            Serial.print("Received bad signal ");
            Serial.println((int)recvByte);
        }

        radio.writeAckPayload(1, &ackVal, 1);
    }
#endif
}
