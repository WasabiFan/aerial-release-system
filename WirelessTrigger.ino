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
uint8_t triggerButtonPin = 2;
uint8_t resetButtonPin = 3;

Switch triggerButton = Switch(triggerButtonPin);
Switch resetButton = Switch(resetButtonPin);

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
    pinMode(triggerButtonPin, INPUT_PULLUP);
    pinMode(resetButtonPin, INPUT_PULLUP);
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
        }
        else {
            radio.read(&recvByte, 1);
            Serial.print("Received ack byte " + recvByte);
        }
    }
    else {
        Serial.println("Unknown failure");
    }
}
#endif

void loop()
{
#if ROLE == ROLE_SENDER
    triggerButton.poll();
    resetButton.poll();

    // Require a long press to activate
    if (triggerButton.longPress()) {
        sendByte(triggerVal);
    }
    else if (resetButton.pushed()) {
        sendByte(resetVal);
    }
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
        else {
            Serial.print("Received bad signal ");
            Serial.println((int)recvByte);
        }

        radio.writeAckPayload(1, &ackVal, 1);
    }
#endif
}
