// Standard libraries
#include <Arduino.h>
#include <printf.h>

// Servo controller
#include <Servo.h>

// Radio comms
#include <RF24_config.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// Display
#include <LiquidCrystal.h>

// Smart switch utils
#include "~Switch.h"

#define ROLE_SENDER 0
#define ROLE_RECEIVER 1

// SET THE TARGET ROLE HERE -------------------------------
#define ROLE ROLE_SENDER
//#define DISABLE_DISPLAY

RF24 radio(7, 8);

#if !defined(DISABLE_DISPLAY) && ROLE == ROLE_SENDER
LiquidCrystal display(10, 9, 5, 4, 3, 2);
#endif
byte addresses[][6] = { *(byte*)"1Node", *(byte*)"2Node" };

const byte triggerVal = 0b11111111;
const byte resetVal = 0b10101010;
const byte ackVal = 0b11110000;

#if ROLE == ROLE_SENDER
#define HEARTBEAT_THRESH_MILLIS 1000
#define DISPLAY_TIME_MILLIS 1000

uint8_t triggerButtonPin = A0;
uint8_t resetButtonPin = A1;

Switch triggerButton = Switch(triggerButtonPin);
Switch resetButton = Switch(resetButtonPin);
unsigned long lastTriggerSignalTime = 0;
bool lastTriggerType = false;

uint8_t heartbeatLEDGreenPin = 10;
uint8_t heartbeatLEDRedPin = 6;

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
    printf_begin();

#if !defined(DISABLE_DISPLAY) && ROLE == ROLE_SENDER
    display.begin(16, 2);
    display.print("Display initialized");
#endif

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
    radio.printDetails();
    radio.writeAckPayload(1, &ackVal, 1);

#if ROLE == ROLE_SENDER
    //pinMode(heartbeatLEDGreenPin, OUTPUT);
    pinMode(heartbeatLEDRedPin, OUTPUT);
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
            Serial.print("Received ack byte ");
            Serial.println((int)recvByte);
        }
    }
    else {
        Serial.println("Unknown failure");
        return 0;
    }

    return recvByte;
}

void validateAck(byte ackResponse) {
    if (ackResponse == ackVal) {
        isCommHealthy = true;
        Serial.println("Comms healthy");
    }
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
        Serial.println("Sending trigger");

        lastTriggerType = true;
        lastTriggerSignalTime = millis();

        validateAck(sendByte(triggerVal));
    }
    else if (resetButton.longPress()) {
        Serial.println("Sending reset");

        lastTriggerType = false;
        lastTriggerSignalTime = millis();

        validateAck(sendByte(resetVal));
    }
    else if (millis() - lastHeartbeat >= HEARTBEAT_THRESH_MILLIS) {
        // Re-use ack byte for heartbeat
        Serial.println("Sending ping");
        validateAck(sendByte(ackVal));
    }

    //analogWrite(heartbeatLEDGreenPin, isCommHealthy * 255);
    analogWrite(heartbeatLEDRedPin, !isCommHealthy * 255);

#if !defined(DISABLE_DISPLAY)
    // NOTE: Always send spaces after text to clear cells past desired content.

    display.setCursor(0, 0);
    display.print(isCommHealthy ? "Comms: HEALTHY  " : "Comms: UNHEALTHY");
    display.setCursor(0, 1);

    if (millis() - lastTriggerSignalTime < DISPLAY_TIME_MILLIS)
        display.print(lastTriggerType ? "Sent TRIGGER" : "Sent RESET  ");
    else
        display.print("UPTIME: " + String(millis() / 1000) + "        ");
#endif

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