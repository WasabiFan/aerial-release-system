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

#define ROLE_GROUND 0
#define ROLE_SKY 1

// SET THE TARGET ROLE HERE -------------------------------
#define ROLE ROLE_SKY
//#define DISABLE_DISPLAY

RF24 radio(7, 8);

#if !defined(DISABLE_DISPLAY) && ROLE == ROLE_GROUND
LiquidCrystal display(10, 9, 5, 4, 3, 2);
#endif
byte addresses[][6] = { *(byte*)"1Node", *(byte*)"2Node", *(byte*)"ANode" };
byte primaryPipeIndex = 1;
byte altitudePipeIndex = 2;

// TODO: Choose numbers that make sense
const byte triggerVal = 0b11111111;
const byte resetVal = 0b10101010;
const byte heightInitialVal = 0b01010101;
const byte ackVal = 0b11110000;

#if ROLE == ROLE_GROUND
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

float lastAltitude = -1;

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

#if !defined(DISABLE_DISPLAY) && ROLE == ROLE_GROUND
    display.begin(16, 2);
    display.print("Display initialized");
#endif

    radio.begin();

    radio.enableAckPayload();
    radio.enableDynamicPayloads();

#if ROLE == ROLE_GROUND
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(primaryPipeIndex, addresses[1]);
    radio.openReadingPipe(altitudePipeIndex, addresses[2]);
#else
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(primaryPipeIndex, addresses[0]);
#endif

    radio.startListening();
    radio.printDetails();
    radio.writeAckPayload(1, &ackVal, 1);

#if ROLE == ROLE_GROUND
    //pinMode(heartbeatLEDGreenPin, OUTPUT);
    pinMode(heartbeatLEDRedPin, OUTPUT);
#else
    actuationServo.attach(servoPin);
    actuationServo.write(RESET_SERVO_ANGLE);
#endif
}


byte sendData(void* val, int numBytes) {
    byte recvByte;

    if (radio.available())
        Serial.println("Data available before request was sent! This probably means something was not read correctly.");

    Serial.print("Sending data ");
    for (int i = 0; i < numBytes; i++)
    {
        Serial.print(((byte*)val)[i]);
        Serial.print(" ");
    }
    Serial.println();

    radio.stopListening();
    bool writeResult = radio.write(val, numBytes);
    radio.startListening();

    if (writeResult) {
        if (!radio.available()) {
            Serial.println("Empty recv!");
            return 0;
        }
        else {
            radio.read(&recvByte, 1);
            radio.writeAckPayload(1, &ackVal, 1);

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

byte sendByte(byte val)
{
    return sendData(&val, 1);
}

void validateAck(byte ackResponse) {
    if (ackResponse == ackVal) {
#if ROLE == ROLE_GROUND
        isCommHealthy = true;
#endif
        Serial.println("Comms healthy");
    }
    else {
#if ROLE == ROLE_GROUND
        isCommHealthy = false;
#endif
        Serial.println("Bad ack received! This probably means that the connection is unstable.");
        Serial.println(ackResponse);
    }

#if ROLE == ROLE_GROUND
    // We have validated that data was successfully sent; we can consider this a heartbeat.
    lastHeartbeat = millis();
#endif
}

bool receiveData(void* recvBuf, uint8_t numBytes) {
    if (radio.available()) {
        radio.read(recvBuf, numBytes);

        radio.writeAckPayload(1, &ackVal, 1);

        return true;
    }
    else
    {
        return false;
    }
}

byte receiveByte() {
    byte val = 0;
    Serial.println(receiveData(&val, 1));

    return val;
}

void loop()
{
#if ROLE == ROLE_GROUND

    triggerButton.poll();
    resetButton.poll();

    // Check for and handle incoming bytes first to make sure
    // that buffers are clear. If they aren't, acks may not work correctly.
    if (radio.available()) {
        Serial.println("Data available");
        byte recvByte = receiveByte();
        if (recvByte == heightInitialVal)
        {
            //receiveData(&lastAltitude, sizeof(float));
            Serial.println("Got altitude");
        }
        else
        {
            Serial.println("Received unknown header data!");
        }
    }

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

        Serial.println(radio.available() ? "Data are AVAILABLE" : "Data ARE NOT available");
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
        display.print("ALTITUDE: " + String(lastAltitude) + "        ");
#endif

#else
    if(radio.available()) {
        byte recvByte = receiveByte();
        
        if (recvByte == triggerVal) {
            Serial.println("TRIGGER");
            actuationServo.write(TRIGGER_SERVO_ANGLE);
        }
        else if (recvByte == resetVal) {
            Serial.println("RESET");
            actuationServo.write(RESET_SERVO_ANGLE);
        }
        else if (recvByte == ackVal) {
            Serial.println("Heartbeat byte received");
            Serial.println(radio.available() ? "AFTER AVAIL" : "AFTER NOT");
            float altitude = 9;

            byte foo[1] = { heightInitialVal };
            //foo[1] = altitude;

            //validateAck(sendByte(heightInitialVal));//sendData(&foo, sizeof(foo)));
        }
        else {
            Serial.print("Received unknown signal ");
            Serial.println((int)recvByte);
        }
    }
#endif
}