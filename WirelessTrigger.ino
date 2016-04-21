#define ROLE_GROUND 0
#define ROLE_SKY 1

// SET THE TARGET ROLE HERE -------------------------------
#define ROLE ROLE_SKY

// Standard libraries
#include <Arduino.h>
#include <printf.h>

// Radio comms
#include <RF24_config.h>
#include <nRF24L01.h>
#include <RF24.h>

#if ROLE == ROLE_GROUND

// Display
#include <LiquidCrystal.h>

// Smart switch utils
#include "~Switch.h"

#else

// Barometric pressure sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Servo controller
#include <Servo.h>

#endif

#define DISABLE_DISPLAY

RF24 radio(7, 8);

#if !defined(DISABLE_DISPLAY) && ROLE == ROLE_GROUND
LiquidCrystal display(10, 9, 5, 4, 3, 2);
#endif
// radio comm lines
byte pipeNames[][6] = { *(byte*)"1Node", *(byte*)"2Node" };
byte primaryPipeIndex = 1;

// radio command bytes
const byte triggerVal = 0b11111111; //  255 decimal
const byte resetVal = 0b10101010; // 170 decimal
const byte heightInitialVal = 0b01010101; // 85 decimal
const byte ackVal = 0b11110000; // 240 decimal

#if ROLE == ROLE_GROUND
#define HEARTBEAT_INTERVAL_MILLIS 1000
#define DISPLAY_TIME_MILLIS 1000

                                // can be digital pinsa
uint8_t triggerButtonPin = A0;
uint8_t resetButtonPin = A1;

Switch triggerButton = Switch(triggerButtonPin);
Switch resetButton = Switch(resetButtonPin);
unsigned long lastTriggerSignalTime = 0;
bool lastTriggerType = false;

// can be digital pins
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


// Parameter is sensor ID
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
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
    radio.openWritingPipe(pipeNames[0]);
    radio.openReadingPipe(primaryPipeIndex, pipeNames[1]);
#else
    radio.openWritingPipe(pipeNames[1]);
    radio.openReadingPipe(primaryPipeIndex, pipeNames[0]);
#endif

    radio.startListening();
    radio.printDetails();
    radio.writeAckPayload(primaryPipeIndex, &ackVal, 1);

#if ROLE == ROLE_GROUND
    //pinMode(heartbeatLEDGreenPin, OUTPUT);
    pinMode(heartbeatLEDRedPin, OUTPUT);
#else
    actuationServo.attach(servoPin);
    actuationServo.write(RESET_SERVO_ANGLE);

    if (!bmp.begin())
        Serial.print(F("BMP085 connection failed"));

#endif
}



#if ROLE == ROLE_SKY
float getAltitude() {
    sensors_event_t event;
    bmp.getEvent(&event);

    float temperature;
    bmp.getTemperature(&temperature);

    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; // TODO: Find a more accurate value?

    return bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);
}

#endif

void loop()
{
#if ROLE == ROLE_GROUND

    triggerButton.poll();
    resetButton.poll();

    // Check for and handle incoming bytes first to make sure
    // that buffers are clear. If they aren't, acks may not work correctly.
    if (radio.available()) {
        Serial.println(F("Data available"));
        byte recvByte = receiveByte();

        if (recvByte == heightInitialVal)
        {
            // TODO: Figure out why the first byte isn't already gone

            // Declare a buffer to store the packet and read the data into it
            byte buffer[sizeof(float) + 1];
            receiveData(&buffer, sizeof(buffer));

            // Dereference the data portion of the packet as a float
            lastAltitude = *(float*)(&(buffer[1]));
            Serial.println("Got altitude " + String(lastAltitude));
        }
        else
        {
            Serial.println(F("Received unknown header data!"));
        }
    }

    // Require a long press to activate
    if (triggerButton.longPress()) {
        Serial.println(F("Sending trigger byte"));

        // Record the last-sent signal
        lastTriggerType = true;
        lastTriggerSignalTime = millis();

        validateAck(sendByte(triggerVal));
    }
    else if (resetButton.longPress()) {
        Serial.println(F("Sending reset byte"));

        // Record the last-sent signal
        lastTriggerType = false;
        lastTriggerSignalTime = millis();

        validateAck(sendByte(resetVal));
    }
    else if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MILLIS) {
        // Re-use ack byte for heartbeat
        Serial.println(F("Sending ping"));
        validateAck(sendByte(ackVal));
    }

    digitalWrite(heartbeatLEDRedPin, isCommHealthy ? LOW : HIGH);

#if !defined(DISABLE_DISPLAY)
    // NOTE: Always send spaces after text to clear cells past desired content.

    display.setCursor(0, 0);
    display.print("Comms: " + (isCommHealthy ? "HEALTHY  " : "UNHEALTHY"));
    display.setCursor(0, 1);

    if (millis() - lastTriggerSignalTime < DISPLAY_TIME_MILLIS) {
        display.print(lastTriggerType ? "Sent TRIGGER" : "Sent RESET      ");
    }
    else {
        String altString = (lastAltitude == -1) ? "Unknown" : String(lastAltitude);
        display.print("ALTITUDE: " + altString + "        ");
    }
#endif

#else
    if (radio.available()) {
        byte recvByte = receiveByte();
        switch (recvByte) {
        case triggerVal:
            Serial.println(F("Trigger byte received"));
            actuationServo.write(TRIGGER_SERVO_ANGLE);
            break;
        case resetVal:
            Serial.println(F("Reset byte received"));
            actuationServo.write(RESET_SERVO_ANGLE);
            break;
        case ackVal:
            Serial.println(F("Heartbeat byte received"));

            // Get the altitude value
            float altitude = getAltitude();

            // Construct the packet. Has a one-byte header.
            byte packetData[sizeof(float) + 1] = { heightInitialVal };
            // Get a pointer to the last 4 bytes of the packet buffer 
            // and set it to the altitude value
            float* packetFloatPortion = (float*)&packetData[1];
            *packetFloatPortion = altitude;

            validateAck(sendData(&packetData, sizeof(packetData)));
            break;
        default:
            Serial.print(F("Received unknown signal "));
            Serial.println((int)recvByte);
            break;
        }
    }
#endif
}

void validateAck(byte ackResponse) {
    boolean correctResponse = (ackResponse == ackVal);

    if (correctResponse) {
        Serial.println(F("Comms healthy"));
    }
    else {
        Serial.println(F("Bad ack received! This probably means that the connection is unstable."));
    }

#if ROLE == ROLE_GROUND
    isCommHealthy = correctResponse;
    // We have validated that data was successfully sent; we can consider this a heartbeat.
    lastHeartbeat = millis();
#endif
}

// receives numBytes bytes from the radio into recvBuf
bool receiveData(void* recvBuf, uint8_t numBytes) {
    if (radio.available()) {
        radio.read(recvBuf, numBytes);
        // Prep an ack for the next time we receive data
        radio.writeAckPayload(primaryPipeIndex, &ackVal, 1);
        return true;
    }
    else {
        return false;
    }
}

// receives a single byte from the radio
byte receiveByte() {
    byte val = 0;
    receiveData(&val, 1);
    return val;
}

// Sends arbitrary data of length numBytes from the given address
byte sendData(void* val, int numBytes) {
    byte recvByte;

    if (radio.available())
        Serial.println(F("Data available before request was sent! This probably means something was not read correctly."));

    // Print bytes that are going to be sent
    Serial.print(F("Sending data {"));
    for (int i = 0; i < numBytes; i++)
    {
        Serial.print(((byte*)val)[i]);
        Serial.print(F(", "));
    }
    Serial.println(F("}"));

    // Stop listening temporarily so we can write
    radio.stopListening();
    bool writeSuccess = radio.write(val, numBytes);
    radio.startListening();

    if (writeSuccess) {
        if (!radio.available()) {
            Serial.println(F("Empty receive buffer! Was expecting an ack."));
            return 0;
        }
        else {
            radio.read(&recvByte, 1);
            radio.writeAckPayload(primaryPipeIndex, &ackVal, 1);

            Serial.print(F("Received ack"));
        }
    }
    else {
        Serial.println(F("Unknown radio write failure"));
        return 0;
    }

    return recvByte;
}

// send a single byte on the radio
byte sendByte(byte val)
{
    return sendData(&val, 1);
}
