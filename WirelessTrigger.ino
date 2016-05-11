#include "Config.h"

// Standard libraries
#include <Arduino.h>
#include <printf.h>

// Radio comms
#include <RF24_config.h>
#include <nRF24L01.h>
#include <RF24.h>

#if ROLE == ROLE_GROUND

// Smart switch utils
#include "~Switch.h"

#elif ROLE == ROLE_SKY

// Barometric pressure sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Servo controller
#include <Servo.h>
#endif

RF24 radio(7, 8);

// radio comm lines
byte pipeNames[][6] = { *(byte*)"1Node", *(byte*)"2Node" };
byte primaryPipeIndex = 1;

// radio command bytes
const byte triggerVal = 0b11111111; //  255 decimal
const byte resetVal = 0b10101010; // 170 decimal
const byte heightInitialVal = 0b01010101; // 85 decimal
const byte ackVal = 0b11110000; // 240 decimal

#if ROLE == ROLE_GROUND

// can be digital pins
uint8_t triggerButtonPin = A0;
uint8_t resetButtonPin = A1;
uint8_t calibButtonPin = A2;

Switch triggerButton = Switch(triggerButtonPin);
Switch resetButton = Switch(resetButtonPin);
Switch calibButton = Switch(calibButtonPin);

// can be digital pins
uint8_t indicatorLEDGreenPin = 5;
uint8_t indicatorLEDBluePin = 3;
uint8_t indicatorLEDRedPin = 6;

unsigned long lastHeartbeat = 0;
bool isCommHealthy = false;

unsigned long lastCommandTime = 0;

float rawCalibrationAltitude = 0;
bool isAltitudeCalibrated = false;
float lastAltitude = NAN; // NAN indicates that altitude is not stored yet
float rawLastAltitude = NAN;

#elif ROLE == ROLE_SKY

// The code is in place to automatically reset the arduino if problems arise.
// It hasn't been proven working, so we aren't wiring it up.
uint8_t resetPin = 9;

// Should be a PWM pin
uint8_t servoPin = 5;
Servo actuationServo;

float lastAltitudes[ALTITUDE_SMOOTHING];
int lastAltitudeIndex;

// Parameter is sensor ID
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
bool bmpConnected = false;

// We use a digital output pin to power the altimiter so that we
//  don't need to splice wires.
// TODO: power this from an actual power pin
uint8_t bmpPowerPin = 4;
#endif

void setup()
{
    Serial.begin(115200);
    initializeRadio();

#if ROLE == ROLE_GROUND
    pinMode(indicatorLEDGreenPin, OUTPUT);
    pinMode(indicatorLEDBluePin, OUTPUT);
    pinMode(indicatorLEDRedPin, OUTPUT);
#elif ROLE == ROLE_SKY
    actuationServo.attach(servoPin);
    actuationServo.write(RESET_SERVO_ANGLE);

    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, HIGH);

    // we don't have enough power pins, so we just power it from an output pin
    pinMode(bmpPowerPin, OUTPUT);
    digitalWrite(bmpPowerPin, HIGH);

    // initialize altimeter
    Serial.print(F("BMP085 connecting... "));
    if (bmp.begin()) {
        bmpConnected = true;
        Serial.println(F("Succeeded"));
    }
    else {
        Serial.println(F("Failed"));
    }
#endif
}

#if ROLE == ROLE_GROUND
void loop() {
    triggerButton.poll();
    resetButton.poll();
    calibButton.poll();

    // Check for and handle incoming bytes first to make sure
    // that buffers are clear. If they aren't, acks may not work correctly.
    if (radio.available()) {
        Serial.println(F("Data available"));
        byte recvByte = receiveByte();

        if (recvByte == heightInitialVal) {
            // TODO: Figure out why the first byte isn't already gone

            // Declare a buffer to store the packet and read the data into it
            byte buffer[sizeof(float) + 1];
            receiveData(&buffer, sizeof(buffer));

            // Dereference the data portion of the packet as a float
            rawLastAltitude = *(float*)(&(buffer[1]));
            if (isAltitudeCalibrated) {
                lastAltitude = rawLastAltitude - rawCalibrationAltitude;
            }
            else {
                lastAltitude = rawLastAltitude;
            }
            Serial.println("Got altitude " + String(lastAltitude));
        }
        else {
            Serial.println(F("Received unknown header data!"));
        }
    }

    // Require a long press to activate
    if (triggerButton.longPress()) {
        Serial.println(F("Sending trigger byte"));
        if (validateAck(sendByte(triggerVal)))
            lastCommandTime = millis();
    }
    else if (resetButton.longPress()) {
        Serial.println(F("Sending reset byte"));
        if (validateAck(sendByte(resetVal)))
            lastCommandTime = millis();
    }
    else if (calibButton.pushed()) {
        if (isnan(rawLastAltitude)) {
            // if we haven't received an altitude yet, we can't calibrate
            Serial.println(F("Tried to calibrate, but haven't received altitude packet yet."));
        }
        else {
            rawCalibrationAltitude = rawLastAltitude;
            Serial.println("New calibrated base altitude: " + String(rawCalibrationAltitude));
            isAltitudeCalibrated = true;

            lastCommandTime = millis();
        }
    }
    else if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MILLIS) {
        // Re-use ack byte for heartbeat
        Serial.println(F("Sending ping"));
        validateAck(sendByte(ackVal));
    }

    //Serial.println("Current calibrated altitude: " + String(lastAltitude));

    bool altitudeAboveThreshold = isAltitudeCalibrated && !isnan(lastAltitude) && lastAltitude >= ALTITUDE_TARGET_THRESH_FEET;
    if (millis() - lastCommandTime <= COMMAND_SIG_DURATION) {
        setStatusLED(255, 255, 0); // Yellow
    }
    else if (isCommHealthy && altitudeAboveThreshold) {
        setStatusLED(0, 255, 0); // Green
    }
    else if (isCommHealthy) {
        setStatusLED(0, 0, 255); // Blue
    }
    else {
        setStatusLED(255, 0, 0); // Red
    }
}

#elif ROLE == ROLE_SKY
void loop() {
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
            handleAck();
            break;
        default:
            Serial.print(F("Received unknown signal: "));
            Serial.println((int)recvByte);
            break;
        }
        checkAltimeterDataIntegrity();
    }
}
#endif

#if ROLE == ROLE_GROUND
void setStatusLED(int r, int g, int b) {
    analogWrite(indicatorLEDRedPin, r);
    analogWrite(indicatorLEDGreenPin, g);
    analogWrite(indicatorLEDBluePin, b);
}
#endif


