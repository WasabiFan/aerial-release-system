
void initializeRadio() {
    printf_begin();

    radio.begin();

    radio.enableAckPayload();
    radio.enableDynamicPayloads();

    // open communication lines based on role
#if ROLE == ROLE_GROUND
    radio.openWritingPipe(pipeNames[0]);
    radio.openReadingPipe(primaryPipeIndex, pipeNames[1]);
#elif ROLE == ROLE_SKY
    radio.openWritingPipe(pipeNames[1]);
    radio.openReadingPipe(primaryPipeIndex, pipeNames[0]);
#endif

    radio.startListening();
    radio.printDetails();
    radio.writeAckPayload(primaryPipeIndex, &ackVal, 1);
}

#if ROLE == ROLE_SKY
void handleAck() {
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
}
#endif

bool validateAck(byte ackResponse) {
    bool isCorrectResponse = (ackResponse == ackVal);

    if (isCorrectResponse) {
        Serial.println(F("Comms healthy"));
    }
    else {
        Serial.print(F("Bad ack received: "));
        Serial.println(ackResponse);
        Serial.println(F("This probably means that the connection is unstable."));
    }

#if ROLE == ROLE_GROUND
    isCommHealthy = isCorrectResponse;
    // We have validated that data was successfully sent; we can consider this a heartbeat.
    lastHeartbeat = millis();
#endif
    return isCorrectResponse;
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

    if (radio.available()) {
        Serial.println(F("Data available before request was sent! This probably means something was not read correctly."));
    }
    // Print bytes that are going to be sent
    Serial.print(F("Sending data {"));
    for (int i = 0; i < numBytes; i++)
    {
        Serial.print(((byte*)val)[i]);
        if (i < numBytes - 1) {
            Serial.print(F(", "));
        }
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

            Serial.println(F("Received ack"));
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
