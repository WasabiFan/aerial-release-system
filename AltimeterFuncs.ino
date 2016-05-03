#if ROLE == ROLE_SKY
// returns smoothed uncalibrated altitude
float getAltitude() {
  if (!bmpConnected) {
    return -1;
  }

  sensors_event_t event;
  bmp.getEvent(&event);

  float temperature;
  bmp.getTemperature(&temperature);

  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

  float currentAlt = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature) * METERS_TO_FEET_RATIO;
  return smoothAltitude(currentAlt);
}

// outputs a running average of the last n (ALTITUDE_SMOOTHING) altitudes
float smoothAltitude(float newAlt) {
  // cycle every time this is called
  lastAltitudes[lastAltitudeIndex] = newAlt;
  lastAltitudeIndex++;
  if (lastAltitudeIndex >= ALTITUDE_SMOOTHING) {
    lastAltitudeIndex = 0;
  }

  // get average
  float sum = 0;
  for (int i = 0; i < ALTITUDE_SMOOTHING; i++) {
    sum += lastAltitudes[i];
  }
  return sum / ALTITUDE_SMOOTHING;
}
// this is to check and see if our altimeter is just returning 0s and reset the arduino
void checkAltimeterDataIntegrity() {
  // only check when our altitude is at the end in order to avoid the initial {0,0,0...}
  if (lastAltitudeIndex != ALTITUDE_SMOOTHING) {
    return;
  }
  bool allZeros = true;
  for (int i = 0; i < ALTITUDE_SMOOTHING; i++) {
    if (lastAltitudes[i] != 0.0f) {
      allZeros = false;
    }
  }
  if (allZeros) {
    Serial.println("RESETTING ARDUINO");
    digitalWrite(resetPin, LOW);
  }

}
#endif
