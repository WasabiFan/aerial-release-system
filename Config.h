#pragma once

#define METERS_TO_FEET_RATIO (1250./381.)

#define ROLE_GROUND 0
#define ROLE_SKY 1

// SET THE TARGET ROLE HERE -------------------------------
#define ROLE ROLE_GROUND

#if ROLE != ROLE_SKY && ROLE != ROLE_GROUND
#error "ROLE must be ROLE_SKY or ROLE_GROUND"
#endif

// time in between "heartbeat" signals - determine connection health and recieve back altitude
#define HEARTBEAT_INTERVAL_MILLIS 300
// The threshold when the indicator LED changes
#define ALTITUDE_TARGET_THRESH_FEET 100

// Angle when servo is in "trigger" position
#define TRIGGER_SERVO_ANGLE 90
// Angle when servo is in reset/ready position
#define RESET_SERVO_ANGLE 0
// Number of altitude samples to average
#define ALTITUDE_SMOOTHING 10

// Time (in ms) that the indicator LED will flash for when a command is successfully registered
#define COMMAND_SIG_DURATION 100