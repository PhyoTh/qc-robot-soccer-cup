/*
  Hiwonder miniAuto control sketch.

  Hardware facts are from Hiwonder's miniAuto examples:
  - Motor PWM pins: D10, D9, D6, D11
  - Motor direction pins: D12, D8, D7, D13
  - Onboard WS2812 RGB data: D2
  - Passive buzzer: D3
  - Servo/gripper: D5
  - Battery divider: A3
  - Glowing ultrasonic sensor: I2C 0x77, distance in millimeters
  - 4-channel line sensor: I2C 0x78, line bits in register 1
*/

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_RouterBridge.h>

#if !defined(ARDUINO_ARCH_ZEPHYR)
#error "This sketch is intentionally targeted to Arduino UNO Q."
#endif

#define HAS_ROUTER_BRIDGE 1
#define CMD_IO Monitor
const char MCU_NAME[] = "uno_q";

// Fixed miniAuto wiring. Keep these in one place so motor/sensor mapping can
// be checked against the board without digging through the control logic.
const uint8_t PIN_RGB = 2;
const uint8_t PIN_BUZZER = 3;
const uint8_t PIN_SERVO = 5;
const uint8_t PIN_BATTERY = A3;

const uint8_t MOTOR_PWM_PIN[4] = {10, 9, 6, 11};
const uint8_t MOTOR_DIR_PIN[4] = {12, 8, 7, 13};
const uint8_t MOTOR_DIR_CANDIDATE_PIN[4] = {12, 8, 7, 13};
const uint8_t HEADER_DIR_SCAN_PIN[] = {2, 3, 4, 5, 7, 8, 12, 13, A0, A1, A2, A3};
const bool MOTOR_POSITIVE_DIR[4] = {true, false, false, true};
const uint8_t MOTOR_PWM_MIN = 2;
const int MAX_DRIVE_MS = 5000;
const int DEFAULT_PULSE_MS = 700;
const int DEFAULT_SPEED = 180;

// Observed with UNO Q on this miniAuto. These labels are used in the serial
// diagnostic commands so students can compare sketch channels to board labels.
// sketch M0/PWM D10 -> board connector M3, forward only with current DIR map
// sketch M1/PWM D9  -> board connector M2, backward only with current DIR map
// sketch M2/PWM D6  -> board connector M1, forward/backward works with DIR D7
// sketch M3/PWM D11 -> board connector M4, forward only with current DIR map
const char MOTOR_BOARD_CONNECTOR[4][3] = {"M3", "M2", "M1", "M4"};

const uint8_t ULTRASONIC_I2C_ADDR = 0x77;
const uint8_t LINE_FOLLOWER_I2C_ADDR = 0x78;
const uint8_t ULTRASONIC_RGB_MODE = 2;
const uint8_t ULTRASONIC_RGB1_R = 3;
const uint8_t ULTRASONIC_RGB_SIMPLE_MODE = 0;

unsigned long driveStopAt = 0;
bool driveTimerActive = false;
uint8_t speedPercent = 55;
bool obstacleAvoidEnabled = false;
unsigned long lastAvoidUpdate = 0;
String commandBuffer;
unsigned long lastCommandByteAt = 0;

// Keep inputs inside the physical/safety ranges expected by the motor and
// timing code. This sketch accepts commands from both Bridge RPC and serial.
int clampInt(int value, int low, int high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint8_t speedToPercent(int speed) {
  speed = abs(speed);
  speed = clampInt(speed, 0, 255);
  return (uint8_t)map(speed, 0, 255, 0, 100);
}

int clampDuration(int durationMs) {
  return clampInt(durationMs, 0, MAX_DRIVE_MS);
}

bool i2cWriteByte(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cWriteData(uint8_t address, uint8_t reg, const uint8_t *values, uint8_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(values[i]);
  }
  return Wire.endTransmission() == 0;
}

int i2cReadData(uint8_t address, uint8_t reg, uint8_t *values, uint8_t len) {
  if (!i2cWriteByte(address, reg)) {
    return -1;
  }

  uint8_t count = 0;
  Wire.requestFrom(address, len);
  while (Wire.available() && count < len) {
    values[count++] = Wire.read();
  }
  return count == len ? count : -1;
}

int readUltrasonicMm() {
  uint8_t bytes[2] = {0, 0};
  if (i2cReadData(ULTRASONIC_I2C_ADDR, 0, bytes, 2) != 2) {
    return -1;
  }
  // Sensor returns little-endian millimeters.
  return (int)(bytes[0] | (bytes[1] << 8));
}

void setUltrasonicColor(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t mode = ULTRASONIC_RGB_SIMPLE_MODE;
  uint8_t rgb[6] = {r, g, b, r, g, b};
  i2cWriteData(ULTRASONIC_I2C_ADDR, ULTRASONIC_RGB_MODE, &mode, 1);
  i2cWriteData(ULTRASONIC_I2C_ADDR, ULTRASONIC_RGB1_R, rgb, 6);
}

bool readLineBits(uint8_t bits[4]) {
  uint8_t data = 0;
  if (i2cReadData(LINE_FOLLOWER_I2C_ADDR, 1, &data, 1) != 1) {
    for (uint8_t i = 0; i < 4; i++) {
      bits[i] = 0;
    }
    return false;
  }

  bits[0] = data & 0x01;
  bits[1] = (data >> 1) & 0x01;
  bits[2] = (data >> 2) & 0x01;
  bits[3] = (data >> 3) & 0x01;
  return true;
}

int readBatteryMv() {
  // Conversion factor comes from the miniAuto battery divider calibration.
  return (int)(analogRead(PIN_BATTERY) * 29.89);
}

void rgbSendBitSlow(bool one) {
  digitalWrite(PIN_RGB, HIGH);
  if (one) {
    delayMicroseconds(1);
  }
  digitalWrite(PIN_RGB, LOW);
  if (!one) {
    delayMicroseconds(1);
  }
}

void rgbSendByteSlow(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    rgbSendBitSlow(value & (1 << bit));
  }
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  // The I2C ultrasonic RGB is the reliable status light on UNO Q. This
  // best-effort WS2812 pulse path is secondary.
  noInterrupts();
  rgbSendByteSlow(g);
  rgbSendByteSlow(r);
  rgbSendByteSlow(b);
  interrupts();
  delayMicroseconds(80);
}

void motorsSetPercent(int motor0, int motor1, int motor2, int motor3) {
  int motors[4] = {
    clampInt(motor0, -100, 100),
    clampInt(motor1, -100, 100),
    clampInt(motor2, -100, 100),
    clampInt(motor3, -100, 100)
  };
  for (uint8_t i = 0; i < 4; i++) {
    // MOTOR_POSITIVE_DIR normalizes each channel so positive percentages mean
    // the same logical wheel direction even when the board wiring differs.
    bool direction = MOTOR_POSITIVE_DIR[i];
    if (motors[i] < 0) {
      direction = !direction;
    }

    uint8_t pwm = 0;
    if (motors[i] != 0) {
      pwm = (uint8_t)map(abs(motors[i]), 0, 100, MOTOR_PWM_MIN, 255);
    }

    digitalWrite(MOTOR_DIR_PIN[i], direction ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN[i], pwm);
  }
}

void stopMotors() {
  motorsSetPercent(0, 0, 0, 0);
  driveTimerActive = false;
}

void pwmOnlySet(uint8_t mask, int8_t reversibleMotor2, uint8_t speed) {
  stopMotors();
  speed = constrain(speed, 0, 255);

  // Fallback diagnostic mode: drive selected PWM channels directly. Only M2 is
  // known to reverse reliably with the current direction wiring.
  for (uint8_t i = 0; i < 4; i++) {
    bool positive = true;
    if (i == 2 && reversibleMotor2 < 0) {
      positive = false;
    }

    bool direction = MOTOR_POSITIVE_DIR[i];
    if (!positive) {
      direction = !direction;
    }
    digitalWrite(MOTOR_DIR_PIN[i], direction ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN[i], (mask & (1 << i)) ? speed : 0);
  }
}

void pwmOnlyCombo(uint8_t mask, int8_t reversibleMotor2, uint16_t durationMs, uint8_t speed) {
  pwmOnlySet(mask, reversibleMotor2, speed);
  if (durationMs > 0) {
    delay(durationMs);
    stopMotors();
  }
}

void velocityController(uint16_t angle, uint8_t velocity, int8_t rot, bool drift) {
  float speedFactor = (rot == 0) ? 1.0 : 0.5;
  float velocityScaled = velocity / sqrt(2.0);
  // The mecanum/omni wheel math treats 0 degrees as forward after the 90 degree
  // offset, then mixes rotation into each wheel.
  float rad = (angle + 90) * PI / 180.0;

  int motor0;
  int motor1;
  int motor2;
  int motor3;
  if (drift) {
    motor0 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor);
    motor1 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor);
    motor2 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor - rot * speedFactor * 2);
    motor3 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor + rot * speedFactor * 2);
  } else {
    motor0 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor + rot * speedFactor);
    motor1 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor - rot * speedFactor);
    motor2 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor - rot * speedFactor);
    motor3 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor + rot * speedFactor);
  }

  motorsSetPercent(motor0, motor1, motor2, motor3);
}

void armDriveTimer(int durationMs) {
  durationMs = clampDuration(durationMs);
  if (durationMs > 0) {
    // A nonzero duration makes the robot stop itself even if the caller does
    // not send a later stop command.
    driveStopAt = millis() + (unsigned long)durationMs;
    driveTimerActive = true;
  } else {
    driveTimerActive = false;
  }
}

void driveRaw(int motor0, int motor1, int motor2, int motor3, int durationMs) {
  motorsSetPercent(
    (int)map(clampInt(motor0, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor1, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor2, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor3, -255, 255), -255, 255, -100, 100)
  );
  armDriveTimer(durationMs);
}

bool driveCommand(String command, int speed, int durationMs) {
  command.trim();
  command.toLowerCase();
  const uint8_t percent = speedToPercent(speed);
  speedPercent = percent;

  // Human-friendly aliases are accepted for both serial and Bridge callers.
  if (command == "stop" || command == "x") {
    stopMotors();
    return true;
  }
  if (command == "forward" || command == "f") {
    velocityController(0, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "backward" || command == "back" || command == "reverse" || command == "b") {
    velocityController(180, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "left" || command == "strafe_left" || command == "a") {
    velocityController(90, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "right" || command == "strafe_right" || command == "d") {
    velocityController(270, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "rotate_left" || command == "turn_left" || command == "q") {
    velocityController(0, 0, percent, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "rotate_right" || command == "turn_right" || command == "e") {
    velocityController(0, 0, -percent, false);
    armDriveTimer(durationMs);
    return true;
  }

  return false;
}

void servoPulse(uint16_t pulseUs) {
  // Minimal software servo pulse. Repeated pulses in setServoAngle give the
  // gripper enough time to move without needing a Servo library dependency.
  digitalWrite(PIN_SERVO, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(PIN_SERVO, LOW);
  delayMicroseconds(20000 - pulseUs);
}

bool setServoAngle(int angle) {
  angle = clampInt(angle, 0, 180);
  uint16_t pulseUs = (uint16_t)map(angle, 0, 180, 1000, 2000);
  for (uint8_t i = 0; i < 30; i++) {
    servoPulse(pulseUs);
  }
  return true;
}

void chirp() {
  for (uint8_t i = 0; i < 3; i++) {
    unsigned long endAt = millis() + 100;
    while (millis() < endAt) {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(250);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(250);
    }
    delay(80);
  }
  digitalWrite(PIN_BUZZER, LOW);
}

String normalized(String input) {
  // Accept friendly shell-style commands plus Hiwonder's pipe-delimited format
  // by turning separators into spaces before tokenizing.
  input.replace(',', ' ');
  input.replace('|', ' ');
  input.replace('(', ' ');
  input.replace(')', ' ');
  input.replace('&', ' ');
  input.trim();
  return input;
}

String tokenAt(String input, uint8_t wanted) {
  input = normalized(input);
  uint8_t index = 0;
  int start = 0;

  while (start < input.length()) {
    while (start < input.length() && input[start] == ' ') {
      start++;
    }
    int end = start;
    while (end < input.length() && input[end] != ' ') {
      end++;
    }
    if (end > start) {
      if (index == wanted) {
        return input.substring(start, end);
      }
      index++;
    }
    start = end + 1;
  }

  return "";
}

String readSensorsJson() {
  uint8_t lineBits[4];
  bool lineOk = readLineBits(lineBits);
  int distanceMm = readUltrasonicMm();
  int distanceCm = distanceMm >= 0 ? distanceMm / 10 : distanceMm;
  int batteryMv = readBatteryMv();

  String json = "{";
  // Keep the payload small and JSON-shaped so Python callers can parse it with
  // json.loads() while serial users can still read it directly.
  json += "\"robot\":\"hiwonder_miniauto\"";
  json += ",\"mcu\":\"";
  json += MCU_NAME;
  json += "\"";
  json += ",\"ir\":-1";
  json += ",\"line_ok\":";
  json += lineOk ? "true" : "false";
  json += ",\"line_digital\":[";
  json += lineBits[0];
  json += ",";
  json += lineBits[1];
  json += ",";
  json += lineBits[2];
  json += ",";
  json += lineBits[3];
  json += "]";
  json += ",\"trace_digital\":[";
  json += lineBits[0];
  json += ",";
  json += lineBits[1];
  json += ",";
  json += lineBits[2];
  json += ",";
  json += lineBits[3];
  json += "]";
  json += ",\"ultrasonic_mm\":";
  json += distanceMm;
  json += ",\"ultrasonic_cm\":";
  json += distanceCm;
  json += ",\"battery_mv\":";
  json += batteryMv;
  json += "}";
  return json;
}

#if HAS_ROUTER_BRIDGE
// Bridge RPC methods mirror the Python MiniAutoRobot client in python/.
bool rpcDrive(String command, int speed, int durationMs) {
  return driveCommand(command, speed, durationMs);
}

bool rpcStop() {
  stopMotors();
  obstacleAvoidEnabled = false;
  return true;
}

String rpcReadSensors() {
  return readSensorsJson();
}

bool rpcServo(int angle) {
  return setServoAngle(angle);
}

bool rpcBuzz() {
  chirp();
  return true;
}

bool rpcLed(bool on) {
  setRgb(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
  setUltrasonicColor(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
  return true;
}

bool rpcDriveRaw(int motor0, int motor1, int motor2, int motor3, int durationMs) {
  driveRaw(motor0, motor1, motor2, motor3, durationMs);
  return true;
}

String rpcHealth() {
  String json = "{\"robot\":\"hiwonder_miniauto\",\"mcu\":\"";
  json += MCU_NAME;
  json += "\",\"bridge\":true,\"serial\":true}";
  return json;
}

void registerBridgeMethods() {
  // provide_safe exposes typed calls to the Python side through
  // Arduino_RouterBridge.
  Bridge.provide_safe("drive", rpcDrive);
  Bridge.provide_safe("stop", rpcStop);
  Bridge.provide_safe("read_sensors", rpcReadSensors);
  Bridge.provide_safe("servo", rpcServo);
  Bridge.provide_safe("buzz", rpcBuzz);
  Bridge.provide_safe("led", rpcLed);
  Bridge.provide_safe("drive_raw", rpcDriveRaw);
  Bridge.provide_safe("health", rpcHealth);
}
#endif

void motorPulse(uint8_t motorIndex) {
  // Simple per-channel hardware test used by serial commands 1..4.
  int motors[4] = {0, 0, 0, 0};
  motors[motorIndex] = 100;
  motorsSetPercent(motors[0], motors[1], motors[2], motors[3]);
  delay(550);
  motors[motorIndex] = -100;
  motorsSetPercent(motors[0], motors[1], motors[2], motors[3]);
  delay(550);
  stopMotors();
}

void setAllDirCandidates(uint8_t value) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(MOTOR_DIR_CANDIDATE_PIN[i], value);
  }
}

void setAllScanPins(uint8_t value) {
  for (uint8_t i = 0; i < sizeof(HEADER_DIR_SCAN_PIN) / sizeof(HEADER_DIR_SCAN_PIN[0]); i++) {
    digitalWrite(HEADER_DIR_SCAN_PIN[i], value);
  }
}

void pulsePwmOnly(uint8_t motorIndex, uint16_t durationMs) {
  analogWrite(MOTOR_PWM_PIN[motorIndex], 180);
  delay(durationMs);
  analogWrite(MOTOR_PWM_PIN[motorIndex], 0);
  delay(250);
}

void directionSweep(uint8_t motorIndex) {
  if (motorIndex > 3) {
    CMD_IO.println(F("ERR dir_sweep motor index must be 0..3"));
    return;
  }

  stopMotors();
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_DIR_CANDIDATE_PIN[i], OUTPUT);
  }

  CMD_IO.print(F("DIR sweep for sketch M"));
  CMD_IO.print(motorIndex);
  CMD_IO.print(F(" / board "));
  CMD_IO.print(MOTOR_BOARD_CONNECTOR[motorIndex]);
  CMD_IO.print(F(" / PWM D"));
  CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
  CMD_IO.println(F("Watch which D pin makes this motor reverse vs LOW baseline."));

  // Compare a LOW baseline with each candidate direction pin driven HIGH.
  for (uint8_t i = 0; i < 4; i++) {
    setAllDirCandidates(LOW);
    CMD_IO.print(F("baseline all DIR LOW, PWM D"));
    CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
    pulsePwmOnly(motorIndex, 500);

    setAllDirCandidates(LOW);
    digitalWrite(MOTOR_DIR_CANDIDATE_PIN[i], HIGH);
    CMD_IO.print(F("candidate DIR D"));
    CMD_IO.print(MOTOR_DIR_CANDIDATE_PIN[i]);
    CMD_IO.print(F(" HIGH, PWM D"));
    CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
    pulsePwmOnly(motorIndex, 700);
  }

  setAllDirCandidates(LOW);
  stopMotors();
  CMD_IO.println(F("DIR sweep done."));
}

void directionHeaderScan(uint8_t motorIndex) {
  if (motorIndex > 3) {
    CMD_IO.println(F("ERR dir_scan motor index must be 0..3"));
    return;
  }

  stopMotors();
  for (uint8_t i = 0; i < sizeof(HEADER_DIR_SCAN_PIN) / sizeof(HEADER_DIR_SCAN_PIN[0]); i++) {
    pinMode(HEADER_DIR_SCAN_PIN[i], OUTPUT);
  }

  CMD_IO.print(F("Header DIR scan for sketch M"));
  CMD_IO.print(motorIndex);
  CMD_IO.print(F(" / board "));
  CMD_IO.print(MOTOR_BOARD_CONNECTOR[motorIndex]);
  CMD_IO.print(F(" / PWM D"));
  CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
  CMD_IO.println(F("Watch for any candidate pin that reverses this motor."));
  CMD_IO.println(F("Non-PWM scan pins: D2,D3,D4,D5,D7,D8,D12,D13,A0,A1,A2,A3"));

  // Broader scan for boards whose direction pins are not on the expected
  // header pins.
  for (uint8_t i = 0; i < sizeof(HEADER_DIR_SCAN_PIN) / sizeof(HEADER_DIR_SCAN_PIN[0]); i++) {
    setAllScanPins(LOW);
    CMD_IO.print(F("baseline scan pins LOW, PWM D"));
    CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
    pulsePwmOnly(motorIndex, 450);

    setAllScanPins(LOW);
    digitalWrite(HEADER_DIR_SCAN_PIN[i], HIGH);
    CMD_IO.print(F("candidate "));
    if (HEADER_DIR_SCAN_PIN[i] >= A0) {
      CMD_IO.print(F("A"));
      CMD_IO.print(HEADER_DIR_SCAN_PIN[i] - A0);
    } else {
      CMD_IO.print(F("D"));
      CMD_IO.print(HEADER_DIR_SCAN_PIN[i]);
    }
    CMD_IO.print(F(" HIGH, PWM D"));
    CMD_IO.println(MOTOR_PWM_PIN[motorIndex]);
    pulsePwmOnly(motorIndex, 650);
  }

  setAllScanPins(LOW);
  stopMotors();
  CMD_IO.println(F("Header DIR scan done."));
}

void comboScan() {
  stopMotors();
  CMD_IO.println(F("PWM-only combo scan."));
  CMD_IO.println(F("Mask bits: 1=M0/boardM3, 2=M1/boardM2, 4=M2/boardM1, 8=M3/boardM4."));
  CMD_IO.println(F("Report masks that move: forward, left-turn, right-turn, or usable wobble."));

  // Try every PWM channel mask so a usable fallback movement can be found even
  // when direction wiring is partially unknown.
  for (int8_t m2Dir = 1; m2Dir >= -1; m2Dir -= 2) {
    CMD_IO.print(F("M2 direction "));
    CMD_IO.println(m2Dir > 0 ? F("positive") : F("negative"));
    for (uint8_t mask = 1; mask < 16; mask++) {
      CMD_IO.print(F("combo mask="));
      CMD_IO.print(mask);
      CMD_IO.print(F(" m2dir="));
      CMD_IO.println(m2Dir);
      pwmOnlyCombo(mask, m2Dir, 450, 165);
      delay(350);
    }
  }

  stopMotors();
  CMD_IO.println(F("PWM-only combo scan done."));
}

void printHelp() {
  CMD_IO.println();
  CMD_IO.println(F("Hiwonder miniAuto commands:"));
  CMD_IO.println(F("  ? help"));
  CMD_IO.println(F("  r read sensors"));
  CMD_IO.println(F("  l RGB blink"));
  CMD_IO.println(F("  z buzzer chirp"));
  CMD_IO.println(F("  u ultrasonic distance"));
  CMD_IO.println(F("  v servo center-open-close-center"));
  CMD_IO.println(F("  1..4 motor tests: 1=M0/board M3, 2=M1/board M2, 3=M2/board M1, 4=M3/board M4"));
  CMD_IO.println(F("  dir_sweep(0..3) find which DIR pin reverses a PWM/motor channel"));
  CMD_IO.println(F("  dir_scan(0..3) broad non-PWM header DIR scan"));
  CMD_IO.println(F("  combo_scan() discover PWM-only fallback movements"));
  CMD_IO.println(F("  pwm_combo(mask,m2dir,ms,speed) run PWM-only fallback combo"));
  CMD_IO.println(F("  f/b/a/d/q/e/x forward/back/left/right/rotate-left/rotate-right/stop"));
  CMD_IO.println(F("Line API: drive(command,speed,ms), stop, read_sensors, servo(angle), buzz, led(on), rgb(r,g,b), drive_raw(m0,m1,m2,m3,ms), health"));
  CMD_IO.println(F("Hiwonder protocol also works: A|2|&, B|255|0|0|&, C|50|&, D|&, E|30|&, F|0|&"));
  CMD_IO.println();
}

void handleSingleCommand(char command) {
  switch (command) {
    case '?':
      printHelp();
      break;
    case 'r':
      CMD_IO.println(readSensorsJson());
      break;
    case 'l':
      setRgb(255, 0, 0);
      setUltrasonicColor(255, 0, 0);
      delay(180);
      setRgb(0, 255, 0);
      setUltrasonicColor(0, 255, 0);
      delay(180);
      setRgb(0, 0, 255);
      setUltrasonicColor(0, 0, 255);
      delay(180);
      setRgb(0, 0, 0);
      setUltrasonicColor(0, 0, 0);
      CMD_IO.println(F("OK led"));
      break;
    case 'z':
      chirp();
      CMD_IO.println(F("OK buzz"));
      break;
    case 'u':
      CMD_IO.print(F("ultrasonic_mm="));
      CMD_IO.println(readUltrasonicMm());
      break;
    case 'v':
      setServoAngle(90);
      setServoAngle(150);
      setServoAngle(30);
      setServoAngle(90);
      CMD_IO.println(F("OK servo"));
      break;
    case '1':
      motorPulse(0);
      CMD_IO.println(F("OK M0 front-left"));
      break;
    case '2':
      motorPulse(1);
      CMD_IO.println(F("OK M1 front-right"));
      break;
    case '3':
      motorPulse(2);
      CMD_IO.println(F("OK M2 rear-right"));
      break;
    case '4':
      motorPulse(3);
      CMD_IO.println(F("OK M3 rear-left"));
      break;
    case 'f':
      driveCommand("forward", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK forward"));
      break;
    case 'b':
      driveCommand("backward", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK backward"));
      break;
    case 'a':
      driveCommand("left", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK left"));
      break;
    case 'd':
      driveCommand("right", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK right"));
      break;
    case 'q':
      driveCommand("rotate_left", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK rotate_left"));
      break;
    case 'e':
      driveCommand("rotate_right", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK rotate_right"));
      break;
    case 'x':
      stopMotors();
      obstacleAvoidEnabled = false;
      CMD_IO.println(F("OK stop"));
      break;
    default:
      CMD_IO.print(F("ERR unknown command "));
      CMD_IO.println(command);
      break;
  }
}

void handleHiwonderProtocol(String line) {
  String function = tokenAt(line, 0);
  function.toUpperCase();

  // Compatibility mode for Hiwonder examples:
  // A=motion, B=RGB, C=speed, D=sensors, E=servo, F=obstacle avoidance.
  if (function == "A") {
    uint8_t state = (uint8_t)tokenAt(line, 1).toInt();
    switch (state) {
      case 0: velocityController(90, speedPercent, 0, false); break;
      case 1: velocityController(45, speedPercent, 0, false); break;
      case 2: velocityController(0, speedPercent, 0, false); break;
      case 3: velocityController(315, speedPercent, 0, false); break;
      case 4: velocityController(270, speedPercent, 0, false); break;
      case 5: velocityController(225, speedPercent, 0, false); break;
      case 6: velocityController(180, speedPercent, 0, false); break;
      case 7: velocityController(135, speedPercent, 0, false); break;
      case 8: stopMotors(); break;
      case 9: velocityController(0, 0, speedPercent, false); break;
      case 10: velocityController(0, 0, -speedPercent, false); break;
      case 11: stopMotors(); break;
      default: stopMotors(); break;
    }
    CMD_IO.println(F("A|OK|$"));
    return;
  }

  if (function == "B") {
    uint8_t r = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 0, 255);
    uint8_t g = (uint8_t)clampInt(tokenAt(line, 2).toInt(), 0, 255);
    uint8_t b = (uint8_t)clampInt(tokenAt(line, 3).toInt(), 0, 255);
    setRgb(r, g, b);
    setUltrasonicColor(r, g, b);
    CMD_IO.println(F("B|OK|$"));
    return;
  }

  if (function == "C") {
    speedPercent = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 10, 100);
    CMD_IO.print(F("C|"));
    CMD_IO.print(speedPercent);
    CMD_IO.println(F("|$"));
    return;
  }

  if (function == "D") {
    CMD_IO.print(F("$"));
    CMD_IO.print(readUltrasonicMm());
    CMD_IO.print(F(","));
    CMD_IO.print(readBatteryMv());
    CMD_IO.println(F("$"));
    return;
  }

  if (function == "E") {
    int increase = clampInt(tokenAt(line, 1).toInt(), 0, 60);
    setServoAngle(90 + increase);
    CMD_IO.println(F("E|OK|$"));
    return;
  }

  if (function == "F") {
    obstacleAvoidEnabled = tokenAt(line, 1).toInt() != 0;
    if (!obstacleAvoidEnabled) {
      stopMotors();
    }
    CMD_IO.println(F("F|OK|$"));
    return;
  }

  CMD_IO.println(F("ERR hiwonder protocol"));
}

void handleLineCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  // Lines ending in '&' are treated as Hiwonder protocol commands such as
  // A|2|&. Everything else uses this sketch's plain text command API.
  if (line.endsWith("&")) {
    handleHiwonderProtocol(line);
    return;
  }

  if (line.length() == 1) {
    handleSingleCommand(line[0]);
    return;
  }

  String command = tokenAt(line, 0);
  command.toLowerCase();

  if (command == "drive") {
    String direction = tokenAt(line, 1);
    int speed = tokenAt(line, 2).length() ? tokenAt(line, 2).toInt() : DEFAULT_SPEED;
    int duration = tokenAt(line, 3).length() ? tokenAt(line, 3).toInt() : 0;
    CMD_IO.println(driveCommand(direction, speed, duration) ? F("OK drive") : F("ERR drive"));
    return;
  }

  if (command == "stop") {
    stopMotors();
    obstacleAvoidEnabled = false;
    CMD_IO.println(F("OK stop"));
    return;
  }

  if (command == "read_sensors" || command == "sensors") {
    CMD_IO.println(readSensorsJson());
    return;
  }

  if (command == "servo") {
    setServoAngle(tokenAt(line, 1).toInt());
    CMD_IO.println(F("OK servo"));
    return;
  }

  if (command == "buzz") {
    chirp();
    CMD_IO.println(F("OK buzz"));
    return;
  }

  if (command == "led") {
    bool on = tokenAt(line, 1).toInt() != 0;
    setRgb(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
    setUltrasonicColor(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
    CMD_IO.println(F("OK led"));
    return;
  }

  if (command == "rgb") {
    uint8_t r = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 0, 255);
    uint8_t g = (uint8_t)clampInt(tokenAt(line, 2).toInt(), 0, 255);
    uint8_t b = (uint8_t)clampInt(tokenAt(line, 3).toInt(), 0, 255);
    setRgb(r, g, b);
    setUltrasonicColor(r, g, b);
    CMD_IO.println(F("OK rgb"));
    return;
  }

  if (command == "drive_raw") {
    int m0 = tokenAt(line, 1).toInt();
    int m1 = tokenAt(line, 2).toInt();
    int m2 = tokenAt(line, 3).toInt();
    int m3 = tokenAt(line, 4).toInt();
    int duration = tokenAt(line, 5).length() ? tokenAt(line, 5).toInt() : 0;
    driveRaw(m0, m1, m2, m3, duration);
    CMD_IO.println(F("OK drive_raw"));
    return;
  }

  if (command == "dir_sweep" || command == "dirsweep") {
    directionSweep((uint8_t)tokenAt(line, 1).toInt());
    return;
  }

  if (command == "dir_scan" || command == "dirscan") {
    directionHeaderScan((uint8_t)tokenAt(line, 1).toInt());
    return;
  }

  if (command == "combo_scan" || command == "comboscan") {
    comboScan();
    return;
  }

  if (command == "pwm_combo" || command == "pwmcombo") {
    uint8_t mask = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 0, 15);
    int8_t m2dir = tokenAt(line, 2).toInt() < 0 ? -1 : 1;
    uint16_t duration = tokenAt(line, 3).length() ? (uint16_t)clampInt(tokenAt(line, 3).toInt(), 0, MAX_DRIVE_MS) : DEFAULT_PULSE_MS;
    uint8_t speed = tokenAt(line, 4).length() ? (uint8_t)clampInt(tokenAt(line, 4).toInt(), 0, 255) : DEFAULT_SPEED;
    pwmOnlyCombo(mask, m2dir, duration, speed);
    CMD_IO.println(F("OK pwm_combo"));
    return;
  }

  if (command == "health") {
    String json = "{\"robot\":\"hiwonder_miniauto\",\"mcu\":\"";
    json += MCU_NAME;
    json += "\",\"serial\":true";
#if HAS_ROUTER_BRIDGE
    json += ",\"bridge\":true";
#endif
    json += "}";
    CMD_IO.println(json);
    return;
  }

  CMD_IO.println(F("ERR unknown line command"));
}

bool isSingleCommandChar(char command) {
  switch (command) {
    case '?':
    case 'r':
    case 'l':
    case 'z':
    case 'u':
    case 'v':
    case '1':
    case '2':
    case '3':
    case '4':
    case 'f':
    case 'b':
    case 'a':
    case 'd':
    case 'q':
    case 'e':
    case 'x':
      return true;
    default:
      return false;
  }
}

void pollSerial() {
  bool received = false;

  // Accept newline-terminated commands, single-key commands, and short commands
  // without a newline by flushing the buffer after a brief idle timeout.
  while (CMD_IO.available()) {
    char incoming = (char)CMD_IO.read();
    received = true;
    lastCommandByteAt = millis();

    if (incoming == '\r' || incoming == '\n') {
      if (commandBuffer.length() > 0) {
        handleLineCommand(commandBuffer);
        commandBuffer = "";
      }
      continue;
    }

    if (commandBuffer.length() == 0 && isSingleCommandChar(incoming) && CMD_IO.available() == 0) {
      handleSingleCommand(incoming);
      continue;
    }

    commandBuffer += incoming;
    if (incoming == '&' || incoming == ')' || commandBuffer.length() >= 96) {
      handleLineCommand(commandBuffer);
      commandBuffer = "";
    }
  }

  if (!received && commandBuffer.length() > 0 && millis() - lastCommandByteAt > 80) {
    handleLineCommand(commandBuffer);
    commandBuffer = "";
  }
}

void updateDriveTimer() {
  if (driveTimerActive && (long)(millis() - driveStopAt) >= 0) {
    stopMotors();
  }
}

void updateObstacleAvoid() {
  if (!obstacleAvoidEnabled || millis() - lastAvoidUpdate < 100) {
    return;
  }

  lastAvoidUpdate = millis();
  int distanceMm = readUltrasonicMm();
  // Basic demo behavior: rotate away from close objects, otherwise move
  // forward at the currently selected speed.
  if (distanceMm > 0 && distanceMm < 400) {
    velocityController(0, 0, speedPercent, false);
  } else {
    velocityController(0, speedPercent, 0, false);
  }
}

void setupPins() {
  pinMode(PIN_RGB, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_PWM_PIN[i], OUTPUT);
    pinMode(MOTOR_DIR_PIN[i], OUTPUT);
  }

  digitalWrite(PIN_BUZZER, LOW);
  stopMotors();
  setRgb(0, 0, 0);
}

void setup() {
  CMD_IO.begin(9600);
  CMD_IO.setTimeout(80);
  Wire.begin();
  setupPins();
  setUltrasonicColor(0, 0, 0);
#if HAS_ROUTER_BRIDGE
  if (Bridge.begin()) {
    registerBridgeMethods();
  }
#endif
  CMD_IO.println(F("Hiwonder miniAuto command sketch ready."));
  printHelp();
}

void loop() {
  pollSerial();
  updateDriveTimer();
  updateObstacleAvoid();
}
