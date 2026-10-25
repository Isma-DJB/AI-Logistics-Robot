#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>

// ============================================================
// I-1.0 SUPERVISED SERIAL FIRMWARE
// Controlled wheels-lifted bounded-forward validation stage.
// ============================================================
// Movement is limited to one supervised 250 ms command.

const char PROTOCOL_NAME[] = "ALR";
const char PROTOCOL_VERSION[] = "1";
const char FIRMWARE_VERSION[] = "i-1.0-bounded-forward-wheels-lifted";

const unsigned long MAX_SEQUENCE_ID = 2147483647UL;

const unsigned long SERIAL_BAUD_RATE = 115200UL;
const size_t REQUEST_BUFFER_SIZE = 96;
const int MAX_TOKENS = 6;

// ============================================================
// MOTOR PINS VALIDATED DURING I-0.9
// ============================================================

const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

const int MOTOR_OFFSET_MIN_PERCENT = 50;
const int MOTOR_OFFSET_MAX_PERCENT = 100;

// These are the physically validated I-0.9 forward values.
// They are used only inside the supervised bounded command.
const int FORWARD_MOTION_SPEED = 120;
const int FORWARD_RESTART_SPEED = 140;

const unsigned long FORWARD_DIRECTION_PAUSE_MS = 80UL;
const unsigned long FORWARD_RESTART_MS = 100UL;
const unsigned long BOUNDED_FORWARD_DURATION_MS = 250UL;
const unsigned long ULTRASONIC_SAMPLE_INTERVAL_MS = 50UL;
const unsigned long FRONTAL_OBSTACLE_DISTANCE_MM = 200UL;

// Enabled only for the controlled wheels-lifted validation stage.
const bool MOTION_EXECUTION_ENABLED = true;

float motorAOffset = 1.0F;
float motorBOffset = 1.0F;

// ============================================================
// SENSOR PINS VALIDATED DURING I-0.9
// ============================================================

const int ECHO_PIN = 4;
const int TRIG_PIN = 3;
const int RIGHT_IR_PIN = 7;
const int LEFT_IR_PIN = 8;
const int LINE_SENSOR_PIN = 2;
const int LDR_PIN = A0;

const int LIGHT_SAMPLE_COUNT = 10;
const int LIGHT_SAMPLE_DELAY_MS = 2;

const unsigned long ULTRASONIC_TIMEOUT_MICROSECONDS =
    30000UL;

enum class MotionPhase : uint8_t {
  IDLE,
  DIRECTION_PAUSE,
  RESTART_BOOST,
  CRUISE
};

// The controller starts latched after every boot.
bool safetyLatched = true;

bool motionExecuting = false;
unsigned long activeMotionSequenceId = 0;
unsigned long motionEndDeadlineMs = 0;
unsigned long motionPhaseDeadlineMs = 0;
unsigned long nextUltrasonicSampleMs = 0;
MotionPhase motionPhase = MotionPhase::IDLE;

char serialBuffer[REQUEST_BUFFER_SIZE];
size_t serialLength = 0;
bool serialOverflow = false;

// ============================================================
// MOTOR SAFETY
// ============================================================

void stopMotors() {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void loadMotorOffsets() {
  const byte savedA = EEPROM.read(0);
  const byte savedB = EEPROM.read(1);

  if (
      savedA >= MOTOR_OFFSET_MIN_PERCENT
      && savedA <= MOTOR_OFFSET_MAX_PERCENT
  ) {
    motorAOffset = savedA / 100.0F;
  } else {
    motorAOffset = 1.0F;
  }

  if (
      savedB >= MOTOR_OFFSET_MIN_PERCENT
      && savedB <= MOTOR_OFFSET_MAX_PERCENT
  ) {
    motorBOffset = savedB / 100.0F;
  } else {
    motorBOffset = 1.0F;
  }
}

int applyMotorOffset(
    int speed,
    float offset
) {
  const int scaledSpeed =
      static_cast<int>(speed * offset);

  return constrain(
      scaledSpeed,
      0,
      255
  );
}

void writeForwardOutputs(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(
      A_1A,
      applyMotorOffset(
          speed,
          motorBOffset
      )
  );
  analogWrite(
      B_1B,
      applyMotorOffset(
          speed,
          motorAOffset
      )
  );
  analogWrite(B_1A, 0);
}

bool readUltrasonicDistanceMillimeters(
    unsigned long& distanceMillimeters
) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long durationMicroseconds = pulseIn(
      ECHO_PIN,
      HIGH,
      ULTRASONIC_TIMEOUT_MICROSECONDS
  );

  if (durationMicroseconds == 0) {
    return false;
  }

  distanceMillimeters =
      durationMicroseconds * 10UL / 58UL;
  return true;
}

int readAverageLightRaw() {
  long total = 0;

  for (
      int sampleIndex = 0;
      sampleIndex < LIGHT_SAMPLE_COUNT;
      sampleIndex++
  ) {
    total += analogRead(LDR_PIN);
    delay(LIGHT_SAMPLE_DELAY_MS);
  }

  return static_cast<int>(
      total / LIGHT_SAMPLE_COUNT
  );
}

void sendSensorSnapshot(unsigned long sequenceId) {
  // Sensor inspection is read-only and must never activate a motor.
  stopMotors();

  const int leftIrRaw = digitalRead(LEFT_IR_PIN);
  const int rightIrRaw = digitalRead(RIGHT_IR_PIN);
  const int lineRaw = digitalRead(LINE_SENSOR_PIN);

  unsigned long distanceMillimeters = 0;
  const bool distanceAvailable =
      readUltrasonicDistanceMillimeters(
          distanceMillimeters
      );

  const int lightRaw = readAverageLightRaw();

  stopMotors();

  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|SENSORS|");
  Serial.print(sequenceId);
  Serial.print('|');
  Serial.print(leftIrRaw);
  Serial.print('|');
  Serial.print(rightIrRaw);
  Serial.print('|');
  Serial.print(lineRaw);
  Serial.print('|');

  if (distanceAvailable) {
    Serial.print(distanceMillimeters);
  } else {
    Serial.print("NONE");
  }

  Serial.print('|');
  Serial.println(lightRaw);
}

// ============================================================
// PROTOCOL RESPONSES
// ============================================================

void sendReady() {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|READY|");
  Serial.println(FIRMWARE_VERSION);
}

void sendPong(unsigned long sequenceId) {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|PONG|");
  Serial.println(sequenceId);
}

void sendAcknowledgement(
    unsigned long sequenceId,
    const char* commandType
) {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|ACK|");
  Serial.print(sequenceId);
  Serial.print('|');
  Serial.println(commandType);
}

void sendCompletion(
    unsigned long sequenceId,
    const char* completionCode
) {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|DONE|");
  Serial.print(sequenceId);
  Serial.print('|');
  Serial.println(completionCode);
}

void sendStatus(unsigned long sequenceId) {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|STATUS|");
  Serial.print(sequenceId);
  Serial.print('|');

  if (safetyLatched) {
    Serial.println("ESTOPPED|LATCHED");
  } else if (motionExecuting) {
    Serial.println("EXECUTING|SAFE");
  } else {
    Serial.println("IDLE|SAFE");
  }
}

void sendError(
    unsigned long sequenceId,
    const char* errorCode
) {
  Serial.print(PROTOCOL_NAME);
  Serial.print('|');
  Serial.print(PROTOCOL_VERSION);
  Serial.print("|ERROR|");
  Serial.print(sequenceId);
  Serial.print('|');
  Serial.println(errorCode);
}

// ============================================================
// BOUNDED FORWARD-MOTION STATE MACHINE
// ============================================================

bool deadlineReached(
    unsigned long currentTimeMs,
    unsigned long deadlineMs
) {
  return static_cast<long>(
      currentTimeMs - deadlineMs
  ) >= 0;
}

void resetActiveMotionState() {
  motionExecuting = false;
  activeMotionSequenceId = 0;
  motionEndDeadlineMs = 0;
  motionPhaseDeadlineMs = 0;
  nextUltrasonicSampleMs = 0;
  motionPhase = MotionPhase::IDLE;
}

void completeActiveMotion(
    const char* completionCode,
    bool latchSafety
) {
  const unsigned long completedSequenceId =
      activeMotionSequenceId;

  stopMotors();
  resetActiveMotionState();

  if (latchSafety) {
    safetyLatched = true;
  }

  sendCompletion(
      completedSequenceId,
      completionCode
  );
}

void startBoundedForwardMotion(
    unsigned long sequenceId
) {
  if (!MOTION_EXECUTION_ENABLED) {
    stopMotors();
    resetActiveMotionState();
    sendError(sequenceId, "BAD_COMMAND");
    return;
  }

  if (safetyLatched) {
    stopMotors();
    sendError(sequenceId, "SAFETY_LATCHED");
    return;
  }

  if (motionExecuting) {
    sendError(sequenceId, "BUSY");
    return;
  }

  const unsigned long currentTimeMs = millis();

  stopMotors();

  motionExecuting = true;
  activeMotionSequenceId = sequenceId;
  motionEndDeadlineMs =
      currentTimeMs + BOUNDED_FORWARD_DURATION_MS;
  motionPhaseDeadlineMs =
      currentTimeMs + FORWARD_DIRECTION_PAUSE_MS;
  nextUltrasonicSampleMs = currentTimeMs;
  motionPhase = MotionPhase::DIRECTION_PAUSE;

  sendAcknowledgement(
      sequenceId,
      "MOVE_FORWARD"
  );
}

bool frontalObstacleDetected(
    unsigned long currentTimeMs
) {
  if (
      digitalRead(LEFT_IR_PIN) == LOW
      || digitalRead(RIGHT_IR_PIN) == LOW
  ) {
    return true;
  }

  if (
      !deadlineReached(
          currentTimeMs,
          nextUltrasonicSampleMs
      )
  ) {
    return false;
  }

  nextUltrasonicSampleMs =
      currentTimeMs + ULTRASONIC_SAMPLE_INTERVAL_MS;

  unsigned long distanceMillimeters = 0;

  if (
      !readUltrasonicDistanceMillimeters(
          distanceMillimeters
      )
  ) {
    return false;
  }

  return (
      distanceMillimeters
      <= FRONTAL_OBSTACLE_DISTANCE_MM
  );
}

void serviceActiveMotion() {
  if (!MOTION_EXECUTION_ENABLED) {
    stopMotors();
    resetActiveMotionState();
    return;
  }

  if (!motionExecuting) {
    stopMotors();
    return;
  }

  if (safetyLatched) {
    completeActiveMotion(
        "EMERGENCY_STOP",
        true
    );
    return;
  }

  if (digitalRead(LINE_SENSOR_PIN) == HIGH) {
    completeActiveMotion(
        "EMERGENCY_STOP",
        true
    );
    return;
  }

  const unsigned long currentTimeMs = millis();

  if (frontalObstacleDetected(currentTimeMs)) {
    completeActiveMotion(
        "BLOCKED",
        false
    );
    return;
  }

  if (
      deadlineReached(
          currentTimeMs,
          motionEndDeadlineMs
      )
  ) {
    completeActiveMotion(
        "OK",
        false
    );
    return;
  }

  if (
      motionPhase == MotionPhase::DIRECTION_PAUSE
      && deadlineReached(
          currentTimeMs,
          motionPhaseDeadlineMs
      )
  ) {
    writeForwardOutputs(FORWARD_RESTART_SPEED);
    motionPhase = MotionPhase::RESTART_BOOST;
    motionPhaseDeadlineMs =
        currentTimeMs + FORWARD_RESTART_MS;
    return;
  }

  if (
      motionPhase == MotionPhase::RESTART_BOOST
      && deadlineReached(
          currentTimeMs,
          motionPhaseDeadlineMs
      )
  ) {
    writeForwardOutputs(FORWARD_MOTION_SPEED);
    motionPhase = MotionPhase::CRUISE;
    return;
  }

  if (
      motionPhase != MotionPhase::DIRECTION_PAUSE
      && motionPhase != MotionPhase::RESTART_BOOST
      && motionPhase != MotionPhase::CRUISE
  ) {
    completeActiveMotion(
        "EMERGENCY_STOP",
        true
    );
  }
}

// ============================================================
// REQUEST PARSING
// ============================================================

bool parseSequenceId(
    const char* token,
    unsigned long& sequenceId
) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }

  unsigned long value = 0;

  for (size_t index = 0; token[index] != '\0'; index++) {
    const char character = token[index];

    if (character < '0' || character > '9') {
      return false;
    }

    const unsigned long digit =
        static_cast<unsigned long>(character - '0');

    if (
        value
        > (MAX_SEQUENCE_ID - digit) / 10UL
    ) {
      return false;
    }

    value = value * 10UL + digit;
  }

  if (value == 0UL) {
    return false;
  }

  sequenceId = value;
  return true;
}

int splitTokens(
    char* line,
    char* tokens[],
    int maximumTokens
) {
  int tokenCount = 0;
  char* context = nullptr;
  char* token = strtok_r(
      line,
      "|",
      &context
  );

  while (token != nullptr) {
    if (tokenCount >= maximumTokens) {
      return -1;
    }

    tokens[tokenCount] = token;
    tokenCount++;

    token = strtok_r(
        nullptr,
        "|",
        &context
    );
  }

  return tokenCount;
}

void handleRequest(char* line) {
  char* tokens[MAX_TOKENS];
  const int tokenCount = splitTokens(
      line,
      tokens,
      MAX_TOKENS
  );

  if (tokenCount < 4) {
    return;
  }

  unsigned long sequenceId = 0;

  if (!parseSequenceId(tokens[3], sequenceId)) {
    return;
  }

  if (strcmp(tokens[0], PROTOCOL_NAME) != 0) {
    sendError(sequenceId, "BAD_FORMAT");
    return;
  }

  if (strcmp(tokens[1], PROTOCOL_VERSION) != 0) {
    sendError(sequenceId, "BAD_VERSION");
    return;
  }

  const char* requestType = tokens[2];

  if (strcmp(requestType, "PING") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    sendPong(sequenceId);
    return;
  }

  if (strcmp(requestType, "STATUS") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    sendStatus(sequenceId);
    return;
  }

  if (strcmp(requestType, "SENSORS") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    if (motionExecuting) {
      sendError(sequenceId, "BUSY");
      return;
    }

    stopMotors();
    sendSensorSnapshot(sequenceId);
    stopMotors();
    return;
  }

  if (strcmp(requestType, "ESTOP") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    stopMotors();
    safetyLatched = true;

    if (motionExecuting) {
      completeActiveMotion(
          "EMERGENCY_STOP",
          true
      );
    } else {
      resetActiveMotionState();
    }

    sendStatus(sequenceId);
    return;
  }

  if (strcmp(requestType, "REARM") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    if (motionExecuting) {
      sendError(sequenceId, "BUSY");
      return;
    }

    stopMotors();
    resetActiveMotionState();
    safetyLatched = false;
    sendStatus(sequenceId);
    return;
  }

  if (strcmp(requestType, "CMD") == 0) {
    if (tokenCount != 5) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    const char* commandType = tokens[4];

    // STOP is always accepted, including during active execution.
    if (strcmp(commandType, "STOP") == 0) {
      stopMotors();

      if (motionExecuting) {
        completeActiveMotion(
            "EMERGENCY_STOP",
            false
        );
      } else {
        resetActiveMotionState();
      }

      sendAcknowledgement(
          sequenceId,
          commandType
      );
      sendCompletion(sequenceId, "OK");
      return;
    }

    if (motionExecuting) {
      sendError(sequenceId, "BUSY");
      return;
    }

    if (safetyLatched) {
      stopMotors();
      sendError(sequenceId, "SAFETY_LATCHED");
      return;
    }

    if (
        strcmp(
            commandType,
            "MOVE_FORWARD"
        ) == 0
    ) {
      startBoundedForwardMotion(sequenceId);
      return;
    }

    stopMotors();
    sendError(sequenceId, "BAD_COMMAND");
    return;
  }

  if (!motionExecuting) {
    stopMotors();
  }

  sendError(sequenceId, "BAD_COMMAND");
}

// ============================================================
// SERIAL INPUT
// ============================================================

void resetSerialLine() {
  serialLength = 0;
  serialOverflow = false;
}

void readSerialRequests() {
  while (Serial.available() > 0) {
    const int incomingValue = Serial.read();

    if (incomingValue < 0) {
      return;
    }

    const char incomingCharacter =
        static_cast<char>(incomingValue);

    if (incomingCharacter == '\r') {
      continue;
    }

    if (incomingCharacter == '\n') {
      if (
          !serialOverflow
          && serialLength > 0
      ) {
        serialBuffer[serialLength] = '\0';
        handleRequest(serialBuffer);
      }

      resetSerialLine();
      continue;
    }

    if (serialOverflow) {
      continue;
    }

    if (
        serialLength
        >= REQUEST_BUFFER_SIZE - 1
    ) {
      serialOverflow = true;
      continue;
    }

    serialBuffer[serialLength] =
        incomingCharacter;
    serialLength++;
  }
}

// ============================================================
// ARDUINO LIFECYCLE
// ============================================================

void setup() {
  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(LINE_SENSOR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  loadMotorOffsets();
  stopMotors();

  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  stopMotors();
  sendReady();
}

void loop() {
  if (
      !MOTION_EXECUTION_ENABLED
      || !motionExecuting
  ) {
    stopMotors();
  }

  readSerialRequests();
  serviceActiveMotion();

  if (!MOTION_EXECUTION_ENABLED) {
    stopMotors();
  }

  delay(1);
}