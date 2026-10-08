#include <Arduino.h>
#include <string.h>

// ============================================================
// I-1.0 SUPERVISED SERIAL FIRMWARE
// Safe handshake stage: no motion command is enabled.
// ============================================================

const char PROTOCOL_NAME[] = "ALR";
const char PROTOCOL_VERSION[] = "1";
const char FIRMWARE_VERSION[] = "i-1.0-stop-only";

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

// The controller starts latched after every boot.
bool safetyLatched = true;

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
  // No incoming message may activate a motor in this stage.
  stopMotors();

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

  if (strcmp(requestType, "ESTOP") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    stopMotors();
    safetyLatched = true;
    sendStatus(sequenceId);
    return;
  }

  if (strcmp(requestType, "REARM") == 0) {
    if (tokenCount != 4) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    stopMotors();
    safetyLatched = false;
    sendStatus(sequenceId);
    return;
  }

  if (strcmp(requestType, "CMD") == 0) {
    stopMotors();

    if (tokenCount != 5) {
      sendError(sequenceId, "BAD_FORMAT");
      return;
    }

    const char* commandType = tokens[4];

    // STOP is always accepted, including while safety remains latched.
    if (strcmp(commandType, "STOP") == 0) {
      sendAcknowledgement(
          sequenceId,
          commandType
      );
      stopMotors();
      sendCompletion(sequenceId, "OK");
      return;
    }

    if (safetyLatched) {
      sendError(sequenceId, "SAFETY_LATCHED");
      return;
    }

    // Movement remains intentionally unavailable.
    sendError(sequenceId, "BAD_COMMAND");
    return;
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

  stopMotors();

  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  stopMotors();
  sendReady();
}

void loop() {
  // Redundant stop is intentional during the handshake stage.
  stopMotors();
  readSerialRequests();
  delay(1);
}