#include <EEPROM.h>

// ============================================================
// MOTEURS ET CAPTEURS DEJA VALIDES
// ============================================================

float motorAOffset = 1.0;
float motorBOffset = 1.0;

const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

const int echoPin = 4;
const int trigPin = 3;

const int rightIR = 7;
const int leftIR = 8;

// Capteur de ligne : 0 = sol rouge, 1 = ruban noir.
const int lineSensor = 2;

// ============================================================
// PHOTORESISTANCE
// ============================================================

const int LDR_PIN = A0;

const int LIGHT_SAMPLE_COUNT = 10;
const int LIGHT_SAMPLE_DELAY_MS = 2;

// La lampe declenche la mission a partir de Base + 30.
const float SIGNAL_DELTA = 30.0;

// Zone neutre utilisee pour suivre lentement la lumiere ambiante.
const float BASELINE_TRACK_DELTA = 15.0;
const float BASELINE_ALPHA = 0.02;

// Valeur initiale a tester pour l'arrivee pres de la lampe.
// Avec le test integre, Base + 220 correspond approximativement
// a une lampe situee entre 20 et 50 cm du capteur.
const float ARRIVAL_DELTA = 220.0;

// Le robot valide la cible uniquement à 25 cm ou moins.
const float TARGET_DISTANCE_CM = 25.0;

const int REQUIRED_LIGHT_READINGS = 5;

float lightBaseline = 0.0;
float lightValue = 0.0;
float lightDelta = 0.0;

int signalCount = 0;
int arrivalCount = 0;

enum RobotState {
  WAITING_FOR_SIGNAL,
  MISSION_ACTIVE,
  TARGET_REACHED
};

RobotState robotState = WAITING_FOR_SIGNAL;

float lastDistance = 0.0;
int lineValue = LOW;
unsigned long lastBoundaryDetection = 0;
unsigned long lastStatusPrint = 0;

// ============================================================
// MESURES
// ============================================================

float readSensorData() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Limite l'attente a 30 ms. Une duree nulle signifie qu'aucun echo
  // ultrasonique n'a ete recu, et non qu'un obstacle se trouve a 0 cm.
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 0.0;
  }

  return duration / 58.00;
}

float readLightAverage() {
  long total = 0;

  for (int i = 0; i < LIGHT_SAMPLE_COUNT; i++) {
    total += analogRead(LDR_PIN);
    delay(LIGHT_SAMPLE_DELAY_MS);
  }

  return total / float(LIGHT_SAMPLE_COUNT);
}

void calibrateLightBaseline() {
  // Environ 3 secondes. La lampe cible doit rester eteinte.
  const int calibrationReadings = 150;
  float total = 0.0;

  for (int i = 0; i < calibrationReadings; i++) {
    total += readLightAverage();
  }

  lightBaseline = total / calibrationReadings;
}

// ============================================================
// COMMANDES DES MOTEURS
// ============================================================

bool forwardRestartRequired = true;

const int FORWARD_RESTART_SPEED = 140;
const int FORWARD_RESTART_MS = 100;
const int FORWARD_DIRECTION_PAUSE_MS = 80;

void writeForwardOutputs(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, int(speed * motorBOffset));
  analogWrite(B_1B, int(speed * motorAOffset * 1.00));
  analogWrite(B_1A, 0);
}

void moveForward(int speed) {
  if (forwardRestartRequired) {
    // Courte pause neutre après un recul ou un virage.
    analogWrite(A_1B, 0);
    analogWrite(A_1A, 0);
    analogWrite(B_1B, 0);
    analogWrite(B_1A, 0);
    delay(FORWARD_DIRECTION_PAUSE_MS);

    // Impulsion brève pour vaincre le blocage mécanique au redémarrage.
    writeForwardOutputs(FORWARD_RESTART_SPEED);
    delay(FORWARD_RESTART_MS);

    forwardRestartRequired = false;
  }

  // Retour immédiat à la vitesse normale calibrée.
  writeForwardOutputs(speed);
}

void moveBackward(int speed) {
  forwardRestartRequired = true;

  analogWrite(A_1B, int(speed * motorAOffset));
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, int(speed * motorBOffset));
}

void backLeft(int speed) {
  forwardRestartRequired = true;

  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void backRight(int speed) {
  forwardRestartRequired = true;

  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

void stopMove() {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void avoidBoundary() {
  unsigned long now = millis();

  bool cornerDetected =
      lastBoundaryDetection != 0 &&
      now - lastBoundaryDetection < 2500;

  lastBoundaryDetection = now;

  if (cornerDetected) {
    Serial.println("=== COIN DETECTE : MANOEUVRE RENFORCEE ===");
  } else {
    Serial.println("=== LIGNE NOIRE : RETOUR VERS LA GRILLE ===");
  }

  stopMove();
  delay(100);

  if (cornerDetected) {
    moveBackward(140);
    delay(300);

    backLeft(120);
    delay(350);
  } else {
    moveBackward(130);
    delay(250);

    backLeft(120);
    delay(220);
  }

  stopMove();
  delay(100);
}

// ============================================================
// LOGIQUE LUMINEUSE
// ============================================================

void updateLightState() {
  lightValue = readLightAverage();
  lightDelta = lightValue - lightBaseline;

  if (robotState == WAITING_FOR_SIGNAL) {
    if (lightDelta >= SIGNAL_DELTA) {
      signalCount++;
    } else {
      signalCount = 0;
    }

    if (signalCount >= REQUIRED_LIGHT_READINGS) {
      robotState = MISSION_ACTIVE;
      signalCount = 0;
      Serial.println("=== SIGNAL CONFIRME : MISSION DEMARREE ===");
    } else if (lightDelta < BASELINE_TRACK_DELTA) {
      // Suivi lent de la lumière ambiante avant la mission.
      lightBaseline =
          (1.0 - BASELINE_ALPHA) * lightBaseline
          + BASELINE_ALPHA * lightValue;
    }

  } else if (robotState == MISSION_ACTIVE) {
    // La cible doit être lumineuse ET suffisamment proche.
    if (lightDelta >= ARRIVAL_DELTA) {
      lastDistance = readSensorData();

      bool targetCloseEnough =
          lastDistance > 2.0 &&
          lastDistance <= TARGET_DISTANCE_CM;

      if (targetCloseEnough) {
        arrivalCount++;
      } else {
        arrivalCount = 0;
      }
    } else {
      arrivalCount = 0;
    }

    if (arrivalCount >= REQUIRED_LIGHT_READINGS) {
      robotState = TARGET_REACHED;
      arrivalCount = 0;
      stopMove();

      Serial.println(
          "=== CIBLE LUMINEUSE PROCHE : ROBOT ARRETE ==="
      );
    }
  }
}

const char* stateName() {
  if (robotState == WAITING_FOR_SIGNAL) {
    return "ATTENTE_SIGNAL";
  }

  if (robotState == MISSION_ACTIVE) {
    return "MISSION_ACTIVE";
  }

  return "CIBLE_ATTEINTE";
}

void printStatus() {
  if (millis() - lastStatusPrint < 500) {
    return;
  }

  lastStatusPrint = millis();

  Serial.print("Lumiere: ");
  Serial.print(lightValue, 1);
  Serial.print(" | Base: ");
  Serial.print(lightBaseline, 1);
  Serial.print(" | Delta: ");
  Serial.print(lightValue - lightBaseline, 1);
  Serial.print(" | Distance: ");
  Serial.print(lastDistance, 1);
  Serial.print(" cm | Ligne: ");
  Serial.print(lineValue == HIGH ? "NOIR" : "SOL");
  Serial.print(" | Etat: ");
  Serial.println(stateName());
}

// ============================================================
// CONDUITE AUTONOME VALIDEE
// ============================================================

void runAutonomousDriving() {
  lineValue = digitalRead(lineSensor);

  // La limite noire conserve la priorité absolue.
  if (lineValue == HIGH) {
    avoidBoundary();
    return;
  }

  int left = digitalRead(leftIR);    // 0 = obstacle, 1 = libre
  int right = digitalRead(rightIR);

  // Obstacle uniquement à gauche.
  if (!left && right) {
    backLeft(120);
  }

  // Obstacle uniquement à droite.
  else if (left && !right) {
    backRight(120);
  }

  // Obstacles détectés des deux côtés.
  else if (!left && !right) {
    moveBackward(140);
  }

  else {
    // Aucun obstacle latéral : contrôle ultrasonique frontal.
    lastDistance = readSensorData();

    if (lastDistance == 0) {
      // Aucun écho : chemin considéré comme libre.
      moveForward(120);
    }

    else if (lastDistance <= 2) {
      // Mesure très proche ou zone aveugle.
      stopMove();
    }

    else if (lastDistance < 20) {
      // Obstacle frontal : recul court et virage léger.
      moveBackward(140);
      delay(250);

      backLeft(120);
      delay(220);

      stopMove();
      delay(100);
    }

    else {
      moveForward(120);
    }
  }
}

// ============================================================
// INITIALISATION ET BOUCLE PRINCIPALE
// ============================================================

void setup() {
  Serial.begin(9600);

  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(lineSensor, INPUT);

  stopMove();

  byte savedA = EEPROM.read(0);
  byte savedB = EEPROM.read(1);

  if (savedA >= 50 && savedA <= 100) {
    motorAOffset = savedA / 100.0;
  } else {
    motorAOffset = 1.0;
  }

  if (savedB >= 50 && savedB <= 100) {
    motorBOffset = savedB / 100.0;
  } else {
    motorBOffset = 1.0;
  }

  delay(1000);

  Serial.println("Calibration lumineuse : lampe cible eteinte...");
  calibrateLightBaseline();

  Serial.print("Base lumineuse = ");
  Serial.println(lightBaseline, 1);
  Serial.print("Offsets A/B = ");
  Serial.print(motorAOffset, 2);
  Serial.print(" / ");
  Serial.println(motorBOffset, 2);
  Serial.println("Robot immobile : attente du signal lumineux.");
}

void loop() {
  lineValue = digitalRead(lineSensor);

  // Pendant une mission, la ligne noire est traitee avant la lumiere
  // et avant les autres capteurs afin d'empecher toute sortie de grille.
  if (robotState == MISSION_ACTIVE && lineValue == HIGH) {
    avoidBoundary();
    printStatus();
    delay(50);
    return;
  }

  updateLightState();

  if (robotState == WAITING_FOR_SIGNAL) {
    stopMove();
  } else if (robotState == MISSION_ACTIVE) {
    runAutonomousDriving();
  } else {
    stopMove();
  }

  printStatus();

  delay(50);
}
