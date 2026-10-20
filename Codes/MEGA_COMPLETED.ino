/*
  ============================================================
  ARDUINO MEGA - MOTION CONTROLLER FOR OUR BIAXIAL MECHANISM
  ============================================================

  This version preserves the supplied MEGA ROUGH CODE's pin map,
  command names, serial baud rate, and overall state-machine layout,
  while implementing encoder-based stretching and homing.

  IMPORTANT HARDWARE / SAFETY NOTES:
  - Test with the elastomer disconnected and the mechanism supported.
  - Keep a reachable power disconnect available during testing.
  - Mechanical travel limits are software limits; there are no far-end
    limit switches in the supplied design.
  - Verify motor directions at low PWM before attaching the elastomer.
  - Encoder calibration is 12,800 counts/output revolution as reported
    from the user's actual motor and counting method (A-channel CHANGE,
    B read for direction). Do not change the interrupt mode without
    recalibrating this value.
  - UNO code is intentionally not changed. Its current CHECK_HOME response
    handling does not transition into the stretch menu; see accompanying
    explanation.
*/

// ============================================================
// MOTOR 1 - SG (X direction)
// ============================================================
#define SG_RPWM 9
#define SG_LPWM 10
#define SG_ENC_A 2
#define SG_ENC_B 3
#define SG_LIMIT 22

// ============================================================
// MOTOR 2 - CG1 (Y direction; mounted on fixed base)
// ============================================================
#define CG1_RPWM 11
#define CG1_LPWM 12
#define CG1_ENC_A 18
#define CG1_ENC_B 19
#define CG1_LIMIT 23

// ============================================================
// MOTOR 3 - CG2 (Y direction; mounted on moving T-rack)
// ============================================================
#define CG2_RPWM 6
#define CG2_LPWM 7
#define CG2_ENC_A 20
#define CG2_ENC_B 21
#define CG2_LIMIT 24

// ============================================================
// MECHANISM / ENCODER CONSTANTS
// ============================================================
const float INITIAL_SIZE_MM = 75.0f;
const float GEAR_PITCH_RADIUS_MM = 14.0f;
const float RACK_TRAVEL_PER_OUTPUT_REV = 2.0f * PI * GEAR_PITCH_RADIUS_MM;
const long ENCODER_COUNTS_PER_OUTPUT_REV = 12800L;

const float MIN_STRETCH = 1.00f;
const float MAX_STRETCH = 5.00f;
const float MAX_SG_TRAVEL_MM = 300.0f;
const float MAX_CG_TRAVEL_MM = 170.0f;  // safety limit per CG rack travel

// Start conservatively. Tune only after no-load testing.
const uint8_t HOME_PWM = 100;
const uint8_t STRETCH_PWM = 100;

// Safety timeouts. These are fault guards, not substitutes for hard stops.
const unsigned long HOME_TIMEOUT_MS = 60000UL;
const unsigned long SWITCH_RELEASE_TIMEOUT_MS = 2500UL;
const unsigned long MIN_STRETCH_TIMEOUT_MS = 15000UL;
const unsigned long MOTION_TIMEOUT_EXTRA_MS = 10000UL;

// ============================================================
// ENCODER COUNTERS
// ============================================================
volatile long sgEncoderCount = 0;
volatile long cg1EncoderCount = 0;
volatile long cg2EncoderCount = 0;

// ============================================================
// LIMIT SWITCH STATES: INPUT_PULLUP means LOW = pressed
// ============================================================
bool sgHomePressed = false;
bool cg1HomePressed = false;
bool cg2HomePressed = false;

// ============================================================
// STATE
// ============================================================
enum MotionState {
  MOTION_IDLE,
  MOTION_HOMING,
  MOTION_STRETCHING
};
MotionState motionState = MOTION_IDLE;

String incomingCommand = "";
float requestedXStretch = 1.00f;
float requestedYStretch = 1.00f;

unsigned long homeStartTime = 0;
unsigned long stretchStartTime = 0;
unsigned long sgStartedMovingAt = 0;
unsigned long cg1StartedMovingAt = 0;
unsigned long cg2StartedMovingAt = 0;

long sgStartCount = 0;
long cg1StartCount = 0;
long cg2StartCount = 0;
long sgTargetCounts = 0;
long cg1TargetCounts = 0;
long cg2TargetCounts = 0;

bool sgMotionDone = false;
bool cg1MotionDone = false;
bool cg2MotionDone = false;
bool sgNeedsReleaseCheck = false;
bool cg1NeedsReleaseCheck = false;
bool cg2NeedsReleaseCheck = false;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void sgEncoderISR();
void cg1EncoderISR();
void cg2EncoderISR();
void stopSG();
void stopCG1();
void stopCG2();
void stopAllMotors();
void moveSGHome();
void moveCG1Home();
void moveCG2Home();
void moveSGStretch();
void moveCG1Stretch();
void moveCG2Stretch();
void updateLimitSwitches();
bool allHomeLimitsPressed();
void readUNOCommands();
void processUNOCommand(String command);
void sendToUNO(const String &message);
void beginHomeProcedure();
void performHomeProcedure();
void beginStretchProcedure();
void performStretchProcedure();
void failMotion();
float calculateXDisplacement(float stretchFactor);
float calculateYTotalDisplacement(float stretchFactor);
float calculateYEachSideDisplacement(float stretchFactor);
long millimetersToEncoderCounts(float mm);
long readSGCount();
long readCG1Count();
long readCG2Count();
long absoluteCountDifference(long a, long b);
unsigned long calculateStretchTimeout(long maxTargetCounts);
void resetEncoderCounts();

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Keep this at 115200 to match the supplied UNO code.
  Serial2.begin(115200);
  Serial.begin(115200);  // USB diagnostics

  pinMode(SG_RPWM, OUTPUT);
  pinMode(SG_LPWM, OUTPUT);
  pinMode(CG1_RPWM, OUTPUT);
  pinMode(CG1_LPWM, OUTPUT);
  pinMode(CG2_RPWM, OUTPUT);
  pinMode(CG2_LPWM, OUTPUT);

  pinMode(SG_ENC_A, INPUT_PULLUP);
  pinMode(SG_ENC_B, INPUT_PULLUP);
  pinMode(CG1_ENC_A, INPUT_PULLUP);
  pinMode(CG1_ENC_B, INPUT_PULLUP);
  pinMode(CG2_ENC_A, INPUT_PULLUP);
  pinMode(CG2_ENC_B, INPUT_PULLUP);

  pinMode(SG_LIMIT, INPUT_PULLUP);
  pinMode(CG1_LIMIT, INPUT_PULLUP);
  pinMode(CG2_LIMIT, INPUT_PULLUP);

  // Mega external interrupt-capable pins: SG A=2, CG1 A=18, CG2 A=20.
  // ISR fires on A CHANGE and reads B for direction. The calibrated
  // 12,800 counts/revolution must correspond to this exact method.
  attachInterrupt(digitalPinToInterrupt(SG_ENC_A), sgEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CG1_ENC_A), cg1EncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CG2_ENC_A), cg2EncoderISR, CHANGE);

  stopAllMotors();
  sendToUNO("MEGA_READY");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  readUNOCommands();
  updateLimitSwitches();

  if (motionState == MOTION_HOMING) {
    performHomeProcedure();
  } else if (motionState == MOTION_STRETCHING) {
    performStretchProcedure();
  }
}

// ============================================================
// ENCODER ISRs: A-channel CHANGE, B determines direction
// ============================================================
void sgEncoderISR() {
  if (digitalRead(SG_ENC_A) == digitalRead(SG_ENC_B)) sgEncoderCount++;
  else sgEncoderCount--;
}

void cg1EncoderISR() {
  if (digitalRead(CG1_ENC_A) == digitalRead(CG1_ENC_B)) cg1EncoderCount++;
  else cg1EncoderCount--;
}

void cg2EncoderISR() {
  if (digitalRead(CG2_ENC_A) == digitalRead(CG2_ENC_B)) cg2EncoderCount++;
  else cg2EncoderCount--;
}

// ============================================================
// ATOMIC ENCODER READS / RESET
// ============================================================
long readSGCount() {
  noInterrupts(); long value = sgEncoderCount; interrupts(); return value;
}
long readCG1Count() {
  noInterrupts(); long value = cg1EncoderCount; interrupts(); return value;
}
long readCG2Count() {
  noInterrupts(); long value = cg2EncoderCount; interrupts(); return value;
}
void resetEncoderCounts() {
  noInterrupts();
  sgEncoderCount = 0;
  cg1EncoderCount = 0;
  cg2EncoderCount = 0;
  interrupts();
}
long absoluteCountDifference(long a, long b) {
  long d = a - b;
  return (d < 0) ? -d : d;
}

// ============================================================
// LIMIT SWITCHES
// ============================================================
void updateLimitSwitches() {
  sgHomePressed = (digitalRead(SG_LIMIT) == LOW);
  cg1HomePressed = (digitalRead(CG1_LIMIT) == LOW);
  cg2HomePressed = (digitalRead(CG2_LIMIT) == LOW);
}

bool allHomeLimitsPressed() {
  updateLimitSwitches();
  return sgHomePressed && cg1HomePressed && cg2HomePressed;
}

// ============================================================
// SERIAL COMMAND READER
// ============================================================
void readUNOCommands() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      incomingCommand.trim();
      if (incomingCommand.length() > 0) processUNOCommand(incomingCommand);
      incomingCommand = "";
    } else if (c != '\r') {
      if (incomingCommand.length() < 80) incomingCommand += c;
      else incomingCommand = "";  // discard overlong command
    }
  }
}

void processUNOCommand(String command) {
  if (motionState != MOTION_IDLE) {
    // Do not accept another motion command while already moving.
    if (command == "HOME" || command.startsWith("STRETCH") || command == "CHECK_HOME") {
      sendToUNO("MOTION_ERROR");
    }
    return;
  }

  if (command == "CHECK_HOME") {
    if (allHomeLimitsPressed()) {
      resetEncoderCounts();
      sendToUNO("HOME_COMPLETE");
    } else {
      sendToUNO("HOME_INCOMPLETE");
    }
    return;
  }

  if (command == "HOME") {
    beginHomeProcedure();
    return;
  }

  if (command.startsWith("STRETCH")) {
    int firstSpace = command.indexOf(' ');
    if (firstSpace < 0) { sendToUNO("MOTION_ERROR"); return; }
    String values = command.substring(firstSpace + 1);
    int secondSpace = values.indexOf(' ');
    if (secondSpace < 0) { sendToUNO("MOTION_ERROR"); return; }

    String xString = values.substring(0, secondSpace);
    String yString = values.substring(secondSpace + 1);
    xString.trim(); yString.trim();
    if (xString.length() == 0 || yString.length() == 0) {
      sendToUNO("MOTION_ERROR"); return;
    }

    requestedXStretch = xString.toFloat();
    requestedYStretch = yString.toFloat();
    beginStretchProcedure();
  }
}

void sendToUNO(const String &message) {
  Serial2.println(message);
  Serial.println(message);  // diagnostics on USB Serial Monitor
}

// ============================================================
// MOTOR OUTPUTS
// Direction convention from user's observation, viewed from above
// looking directly at the vertically upward gearbox output shaft:
// RPWM => output shaft anticlockwise; LPWM => clockwise.
// SG: stretch CW / home ACW.
// CG1: stretch CW / home ACW.
// CG2: stretch ACW / home CW.
// ============================================================
void moveSGHome() {
  analogWrite(SG_RPWM, HOME_PWM); analogWrite(SG_LPWM, 0);  // ACW
}
void moveCG1Home() {
  analogWrite(CG1_RPWM, HOME_PWM); analogWrite(CG1_LPWM, 0);  // ACW
}
void moveCG2Home() {
  analogWrite(CG2_RPWM, 0); analogWrite(CG2_LPWM, HOME_PWM); // CW
}
void moveSGStretch() {
  analogWrite(SG_RPWM, 0); analogWrite(SG_LPWM, STRETCH_PWM); // CW
}
void moveCG1Stretch() {
  analogWrite(CG1_RPWM, 0); analogWrite(CG1_LPWM, STRETCH_PWM); // CW
}
void moveCG2Stretch() {
  analogWrite(CG2_RPWM, STRETCH_PWM); analogWrite(CG2_LPWM, 0); // ACW
}

void stopSG() { analogWrite(SG_RPWM, 0); analogWrite(SG_LPWM, 0); }
void stopCG1() { analogWrite(CG1_RPWM, 0); analogWrite(CG1_LPWM, 0); }
void stopCG2() { analogWrite(CG2_RPWM, 0); analogWrite(CG2_LPWM, 0); }
void stopAllMotors() { stopSG(); stopCG1(); stopCG2(); }

// ============================================================
// HOME PROCEDURE
// ============================================================
void beginHomeProcedure() {
  updateLimitSwitches();
  if (allHomeLimitsPressed()) {
    stopAllMotors();
    resetEncoderCounts();
    motionState = MOTION_IDLE;
    sendToUNO("HOME_ALREADY");
    return;
  }

  stopAllMotors();
  homeStartTime = millis();
  motionState = MOTION_HOMING;
  sendToUNO("HOME_PROCESSING");
}

void performHomeProcedure() {
  updateLimitSwitches();
  if (millis() - homeStartTime > HOME_TIMEOUT_MS) {
    failMotion();
    return;
  }

  // Each axis homes independently and stops as soon as its own switch presses.
  if (!sgHomePressed) moveSGHome(); else stopSG();
  if (!cg1HomePressed) moveCG1Home(); else stopCG1();
  if (!cg2HomePressed) moveCG2Home(); else stopCG2();

  if (allHomeLimitsPressed()) {
    stopAllMotors();
    resetEncoderCounts();
    motionState = MOTION_IDLE;
    sendToUNO("HOME_COMPLETE");
  }
}

// ============================================================
// STRETCH TARGET CALCULATIONS
// ============================================================
float calculateXDisplacement(float stretchFactor) {
  return (stretchFactor - 1.0f) * INITIAL_SIZE_MM;
}
float calculateYTotalDisplacement(float stretchFactor) {
  return (stretchFactor - 1.0f) * INITIAL_SIZE_MM;
}
float calculateYEachSideDisplacement(float stretchFactor) {
  return calculateYTotalDisplacement(stretchFactor) / 2.0f;
}
long millimetersToEncoderCounts(float mm) {
  float revolutions = mm / RACK_TRAVEL_PER_OUTPUT_REV;
  return (long)lroundf(revolutions * (float)ENCODER_COUNTS_PER_OUTPUT_REV);
}

unsigned long calculateStretchTimeout(long maxTargetCounts) {
  // 10 RPM => about 2,133 output-shaft counts/s at the calibrated resolution.
  // Add a generous margin for load, acceleration, and slow movement.
  const float expectedCountsPerSecond =
    (float)ENCODER_COUNTS_PER_OUTPUT_REV * 10.0f / 60.0f;
  float estimatedSeconds = (float)maxTargetCounts / expectedCountsPerSecond;
  unsigned long computed = (unsigned long)(estimatedSeconds * 3.0f * 1000.0f)
                         + MOTION_TIMEOUT_EXTRA_MS;
  if (computed < MIN_STRETCH_TIMEOUT_MS) computed = MIN_STRETCH_TIMEOUT_MS;
  return computed;
}

void beginStretchProcedure() {
  if (requestedXStretch < MIN_STRETCH || requestedXStretch > MAX_STRETCH ||
      requestedYStretch < MIN_STRETCH || requestedYStretch > MAX_STRETCH) {
    sendToUNO("MOTION_ERROR"); return;
  }

  // Stretch must begin from the physical home condition.
  if (!allHomeLimitsPressed()) {
    sendToUNO("MOTION_ERROR"); return;
  }

  float xMM = calculateXDisplacement(requestedXStretch);
  float yMM = calculateYEachSideDisplacement(requestedYStretch);

  // Enforce the specified safe software travel limits before any motor starts.
  if (xMM < 0.0f || xMM > MAX_SG_TRAVEL_MM ||
      yMM < 0.0f || yMM > MAX_CG_TRAVEL_MM) {
    sendToUNO("MOTION_ERROR"); return;
  }

  sgTargetCounts = millimetersToEncoderCounts(xMM);
  cg1TargetCounts = millimetersToEncoderCounts(yMM);
  cg2TargetCounts = millimetersToEncoderCounts(yMM);

  sgStartCount = readSGCount();
  cg1StartCount = readCG1Count();
  cg2StartCount = readCG2Count();

  sgMotionDone = (sgTargetCounts == 0);
  cg1MotionDone = (cg1TargetCounts == 0);
  cg2MotionDone = (cg2TargetCounts == 0);

  sgNeedsReleaseCheck = !sgMotionDone;
  cg1NeedsReleaseCheck = !cg1MotionDone;
  cg2NeedsReleaseCheck = !cg2MotionDone;
  sgStartedMovingAt = 0;
  cg1StartedMovingAt = 0;
  cg2StartedMovingAt = 0;

  stopAllMotors();
  if (sgMotionDone) stopSG(); else moveSGStretch();
  if (cg1MotionDone) stopCG1(); else moveCG1Stretch();
  if (cg2MotionDone) stopCG2(); else moveCG2Stretch();

  long maxTarget = sgTargetCounts;
  if (cg1TargetCounts > maxTarget) maxTarget = cg1TargetCounts;
  if (cg2TargetCounts > maxTarget) maxTarget = cg2TargetCounts;
  stretchStartTime = millis();
  // Store calculated deadline in stretchStartedAt as a duration end reference.
  // (The actual deadline is computed and checked in performStretchProcedure.)
  motionState = MOTION_STRETCHING;
  sendToUNO("STRETCH_PROCESSING");
}

void performStretchProcedure() {
  unsigned long elapsed = millis() - stretchStartTime;
  long maxTarget = sgTargetCounts;
  if (cg1TargetCounts > maxTarget) maxTarget = cg1TargetCounts;
  if (cg2TargetCounts > maxTarget) maxTarget = cg2TargetCounts;
  if (elapsed > calculateStretchTimeout(maxTarget)) {
    failMotion(); return;
  }

  updateLimitSwitches();

  // A requested movement must release its home switch promptly. If a switch
  // remains pressed, the axis may be blocked or the switch/wiring may be wrong.
  if (sgNeedsReleaseCheck && sgHomePressed) {
    if (elapsed > SWITCH_RELEASE_TIMEOUT_MS) { failMotion(); return; }
  } else if (sgNeedsReleaseCheck) {
    sgNeedsReleaseCheck = false; sgStartedMovingAt = millis();
  }
  if (cg1NeedsReleaseCheck && cg1HomePressed) {
    if (elapsed > SWITCH_RELEASE_TIMEOUT_MS) { failMotion(); return; }
  } else if (cg1NeedsReleaseCheck) {
    cg1NeedsReleaseCheck = false; cg1StartedMovingAt = millis();
  }
  if (cg2NeedsReleaseCheck && cg2HomePressed) {
    if (elapsed > SWITCH_RELEASE_TIMEOUT_MS) { failMotion(); return; }
  } else if (cg2NeedsReleaseCheck) {
    cg2NeedsReleaseCheck = false; cg2StartedMovingAt = millis();
  }

  // Since these are home switches, activation again during outward stretching
  // is unexpected and treated as a fault for axes that have a nonzero target.
  if ((!sgMotionDone && !sgNeedsReleaseCheck && sgHomePressed) ||
      (!cg1MotionDone && !cg1NeedsReleaseCheck && cg1HomePressed) ||
      (!cg2MotionDone && !cg2NeedsReleaseCheck && cg2HomePressed)) {
    failMotion(); return;
  }

  // Absolute count difference avoids depending on whether the ISR's positive
  // sign matches the mechanical CW convention. Direction is set by motor pins.
  if (!sgMotionDone && absoluteCountDifference(readSGCount(), sgStartCount) >= sgTargetCounts) {
    stopSG(); sgMotionDone = true;
  }
  if (!cg1MotionDone && absoluteCountDifference(readCG1Count(), cg1StartCount) >= cg1TargetCounts) {
    stopCG1(); cg1MotionDone = true;
  }
  if (!cg2MotionDone && absoluteCountDifference(readCG2Count(), cg2StartCount) >= cg2TargetCounts) {
    stopCG2(); cg2MotionDone = true;
  }

  if (sgMotionDone && cg1MotionDone && cg2MotionDone) {
    stopAllMotors();
    motionState = MOTION_IDLE;
    sendToUNO("STRETCH_COMPLETE");
  }
}

// ============================================================
// GLOBAL FAULT HANDLER
// ============================================================
void failMotion() {
  stopAllMotors();
  motionState = MOTION_IDLE;
  sendToUNO("MOTION_ERROR");
}
