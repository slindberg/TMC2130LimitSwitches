#include <TMC2130Stepper.h>

//#define DEBUG

#define UNUSED_PIN 4 // Unconnected, used to initialize TMC instance
#define DIR_PIN 3 // Direction input
#define CS_PIN 10 // SPI chip select output, can be 10 because we're master
#define STALL_PIN 2 // Stallguard active/inactive input
#define BOTTOM_LIMIT_PIN 8 // Bottom limit switch output
#define BOTTOM_LED_PIN 6 // Bottom limit LED indicator
#define TOP_LIMIT_PIN 9 // Top limit switch output
#define TOP_LED_PIN 7 // Top limit LED indicator
#define STALL_VALUE_PIN A0 // Analog pin used to set stall value
#define MIN_SPEED_PIN A1 // Analog pin used to set min speed value
#define STEP_PIN UNUSED_PIN // Step output pin, only used in debug mode

// Scales and 10 bit analog value from a trimpot to achieve the desired range
const float stallValueScalingFactor = 0.009; // 0-9
const float minSpeedScalingFactor = 1.5; // 0-1500

// State vars
bool isAtTop;
bool isAtBottom;

// The stepper lib constructor takes arguments for enable, step, and dir pins, simply
// to initialize their mode. Because the driver is controlled by a raspberry pi, the
// pins don't matter, but a valid pin must be supplied that doesn't clash with others.
TMC2130Stepper stepper = TMC2130Stepper(UNUSED_PIN, UNUSED_PIN, UNUSED_PIN, CS_PIN);

void setup() {
  pinMode(DIR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIR_PIN), dirChanged, CHANGE);

  pinMode(STALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallChanged, CHANGE);

  pinMode(BOTTOM_LIMIT_PIN, OUTPUT);
  pinMode(BOTTOM_LED_PIN, OUTPUT);
  setIsAtBottom(false);

  pinMode(TOP_LIMIT_PIN, OUTPUT);
  pinMode(TOP_LED_PIN, OUTPUT);
  setIsAtTop(false);

  pinMode(STALL_VALUE_PIN, INPUT);
  pinMode(MIN_SPEED_PIN, INPUT);

  #ifdef DEBUG
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  Serial.begin(9600);
  #endif

  initStepper();
}

void loop() {
  #ifdef DEBUG
  static uint32_t lastTime = 0;
  uint32_t now = millis();
  if (now - lastTime > 500) {
    lastTime = now;
    Serial.print(stepper.microstep_time(), DEC);
    Serial.print(", ");
    Serial.print(stepper.sg_result(), DEC);
    Serial.println();
  }

  if (!isAtTop && !isAtBottom) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(600);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(20);
  }
  #endif
}

void initStepper() {
  // Read values from trimpots for on the fly tuning
  int stallValue = analogRead(STALL_VALUE_PIN) * stallValueScalingFactor;
  int minSpeed = analogRead(MIN_SPEED_PIN) * minSpeedScalingFactor;

  #ifdef DEBUG
  Serial.print("Stall value: ");
  Serial.println(stallValue);
  Serial.print("Coolstep min speed: ");
  Serial.println(minSpeed);
  #endif

  stepper.begin(); // Initialize pin modes and SPI interface
  stepper.external_ref(true); // Use the external reference voltage to scale current
  stepper.run_current(31); // Set internal run current scaling to maximum
  stepper.hold_current(7); // Set internal hold (idle) current scaling to 25%
  stepper.shaft_dir(1); // Reverse default direction
  stepper.microsteps(16); // One step pulse is 1/16 of full step
  stepper.interpolate(true); // Turn on sub-step interpolation
  stepper.diag1_stall(true); // Diag1 pin activates on motor stall
  stepper.sg_stall_value(stallValue); // Controls when stall value reads 0 [-64..63]
  stepper.coolstep_min_speed(minSpeed); // enable stallGuard only when moving
}

bool isDirUp() {
  return (bool)digitalRead(DIR_PIN);
}

bool isStalled() {
  // Stall pin is active low
  return !digitalRead(STALL_PIN);
}

void setIsAtBottom(bool state) {
  isAtBottom = state;
  digitalWrite(BOTTOM_LIMIT_PIN, !isAtBottom);
  digitalWrite(BOTTOM_LED_PIN, isAtBottom);
}

void setIsAtTop(bool state) {
  isAtTop = state;
  digitalWrite(TOP_LIMIT_PIN, !isAtTop);
  digitalWrite(TOP_LED_PIN, isAtTop);
}

// Interrupt handler for direction switching, used to clear state vars
void dirChanged() {
  bool isUp = isDirUp();

  if (isAtBottom && isUp) {
    setIsAtBottom(false);
  }

  if (isAtTop && !isUp) {
    setIsAtTop(false);
  }

  #ifdef DEBUG
  Serial.print("Direction: ");
  Serial.println(isUp ? "up" : "down");
  #endif
}

// Interrupt handler for stall flag, used to set limit state vars based on current dir
void stallChanged() {
  bool stalled = isStalled();

  if (stalled) {
    if (isDirUp()) {
      setIsAtTop(true);
    } else {
      setIsAtBottom(true);
    }
  }

  #ifdef DEBUG
  Serial.print(stalled ? "Stalled: " : "Not stalled: ");
  Serial.println(stepper.sg_result(), DEC);
  #endif
}

