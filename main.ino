#include <CD74HC4067.h>
#include <Servo.h>
#include <math.h>

// ---- PID Coefficients ----
const float P_GAIN = 3;  
const float I_GAIN = 0.1;
const float D_GAIN = 40;  


// ---- Mechanical Constants ----
const float DEG2RAD = M_PI / 180.0;
const float RAD2DEG = 180.0 / M_PI;
const float R1 = 50.0;                   // Servo arm length [mm]
const float R2 = 39.2;                   // Passive link length [mm]
const float BASE_R = 32.9 / sqrt(3.0);   // Base triangle radius [mm]
const float PLAT_R = 107.9 / sqrt(3.0);  // Platform triangle radius [mm]
const float PLATE_HEIGHT = 60;           // TOP PCB height [mm]

// ---- Pin Assignments ----
// Control
const int BUTTON_PIN = A1;
const int LED_PIN = 8;

// IR Sensor
const int IR_LED_PIN = 7;
const int IR_RECEIVER_PIN = A0;

// Digital Potentiometer (MCP42xx)
const int DIGIPOT_CS = 4;
const int DIGIPOT_DIN = 1;
const int DIGIPOT_SCLK = 0;

// Servo pins 
const int SERVO_PIN_A = 10;
const int SERVO_PIN_B = 9;
const int SERVO_PIN_C = 11;

// ---- Smoothing Factors ----
const float IR_ALPHA  = 0.8;    // IR signal low-pass filter
const float alpha = 0.7;  // Smoothing factor 

// ---- Globals ----
// IR measurements and center tracking
int ambientLight[16] = { 0 };
int irLight[16] = { 0 };
float irSignal[16] = { 0.0 };

float centerX = 0.0;
float centerY = 0.0;
float setpointX = 0.0;
float setpointY = 0.0;

// PID state
float lastErrorX = 0.0;
float lastErrorY = 0.0;
float integralX = 0.0;
float integralY = 0.0;

float outputX = 0, outputY = 0;  // OUTPUTS OF PID

// Ball tracking
bool ballWasOnPlate = false;
unsigned long ballLostTime = 0;
unsigned long ballFoundTime = 0;

// BaBot Modes
enum BaBotMode {
  ON,
  OFF,
  ASSEMBLY
};
BaBotMode mode = ON;  // Initial state

// === ENUM FOR PRESS TYPES ===
enum ButtonPress {
  NO_PRESS,
  SINGLE_PRESS,
  DOUBLE_PRESS,
  LONG_PRESS
};


// === TIMING CONFIG ===
const unsigned long DEBOUNCE_TIME = 50;
const unsigned long DOUBLE_PRESS_GAP = 500;
const unsigned long LONG_PRESS_TIME = 1000;

// === STATE VARIABLES ===
bool buttonWasPressed = false;
unsigned long buttonDownTime = 0;
unsigned long lastPressTime = 0;
int pressCount = 0;

// OptimalPot
int optimalPot;

// ---- Objects ----
CD74HC4067 mux(5, 13, 6, 12);  // S0,S1 -> 13, S2->6, S3->12
Servo servoA, servoB, servoC;


// ---- Arduino Setup ----
void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IR_LED_PIN, OUTPUT);
  pinMode(DIGIPOT_CS, OUTPUT);
  pinMode(DIGIPOT_DIN, OUTPUT);
  pinMode(DIGIPOT_SCLK, OUTPUT);
  digitalWrite(DIGIPOT_CS, HIGH);

  servoA.attach(SERVO_PIN_A);
  servoB.attach(SERVO_PIN_B);
  servoC.attach(SERVO_PIN_C);

  // Initialize platform to neutral
  movePlatform(0, 0, PLATE_HEIGHT);

  Serial.begin(115200);
  delay(1000);
}

int skipCounter = 0;
// ---- Main Loop ----
void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  measureIR();

  setDigitalPot(235); //255=0kOhm 0=100kOhm
  
  float rawX, rawY;
  calculateWeightedCenter(irSignal, rawX, rawY);
  // Apply Exponential Moving Average
  centerX = alpha * rawX + (1 - alpha) * centerX;
  centerY = alpha * rawY + (1 - alpha) * centerY;

  // sendSerialData();

  ButtonPress result = checkButton();

  switch (result) {
    case SINGLE_PRESS:
      // Serial.println("Single Press");
      if (mode == OFF) {
        mode = ON;
      } else if (mode == ON) {
        mode = OFF;
      }
      break;
    case DOUBLE_PRESS:
      // Serial.println("Double Press");
      break;
    case LONG_PRESS:
      if (mode != ASSEMBLY) {
        mode = ASSEMBLY;
      } else if (mode == ASSEMBLY) {
        mode = OFF;
      }
      // Serial.println("Long Press");
      break;
    default:
      break;
  }

  switch (mode) {
    case ON:
      blinkLED(300);
      if (ballOnPlate()) {
        digitalWrite(LED_PIN, HIGH);  //Turn on light
        if (ballWasOnPlate == false) {
          ballFoundTime = millis();
          skipCounter = 2;  // Ignore next 10 cycles
          lastErrorX, lastErrorY = setpointX - centerX, setpointY - centerY;
        }
        ballWasOnPlate = true;
        ballLostTime = millis();  // Reset the timer since the ball is detected
        if (skipCounter > 0) {
          skipCounter--;
          return; // Skip rest of loop
        }

        // PID
        pidControl(centerX, setpointX, lastErrorX, integralX, outputX);
        pidControl(centerY, setpointY, lastErrorY, integralY, outputY);

        movePlatform(outputX, outputY, PLATE_HEIGHT);

      } else {
        if (ballWasOnPlate && millis() - ballLostTime < 1000) {
          // hold last
          movePlatform(outputX, outputY, PLATE_HEIGHT);
        } 
        else {
          ballWasOnPlate = false;
          integralX = integralY = 0;
          lastErrorX = lastErrorY = 0;
          setpointX = setpointY = 0;
          movePlatform(0, 0, PLATE_HEIGHT);
        }
      }
      break;

    case OFF:
      digitalWrite(LED_PIN, LOW);  //Turn off light
      movePlatform(0, 0, PLATE_HEIGHT);
      break;

    case ASSEMBLY:
      blinkLED(50);
      moveServos(0, 0, 0);
      break;

    default:
      break;
  }
}

// ---- Utility Functions ----

void blinkLED(unsigned long interval) {
  static unsigned long lastToggle = 0;
  if (millis() - lastToggle >= interval) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastToggle = millis();
  }
}

void setDigitalPot(byte val) {
  digitalWrite(DIGIPOT_CS, LOW);
  for (int i = 7; i >= 0; --i) {
    digitalWrite(DIGIPOT_DIN, (val & (1 << i)) ? HIGH : LOW);
    digitalWrite(DIGIPOT_SCLK, LOW);
    delayMicroseconds(10);
    digitalWrite(DIGIPOT_SCLK, HIGH);
    delayMicroseconds(10);
  }
  digitalWrite(DIGIPOT_CS, HIGH);
}

void measureIR() {
  // Ambient
  digitalWrite(IR_LED_PIN, LOW);
  delay(1);
  for (int i = 0; i < 16; i++) {
    mux.channel(i);
    delayMicroseconds(400);
    ambientLight[i] = analogRead(IR_RECEIVER_PIN);
  }
  // IR On
  digitalWrite(IR_LED_PIN, HIGH);
  delay(4);
  for (int i = 0; i < 16; i++) {
    mux.channel(i);
    delayMicroseconds(400);
    irLight[i] = analogRead(IR_RECEIVER_PIN);
  }
  // Compute signal
  for (int i = 0; i < 16; i++) {
    float delta = irLight[i] - ambientLight[i];
    irSignal[i] = IR_ALPHA * delta + (1 - IR_ALPHA) * irSignal[i];
  }

  Serial.print("irLight ");
  Serial.println(irLight[0]);
  Serial.print("ambientLight ");
  Serial.println(ambientLight[0]);
}

const float BALL_THRESHOLD = 200.0;
bool ballOnPlate() {
  float minV = irSignal[0], maxV = irSignal[0];
  for (int i = 1; i < 16; i++) {
    minV = min(minV, irSignal[i]);
    maxV = max(maxV, irSignal[i]);
  }
  return (maxV - minV >= BALL_THRESHOLD);
}

void pidControl(float input, float target, float &lastErr, float &integ, float &out) {
  float error = target - input;
  integ += I_GAIN * error;
  float deriv = D_GAIN * (error - lastErr);
  out = P_GAIN * error + integ + deriv;
  lastErr = error;
}

void movePlatform(float rollDeg, float pitchDeg, float height) {
  float roll = -rollDeg * DEG2RAD;
  float pitch = -pitchDeg * DEG2RAD;
  float baseAngle[3] = { 0, 120 * DEG2RAD, 240 * DEG2RAD };
  float platX[3], platY[3], platZ[3], angles[3];

  // Transform platform points
  for (int i = 0; i < 3; i++) {
    float a = baseAngle[i];
    float px = PLAT_R * cos(a);
    float py = PLAT_R * sin(a);
    float pz = height;

    // Pitch
    float x1 = px * cos(pitch) + pz * sin(pitch);
    float z1 = -px * sin(pitch) + pz * cos(pitch);
    // Roll
    float y1 = py * cos(roll) - z1 * sin(roll);
    float z2 = py * sin(roll) + z1 * cos(roll);

    platX[i] = x1;
    platY[i] = y1;
    platZ[i] = z2;
  }
  // Calculate servo angles
  for (int i = 0; i < 3; i++) {
    float a = baseAngle[i];
    float bx = BASE_R * cos(a);
    float by = BASE_R * sin(a);
    float dx = platX[i] - bx;
    float dy = platY[i] - by;
    float dz = platZ[i];
    float dxl = dx * cos(a) + dy * sin(a);
    float dyl = dz;
    float d = sqrt(dxl * dxl + dyl * dyl);
    float theta = atan2(dyl, dxl) - acos(constrain((R1 * R1 + d * d - R2 * R2) / (2 * R1 * d), -1, 1));
    angles[i] = theta * RAD2DEG;
  }
  moveServos(angles[0], angles[1], angles[2]);  // ordre a changer pour respecter pcb
}

void moveServos(float a, float b, float c) {
  a = constrain(a, -10, 65);
  b = constrain(b, -10, 65);
  c = constrain(c, -10, 65);
  servoA.write(100 - a);  // BABOT blanc PCBWAY
  servoB.write(100 - b);
  servoC.write(100 - c);
}

void calculateWeightedCenter(const float arr[], float &x, float &y) {
  // If insufficient contrast, return (0,0)
  float minV = arr[0], maxV = arr[0];
  for (int i = 1; i < 16; i++) {
    minV = min(minV, arr[i]);
    maxV = max(maxV, arr[i]);
  }

  if(!ballOnPlate()){
    x = y = 0;
    return;
  }

  const float coordsX[16] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 };
  const float coordsY[16] = { 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 };
  float sumW = 0, wx = 0, wy = 0;
  for (int i = 0; i < 16; i++) {
    float norm = pow((arr[i] - minV) / (maxV - minV), 4);
    wx += coordsX[i] * norm;
    wy += coordsY[i] * norm;
    sumW += norm;
  }
  x = wx / sumW - 1.5;
  y = wy / sumW - 1.5;
}


void sendSerialData() {
  for (int i = 0; i < 16; i++) {
    Serial.print(irSignal[i]);
    Serial.print(',');
  }
  Serial.print(centerX);
  Serial.print(',');
  Serial.print(centerY);
  Serial.print(',');
  Serial.print(setpointX);
  Serial.print(',');
  Serial.println(setpointY);
}


// === BUTTON CHECK FUNCTION ===
ButtonPress checkButton() {
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  ButtonPress result = NO_PRESS;

  // === DETECT PRESS ===
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    // Button just pressed
    buttonDownTime = now;
    buttonWasPressed = true;
  }

  // === DETECT RELEASE ===
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    unsigned long pressDuration = now - buttonDownTime;
    buttonWasPressed = false;

    if (pressDuration >= LONG_PRESS_TIME) {
      result = LONG_PRESS;
      pressCount = 0;
    } else {
      pressCount++;
      lastPressTime = now;
    }
  }

  // === HANDLE SINGLE/DOUBLE ===
  if (pressCount > 0 && (now - lastPressTime > DOUBLE_PRESS_GAP)) {
    if (pressCount == 1) {
      result = SINGLE_PRESS;
    } else if (pressCount == 2) {
      result = DOUBLE_PRESS;
    }
    pressCount = 0;
  }

  lastButtonState = currentButtonState;
  return result;
}
