#include <CD74HC4067.h>
#include <Servo.h>
#include <math.h>

// ---- PID Coefficients ----
const float P_GAIN = 2.0;
const float I_GAIN = 0.1;
const float D_GAIN = 30.0;

// ---- Smoothing Factors ----
const float EMA_ALPHA = 0.9;    // Exponential moving average
const float IR_ALPHA  = 0.5;    // IR signal low-pass filter

// ---- Mechanical Constants ----
const float DEG2RAD  = M_PI / 180.0;
const float RAD2DEG  = 180.0 / M_PI;
const float R1       = 50.0;                           // Servo arm length [mm]
const float R2       = 39.2;                           // Passive link length [mm]
const float BASE_R   = 32.9 / sqrt(3.0);               // Base triangle radius [mm]
const float PLAT_R   = 107.9 / sqrt(3.0);              // Platform triangle radius [mm]

// ---- Pin Assignments ----
// Control
const int BUTTON_PIN     = A1;
const int LED_PIN        = 8;

// IR Sensor
const int IR_LED_PIN     = 7;
const int IR_RECEIVER_PIN= A0;

// Digital Potentiometer (MCP42xx)
const int DIGIPOT_CS     = 4;
const int DIGIPOT_DIN    = 1;
const int DIGIPOT_SCLK   = 0;

// Servo pins
const int SERVO_PIN_A    = 10;
const int SERVO_PIN_B    = 9;
const int SERVO_PIN_C    = 11;

// Button press timings [ms]
const unsigned long SHORT_PRESS_TIME  = 50;
const unsigned long LONG_PRESS_TIME   = 1000;
const unsigned long DOUBLE_PRESS_TIME = 350;

// ---- Globals ----
// IR measurements and center tracking
int ambientLight[16] = {0};
int irLight[16]      = {0};
float irSignal[16]   = {0.0};

float centerX        = 0.0;
float centerY        = 0.0;
float setpointX      = 0.0;
float setpointY      = 0.0;

// PID state
float lastErrorX     = 0.0;
float lastErrorY     = 0.0;
float integralX      = 0.0;
float integralY      = 0.0;

// Ball tracking
bool ballWasOnPlate  = false;
unsigned long ballLostTime = 0;

// Button state
bool buttonPressed       = false;
unsigned long pressStart = 0;
unsigned long lastPress  = 0;
bool singlePressFlag     = false;

// Trajectory
float trajectoryAngle = 0.0;

// ---- Objects ----
CD74HC4067 mux(5, 13, 6, 12);  // S0,S1 -> 13, S2->6, S3->12
Servo servoA, servoB, servoC;

// ---- Kalman Filter Definition ----
struct KalmanFilter {
  float x = 0, v = 0, p = 1;
  const float q = 0.3, r = 1.0;
  void update(float z, float dt) {
    // Prediction
    x += v * dt;
    p += q;
    // Correction
    float k = p / (p + r);
    v  += k * ((z - x) / dt);
    x  += k * (z - x);
    p *= (1 - k);
  }
};
KalmanFilter kfX, kfY;

// ---- Function Prototypes ----
void     blinkLED(unsigned long interval);
void     measureIR();
void     setDigitalPot(byte value);
bool     ballOnPlate();
void     computeCenter(float rawX, float rawY);
void     pidControl(float input, float setpoint, float &lastError, float &integral, float &output);
void     movePlatform(float rollDeg, float pitchDeg, float height);
void     moveServos(float a, float b, float c);
void     checkButton();
void     calculateWeightedCenter(const float ir[], float &x, float &y);
void     sendSerialData();
void     setTrajectory(float radius, float speed);

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
  movePlatform(0, -20, 60);
  delay(1000);

  Serial.begin(115200);
}

// ---- Main Loop ----
void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  blinkLED(300);
  setDigitalPot(255);
  measureIR();
  checkButton();
  sendSerialData();

  if (ballOnPlate()) {
    ballWasOnPlate = true;
    ballLostTime = now;

    // Raw center calculation
    float rawX, rawY;
    calculateWeightedCenter(irSignal, rawX, rawY);

    // Kalman & EMA filters
    kfX.update(rawX, dt);
    kfY.update(rawY, dt);
    centerX = EMA_ALPHA * rawX + (1 - EMA_ALPHA) * kfX.x;
    centerY = EMA_ALPHA * rawY + (1 - EMA_ALPHA) * kfY.x;

    // PID
    float outputX, outputY;
    pidControl(centerX, setpointX, lastErrorX, integralX, outputX);
    pidControl(centerY, setpointY, lastErrorY, integralY, outputY);

    movePlatform(outputX, outputY, 60);
  }
  else {
    if (ballWasOnPlate && now - ballLostTime < 1000) {
      // hold last
      movePlatform(0, 0, 60);
    } else {
      ballWasOnPlate = false;
      integralX = integralY = 0;
      lastErrorX = lastErrorY = 0;
      setpointX = setpointY = 0;
      movePlatform(0, -20, 60);
    }
  }

  lastTime = now;
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
    delayMicroseconds(250);
    ambientLight[i] = analogRead(IR_RECEIVER_PIN);
  }
  // IR On
  digitalWrite(IR_LED_PIN, HIGH);
  delay(5);
  for (int i = 0; i < 16; i++) {
    mux.channel(i);
    delayMicroseconds(250);
    irLight[i] = analogRead(IR_RECEIVER_PIN);
  }
  // Compute signal
  for (int i = 0; i < 16; i++) {
    float delta = irLight[i] - ambientLight[i];
    irSignal[i] = IR_ALPHA * delta + (1 - IR_ALPHA) * irSignal[i];
  }
}

bool ballOnPlate() {
  long sum = 0;
  int maxVal = irSignal[0];
  for (int i = 0; i < 16; i++) {
    sum += irSignal[i];
    maxVal = max(maxVal, int(irSignal[i]));
  }
  float avg = sum / 16.0;
  return maxVal > 1.5 * avg;
}

void pidControl(float input, float target, float &lastErr, float &integ, float &out) {
  float error = target - input;
  integ += I_GAIN * error;
  float deriv = D_GAIN * (error - lastErr);
  out = P_GAIN * error + integ + deriv;
  lastErr = error;
}

void movePlatform(float rollDeg, float pitchDeg, float height) {
  float roll  = -rollDeg * DEG2RAD;
  float pitch = -pitchDeg * DEG2RAD;
  float baseAngle[3]    = {0, 120 * DEG2RAD, 240 * DEG2RAD};
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

    platX[i] = x1; platY[i] = y1; platZ[i] = z2;
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
    float d = sqrt(dxl*dxl + dyl*dyl);
    float theta = atan2(dyl, dxl) - acos(constrain((R1*R1 + d*d - R2*R2)/(2*R1*d), -1, 1));
    angles[i] = theta * RAD2DEG;
  }
  moveServos(angles[0], angles[1], angles[2]);
}

void moveServos(float a, float b, float c) {
  a = constrain(a, -10, 65);
  b = constrain(b, -10, 65);
  c = constrain(c, -10, 65);
  servoA.write(100 - a);
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
  if (maxV - minV < 150) { x = y = 0; return; }

  const float coordsX[16] = {0,1,2,3, 0,1,2,3, 0,1,2,3, 0,1,2,3};
  const float coordsY[16] = {0,0,0,0, 1,1,1,1, 2,2,2,2, 3,3,3,3};
  float sumW=0, wx=0, wy=0;
  for (int i = 0; i < 16; i++) {
    float norm = pow((arr[i] - minV)/(maxV - minV), 4);
    wx += coordsX[i] * norm;
    wy += coordsY[i] * norm;
    sumW += norm;
  }
  x = wx/sumW - 1.5;
  y = wy/sumW - 1.5;
}

void checkButton() {
  bool state = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  static bool lastState = LOW;

  if (state && !lastState) {
    pressStart = now;
    buttonPressed = true;
  }
  if (buttonPressed && state && (now - pressStart > LONG_PRESS_TIME)) {
    Serial.println("Long Press Detected");
    buttonPressed = false;
  }
  if (buttonPressed && !state) {
    unsigned long dur = now - pressStart;
    if (dur >= SHORT_PRESS_TIME && dur < LONG_PRESS_TIME) {
      if (now - lastPress < DOUBLE_PRESS_TIME) {
        Serial.println("Double Press Detected");
        singlePressFlag = false;
      } else {
        singlePressFlag = true;
      }
      lastPress = now;
    }
    buttonPressed = false;
  }
  if (singlePressFlag && now - lastPress > DOUBLE_PRESS_TIME) {
    Serial.println("Single Press Detected");
    singlePressFlag = false;
  }
  lastState = state;
}

void sendSerialData() {
  for (int i = 0; i < 16; i++) {
    Serial.print(irSignal[i]);
    Serial.print(',');
  }
  Serial.print(centerX); Serial.print(',');
  Serial.print(centerY); Serial.print(',');
  Serial.print(setpointX); Serial.print(',');
  Serial.println(setpointY);
}

void setTrajectory(float radius, float speed) {
  unsigned long now = millis();
  static unsigned long lastT = 0;
  float dt = (now - lastT)/2000.0;
  trajectoryAngle += speed * dt;
  setpointX = radius * cos(trajectoryAngle);
  setpointY = radius * sin(trajectoryAngle);
  lastT = now;
}
