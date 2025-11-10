#define PRINT_DEBUG_BUILD  // Uncomment to print live MPU & PID data in Serial Monitor

#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2

// --- MPU control/status vars ---
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// --- PID settings ---
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT 255
#define PID_SAMPLE_TIME_IN_MILLI 10

#define SETPOINT_PITCH_ANGLE_OFFSET -2.2
#define MIN_ABSOLUTE_SPEED 0
 
// --- Fall detection & recovery ---
#define MAX_FALL_ANGLE 60  // degrees
#define FALL_HOLD_TIME  3000// ms to keep motors stopped after fall
#define BALANCE_DEADZONE 1.5  // degrees within which motors stop (no drift)

bool robotFallen = false;
unsigned long fallDetectedTime = 0;

double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;==
// --- PID Tuning Values ---
#define PID_PITCH_KP 29
#define PID_PITCH_KI 85
#define PID_PITCH_KD 0.9

#define PID_YAW_KP 0.5
#define PID_YAW_KI 0.5
#define PID_YAW_KD 0

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

// --- Motor pins ---
int enableMotor1 = 9;
int motor1Pin1 = 5;
int motor1Pin2 = 6;
int motor2Pin1 = 7;
int motor2Pin2 = 8;
int enableMotor2 = 10;

// === Function Prototypes ===
void rotateMotor(int speed1, int speed2);
void setupPID();
void setupMotors();
void setupMPU();

void setupPID() {
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setupMotors() {
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  rotateMotor(0, 0);
}

void setupMPU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // Your MPU6050 calibration offsets (adjust if needed)
  mpu.setXAccelOffset(-150);
  mpu.setYAccelOffset(-1807);
  mpu.setZAccelOffset(3522);
  mpu.setXGyroOffset(29);
  mpu.setYGyroOffset(-93);
  mpu.setZGyroOffset(-30);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("❌ MPU DMP Initialization Failed!");
  }
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupMPU();
  setupPID();

  Serial.println("\n=== Self-Balancing Robot PID System ===");
  Serial.println("Pitch PID values:");
  Serial.print("Kp: "); Serial.println(PID_PITCH_KP);
  Serial.print("Ki: "); Serial.println(PID_PITCH_KI);
  Serial.print("Kd: "); Serial.println(PID_PITCH_KD);

  Serial.println("Yaw PID values:");
  Serial.print("Kp: "); Serial.println(PID_YAW_KP);
  Serial.print("Ki: "); Serial.println(PID_YAW_KI);
  Serial.print("Kd: "); Serial.println(PID_YAW_KD);
  Serial.println("=====================================");
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    yawGyroRate = gy.z;
    pitchGyroAngle = ypr[1] * 180 / M_PI;

    // === FALL DETECTION ===
    if (abs(pitchGyroAngle - setpointPitchAngle) > MAX_FALL_ANGLE) {
      if (!robotFallen) {
        robotFallen = true;
        fallDetectedTime = millis();
        rotateMotor(0, 0);
        Serial.println("⚠️ Robot FALL detected! Motors stopped.");
      }
    }

    // === HOLD MOTOR OFF after fall ===
if (robotFallen) {
      rotateMotor(0, 0);
      if (millis() - fallDetectedTime > FALL_HOLD_TIME) {
        robotFallen = false; // allow restart after delay
        Serial.println("✅ Ready again.");
      }
      return; // skip rest of control loop
    }
    // === Compute PID only if upright ===
    pitchPID.Compute();
    yawPID.Compute();

    // === BALANCE DEADZONE (stop small drift) ===
    if (abs(pitchGyroAngle - setpointPitchAngle) < BALANCE_DEADZONE) {
      rotateMotor(0, 0);
    } else {
      rotateMotor(pitchPIDOutput + yawPIDOutput, pitchPIDOutput - yawPIDOutput);
    }

#ifdef PRINT_DEBUG_BUILD
    Serial.print("Pitch: ");
    Serial.print(pitchGyroAngle);
    Serial.print(" | Setpoint: ");
    Serial.print(setpointPitchAngle);
    Serial.print(" | PID Output: ");
    Serial.print(pitchPIDOutput);
    Serial.print(" | Yaw Output: ");
    Serial.println(yawPIDOutput);
#endif
  }
}

// === Motor control function ===
void rotateMotor(int speed1, int speed2) {
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;
  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);

  analogWrite(enableMotor1, speed1);
  analogWrite(enableMotor2, speed2);
}

