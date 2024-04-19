#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>
#include <Wire.h>
#include <PID_v1_bc.h>

#define LC 51.0
#define L1 65.0
#define L2 121.0
#define WIDTH 120.0
#define HEIGHT 60.0
#define NUM_POINTS 10
#define INTERVAL 100
#define NUM_LEGS 6
#define MAX_ROLL_ANGLE 20
#define MAX_PITCH_ANGLE 20
#define MAX_YAW_ANGLE 35

double Setpoint = 0.0;
double PIDRollInput, PIDRollOutput;
double PIDPitchInput, PIDPitchOutput;
double kp = 1.5;
double ki = 3.0;
double kd = 0.001;
PID bodyRollPID(&PIDRollInput, &PIDRollOutput, &Setpoint, kp, ki, kd, DIRECT);
PID bodyPitchPID(&PIDPitchInput, &PIDPitchOutput, &Setpoint, kp, ki, kd, DIRECT);

float currentAngles[3];
float points[NUM_POINTS][2];
float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;
float xCoord[6], yCoord[6], zCoord[6];
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
float pitchDeg, rollDeg;
float xOffsets[] = { 110.4, 0, -110.4, -110.4, 0, 110.4 };
float yOffsets[] = { 58.4, 90.8, 58.4, -58.4, -90.8, -58.4 };
float cosTheta, sinTheta;
float tempX, tempY, tempZ;
float calibration[6][3] = {
  { -4.0, -7.0, 4.0 },
  { 3.0, 2.0, -4.0 },
  { 1.0, -11.0, -8.0 },
  { -3.0, -5.0, -4.0 },
  { -7.0, -6.0, -5.0 },
  { -5.0, -3.0, -1.0 }
};

unsigned long previousTime = 0;
unsigned long currentTime = millis();

int dataIn = 0;
int roll = 0, pitch = 0, yaw = 0;
int step = 0;
int mode = 0;
int balanceRoll = 0, balancePitch = 0;
int prevRollDeg, prevPitchDeg;

int16_t ax, ay, az;

bool isTiltEnabled = false;
bool isSelfBalanceEnabled = false;
bool isSelfBalanceCustomEnabled = false;

Servo coxa1, femur1, tibia1;
Servo coxa2, femur2, tibia2;
Servo coxa3, femur3, tibia3;
Servo coxa4, femur4, tibia4;
Servo coxa5, femur5, tibia5;
Servo coxa6, femur6, tibia6;

SoftwareSerial Bluetooth(12, 9);

MPU6050 mpu;

void initCommunication();
void initPID();
void setupServos();
void calculatePoints();
float getZFromEquationGivenX(int x);
void readBluetooth();
void updateEndPoints();
void calculateBalanceAngles();
void getImuMesures();
void filterPitchAndRoll();
void calculateRollPitchPID();
void calculateRollPitchCustom();
void rotateX(float angle);
void rotateY(float angle);
void rotateZ(float angle);
void updateMotors();
void calculateAngles(float x, float y, float z, int legNumber);
void getCoxaForLegNumber(float x, float y, int legNumber);
void checkForEmotes();
void wave();
void hype();
void attack();

void setup() {
  initCommunication();
  initPID();
  setupServos();
  calculatePoints();
}

void loop() {
  readBluetooth();
  updateEndPoints();
  updateMotors();
  checkForEmotes();
}

void initCommunication() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  Bluetooth.setTimeout(1);
  Wire.begin();
  mpu.initialize();
}

void initPID() {
  bodyRollPID.SetMode(AUTOMATIC);
  bodyRollPID.SetOutputLimits(-MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
  bodyPitchPID.SetMode(AUTOMATIC);
  bodyPitchPID.SetOutputLimits(-MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
}

void setupServos() {
  coxa1.attach(22, 610, 2400);
  femur1.attach(23, 610, 2400);
  tibia1.attach(24, 610, 2400);

  coxa2.attach(29, 610, 2400);
  femur2.attach(28, 610, 2400);
  tibia2.attach(27, 610, 2400);

  coxa3.attach(31, 610, 2400);
  femur3.attach(32, 610, 2400);
  tibia3.attach(33, 610, 2400);

  coxa4.attach(36, 610, 2400);
  femur4.attach(35, 610, 2400);
  tibia4.attach(34, 610, 2400);

  coxa5.attach(38, 610, 2400);
  femur5.attach(39, 610, 2400);
  tibia5.attach(40, 610, 2400);

  coxa6.attach(43, 610, 2400);
  femur6.attach(44, 610, 2400);
  tibia6.attach(45, 610, 2400);
}

void calculatePoints() {
  for (int i = 0, x = -(WIDTH / 2); i <= NUM_POINTS / 2 - 1; i++, x += WIDTH / (NUM_POINTS / 2)) {
    points[i][0] = x;
    points[i][1] = getZFromEquationGivenX(x) - 80.0;
  }

  for (int i = NUM_POINTS / 2, x = WIDTH / 2; i <= NUM_POINTS - 1; i++, x -= WIDTH / (NUM_POINTS / 2)) {
    points[i][0] = x;
    points[i][1] = -80.0;
  }
}

float getZFromEquationGivenX(int x) {
  return -(HEIGHT / sq(WIDTH / 2)) * sq(x) + HEIGHT;
}

void readBluetooth() {
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();

    switch (dataIn) {
      case 0:
        step = 0;
        mode = 0;
        isTiltEnabled = false;
        isSelfBalanceEnabled = false;
        pitch = 0;
        roll = 0;
        yaw = 0;
        break;

      case 2:
        mode = 2;
        break;

      case 3:
        mode = 3;
        break;

      case 4:
        mode = 4;
        break;

      case 5:
        mode = 5;
        break;

      case 6:
        step = 0;
        mode = 6;
        break;

      case 7:
        step = 0;
        mode = 7;
        break;

      case 8:
        step = 0;
        mode = 8;
        break;

      case 9:
        mode = 9;
        break;

      case 10:
        mode = 10;
        break;

      case 11:
        mode = 11;
        isTiltEnabled = true;
        break;

      case 12:
        mode = 12;
        isSelfBalanceEnabled = true;
        break;
      case 13:
        mode = 13;
        isSelfBalanceCustomEnabled = true;
        break;

      default:
        if (dataIn >= 20 && dataIn <= 55) {
          if (isTiltEnabled) {
            roll = map(dataIn, 20, 55, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
          } else {
            offsetX = map(dataIn, 20, 55, -50, 50);
          }
        }

        if (dataIn >= 60 && dataIn <= 95) {
          if (isTiltEnabled) {
            pitch = map(dataIn, 60, 95, MAX_PITCH_ANGLE, -MAX_PITCH_ANGLE);
          } else {
            offsetY = map(dataIn, 60, 95, -50, 50);
          }
        }

        if (dataIn >= 100 && dataIn <= 135) {
          if (isTiltEnabled) {
            yaw = map(dataIn, 100, 135, -MAX_YAW_ANGLE, MAX_YAW_ANGLE);
          } else {
            offsetZ = map(dataIn, 100, 135, 20, -67);
          }
        }
        break;
    }
  }
}

void updateEndPoints() {
  if (mode == 0 || mode == 6 || mode == 7 || mode == 8 || mode == 11 || mode == 12 || mode == 13) {
    xCoord[1] = 0.0 + offsetX;
    zCoord[0] = zCoord[1] = zCoord[2] = zCoord[3] = zCoord[4] = zCoord[5] = -80.0 + offsetZ;
  } else {
    if (mode == 9 || mode == 10) {
      yCoord[1] = points[step][0] + offsetY;
    } else {
      xCoord[1] = points[step][0] + offsetX;
    }

    zCoord[1] = zCoord[3] = zCoord[5] = points[step][1] + offsetZ;
    zCoord[0] = zCoord[2] = zCoord[4] = (step <= NUM_POINTS / 2 - 1) ? (-80.0 + offsetZ) : (points[step - NUM_POINTS / 2][1] + offsetZ);

    currentTime = millis();
    if (currentTime - previousTime > INTERVAL) {
      step = (mode == 2 || mode == 3 || mode == 10) ? (step + 1) : (step - 1);
      previousTime = currentTime;
    }

    if (step == NUM_POINTS) {
      step = 0;
    } else if (step == -1) {
      step = NUM_POINTS - 1;
    }
  }

  xCoord[0] = (mode == 9 || mode == 10) ? (82.0 + offsetX) : (-xCoord[1] + 82.0 + 2 * offsetX);
  xCoord[1] = (mode == 9 || mode == 10) ? (0.0 + offsetX) : (xCoord[1]);
  xCoord[2] = (mode == 9 || mode == 10) ? (-82.0 + offsetX) : (-xCoord[1] - 82.0 + 2 * offsetX);
  xCoord[3] = (mode == 9 || mode == 10) ? (-82.0 + offsetX) : (mode == 2 || mode == 5) ? (xCoord[1] - 82.0)
                                                                                       : (-xCoord[1] - 82.0 + 2 * offsetX);
  xCoord[4] = (mode == 9 || mode == 10) ? (0.0 + offsetX) : (mode == 2 || mode == 5) ? (-xCoord[1] + 2 * offsetX)
                                                                                     : (xCoord[1]);
  xCoord[5] = (mode == 9 || mode == 10) ? (82.0 + offsetX) : (mode == 2 || mode == 5) ? (xCoord[1] + 82.0)
                                                                                      : (-xCoord[1] + 82.0 + 2 * offsetX);

  yCoord[0] = (mode == 9 || mode == 10) ? (-yCoord[1] + 82.0 + 2 * offsetY) : (82.0 + offsetY);
  yCoord[2] = (mode == 9 || mode == 10) ? (-yCoord[1] + 82.0 + 2 * offsetY) : (82.0 + offsetY);
  yCoord[3] = (mode == 9 || mode == 10) ? (yCoord[1] - 82.0) : (-82.0 + offsetY);
  yCoord[4] = (mode == 9 || mode == 10) ? (-yCoord[1] - 116.0 - 10.0 + 2 * offsetY) : (-116.0 + offsetY);
  yCoord[5] = (mode == 9 || mode == 10) ? (yCoord[1] - 82.0) : (-82.0 + offsetY);
  yCoord[1] = (mode == 9 || mode == 10) ? (yCoord[1] + 116.0 + 10.0) : (116.0 + offsetY);

  if (isSelfBalanceEnabled || isSelfBalanceCustomEnabled) {
    calculateBalanceAngles();
  }

  if (isTiltEnabled || isSelfBalanceEnabled || isSelfBalanceCustomEnabled) {
    rotateX(roll);
    rotateY(pitch);
    rotateZ(yaw);
  }
}

void calculateBalanceAngles() {
  getImuMesures();
  filterPitchAndRoll();

  if (isSelfBalanceEnabled == true) {
    calculateRollPitchPID();
  } else if (isSelfBalanceCustomEnabled) {
    calculateRollPitchCustom();
  }
}

void getImuMesures() {
  mpu.getAcceleration(&ax, &ay, &az);

  rollDeg = atan2(ay, az) * RAD_TO_DEG;
  pitchDeg = atan2(ax, az) * RAD_TO_DEG;
}

void filterPitchAndRoll() {
  rollDeg = 0.2 * rollDeg + 0.8 * prevRollDeg;
  pitchDeg = 0.2 * pitchDeg + 0.8 * prevPitchDeg;

  prevRollDeg = rollDeg;
  prevPitchDeg = pitchDeg;
}

void calculateRollPitchPID() {
  PIDRollInput = rollDeg;
  PIDPitchInput = pitchDeg;
  bodyRollPID.Compute();
  bodyPitchPID.Compute();

  roll = PIDRollOutput;
  pitch = PIDPitchOutput;

  delay(50);
}

void calculateRollPitchCustom() {
  balanceRoll += -rollDeg;
  balancePitch += -pitchDeg;

  balanceRoll = constrain(balanceRoll, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
  balancePitch = constrain(balancePitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);

  roll = balanceRoll;
  pitch = balancePitch;

  delay(100);
}

void rotateX(float angle) {
  cosTheta = cos(angle * DEG_TO_RAD);
  sinTheta = sin(angle * DEG_TO_RAD);

  for (int i = 0; i <= NUM_LEGS - 1; i++) {
    tempY = yCoord[i] + yOffsets[i];
    tempZ = zCoord[i];

    yCoord[i] = tempY * cosTheta - tempZ * sinTheta - yOffsets[i];
    zCoord[i] = tempY * sinTheta + tempZ * cosTheta;
  }
}

void rotateY(float angle) {
  cosTheta = cos(angle * DEG_TO_RAD);
  sinTheta = sin(angle * DEG_TO_RAD);

  for (int i = 0; i <= NUM_LEGS - 1; i++) {
    tempX = xCoord[i] + xOffsets[i];
    tempZ = zCoord[i];

    xCoord[i] = tempX * cosTheta + tempZ * sinTheta - xOffsets[i];
    zCoord[i] = -tempX * sinTheta + tempZ * cosTheta;
  }
}

void rotateZ(float angle) {
  cosTheta = cos(angle * DEG_TO_RAD);
  sinTheta = sin(angle * DEG_TO_RAD);

  for (int i = 0; i <= NUM_LEGS - 1; i++) {
    tempX = xCoord[i] + xOffsets[i];
    tempY = yCoord[i] + yOffsets[i];

    xCoord[i] = tempX * cosTheta - tempY * sinTheta - xOffsets[i];
    yCoord[i] = tempX * sinTheta + tempY * cosTheta - yOffsets[i];
  }
}

void updateMotors() {
  calculateAngles(xCoord[0], yCoord[0], zCoord[0], 1);

  coxa1.write(int(currentAngles[0]));
  femur1.write(int(currentAngles[1]));
  tibia1.write(int(currentAngles[2]));

  calculateAngles(xCoord[4], yCoord[4], zCoord[4], 5);

  coxa5.write(int(currentAngles[0]));
  femur5.write(int(currentAngles[1]));
  tibia5.write(int(currentAngles[2]));

  calculateAngles(xCoord[2], yCoord[2], zCoord[2], 3);

  coxa3.write(int(currentAngles[0]));
  femur3.write(int(currentAngles[1]));
  tibia3.write(int(currentAngles[2]));

  calculateAngles(xCoord[5], yCoord[5], zCoord[5], 6);

  coxa6.write(int(currentAngles[0]));
  femur6.write(int(currentAngles[1]));
  tibia6.write(int(currentAngles[2]));

  calculateAngles(xCoord[1], yCoord[1], zCoord[1], 2);

  coxa2.write(int(currentAngles[0]));
  femur2.write(int(currentAngles[1]));
  tibia2.write(int(currentAngles[2]));

  calculateAngles(xCoord[3], yCoord[3], zCoord[3], 4);

  coxa4.write(int(currentAngles[0]));
  femur4.write(int(currentAngles[1]));
  tibia4.write(int(currentAngles[2]));
}

void calculateAngles(float x, float y, float z, int legNumber) {
  L0 = sqrt(sq(x) + sq(y)) - LC;
  L3 = sqrt(sq(L0) + sq(z));

  if (L3 < (L2 + L1) && L3 > (L2 - L1)) {
    getCoxaForLegNumber(x, y, legNumber);

    phi_tibia = acos((sq(L1) + sq(L2) - sq(L3)) / (2 * L1 * L2));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + calibration[legNumber - 1][2];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    gamma_femur = atan2(z, L0);
    phi_femur = acos((sq(L1) + sq(L3) - sq(L2)) / (2 * L1 * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + calibration[legNumber - 1][1];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    currentAngles[0] = theta_coxa;
    currentAngles[1] = theta_femur;
    currentAngles[2] = theta_tibia;
  }
}

void getCoxaForLegNumber(float x, float y, int legNumber) {
  theta_coxa = atan2(x, y) * RAD_TO_DEG + calibration[legNumber - 1][0];

  switch (legNumber) {
    case 1:
      theta_coxa += 45.0;
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 2:
      theta_coxa += 90.0;
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 3:
      theta_coxa += 135.0;
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 4:
      if (theta_coxa < 0) {
        theta_coxa += 225.0;
      } else {
        theta_coxa -= 135.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 5:
      if (theta_coxa < 0) {
        theta_coxa += 270.0;
      } else {
        theta_coxa -= 90.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 6:
      if (theta_coxa < 0) {
        theta_coxa += 315.0;
      } else {
        theta_coxa -= 45.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;
  }
}

void checkForEmotes() {
  if (mode == 6) {
    wave();
  } else if (mode == 7) {
    hype();
  } else if (mode == 8) {
    attack();
  }
}

void wave() {
  femur1.write(180);
  tibia1.write(90);
  delay(200);

  for (int i = 0; i <= 2; i++) {
    coxa1.write(45);
    delay(400);
    coxa1.write(135);
    delay(400);
  }

  for (int i = 0; i <= 3; i++) {
    tibia1.write(70);
    delay(300);
    tibia1.write(170);
    delay(300);
  }
}

void hype() {
  calculateAngles(xCoord[0], yCoord[0], zCoord[0], 1);
  coxa1.write(int(currentAngles[0]));
  femur1.write(int(currentAngles[1]));
  tibia1.write(int(currentAngles[2]));

  calculateAngles(xCoord[2], yCoord[2], zCoord[2], 3);
  coxa3.write(int(currentAngles[0]));
  femur3.write(int(currentAngles[1]));
  tibia3.write(int(currentAngles[2]));

  calculateAngles(xCoord[4], yCoord[4], zCoord[4], 5);
  coxa5.write(int(currentAngles[0]));
  femur5.write(int(currentAngles[1]));
  tibia5.write(int(currentAngles[2]));

  delay(350);

  coxa2.write(90);
  femur2.write(180);
  tibia2.write(90);

  coxa4.write(90);
  femur4.write(180);
  tibia4.write(90);

  coxa6.write(90);
  femur6.write(180);
  tibia6.write(90);

  delay(350);

  calculateAngles(xCoord[1], yCoord[1], zCoord[1], 2);
  coxa2.write(int(currentAngles[0]));
  femur2.write(int(currentAngles[1]));
  tibia2.write(int(currentAngles[2]));

  calculateAngles(xCoord[3], yCoord[3], zCoord[3], 4);
  coxa4.write(int(currentAngles[0]));
  femur4.write(int(currentAngles[1]));
  tibia4.write(int(currentAngles[2]));

  calculateAngles(xCoord[5], yCoord[5], zCoord[5], 6);
  coxa6.write(int(currentAngles[0]));
  femur6.write(int(currentAngles[1]));
  tibia6.write(int(currentAngles[2]));

  delay(350);

  coxa1.write(90);
  femur1.write(180);
  tibia1.write(90);

  coxa3.write(90);
  femur3.write(180);
  tibia3.write(90);

  coxa5.write(90);
  femur5.write(180);
  tibia5.write(90);

  delay(350);
}

void attack() {
  coxa2.write(145);
  coxa5.write(35);
  coxa1.write(135);
  coxa6.write(45);

  for (int i = 0; i <= 2; i++) {
    femur1.write(140);
    tibia1.write(145);
    tibia6.write(180);
    femur6.write(180);
    delay(500);
    femur1.write(180);
    tibia1.write(180);
    femur6.write(140);
    tibia6.write(145);
    delay(500);
  }
}