#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#define LC 51.0
#define L1 65.0
#define L2 121.0

SoftwareSerial Bluetooth(12, 9);
int dataIn = 0;

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
unsigned long interval = 60;
float currentAngles[3];
float points[10][2];
float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;
float x1, x2, x3, x4, x5, x6;
float y1, y2, y3, y4, y5, y6;
float z1, z2, z3, z4, z5, z6;
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
int step = 0;
int mode = 0;
bool haveMotorsArrived = false;

Servo coxa1, femur1, tibia1;
Servo coxa2, femur2, tibia2;
Servo coxa3, femur3, tibia3;
Servo coxa4, femur4, tibia4;
Servo coxa5, femur5, tibia5;
Servo coxa6, femur6, tibia6;

void setupServos();
void calculatePoints();
float getZFromEquationGivenX(int x);
void readBluetooth();
void updateEndPoints();
void updateMotors();
void calculateAngles(float x, float y, float z, int legNumber);
void getCoxaForLegNumber(float x, float y, int legNumber);
void checkForEmotes();
void wave();
void hype();

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  Bluetooth.setTimeout(1);
  setupServos();
  calculatePoints();
}

void loop() {
  readBluetooth();
  updateEndPoints();

  currentTime = millis();
  if (currentTime - previousTime > interval) {
    updateMotors();
    haveMotorsArrived = true;
    previousTime = currentTime;
  } else {
    haveMotorsArrived = false;
  }

  checkForEmotes();
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
  for (int i = 0, x = -40; i <= 4; i++, x += 16) {
    points[i][0] = x;
    points[i][1] = getZFromEquationGivenX(x) - 80.0;
  }

  for (int i = 5, x = 40; i <= 9; i++, x -= 16) {
    points[i][0] = x;
    points[i][1] = -80.0;
  }
}

float getZFromEquationGivenX(int x) {
  return -0.025 * sq(x) + 40.0;
}

void readBluetooth() {
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();

    switch (dataIn) {
      case 0:
        step = 0;
        mode = 0;
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

      default:
        if (dataIn >= 10 && dataIn <= 45) {
          offsetX = map(dataIn, 10, 45, -50, 50);
        }
        
        if (dataIn >= 50 && dataIn <= 85) {
          offsetY = map(dataIn, 50, 85, -27, 27);
        }
        
        if (dataIn >= 90 && dataIn <= 125) {
          offsetZ = map(dataIn, 90, 125, 20, -78);
        }
        break;
    }
  }
}

void updateEndPoints() {
  if (mode == 0 || mode == 6 || mode == 7) {
    x2 = 0.0 + offsetX;
    z1 = z2 = z3 = z4 = z5 = z6 = -80.0 + offsetZ;
  } else {
    x2 = points[step][0] + offsetX;
    z2 = z4 = z6 = points[step][1] + offsetZ;
    z1 = z3 = z5 = (step <= 4) ? (-80.0 + offsetZ) : (points[step - 5][1] + offsetZ);

    if (haveMotorsArrived) {
      step = (mode == 2 || mode == 3) ? (step + 1) : (step - 1);
    }
    
    if (step == 10) {
      step = 0;
    } else if (step == -1) {
      step = 9;
    }
  }

  x1 = -x2 + 82.0 + 2 * offsetX;
  x3 = -x2 - 82.0 + 2 * offsetX;
  x4 = (mode == 2 || mode == 5) ? (x2 - 82.0) : (-x2 - 82.0 + 2 * offsetX);
  x5 = (mode == 2 || mode == 5) ? (-x2 + 2 * offsetX) : (x2);
  x6 = (mode == 2 || mode == 5) ? (x2 + 82.0) : (-x2 + 82.0 + 2 * offsetX);
  y1 = 82.0 + offsetY;
  y2 = 116.0 + offsetY;
  y3 = 82.0 + offsetY;
  y4 = -82.0 + offsetY;
  y5 = -116.0 + offsetY;
  y6 = -82.0 + offsetY;
}

void updateMotors() {
  calculateAngles(x1, y1, z1, 1);
  
  coxa1.write(int(currentAngles[0]));
  femur1.write(int(currentAngles[1]));
  tibia1.write(int(currentAngles[2]));

  calculateAngles(x5, y5, z5, 5);

  coxa5.write(int(currentAngles[0]));
  femur5.write(int(currentAngles[1]));
  tibia5.write(int(currentAngles[2]));

  calculateAngles(x3, y3, z3, 3);

  coxa3.write(int(currentAngles[0]));
  femur3.write(int(currentAngles[1]));
  tibia3.write(int(currentAngles[2]));

  calculateAngles(x6, y6, z6, 6);

  coxa6.write(int(currentAngles[0]));
  femur6.write(int(currentAngles[1]));
  tibia6.write(int(currentAngles[2]));

  calculateAngles(x2, y2, z2, 2);

  coxa2.write(int(currentAngles[0]));
  femur2.write(int(currentAngles[1]));
  tibia2.write(int(currentAngles[2]));

  calculateAngles(x4, y4, z4, 4);

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

    if (isnan(theta_coxa) || isnan(theta_femur) || isnan(theta_tibia)) {
      Serial.println("\nTrue");
    }
  } else {
    Serial.println("\nTrue 2");
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
  updateMotors();
  if (mode == 6) {
    wave();
  } else if (mode == 7) {
    hype();
  }
}

void wave() {
  femur1.write(180);
  tibia1.write(90);
  delay(200);

  for(int i = 0; i <= 2; i++) {
    coxa1.write(45);
    delay(400);
    coxa1.write(135);
    delay(400);
  }

  for(int i = 0; i <= 3; i++) {
    tibia1.write(70);
    delay(300);
    tibia1.write(170);
    delay(300);
  }
}

void hype() {
  calculateAngles(x1, y1, z1, 1);
  coxa1.write(int(currentAngles[0]));
  femur1.write(int(currentAngles[1]));
  tibia1.write(int(currentAngles[2]));

  calculateAngles(x3, y3, z3, 3);
  coxa3.write(int(currentAngles[0]));
  femur3.write(int(currentAngles[1]));
  tibia3.write(int(currentAngles[2]));

  calculateAngles(x5, y5, z5, 5);
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

  calculateAngles(x2, y2, z2, 2);
  coxa2.write(int(currentAngles[0]));
  femur2.write(int(currentAngles[1]));
  tibia2.write(int(currentAngles[2]));

  calculateAngles(x4, y4, z4, 4);
  coxa4.write(int(currentAngles[0]));
  femur4.write(int(currentAngles[1]));
  tibia4.write(int(currentAngles[2]));

  calculateAngles(x6, y6, z6, 6);
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