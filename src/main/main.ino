#include <math.h>                                                                    // Include math.h to use its mathematical functions
#include <Servo.h>                                                                   // Include Servo.h to control the servo motors
#include <SoftwareSerial.h>

#define LC 51.0                                                                      // Coxa length
#define L1 65.0                                                                      // Femur length
#define L2 121.0                                                                     // Tibia length

SoftwareSerial Bluetooth(12, 9); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)
int dataIn = 0;

float calibration[6][3] = {
  {-4.0, -7.0, 4.0},                                                                 // Leg 1 calibration: coxa, femur, tibia
  {3.0, 2.0, -4.0},                                                                  // Leg 2 calibration: coxa, femur, tibia
  {1.0, -11.0, -8.0},                                                                // Leg 3 calibration: coxa, femur, tibia
  {-3.0, -5.0, -4.0},                                                                // Leg 4 calibration: coxa, femur, tibia
  {-7.0, -6.0, -5.0},                                                                // Leg 5 calibration: coxa, femur, tibia
  {-5.0, -3.0, -1.0}                                                                 // Leg 6 calibration: coxa, femur, tibia
};

float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

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
      }
      else {
        theta_coxa -= 135.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 5:
      if (theta_coxa < 0) {
        theta_coxa += 270.0;
      }
      else {
        theta_coxa -= 90.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;

    case 6:
      if (theta_coxa < 0) {
        theta_coxa += 315.0;
      }
      else {
        theta_coxa -= 45.0;
      }
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      break;
  }
}

float getZFromEquationGivenX(int x) {
  return -0.025 * sq(x) + 40.0;
}

Servo coxa1;
Servo femur1;
Servo tibia1;

Servo coxa2;
Servo femur2;
Servo tibia2;

Servo coxa3;
Servo femur3;
Servo tibia3;

Servo coxa4;
Servo femur4;
Servo tibia4;

Servo coxa5;
Servo femur5;
Servo tibia5;

Servo coxa6;
Servo femur6;
Servo tibia6;

float currentAngles[3];                                                           // Indexes: coxa at 0, femur at 1, tibia at 2
float points[10][2];                                                              // X and Z coordinates

void calculateAngles(float currentPoint[], int legNumber) {
  float x = currentPoint[0];                                                      // Extract x coordinate
  float y = currentPoint[1];                                                      // Extract y coordinate
  float z = currentPoint[2];                                                      // Extract z coordinate

  L0 = sqrt(sq(x) + sq(y)) - LC;                                    // Calculate the distance from the coxa joint to the leg endpoint projection on the yz-plane
  L3 = sqrt(sq(L0) + sq(z));                                        // Calculate the distance from the femur joint to the leg endpoint

  if(L3 < (L2 + L1) && L3 > (L2 - L1)) {
    getCoxaForLegNumber(x, y, legNumber);                           // Call function to calculate the coxa angle given leg number

    phi_tibia = acos((sq(L1) + sq(L2) - sq(L3)) / (2 * L1 * L2));// Calculate tibia angle in radians
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + calibration[legNumber - 1][2];    // Convert tibia angle to degrees
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    gamma_femur = atan2(z, L0);                                                      // Calculate the angle between the leg and the yz-plane (femur-tibia joint angle)
    phi_femur = acos((sq(L1) + sq(L3) - sq(L2)) / (2 * L1 * L3));    // Calculate the angle between the femur and tibia segments in the yz-plane using the Law of Cosines
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + calibration[legNumber - 1][1];                                                 // Convert angle to degrees
    theta_femur = constrain(theta_femur, 0.0, 180.0);                                           // Calculate femur angle by summing the angles in the yz-plane, with an empirically determined correction

    currentAngles[0] = theta_coxa;                                                      // Store calculated coxa angle
    currentAngles[1] = theta_femur;                                                     // Store calculated femur angle
    currentAngles[2] = theta_tibia;                                                     // Store calculated tibia angle

    if (isnan(theta_coxa) || isnan(theta_femur) || isnan(theta_tibia)) {
      Serial.println("\nTrue");
    }
  }
  else {
    Serial.println("\nTrue 2");
  }
}

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600); // Default baud rate of the Bluetooth module
  Bluetooth.setTimeout(1);
  delay(20);

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

  for (int i = 0, x = -40; i <= 4; i++, x += 16) {
    points[i][0] = x;
    points[i][1] = getZFromEquationGivenX(x) - 80.0;
  }

  for (int i = 5, x = 40; i <= 9; i++, x -= 16) {
    points[i][0] = x;
    points[i][1] = -80.0;
  }

  delay(20);
}

int step = 0;
int mode = 0;
float x, z, zInv;

void loop() {
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();

    if (dataIn == 2) {
      step = 0;
      mode = 2;
    }
    else if (dataIn == 5) {
      step = 9;
      mode = 5;
    }
    else if (dataIn == 0) {
      step = 0;
      mode = 0;
    }
  }

  if (mode == 2) {
    x = points[step][0];
    z = points[step][1];

    if (step <= 4) {
      zInv = -80.0;
    }
    else {
      zInv = points[step - 5][1];
    }

    step++;

    if (step == 10) {
      step = 0;
    }
  }
  else if (mode == 5) {
    x = points[step][0];
    z = points[step][1];

    if (step <= 4) {
      zInv = -80.0;
    }
    else {
      zInv = points[step - 5][1];
    }

    step--;

    if (step == -1) {
      step = 9;
    }
  }
  else if (mode == 0) {
    step = 0;
    x = 0;
    z = -80;
    zInv = -80;
  }

  float currLegPoint1[] = {-x + 82.0, 82.0, zInv};
  float currLegPoint2[] = {x, 116.0, z};
  float currLegPoint3[] = {-x - 82.0, 82.0, zInv};
  float currLegPoint4[] = {x - 82.0, -82.0, z};
  float currLegPoint5[] = {-x, -116.0, zInv};
  float currLegPoint6[] = {x + 82.0, -82.0, z};

  calculateAngles(currLegPoint1, 1);

  coxa1.write(int(currentAngles[0]));
  femur1.write(int(currentAngles[1]));
  tibia1.write(int(currentAngles[2]));

  calculateAngles(currLegPoint5, 5);
      
  coxa5.write(int(currentAngles[0]));
  femur5.write(int(currentAngles[1]));
  tibia5.write(int(currentAngles[2]));

  calculateAngles(currLegPoint3, 3);
      
  coxa3.write(int(currentAngles[0]));
  femur3.write(int(currentAngles[1]));
  tibia3.write(int(currentAngles[2]));

  calculateAngles(currLegPoint6, 6);
      
  coxa6.write(int(currentAngles[0]));
  femur6.write(int(currentAngles[1]));
  tibia6.write(int(currentAngles[2]));

  calculateAngles(currLegPoint2, 2);
      
  coxa2.write(int(currentAngles[0]));
  femur2.write(int(currentAngles[1]));
  tibia2.write(int(currentAngles[2]));

  calculateAngles(currLegPoint4, 4);
      
  coxa4.write(int(currentAngles[0]));
  femur4.write(int(currentAngles[1]));
  tibia4.write(int(currentAngles[2]));

  delay(50);
}
