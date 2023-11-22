#include <math.h>                                                                  // Include math.h to use its mathematical functions

#define LC 51                                                                      // Coxa length
#define L1 65                                                                      // Femur length
#define L2 121                                                                     // Tibia length

double point[] = {0.0, 116.0, -121.0};                                             // Indexes: x at 0, y at 1, z at 2
double* currentAngles = (double*) malloc(sizeof(double) * 3);                      // Indexes: coxa at 0, femur at 1, tibia at 2

void calculateAngles(double currentPoint[]) {
  double x = currentPoint[0];                                                      // Extract x coordinate
  double y = currentPoint[1];                                                      // Extract y coordinate
  double z = currentPoint[2];                                                      // Extract z coordinate

  double radCoxa = atan2(y, x);                                                    // Calculate coxa angle in radians
  double degCoxa = degrees(radCoxa);                                               // Convert coxa angle to degrees

  double L0 = sqrt(pow(x, 2) + pow(y, 2)) - LC;                                    // Calculate the distance from the coxa joint to the leg endpoint projection on the yz-plane
  double L3 = sqrt(pow(L0, 2) + pow(z, 2));                                        // Calculate the distance from the femur joint to the leg endpoint

  double radTibia = acos((pow(L1, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * L1 * L2));  // Calculate tibia angle in radians
  double degTibia = degrees(radTibia) - 23.0;                                      // Convert tibia angle to degrees

  double radA = atan2(z, L0);                                                      // Calculate the angle between the leg and the yz-plane (femur-tibia joint angle)
  double degA = degrees(radA);                                                     // Convert angle to degrees
  double radB = acos((pow(L1, 2) + pow(L3, 2) - pow(L2, 2)) / (2 * L1 * L3));      // Calculate the angle between the femur and tibia segments in the yz-plane using the Law of Cosines
  double degB = degrees(radB);                                                     // Convert angle to degrees
  double degFemur = degA + degB + 104.0;                                           // Calculate femur angle by summing the angles in the yz-plane, with an empirically determined correction

  currentAngles[0] = degCoxa;                                                      // Store calculated coxa angle
  currentAngles[1] = degFemur;                                                     // Store calculated femur angle
  currentAngles[2] = degTibia;                                                     // Store calculated tibia angle
}

void setup() {
  Serial.begin(9600);                                                              // Initialize serial communication

  calculateAngles(point);                                                          // Calculate angles for the hard-coded point
  
  Serial.println(currentAngles[0]);                                                // Print coxa angle
  Serial.println(currentAngles[1]);                                                // Print femur angle
  Serial.println(currentAngles[2]);                                                // Print tibia angle
}

void loop() {

}
