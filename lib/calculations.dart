import 'dart:math';

// Sensor data structure
class SensorData {
  final double accX;
  final double accY;
  final double accZ;

  SensorData(this.accX, this.accY, this.accZ);
}

// Function to calculate angle between two vectors
double calculateAngle(SensorData vector1, SensorData vector2) {
  double dotProduct = vector1.accX * vector2.accX +
      vector1.accY * vector2.accY +
      vector1.accZ * vector2.accZ;

  double magnitude1 = sqrt(vector1.accX * vector1.accX +
      vector1.accY * vector1.accY +
      vector1.accZ * vector1.accZ);
  double magnitude2 = sqrt(vector2.accX * vector2.accX +
      vector2.accY * vector2.accY +
      vector2.accZ * vector2.accZ);

  return acos(dotProduct / (magnitude1 * magnitude2)) * (180 / pi); // Convert to degrees
}

// Function to calculate lateral tilt based on the X and Z components
double calculateLateralTilt(SensorData sensor) {
  return atan2(sensor.accZ, sensor.accX) * (180 / pi); // Convert to degrees
}

// Function to check posture based on angles and additional detections
String assessPosture(SensorData c7, SensorData t8, SensorData l4) {
  // Calculate angles between segments
  double angleC7T8 = calculateAngle(c7, t8);
  double angleT8L4 = calculateAngle(t8, l4);

  // Calculate lateral tilt
  double lateralTiltC7 = calculateLateralTilt(c7);
  double lateralTiltT8 = calculateLateralTilt(t8);
  double lateralTiltL4 = calculateLateralTilt(l4);

  // Define threshold values for poor posture
  const double slouchingThreshold = 15.0; // degrees
  const double hunchingThreshold = 20.0; // degrees
  const double lateralBendingThreshold = 10.0; // degrees
  const double forwardTiltThreshold = 25.0; // degrees

  String assessment = "Posture Good: Alignment appears normal.";

  if (angleC7T8 > slouchingThreshold || angleT8L4 > slouchingThreshold) {
    assessment = "Posture Alert: Potential slouching detected.";
  }
  if (angleC7T8 > hunchingThreshold && angleT8L4 > hunchingThreshold) {
    assessment = "Posture Alert: Hunching detected!";
  }
  if (lateralTiltC7.abs() > lateralBendingThreshold || lateralTiltT8.abs() > lateralBendingThreshold || lateralTiltL4.abs() > lateralBendingThreshold) {
    assessment = "Posture Alert: Excessive lateral bending detected!";
  }
  if (angleC7T8 > forwardTiltThreshold || angleT8L4 > forwardTiltThreshold) {
    assessment = "Posture Alert: Excessive forward tilt detected!";
  }

  return assessment;
}

