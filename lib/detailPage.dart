import 'dart:convert';
import 'dart:typed_data';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

// Sensor data structure
class SensorData {
  final double accX;
  final double accY;
  final double accZ;
  final double gyroX;
  final double gyroY;
  final double gyroZ;

  SensorData(this.accX, this.accY, this.accZ, this.gyroX, this.gyroY, this.gyroZ);
}

// Function to assess posture based on accelerometer data
String assessPosture(SensorData sensor) {
  // Assuming the sensor is placed on the back (e.g., at T8), we can assess posture based on the tilt.
  // We'll calculate the angle between the gravity vector and the sensor's Z-axis.

  // Calculate the magnitude of the accelerometer vector
  double accMagnitude = sqrt(sensor.accX * sensor.accX +
      sensor.accY * sensor.accY +
      sensor.accZ * sensor.accZ);

  // Normalize the accelerometer readings
  double normAccX = sensor.accX / accMagnitude;
  double normAccY = sensor.accY / accMagnitude;
  double normAccZ = sensor.accZ / accMagnitude;

  // Calculate the tilt angle in degrees
  double tiltAngle = acos(normAccZ) * (180 / pi);

  const double tiltThreshold = 15.0; // degrees

  if (tiltAngle > tiltThreshold) {
    return "Posture Alert: Potential slouching detected.";
  }

  return "Posture Good: Alignment appears normal.";
}

class DetailPage extends StatefulWidget {
  const DetailPage({Key? key, required this.server}) : super(key: key);
  final BluetoothDevice server;

  @override
  State<DetailPage> createState() => _DetailPageState();
}

class _DetailPageState extends State<DetailPage> {
  BluetoothConnection? connection;
  bool isConnecting = true;
  bool get isConnected => connection != null && connection!.isConnected;
  bool isDisconnecting = false;

  // Sensor data
  String accX = '0.0', accY = '0.0', accZ = '0.0';
  String gyroX = '0.0', gyroY = '0.0', gyroZ = '0.0';

  String postureAssessment = "Posture assessment will appear here.";

  @override
  void initState() {
    super.initState();
    _getBTConnection();
  }

  @override
  void dispose() {
    if (isConnected) {
      isDisconnecting = true;
      connection!.dispose();
      connection = null;
    }
    super.dispose();
  }

  void _getBTConnection() {
    BluetoothConnection.toAddress(widget.server.address).then((_connection) {
      connection = _connection;
      setState(() {
        isConnecting = false;
        isDisconnecting = false;
      });

      _sendMessage("MY_SECRET_KEY");
      connection!.input!.listen(_onDataReceived).onDone(_showDisconnectionNotification);
    }).catchError((err) {
      Navigator.of(context).pop();
    });
  }

  void _showDisconnectionNotification() {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text("Disconnected from Device"), duration: Duration(seconds: 2)),
      );

      Future.delayed(const Duration(seconds: 2), () {
        Navigator.of(context).pop();
      });
    }
  }

  // Parsing the sensor data from ESP32
  void _onDataReceived(Uint8List data) {
    String dataString = String.fromCharCodes(data).trim();

    if (dataString.isEmpty || dataString.contains("Authentication successful")) return;

    if (dataString.contains("ACC:") && dataString.contains("GYRO:")) {
      try {
        // Split the data into accelerometer and gyroscope parts
        final sensorDataList = dataString.split("|").map((s) => s.trim()).toList();
        if (sensorDataList.length == 2) {
          // Parse accelerometer data
          final accData = _parseSensorData(sensorDataList[0]);
          // Parse gyroscope data
          final gyroData = _parseSensorData(sensorDataList[1]);

          // Update sensor data
          setState(() {
            accX = accData['X']!.toStringAsFixed(2);
            accY = accData['Y']!.toStringAsFixed(2);
            accZ = accData['Z']!.toStringAsFixed(2);

            gyroX = gyroData['X']!.toStringAsFixed(2);
            gyroY = gyroData['Y']!.toStringAsFixed(2);
            gyroZ = gyroData['Z']!.toStringAsFixed(2);
          });

          // Perform posture assessment
          SensorData sensor = SensorData(
            accData['X']!,
            accData['Y']!,
            accData['Z']!,
            gyroData['X']!,
            gyroData['Y']!,
            gyroData['Z']!,
          );

          setState(() {
            postureAssessment = assessPosture(sensor);
          });
        } else {
          print("Error: Unexpected data format");
        }
      } catch (e) {
        print("Error parsing data: $e");
      }
    } else {
      print("Received incomplete data: [$dataString]");
    }
  }

  Map<String, double> _parseSensorData(String sensorString) {
    final regex = RegExp(r'[A-Z]+:\s*X=(-?\d+\.?\d*)\s*Y=(-?\d+\.?\d*)\s*Z=(-?\d+\.?\d*)');
    final match = regex.firstMatch(sensorString);
    if (match != null && match.groupCount == 3) {
      return {
        'X': double.parse(match.group(1)!),
        'Y': double.parse(match.group(2)!),
        'Z': double.parse(match.group(3)!),
      };
    } else {
      throw FormatException("Failed to parse sensor data: $sensorString");
    }
  }

  void _sendMessage(String text) async {
    if (text.isNotEmpty && connection != null) {
      try {
        connection?.output.add(Uint8List.fromList(utf8.encode(text)));
        await connection?.output.allSent;
      } catch (e) {
        print("Failed to send message: $e");
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: isConnecting
            ? const Text('Connecting to Device ...')
            : isConnected
            ? const Text("Connected")
            : const Text("Disconnected"),
        centerTitle: true,
      ),
      body: SafeArea(
        child: isConnected
            ? SingleChildScrollView(
          child: Padding(
            padding: const EdgeInsets.all(16.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                SensorCard(
                  title: "Sensor Data",
                  accData: "X: $accX, Y: $accY, Z: $accZ",
                  gyroData: "X: $gyroX, Y: $gyroY, Z: $gyroZ",
                ),
                const SizedBox(height: 20),
                Text(
                  postureAssessment,
                  style: TextStyle(
                    fontSize: 20,
                    fontWeight: FontWeight.bold,
                    color: (postureAssessment.contains("Posture Alert"))
                        ? Colors.red
                        : Colors.green,
                  ),
                  textAlign: TextAlign.center,
                ),
              ],
            ),
          ),
        )
            : const Center(child: Text("Connect to a device")),
      ),
    );
  }
}

class SensorCard extends StatelessWidget {
  final String title;
  final String accData;
  final String gyroData;

  const SensorCard({
    required this.title,
    required this.accData,
    required this.gyroData,
  });

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.symmetric(vertical: 10),
      child: ListTile(
        title: Text(
          title,
          style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 8),
            Text("Accelerometer: $accData"),
            Text("Gyroscope: $gyroData"),
          ],
        ),
      ),
    );
  }
}