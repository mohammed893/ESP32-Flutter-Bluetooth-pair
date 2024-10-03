import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';

class BluetoothDeviceListEntry extends ListTile {
  BluetoothDeviceListEntry({
    required BluetoothDevice device,
    int? rssi,
    GestureTapCallback? onTap,
    GestureLongPressCallback? onLongPress,
    bool enabled = true,
  }) : super(
    onTap: onTap,
    onLongPress: onLongPress,
    enabled: enabled,
      leading: device.name == "FlexiScan Device"
          ? FaIcon(
        FontAwesomeIcons.shirt,
        size: 25,  // Adjust size as needed
        color: Colors.grey,  // Gold color
        shadows: [
          BoxShadow(
            color: Colors.green.withOpacity(1),
            spreadRadius: 100,
            blurRadius: 10,
            offset: const Offset(0, 0), // Position of shadow
          ),
        ],
      )
          :const Icon(Icons.devices),
    title: Text(device.name ?? ""),
    // subtitle: Text(device.address.toString()),
    trailing: Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        rssi != null
            ? Container(
          margin: new EdgeInsets.all(8.0),
          child: DefaultTextStyle(
            style: _computeTextStyle(rssi),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                Text(rssi.toString()),
                Text('dBm'),
              ],
            ),
          ),
        )
            : Container(width: 0, height: 0),
        device.isConnected
            ? Icon(Icons.import_export)
            : Container(width: 0, height: 0),
        device.isBonded
            ? Icon(Icons.link)
            : Container(width: 0, height: 0),
      ],
    ),
  );

  static TextStyle _computeTextStyle(int rssi) {
    /**/ if (rssi >= -35)
      return TextStyle(color: Colors.greenAccent[700]);
    else if (rssi >= -45)
      return TextStyle(
          color: Color.lerp(
              Colors.greenAccent[700], Colors.lightGreen, -(rssi + 35) / 10));
    else if (rssi >= -55)
      return TextStyle(
          color: Color.lerp(
              Colors.lightGreen, Colors.lime[600], -(rssi + 45) / 10));
    else if (rssi >= -65)
      return TextStyle(
          color: Color.lerp(Colors.lime[600], Colors.amber, -(rssi + 55) / 10));
    else if (rssi >= -75)
      return TextStyle(
          color: Color.lerp(
              Colors.amber, Colors.deepOrangeAccent, -(rssi + 65) / 10));
    else if (rssi >= -85)
      return TextStyle(
          color: Color.lerp(
              Colors.deepOrangeAccent, Colors.redAccent, -(rssi + 75) / 10));
    else
      /*code symmetry*/
      return TextStyle(color: Colors.redAccent);
  }
}