import 'package:flexiscan101/detailPage.dart';
import 'package:flexiscan101/ex1.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(

        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});
  final String title;
  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> with WidgetsBindingObserver {
  BluetoothState _bluetoothState = BluetoothState.UNKNOWN;
  List<BluetoothDevice> devices = [];
  @override
  void initState(){
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _getBTState();
    _stateChangeListen();
    _listBondedDevices();
  }
  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }
  @override
  void didChangeDependencies(){
    print("didChangeDependencies"); //after init state and after deps changes
    super.didChangeDependencies();
  }
  @override
  void didChangeAppLifeCycle(AppLifecycleState state) {
    if(state.index == 0){
      if(_bluetoothState.isEnabled){
        _listBondedDevices();
      }
    }
    super.activate();
  }

  _getBTState(){
    FlutterBluetoothSerial.instance.state.then((state){
      _bluetoothState = state;
      if(_bluetoothState.isEnabled){
        _listBondedDevices();
      }
      setState(() {

      });
    });
  }
  _stateChangeListen(){
    FlutterBluetoothSerial.instance.onStateChanged().listen((BluetoothState state){
      _bluetoothState = state;
      if(_bluetoothState.isEnabled){
        _listBondedDevices();
      }else{
        devices.clear();
      }
      print("State isEnabled: ${state.isEnabled}");
      setState(() {
      });
    });
  }

  _listBondedDevices(){
    FlutterBluetoothSerial.instance.getBondedDevices().then((List<BluetoothDevice> bondedDevices){
      devices = bondedDevices;
      setState(() {
      });
    });
  }

  @override
  Widget build(BuildContext context) {

    return Scaffold(
      appBar: AppBar(
      ),
      body: Container(child: Column(children: <Widget>[
        SwitchListTile(value: _bluetoothState.isEnabled,
            onChanged: (bool val){
              future() async{
                if(val){
                  await FlutterBluetoothSerial.instance.requestEnable();
                }else{
                  await FlutterBluetoothSerial.instance.requestDisable();
                }
                future().then((_){
                  setState(() {
                  });
                });
              }
            } ,
            title: Text("Enable Bluetooth")),
        ListTile(title: Text("Bluetooth Status"
        ),
          subtitle: Text(_bluetoothState.toString()),
          trailing: ElevatedButton(onPressed: (){
            FlutterBluetoothSerial.instance.openSettings();
          }, child: Text("Settings")),
        ),
        Expanded(child: ListView(children: devices.map((_device) => BluetoothDeviceListEntry(device: _device,
        enabled: true,
        onTap: (){
          print("Item");
          _startConnection(context, _device);
        },)).toList(),))
      ],
      ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: (){},
        tooltip: 'Increment',
        child: const Icon(Icons.add),
      ), // This trailing comma makes auto-formatting nicer for build methods.
    );
  }
  void _startConnection (BuildContext context , BluetoothDevice server){
    Navigator.of(context).push(MaterialPageRoute(builder: (context){
      return DetailPage(server: server);
    }));
  }
}
