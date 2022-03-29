/*
  Arduino Nano 33 BLE Getting Started
  BLE peripheral with a simple Hello World greeting service that can be viewed
  on a mobile phone
  Adapted from Arduino BatteryMonitor example
*/

#include <ArduinoBLE.h>
//#include <Arduino_LSM9DS1.h>

#define GPSSerial Serial1

BLEService BTserial;
static const char* greeting = "Hello World!";

BLEService greetingService("180C");  // User defined service
BLEStringCharacteristic greetingCharacteristic("2A56",  // standard 16-bit characteristic UUID
    BLERead, 13); // remote clients will only be able to read this

BLEService batteryService("1101");
BLEUnsignedCharCharacteristic batteryLevelChar("2101", BLERead | BLENotify);

// BLEService imuService("3303");
// BLEUnsignedCharCharacteristic accelerometerChar("4404", BLERead | BLENotify);
// BLEUnsignedCharCharacteristic gyroscopeChar("5505", BLERead | BLENotify);

// BLEService gyroscopeService("1101");
// BLEUnsignedCharCharacteristic gyroscopeChar("2101", BLERead | BLENotify);

// BLEService GPSService("6606");
// BLEUnsignedCharCharacteristic gpsServiceChar("7707", BLERead | BLENotify);

float x, y, z;
int degreesX = 0;
int degreesY = 0;
String accelerometer_data; 


void setup() {
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE");  // Set name for connection
  BLE.setAdvertisedService(greetingService); // Advertise service
  greetingService.addCharacteristic(greetingCharacteristic); // Add characteristic to service
  BLE.addService(greetingService); // Add service
  greetingCharacteristic.setValue(greeting); // Set greeting string

  BLE.setLocalName("BatteryMonitor");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);

  // BLE.setLocalName("IMU");
  // BLE.setAdvertisedService(imuService);
  // imuService.addCharacteristic(accelerometerChar);
  // imuService.addCharacteristic(gyroscopeChar);
  // BLE.addService(imuService);


  // BLE.setLocalName("Gyroscope");
  // BLE.setAdvertisedService(gyroscopeService);
  // gyroscopeService.addCharacteristic(gyroscopeChar);
  // BLE.addService(gyroscopeService);

  // BLE.setLocalName("GPS");
  // BLE.setAdvertisedService(GPSService);
  // GPSService.addCharacteristic(gpsServiceChar);
  // BLE.addService(GPSService);


  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");

    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  
  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println("Hz");
}

void loop() {
  BLEDevice central = BLE.central();  // Wait for a BLE central to connect

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central MAC: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()){
      // battery
      int battery = analogRead(A0);
      int batteryLevel = map(battery, 0, 1023, 0, 100);
      Serial.print("Battery Level % is now: ");
      Serial.println(batteryLevel);
      batteryLevelChar.writeValue(batteryLevel);
      delay(200);


      // accelerometer
      // if (IMU.accelerationAvailable()) {
      //   IMU.readAcceleration(x, y, z);
      //       Serial.print(x);
      //       Serial.print('\t');
      //       Serial.print(y);
      //       Serial.print('\t');
      //       Serial.println(z);
      //       accelerometer_data = Serial.readString();
            //BLE.print(Arduino_data);

      //} // keep looping while connected
    
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
    }

    

//    if (x > 0.1) {
//      x = 100 * x;
//      degreesX = map(x, 0, 97, 0, 90);
//      Serial.print("Tilting up ");
//      Serial.print(degreesX);
//      Serial.println("  degrees");
//    }
//    if (x < -0.1) {
//      x = 100 * x;
//      degreesX = map(x, 0, -100, 0, 90);
//      Serial.print("Tilting down ");
//      Serial.print(degreesX);
//      Serial.println("  degrees");
//    }
//    if (y > 0.1) {
//      y = 100 * y;
//      degreesY = map(y, 0, 97, 0, 90);
//      Serial.print("Tilting left ");
//      Serial.print(degreesY);
//      Serial.println("  degrees");
//    }
//    if (y < -0.1) {
//      y = 100 * y;
//      degreesY = map(y, 0, -100, 0, 90);
//      Serial.print("Tilting right ");
//      Serial.print(degreesY);
//      Serial.println("  degrees");
//    }
//    delay(1000);
  
//    if (Serial.available())
//    {
//      accelerometer_data = Serial.readString();
//      BTserial.println(Arduino_data);
//    }
  }
}
