#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

//  // BLE Battery Service
// BLEService batteryService("180F");

 // BLE IMU Service
BLEService imuService("180F");

// // BLE Battery Level Characteristic
// BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
//     BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

// BLE accelerometer Characteristic
BLEFloatCharacteristic accelerometerChar("2A19", 
    BLERead | BLENotify); 

int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

float oldAccelerometer = 0;  // last battery level reading from analog input
float oldAccX = 0;
float x, y, z;

void setup() {
    Serial.begin(9600);    // initialize serial communication
    while (!Serial);

    pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

    // begin initialization
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");

        while (1);
    }

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    /* Set a local name for the BLE device
        This name will appear in advertising packets
        and can be used by remote devices to identify this BLE device
        The name can be changed but maybe be truncated based on space left in advertisement packet
    */
    // BLE.setLocalName("BatteryMonitor");
    // BLE.setAdvertisedService(batteryService); // add the service UUID
    // batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
    // BLE.addService(batteryService); // Add the battery service
    // batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic

    BLE.setLocalName("IMU");
    BLE.setAdvertisedService(imuService); // add the service UUID
    imuService.addCharacteristic(accelerometerChar); // add the battery level characteristic
    BLE.addService(imuService); // Add the battery service
    accelerometerChar.writeValue(oldAccX); // set initial value for this characteristic
    /* Start advertising BLE.  It will start continuously transmitting BLE
        advertising packets and will be visible to remote BLE central devices
        until it receives a new connection */

    // start advertising
    BLE.advertise();

    Serial.println("Bluetooth device active, waiting for connections...");
    }

void loop() {
    // wait for a BLE central
    BLEDevice central = BLE.central();

    // if a central is connected to the peripheral:
    if (central) {
        Serial.print("Connected to central: ");
        // print the central's BT address:
        Serial.println(central.address());
        // turn on the LED to indicate the connection:
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("led on");
        // check the battery level every 200ms
        // while the central is connected:
        while (central.connected()) {
            long currentMillis = millis();
            // if 200ms have passed, check the battery level:
            if (currentMillis - previousMillis >= 200) {
                previousMillis = currentMillis;
                Serial.println(currentMillis);
                // updateBatteryLevel();
                updateAccelerometer();
            }
        }
        Serial.println("before led off");
        // when the central disconnects, turn off the LED:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}

// void updateBatteryLevel() {
//     /* Read the current voltage level on the A0 analog input pin.
//         This is used here to simulate the charge level of a battery.
//     */
//     int battery = analogRead(A0);
//     int batteryLevel = map(battery, 0, 1023, 0, 100);

//     if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
//         Serial.print("Battery Level % is now: "); // print it
//         Serial.println(batteryLevel);
//         batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
//         oldBatteryLevel = batteryLevel;           // save the level for next comparison
//     }
// }

void updateAccelerometer() {
    Serial.println("updateAccel");
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
    }
   
    float accX = x; 
    Serial.println(x); 
    // if (x > 0.1) {
    //     x = 100 * x;
    //     degreesX = map(x, 0, 97, 0, 90);
    //     Serial.print("Tilting up ");
    //     Serial.print(degreesX);
    //     Serial.println("  degrees");
    // }
    // if (x < -0.1) {
    //     x = 100 * x;
    //     degreesX = map(x, 0, -100, 0, 90);
    //     Serial.print("Tilting down ");
    //     Serial.print(degreesX);
    //     Serial.println("  degrees");
    // }
    // if (y > 0.1) {
    //     y = 100 * y;
    //     degreesY = map(y, 0, 97, 0, 90);
    //     Serial.print("Tilting left ");
    //     Serial.print(degreesY);
    //     Serial.println("  degrees");
    // }
    // if (y < -0.1) {
    //     y = 100 * y;
    //     degreesY = map(y, 0, -100, 0, 90);
    //     Serial.print("Tilting right ");
    //     Serial.print(degreesY);
    //     Serial.println("  degrees");
    // }
    // delay(1000);
    

    if (accX != oldAccX) {      // if the battery level has changed
        Serial.print("X accelerometer value is now: "); // print it
        Serial.println(accX);
        accelerometerChar.writeValue(accX);  // and update the battery level characteristic
        oldAccX = accX;           // save the level for next comparison
    }
}