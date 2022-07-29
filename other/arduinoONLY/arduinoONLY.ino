#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

 // BLE Services: battery, accelerometer, GPS 
BLEService batteryService("180F");
BLEService imuService("7ee45d33-a08e-4758-81ce-eaca164f2d02");

// BLE Characteristics: battery, accelerometer, GPS 
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

//BLECharacteristic accelerometerChar()
BLEFloatCharacteristic accXChar("36029e64-8b3a-4fb2-bf23-937e835b412f", 
    BLERead | BLENotify); 
BLEFloatCharacteristic accYChar("6700d79b-2b91-4a5e-9f36-1a5803e7db8f", 
    BLERead | BLENotify);
BLEFloatCharacteristic accZChar("9bdb03de-51ae-466d-91f2-a12466719ce0", 
    BLERead | BLENotify); 

uint32_t timer = millis();

long previousMillis = 0;  // last time the battery level was checked, in ms

int oldBatteryLevel = 0;  // last battery level reading from analog input
float oldAccX = 0;
float oldAccY = 0;
float oldAccZ = 0;

float x, y, z;

void setup() {
    Serial.begin(115200);    // initialize serial communication
    
    // while (!Serial);

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
    BLE.setLocalName("Nano33BLE");

    batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
    BLE.addService(batteryService); // Add the battery service
    batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic

    BLE.setAdvertisedService(imuService);  // add the service UUID
    imuService.addCharacteristic(accXChar); 
    imuService.addCharacteristic(accYChar);
    imuService.addCharacteristic(accZChar);
    BLE.addService(imuService);
    accXChar.writeValue(oldAccX); 
    accYChar.writeValue(oldAccY); 
    accZChar.writeValue(oldAccZ); 

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
        // Serial.println("led on");
        // check the battery level every 200ms
        // while the central is connected:
        while (central.connected()) {
            long currentMillis = millis();
            // if 200ms have passed, check the battery level:
            if (currentMillis - previousMillis >= 200) {
                previousMillis = currentMillis;
                // Serial.println(currentMillis);
                updateBatteryLevel();
                updateAccelerometer();
            }
        }
        // Serial.println("before led off");
        // when the central disconnects, turn off the LED:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}

void updateBatteryLevel() {
    /* Read the current voltage level on the A0 analog input pin.
        This is used here to simulate the charge level of a battery.
    */
    int battery = analogRead(A0);
    int batteryLevel = map(battery, 0, 1023, 0, 100);

    if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
        // Serial.print("Battery Level % is now: "); // print it
        // Serial.println(batteryLevel);
        batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
        oldBatteryLevel = batteryLevel;           // save the level for next comparison
    }
}

void updateAccelerometer() {
    // Serial.println("updateAccel");
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
    }
   
    float accX = x; 
    // Serial.println(x); 
    float accY = y; 
    // Serial.println(y); 
    float accZ = z; 
    // Serial.println(z); 

    if (accX != oldAccX) {      // if the battery level has changed
        // Serial.print("X accelerometer value is now: "); // print it
        // Serial.println(accX);
        accXChar.writeValue(accX);  // and update the battery level characteristic
        oldAccX = accX;           // save the level for next comparison
    }
    if (accY != oldAccY) {      // if the battery level has changed
        // Serial.print("Y accelerometer value is now: "); // print it
        // Serial.println(accY);
        accYChar.writeValue(accY);  // and update the battery level characteristic
        oldAccY = accY;           // save the level for next comparison
    }
    if (accZ != oldAccZ) {      // if the battery level has changed
        // Serial.print("Z accelerometer value is now: "); // print it
        // Serial.println(accZ);s
        accZChar.writeValue(accZ);  // and update the battery level characteristic
        oldAccZ = accZ;           // save the level for next comparison
    }
}
