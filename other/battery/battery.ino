#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <string>

using std::string;

#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

 // BLE Services: battery, accelerometer, GPS 
BLEService batteryService("180F");
BLEService imuService("7ee45d33-a08e-4758-81ce-eaca164f2d02");
BLEService gpsService("3791880d-64bb-4b94-9088-7d0047ca6519");

// BLE Characteristics: battery, accelerometer, GPS 
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEFloatCharacteristic accXChar("36029e64-8b3a-4fb2-bf23-937e835b412f", 
    BLERead | BLENotify); 
BLEFloatCharacteristic accYChar("6700d79b-2b91-4a5e-9f36-1a5803e7db8f", 
    BLERead | BLENotify);
BLEFloatCharacteristic accZChar("9bdb03de-51ae-466d-91f2-a12466719ce0", 
    BLERead | BLENotify); 

// BLECharacteristic accelChar("c34ba939-708e-478a-8a77-df2a22a32587", 
//     BLERead | BLENotify, 11, false);

BLEFloatCharacteristic latitudeChar("921f7b4a-d0f3-46a8-8ca6-bd351e178fdd", 
    BLERead | BLENotify); 
BLEFloatCharacteristic longitudeChar("a8e45cb5-0b64-4ba4-a2ce-103f42b92015", 
    BLERead | BLENotify); 
BLEFloatCharacteristic speedChar("6fa60c0e-e64c-4913-bbcc-6a635bcb35fc", 
    BLERead | BLENotify); 
BLEFloatCharacteristic altitudeChar("f3e7b2ea-b42b-496e-86d3-7ef9b880cd6e", 
    BLERead | BLENotify); 
BLEFloatCharacteristic fixChar("e70fa3a2-5f38-45db-acec-78af96e54b9f", 
    BLERead | BLENotify); 

uint32_t timer = millis();

long previousMillis = 0;  // last time the battery level was checked, in ms

int oldBatteryLevel = 0;  // last battery level reading from analog input
float oldAccX = 0;
float oldAccY = 0;
float oldAccZ = 0;
// string oldAccel = "";
float oldLatitude = 0; 
float oldLongitude = 0;
float oldSpeed = 0; 
float oldAltitude = 0; 
int oldFix = 0;

float x, y, z;

void setup() {
    Serial.begin(115200);    // initialize serial communication
    GPS.begin(9600);
    
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

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

    /* Set a local name for the BLE device
        This name will appear in advertising packets
        and can be used by remote devices to identify this BLE device
        The name can be changed but maybe be truncated based on space left in advertisement packet
    */
    BLE.setLocalName("Nano33BLE");

    batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
    BLE.addService(batteryService); // Add the battery service
    batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic

    // imuService.addCharacteristic(accelChar);
    imuService.addCharacteristic(accXChar); 
    imuService.addCharacteristic(accYChar);
    imuService.addCharacteristic(accZChar);
    BLE.addService(imuService);
    accXChar.writeValue(oldAccX); 
    accYChar.writeValue(oldAccY); 
    accZChar.writeValue(oldAccZ); 
    // char arr[1]; 
    // strcpy(arr, oldAccel.c_str()); 
    // accelChar.writeValue(arr);

    BLE.setAdvertisedService(gpsService);  // add the service UUID
    imuService.addCharacteristic(latitudeChar); 
    imuService.addCharacteristic(longitudeChar);
    imuService.addCharacteristic(speedChar);
    imuService.addCharacteristic(altitudeChar);
    BLE.addService(gpsService);
    latitudeChar.writeValue(oldLatitude); 
    longitudeChar.writeValue(oldLongitude); 
    speedChar.writeValue(oldSpeed);
    altitudeChar.writeValue(oldAltitude);
    fixChar.writeValue(oldFix);

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
                updateGPS();
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
   
    // string accX(std::to_string(x));
    // Serial.println(x); 
    // string accY(std::to_string(y)); 
    // Serial.println(y); 
    // string accZ(std::to_string(z)); 
    // Serial.println(z); 

    // string accel = accX + "," + accY + "," + accZ;

    float accX = x; 
    float accY = y; 
    float accZ = z;
    
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
    // if (accel != oldAccel) {      // if the battery level has changed
    //     Serial.print("Accelerometer value is now: "); // print it
    //     Serial.print(accel);
    //     char accelArr[accel.length() + 1]; 
    //     strcpy(accelArr, accel.c_str()); 
    //     accelChar.writeValue(accelArr);    // and update the battery level characteristic
    //     oldAccel = accel;           // save the level for next comparison
    // }
}

void updateGPS() {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }

    int fix = GPS.fix;
    float latitude = GPS.latitude;
    float longitude = GPS.longitude; 
    float speed = GPS.speed; 
    float altitude = GPS.altitude; 

    if (fix != oldFix) {      // if the battery level has changed
        fixChar.writeValue(fix);  // and update the battery level characteristic
        oldFix = fix;           // save the level for next comparison
    }
    if (latitude != oldLatitude) {      // if the battery level has changed
        latitudeChar.writeValue(latitude);  // and update the battery level characteristic
        oldLatitude = latitude;           // save the level for next comparison
    }
    if (longitude != oldLongitude) {      // if the battery level has changed
        longitudeChar.writeValue(longitude);  // and update the battery level characteristic
        oldLongitude = longitude;           // save the level for next comparison
    }
    if (speed != oldSpeed) {      // if the battery level has changed
        speedChar.writeValue(speed);  // and update the battery level characteristic
        oldSpeed = speed;           // save the level for next comparison
    }
    if (altitude != oldAltitude) {      // if the battery level has changed
        altitudeChar.writeValue(altitude);  // and update the battery level characteristic
        oldAltitude = altitude;           // save the level for next comparison
    }

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}
