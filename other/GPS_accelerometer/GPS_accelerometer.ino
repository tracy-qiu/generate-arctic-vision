#include <Arduino_LSM9DS1.h>
#define GPSSerial Serial1

float x, y, z;
float a, b, c;
int degreesX = 0;
int degreesY = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

 // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  if (x > 0.1) {
    x = 100 * x;
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < -0.1) {
    x = 100 * x;
    degreesX = map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (y > 0.1) {
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (y < -0.1) {
    y = 100 * y;
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }

  // if (IMU.gyroscopeAvailable()) {
  //       IMU.readGyroscope(a, b, c);

  //       Serial.print(a);
  //       Serial.print('\t');
  //       Serial.print(b);
  //       Serial.print('\t');
  //       Serial.println(c);
  }
  delay(1000);
}
