#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

String device_name = "ESP32-BT";

BluetoothSerial SerialBT;

// Sensor pins
const int trigPin1 = 5;
const int echoPin1 = 18;
const int trigPin2 = 19;   
const int echoPin2 = 21;   
const int trigPin3 = 22;   
const int echoPin3 = 23; 

// Buzzer pin
const int buzzerPin = 25;  // Change this to your buzzer pin

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

// Distance thresholds for buzzer (in cm)
const float MIN_DISTANCE = 14.0;   // Buzzer starts when object is within 3cm
const float MAX_DISTANCE = 0.0;   // Buzzer stops when object is beyond 10cm

// Variables for sensors
long duration1, duration2, duration3;
float distanceCm1, distanceCm2, distanceCm3;
float shortestDistance;

// Buzzer variables
bool buzzerActive = false;
unsigned long previousBuzzerMillis = 0;
const long buzzerInterval = 200;  // Buzzer beep interval in milliseconds

void setup() {
  Serial.begin(115200); // Starts the serial communication

  // Setup sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  // Setup buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  SerialBT.begin(device_name);  //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop() {
  // Read all sensors
  readSensor1();
  readSensor2();
  readSensor3();

  delay(25);
  
  // Find the shortest distance
  findShortestDistance();
  
  // Control buzzer based on distance
  controlBuzzer();
  
  // Print results
  printDistances();
  
  delay(150);
}

void readSensor1() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distanceCm1 = duration1 * SOUND_SPEED/2;
}

void readSensor2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distanceCm2 = duration2 * SOUND_SPEED/2;
}

void readSensor3() {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distanceCm3 = duration3 * SOUND_SPEED/2;
}

void findShortestDistance() {
  shortestDistance = distanceCm1;
  int sensorNumber = 1;
  
  if (distanceCm2 < shortestDistance) {
    shortestDistance = distanceCm2;
    sensorNumber = 2;
  }
  
  if (distanceCm3 < shortestDistance) {
    shortestDistance = distanceCm3;
    sensorNumber = 3;
  }
}

void controlBuzzer() {
  // Check if any sensor detects an object within the alert range
  bool objectInRange = (distanceCm1 >= MIN_DISTANCE && distanceCm1 <= MAX_DISTANCE) ||
                      (distanceCm2 >= MIN_DISTANCE && distanceCm2 <= MAX_DISTANCE) ||
                      (distanceCm3 >= MIN_DISTANCE && distanceCm3 <= MAX_DISTANCE);

  if (objectInRange) {
    // Object detected in range - activate buzzer with beeping
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousBuzzerMillis >= buzzerInterval) {
      previousBuzzerMillis = currentMillis;
      
      // Toggle buzzer state for beeping effect
      if (buzzerActive) {
        digitalWrite(buzzerPin, LOW);
        buzzerActive = false;
      } else {
        digitalWrite(buzzerPin, 150);
        buzzerActive = true;
      }
    }
  } else {
    // No object in range - turn off buzzer
    digitalWrite(buzzerPin, LOW);
    buzzerActive = false;
  }
}

void printDistances() {
  // Print all distances for debugging
  SerialBT.print("S1: "); SerialBT.print(distanceCm1);
  SerialBT.print(" | S2: "); SerialBT.print(distanceCm2);
  SerialBT.print(" | S3: "); SerialBT.print(distanceCm3);
  SerialBT.print(" | Shortest: "); SerialBT.println(shortestDistance);
  
  Serial.print("S1: "); Serial.print(distanceCm1);
  Serial.print(" | S2: "); Serial.print(distanceCm2);
  Serial.print(" | S3: "); Serial.print(distanceCm3);
  Serial.print(" | Shortest: "); Serial.println(shortestDistance);
  
  // Print buzzer status
  if (buzzerActive) {
    SerialBT.println("BUZZER: ACTIVE - Object in range!");
    Serial.println("BUZZER: ACTIVE - Object in range!");
  }
}