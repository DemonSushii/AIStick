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
const int trigPin1 = 5;  //gray
const int echoPin1 = 18;  //white

const int trigPin2 = 19;  //orange
const int echoPin2 = 21;  //red 

const int trigPin3 = 22;  //yellow
const int echoPin3 = 23;  //green

// Buzzer pin
//const int buzzerPin = 25;  // Change this to your buzzer pin

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
long buzzerInterval = 1000;  // Time between beep starts
long beepDuration = 100;     // Duration of each beep (constant short beep)

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
  //controlBuzzer();
  
  // Print results
  printDistances();
  
  delay(300);  //Slowing down the process
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

void controlBuzzer() {                            // Funcion off
  // Find the closest distance among all sensors
  float closestDistance = distanceCm1;
  if (distanceCm2 < closestDistance) closestDistance = distanceCm2;
  if (distanceCm3 < closestDistance) closestDistance = distanceCm3;
  
  // Define three ranges with different beep speeds
  const float RANGE_3_FAR = 130.0;   //70-130cm: Slow intermittent beeping
  const float RANGE_3_NEAR = 70.0;
  const float RANGE_2_FAR =70.0;   // 30-70cm: Medium intermittent beeping
  const float RANGE_2_NEAR =30.0;
  const float RANGE_1_FAR =30.0;   // 0.4-30cm: Fast intermittent beeping
  const float RANGE_1_NEAR = 0.4;
  
  unsigned long currentMillis = millis();
  
  // Check if object is in any range
  bool objectInRange = (closestDistance >= RANGE_1_NEAR && closestDistance <= RANGE_3_FAR);
  
  if (objectInRange) {
    // Determine which range the object is in and set beep speed accordingly
    long beepOnTime, beepOffTime;
    
    if (closestDistance >= RANGE_1_NEAR && closestDistance <= RANGE_1_FAR) {
      // Range 1: Close (0.4-30cm) - Fast intermittent beeping 
      beepOnTime = 30;   // Beep ON for 80ms
      beepOffTime = 70;  // Beep OFF for 100ms
      delay(10);
      beepOnTime = 30;   // Beep ON for 80ms
      beepOffTime = 70;  // Beep OFF for 100ms
      delay(10);
    } 
    else if (closestDistance > RANGE_1_FAR && closestDistance <= RANGE_2_FAR) {
      // Range 2: Medium (30-70cm) - Medium intermittent beeping
      beepOnTime = 150;   // Beep ON for 150ms
      beepOffTime = 700;  // Beep OFF for 400ms
      delay(10);
    } 
    else {
      // Range 3: Far (70-130cm) - Slow intermittent beeping
      beepOnTime = 200;   // Beep ON for 200ms
      beepOffTime = 1200;  // Beep OFF for 1000ms
      delay(10);
    }
    
    // Handle intermittent beeping
    if (!buzzerActive) {
      // Time to start a new beep
      if (currentMillis - previousBuzzerMillis >= beepOffTime) {
        digitalWrite(buzzerPin, HIGH);
        buzzerActive = true;
        previousBuzzerMillis = currentMillis;
      }
    } else {
      // Beep is active, check if time to turn it off
      if (currentMillis - previousBuzzerMillis >= beepOnTime) {
        digitalWrite(buzzerPin, LOW);
        buzzerActive = false;
        previousBuzzerMillis = currentMillis;
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
  SerialBT.print(" | S1: "); SerialBT.println(distanceCm1);
  SerialBT.print(" | S2: "); SerialBT.println(distanceCm2);
  SerialBT.print(" | S3: "); SerialBT.println(distanceCm3);
  SerialBT.print(" | Shortest: "); SerialBT.printlnln(shortestDistance);
  
  Serial.print(" | S1: "); Serial.println(distanceCm1);
  Serial.print(" | S2: "); Serial.println(distanceCm2);
  Serial.print(" | S3: "); Serial.println(distanceCm3);
  Serial.print(" | Shortest: "); Serial.printlnln(shortestDistance);
  
  // Print buzzer status
  if (buzzerActive) {
    SerialBT.println("BUZZER: ACTIVE - Object in range!");
    Serial.println("BUZZER: ACTIVE - Object in range!");
  }
}