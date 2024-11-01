// Define motor control pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 7

// Define ultrasonic sensor pins
#define TRIG_PIN 8
#define ECHO_PIN 9

// Define LED pin
#define LED_PIN 10

// Define state variables
bool isObstacleDetected = false;

void setup() {
  // Set motor control pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize Serial communication with Bluetooth module
  Serial.begin(9600);
}

void loop() {
  long duration, distance;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo time
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in cm
  distance = (duration / 2) / 29.1;

  // Check if an object is within 20 cm
  if (distance < 20) {
    // Object detected
    isObstacleDetected = true;
    // Stop the car
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    // Turn on the LED
    digitalWrite(LED_PIN, HIGH);
  } else {
    // No object detected
    isObstacleDetected = false;
    // Turn off the LED
    digitalWrite(LED_PIN, LOW);
  }

  if (Serial.available()) {
    char command = Serial.read(); // Read the command from Bluetooth module

    switch (command) {
      case 'F': // Move Forward
        if (!isObstacleDetected) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
          analogWrite(ENA, 255); // Full speed
          analogWrite(ENB, 255); // Full speed
        }
        break;

      case 'B': // Move Backward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 255); // Full speed
        analogWrite(ENB, 255); // Full speed
        break;

      case 'L': // Turn Left
        digitalWrite(IN1, LOW); // Stop left motor
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); // Right motor forward
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0); // Stop left motor
        analogWrite(ENB, 255); // Full speed right motor
        break;

      case 'R': // Turn Right
        digitalWrite(IN1, HIGH); // Left motor forward
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); // Stop right motor
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 255); // Full speed left motor
        analogWrite(ENB, 0); // Stop right motor
        break;

      case 'S': // Stop
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        break;

      default:
        // Invalid command
        break;
    }
  }
}
