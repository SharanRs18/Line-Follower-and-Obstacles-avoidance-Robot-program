# Line-Follower-and-Obstacles-avoidance-Robot-program
// Define pins for line sensors
#define LEFT_SENSOR_PIN A0
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A2

// Define pins for ultrasonic sensor
#define TRIG_PIN 9
#define ECHO_PIN 10

// Define pins for motor driver
#define ENA 3
#define ENB 5
#define IN1 2
#define IN2 4
#define IN3 7
#define IN4 8

// Define threshold for line detection
#define LINE_THRESHOLD 500

// Define threshold for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD 20

void setup() {
  // Initialize line sensors
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(CENTER_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read line sensors
  int leftSensorValue = analogRead(LEFT_SENSOR_PIN);
  int centerSensorValue = analogRead(CENTER_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_SENSOR_PIN);

  // Debugging line sensor values
  Serial.print("Left: ");
  Serial.print(leftSensorValue);
  Serial.print(" Center: ");
  Serial.print(centerSensorValue);
  Serial.print(" Right: ");
  Serial.println(rightSensorValue);

  // Read distance from ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert to cm

  // Debugging distance value
  Serial.print("Distance: ");
  Serial.println(distance);

  // Obstacle avoidance
  if (distance < OBSTACLE_THRESHOLD) {
    stop();
    delay(500); // Stop for a while
    avoidObstacle();
    return;
  }

  // Line following logic
  if (centerSensorValue < LINE_THRESHOLD) {
    // Move forward
    moveForward();
  } else if (leftSensorValue < LINE_THRESHOLD) {
    // Turn left
    turnLeft();
  } else if (rightSensorValue < LINE_THRESHOLD) {
    // Turn right
    turnRight();
  } else {
    // Stop if no line is detected
    stop();
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200); // Adjust speed as necessary
  analogWrite(ENB, 200); // Adjust speed as necessary
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200); // Adjust speed as necessary
  analogWrite(ENB, 200); // Adjust speed as necessary
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200); // Adjust speed as necessary
  analogWrite(ENB, 200); // Adjust speed as necessary
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void avoidObstacle() {
  // Example obstacle avoidance routine (simple backward and turn)
  // Move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200); // Adjust speed as necessary
  analogWrite(ENB, 200); // Adjust speed as necessary
  delay(500); // Move backward for a while

  // Turn right to avoid obstacle
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200); // Adjust speed as necessary
  analogWrite(ENB, 200); // Adjust speed as necessary
  delay(500); // Turn for a while
}
