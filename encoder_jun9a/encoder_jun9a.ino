// Pin definitions
const int pwmPin1 = 3;  // PWM control pin for motor1 speed
const int dirPin1 = 2;  // Direction control pin for motor1
const int pwmPin2 = 5;  // PWM control pin for motor2 speed
const int dirPin2 = 4;  // Direction control pin for motor2

// Define encoder pins for motor 1
const int encoderPinA2 = 18;
const int encoderPinB2 = 19;

// Define encoder pins for motor 2
const int encoderPinA1 = 20;
const int encoderPinB1 = 21;

// Variables to store the state of the encoders
volatile long encoderCount1 = 0; // Use long to store larger values
volatile int lastEncoded1 = 0;
volatile bool updateFlag1 = false;  // Flag to indicate an update to encoder 1 count

volatile long encoderCount2 = 0; // Use long to store larger values
volatile int lastEncoded2 = 0;
volatile bool updateFlag2 = false;  // Flag to indicate an update to encoder 2 count

// Global flag to indicate if motors are running
volatile bool running = false;

// Enum for states
enum State {
  FORWARD,
  STOP_FORWARD,
  BACKWARD,
  STOP_BACKWARD
};

State currentState = FORWARD; // Initial state

unsigned long stateStartTime = 0; // Variable to store the start time of the current state

// Interrupt service routine for encoder 1 pin A
void handleEncoderA1() {
  int MSB = digitalRead(encoderPinA1); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB1); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Combine the two bits
  int sum = (lastEncoded1 << 2) | encoded; // Sum the previous and current states

  // Update the encoder count based on the state transitions
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount1--;

  lastEncoded1 = encoded; // Store the current state as the last state
  updateFlag1 = true;  // Set the flag to indicate an update
}

// Interrupt service routine for encoder 1 pin B
void handleEncoderB1() {
  handleEncoderA1(); // Call the same handler for simplicity
}

// Interrupt service routine for encoder 2 pin A
void handleEncoderA2() {
  int MSB = digitalRead(encoderPinA2); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Combine the two bits
  int sum = (lastEncoded2 << 2) | encoded; // Sum the previous and current states

  // Update the encoder count based on the state transitions
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount2--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount2++;

  lastEncoded2 = encoded; // Store the current state as the last state
  updateFlag2 = true;  // Set the flag to indicate an update
}

// Interrupt service routine for encoder 2 pin B
void handleEncoderB2() {
  handleEncoderA2(); // Call the same handler for simplicity
}

// Function declarations
void stopMotor();
void setSpeed(int speed);
void adjustSpeed();
void moveForward(int speed);
void moveBackward(int speed);

void setup() {
  // Set pin modes
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Initially stop the motors
  stopMotor();

  // Set encoder pins as inputs
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

  // Enable pullup resistors
  digitalWrite(encoderPinA1, HIGH);
  digitalWrite(encoderPinB1, HIGH);
  digitalWrite(encoderPinA2, HIGH);
  digitalWrite(encoderPinB2, HIGH);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), handleEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), handleEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), handleEncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), handleEncoderB2, CHANGE);
}

void loop() {
  // State machine logic
  switch (currentState) {
    case FORWARD:
      moveForward(80); // Move forward
      if (millis() - stateStartTime >= 5000) { // Check if 5 seconds have elapsed
        currentState = STOP_FORWARD; // Transition to stop forward state
        stateStartTime = millis(); // Update state start time
      }
      break;

    case STOP_FORWARD:
      stopMotor(); // Stop the motors
      if (millis() - stateStartTime >= 2000) { // Check if 2 seconds have elapsed
        currentState = BACKWARD; // Transition to backward state
        stateStartTime = millis(); // Update state start time
      }
      break;

    case BACKWARD:
      moveBackward(80); // Move backward
      if (millis() - stateStartTime >= 5000) { // Check if 5 seconds have elapsed
        currentState = STOP_BACKWARD; // Transition to stop backward state
        stateStartTime = millis(); // Update state start time
      }
      break;

    case STOP_BACKWARD:
      stopMotor(); // Stop the motors
      if (millis() - stateStartTime >= 2000) { // Check if 2 seconds have elapsed
        currentState = FORWARD; // Transition to forward state
        stateStartTime = millis(); // Update state start time
      }
      break;
  }

  // Print encoder readings
  Serial.print("Encoder Count 1: ");
  Serial.print(encoderCount1);
  Serial.print(" Encoder Count 2: ");
  Serial.println(encoderCount2);
}

void moveForward(int speed) {
  // Reset encoder counts
  encoderCount1 = 0;
  encoderCount2 = 0;

  // Set the direction for forward movement
  digitalWrite(dirPin1, HIGH); // Motor 1 forward
  digitalWrite(dirPin2, HIGH); // Motor 2 forward

  // Set the speed
  setSpeed(speed);

  running = true;  // Set the running flag
}

void moveBackward(int speed) {
  // Reset encoder counts
  encoderCount1 = 0;
  encoderCount2 = 0;

  // Set the direction for backward movement
  digitalWrite(dirPin1, LOW);  // Motor 1 backward
  digitalWrite(dirPin2, LOW);  // Motor 2 backward

  // Set the speed
  setSpeed(speed);

  running = true;  // Set the running flag
}

void adjustSpeed() {
  int baseSpeed = 80; // Maximum speed
  int speed1 = baseSpeed;
  int speed2 = baseSpeed;

  // Check if the difference between absolute encoder counts exceeds 400
  if (abs(abs(encoderCount1) - abs(encoderCount2)) > 400) {
    if (encoderCount1 > encoderCount2) {
      speed1 = baseSpeed * 0.5; // Reduce speed of motor 1
      Serial.println("Adjusting speed: Reducing Motor 1 speed.");
    } else {
      speed2 = baseSpeed * 0.5; // Reduce speed of motor 2
      Serial.println("Adjusting speed: Reducing Motor 2 speed.");
    }
  }
  
  // Set the speed
  analogWrite(pwmPin1, speed1);
  analogWrite(pwmPin2, speed2);
}

void setSpeed(int speed) {
  analogWrite(pwmPin1, speed);
  analogWrite(pwmPin2, speed);
}

void stopMotor() {
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
}
