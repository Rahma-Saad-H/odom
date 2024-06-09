#include <ros.h>
#include <std_msgs/Int64.h>

// Global variables
ros::NodeHandle nh;

std_msgs::Int64 encoderCount1;
std_msgs::Int64 encoderCount2;

ros::Publisher encoder1_pub("encoder1", &encoderCount1);
ros::Publisher encoder2_pub("encoder2", &encoderCount2);

// Pin definitions
const int pwmPin1 = 3;  // PWM control pin for motor1 speed
const int dirPin1 = 2;  // Direction control pin for motor1
const int pwmPin2 = 5;  // PWM control pin for motor2 speed
const int dirPin2 = 4;  // Direction control pin for motor2

// Define encoder pins for motor 1
const int encoderPinA1 = 18;
const int encoderPinB1 = 19;

// Define encoder pins for motor 2
const int encoderPinA2 = 20;
const int encoderPinB2 = 21;

// Variables to store the state of the encoders
volatile long encoderCountValue1 = 0; // Use long to store larger values
volatile int lastEncodedValue1 = 0;
volatile bool updateFlag1 = false;  // Flag to indicate an update to encoder 1 count

volatile long encoderCountValue2 = 0; // Use long to store larger values
volatile int lastEncodedValue2 = 0;
volatile bool updateFlag2 = false;  // Flag to indicate an update to encoder 2 count

// State machine states
enum State {
  FORWARD,
  STOP_FORWARD,
  BACKWARD,
  STOP_BACKWARD
};

State currentState = FORWARD; // Initial state

unsigned long stateStartTime = 0; // Variable to store the start time of the current state

// Function declarations
void publishEncoders();
void handleEncoderA1();
void handleEncoderB1();
void handleEncoderA2();
void handleEncoderB2();
void moveForward(int speed);
void moveBackward(int speed);
void stopMotor();
void setSpeed(int speed);
void adjustSpeed();

void setup() {
  // Initialize ROS
  nh.initNode();
  nh.advertise(encoder1_pub);
  nh.advertise(encoder2_pub);

  // Set pin modes
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Set encoder pins as inputs
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

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
      moveForward(50); // Move forward
      if (millis() - stateStartTime >= 7000) { // Check if 7 seconds have elapsed
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
      moveBackward(50); // Move backward
      if (millis() - stateStartTime >= 7000) { // Check if 7 seconds have elapsed
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

  // Publish encoder readings
  encoder1_pub.publish(&encoderCount1);
  encoder2_pub.publish(&encoderCount2);

  // ROS spin
  nh.spinOnce();
  delay(10); // Adjust delay as needed
}

void publishEncoders() {
  if (updateFlag1) {
    encoderCount1.data = encoderCountValue1;
    encoder1_pub.publish(&encoderCount1);
    updateFlag1 = false;
  }

  if (updateFlag2) {
    encoderCount2.data = encoderCountValue2;
    encoder2_pub.publish(&encoderCount2);
    updateFlag2 = false;
  }
}



void handleEncoderA1() {
  int MSB = digitalRead(encoderPinA1); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB1); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Combine the two bits
  int sum = (lastEncodedValue1 << 2) | encoded; // Sum the previous and current states

  // Update the encoder count based on the state transitions
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountValue1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountValue1--;

  lastEncodedValue1 = encoded; // Store the current state as the last state
  updateFlag1 = true;  // Set the flag to indicate an update
}

void handleEncoderB1() {
  handleEncoderA1(); // Call the same handler for simplicity
}

void handleEncoderA2() {
  int MSB = digitalRead(encoderPinA2); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Combine the two bits
  int sum = (lastEncodedValue2 << 2) | encoded; // Sum the previous and current states

  // Update the encoder count based on the state transitions
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountValue2++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountValue2--;

  lastEncodedValue2 = encoded; // Store the current state as the last state
  updateFlag2 = true;  // Set the flag to indicate an update
}

void handleEncoderB2() {
  handleEncoderA2(); // Call the same handler for simplicity
}

void moveForward(int speed) {
  // Reset encoder counts
  encoderCountValue1 = 0;
  encoderCountValue2 = 0;

  // Set the direction for forward movement
  digitalWrite(dirPin1, HIGH); // Motor 1 forward
  digitalWrite(dirPin2, HIGH); // Motor 2 forward

  // Set the speed
  setSpeed(speed);
}

void moveBackward(int speed) {
  // Reset encoder counts
  encoderCountValue1 = 0;
  encoderCountValue2 = 0;

  // Set the direction for backward movement
  digitalWrite(dirPin1, LOW);  // Motor 1 backward
  digitalWrite(dirPin2, LOW);  // Motor 2 backward

  // Set the speed
  setSpeed(speed);
}

void stopMotor() {
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
}

void setSpeed(int speed) {
  analogWrite(pwmPin1, speed);
  analogWrite(pwmPin2, speed);
}
