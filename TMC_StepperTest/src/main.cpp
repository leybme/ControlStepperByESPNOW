#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <TMCStepper.h>

// Define pins
#define EN_PIN 36           // Enable pin
#define DIR_PIN 4           // Direction pin
#define STEP_PIN 5          // Step pin
#define SERIAL_PORT Serial2 // UART port for TMC2209
#define SERIAL_TX 38        // TX pin for UART
#define SERIAL_RX 37        // RX pin for UART

// #define SERIAL_TX 18     // TX pin for UART
// #define SERIAL_RX 17        // RX pin for UART
#define SERIAL_TX 37 // TX pin for UART
#define SERIAL_RX 38 // RX pin for UART
#define DIR2 12
#define STEP2 13
#define DIAG2 14
#define DIR_PIN 47
#define STEP_PIN 48
#define DIAG1 21
#define STEP_EN 36
#define STEP_TX 38
#define STEP_RX 37
#define R_SENSE 0.11f       // Sense resistor value
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define BUTTON_LEFT 42      // Left button pin
#define BUTTON_RIGHT 41     // Right button pin
#define BUTTON_SELECT 40    // Select button pin

// Create TMC2209 driver instance
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, 0b00);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, 0b10);
// Motor specifications
const int stepsPerRev = 200;                           // Steps per revolution for 1.8° stepper
const int microsteps = 16;                             // Microstepping setting
const int microstepsPerRev = stepsPerRev * microsteps; // Total microsteps per revolution

// Structure to receive data via ESP-NOW
typedef struct struct_message
{
  int command;
  int value;
} struct_message;

struct_message incomingMessage;

// Function to move the motor a specified number of steps
void moveSteps(int steps)
{
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP2, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(STEP2, LOW);
    delayMicroseconds(500);
  }
}

// Callback function executed when data is received via ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  int command = incomingMessage.command;
  int value = incomingMessage.value;

  // Process the received command
  switch (command)
  {
  case 0:                       // Stop motor
    digitalWrite(EN_PIN, HIGH); // Disable driver
    break;
  case 1:                        // Move steps forward
    digitalWrite(EN_PIN, LOW);   // Enable driver
    digitalWrite(DIR_PIN, HIGH); // Set direction forward
    moveSteps(value);
    break;
  case 2:                       // Move steps backward
    digitalWrite(EN_PIN, LOW);  // Enable driver
    digitalWrite(DIR_PIN, LOW); // Set direction backward
    moveSteps(value);
    break;
  case 3: // Set motor current
    driver.rms_current(value);
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200); // Initialize serial communication for debugging

  // Initialize pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Disable driver
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);

  // Initialize UART for TMC2209
  SERIAL_PORT.begin(19200, SERIAL_8N1, SERIAL_RX, SERIAL_TX);

  // Initialize TMC2209 driver settings
  driver.begin();
  driver.toff(5);
  driver.rms_current(600); // Set motor RMS current
  driver.microsteps(microsteps);
  driver.pdn_disable(true); // Use UART for configuration
  driver.I_scale_analog(false);
  driver.en_spreadCycle(false); // Enable stealthChop
   driver2.begin();
  driver2.toff(5);
  driver2.rms_current(600); // Set motor RMS current
  driver2.microsteps(microsteps);
  driver2.pdn_disable(true); // Use UART for configuration
  driver2.I_scale_analog(false);
  driver2.en_spreadCycle(false); // Enable stealthChop
  if (microsteps == driver.microsteps())
  {
    Serial.println("Microsteps set successfully");
  }
  else
  {
    Serial.println("Error setting microsteps");
  }
  // print microsteps
  Serial.print("Microsteps: ");
  Serial.println(driver.microsteps());

  // Register the receive callback
  // esp_now_register_recv_cb(OnDataRecv);

  // Move stepper 10 degrees to the right at startup
  digitalWrite(EN_PIN, LOW);                       // Enable driver
  digitalWrite(DIR_PIN, HIGH);                     // Set direction forward (right)
  int stepsToMove = (microstepsPerRev * 10) / 360; // Calculate steps for 10 degrees
  Serial.print("Moving stepper 10 degrees to the right: ");
  Serial.print(stepsToMove);
  Serial.println(" microsteps");
  moveSteps(stepsToMove);
  // Optionally disable driver after movement
  digitalWrite(EN_PIN, HIGH); // Disable driver
}

void loop()
{
  // Read button states (active LOW)
  bool leftPressed = !digitalRead(BUTTON_LEFT);
  bool rightPressed = !digitalRead(BUTTON_RIGHT);
  bool selectPressed = !digitalRead(BUTTON_SELECT);

  if (leftPressed)
  {
    Serial.println("Left button pressed");
    // Move motor to the left
    digitalWrite(EN_PIN, LOW);                       // Enable driver
    digitalWrite(DIR_PIN, LOW);                      // Set direction backward (left)
    int stepsToMove = (microstepsPerRev * 45) / 360; // Move 45 degrees per press
    moveSteps(stepsToMove);
    digitalWrite(EN_PIN, HIGH); // Disable driver after movement
    delay(200);                 // Debounce delay
  }
  else if (rightPressed)
  {
    Serial.println("Right button pressed");
    // Move motor to the right
    digitalWrite(EN_PIN, LOW);                       // Enable driver
    digitalWrite(DIR_PIN, HIGH);                     // Set direction forward (right)
    int stepsToMove = (microstepsPerRev * 45) / 360; // Move 45 degrees per press
    moveSteps(stepsToMove);
    digitalWrite(EN_PIN, HIGH); // Disable driver after movement
    delay(200);                 // Debounce delay
  }
  else if (selectPressed)
  {
    Serial.println("Select button pressed");
    // Move motor 360 degrees to the right
    digitalWrite(EN_PIN, LOW);                        // Enable driver
    digitalWrite(DIR_PIN, HIGH);                      // Set direction forward (right)
    int stepsToMove = (microstepsPerRev * 360) / 360; // Move 360 degrees
    moveSteps(stepsToMove);
    digitalWrite(EN_PIN, HIGH); // Disable driver after movement
    delay(200);                 // Debounce delay
  }

  // Small delay to prevent rapid polling
  delay(50);
    digitalWrite(EN_PIN, LOW);                       // Enable driver
  int stepsToMove = (microstepsPerRev * 90) / 360; // Move 45 degrees per press
  moveSteps(stepsToMove);
    digitalWrite(EN_PIN, HIGH);                       // Enable driver
  delay(2000);
}
