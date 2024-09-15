// Sender code for the ESP32-based remote control
// This code sends commands to the receiver via ESP-NOW

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Structure to send data via ESP-NOW
typedef struct struct_message
{
  int command;
  int value;
} struct_message;

struct_message outgoingMessage;
void discoverPeers();
// Variables for timing
unsigned long previousMillis = 0;
const long interval = 5000; // 5 seconds
bool moveLeft = true;       // Flag to alternate direction

// Motor specifications (must match receiver's settings)
const int stepsPerRev = 200;                           // Steps per revolution for 1.8° stepper
const int microsteps = 16;                             // Microstepping setting
const int microstepsPerRev = stepsPerRev * microsteps; // Total microsteps per revolution

const int stepsFor15Degrees = (microstepsPerRev * 15) / 360; // Calculate steps for 15 degrees

// List to store discovered peers
#define MAX_PEERS 10
uint8_t peerAddresses[MAX_PEERS][6];
int peerCount = 0;

void setup()
{
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial.println("ESP-NOW Sender Setup with Auto-Discovery");

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure we start disconnected from any network

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Optionally set primary master key if encryption is used
  // esp_now_set_pmk((uint8_t *)"pmk1234567890123");

  // Perform initial scan to discover peers
  discoverPeers();
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // If no peers are known, attempt to discover them
    if (peerCount == 0)
    {
      discoverPeers();
    }

    // Send commands to all known peers
    for (int i = 0; i < peerCount; i++)
    {
      // Prepare the message
      if (moveLeft)
      {
        Serial.println("Sending command to move left 15 degrees");
        outgoingMessage.command = 2;               // Command 2: move steps backward (left)
        outgoingMessage.value = stepsFor15Degrees; // Number of steps to move
      }
      else
      {
        Serial.println("Sending command to move right 15 degrees");
        outgoingMessage.command = 1; // Command 1: move steps forward (right)
        outgoingMessage.value = stepsFor15Degrees;
      }

      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(peerAddresses[i], (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));

      if (result == ESP_OK)
      {
        Serial.print("Sent to peer ");
        Serial.println(i);
      }
      else
      {
        Serial.print("Error sending to peer ");
        Serial.println(i);
      }
    }

    // Alternate direction
    moveLeft = !moveLeft;
  }
}

void discoverPeers()
{
  Serial.println("Starting Wi-Fi scan to discover peers...");
  int8_t scanResults = WiFi.scanNetworks();

  if (scanResults == 0)
  {
    Serial.println("No Wi-Fi networks found");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" networks");

    for (int i = 0; i < scanResults; ++i)
    {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      uint8_t *bssid = WiFi.BSSID(i);
      String bssidStr = WiFi.BSSIDstr(i);

      // Check if the SSID matches our receiver's identifier
      if (ssid == "ESP_NOW_RECEIVER")
      {
        Serial.print("Found receiver at ");
        Serial.println(bssidStr);

        // Add peer
        if (peerCount < MAX_PEERS)
        {
          memcpy(peerAddresses[peerCount], bssid, 6);

          esp_now_peer_info_t peerInfo;
          memset(&peerInfo, 0, sizeof(peerInfo));
          memcpy(peerInfo.peer_addr, bssid, 6);
          peerInfo.channel = 0; // Default channel
          peerInfo.encrypt = false;

          if (esp_now_add_peer(&peerInfo) == ESP_OK)
          {
            Serial.println("Peer added successfully");
            peerCount++;
          }
          else
          {
            Serial.println("Failed to add peer");
          }
        }
        else
        {
          Serial.println("Max peers reached");
        }
      }
    }
  }

  // Clean up scan results
  WiFi.scanDelete();
}
