#include <SPI.h>
#include <WiFi.h>

// WiFi Credentials
IPAddress ip(192, 168, 248, 108);
char ssid[] = "Meet's OnePlus";
char password[] = "123456789";
char b[100]; // Buffer to store received command
WiFiServer server(5555);

// Define LED Pins
#define GREEN_LED 2   // Conveyor ON
#define BLUE_LED  3   // Drilling ON
#define RED_LED   4   // Emergency STOP

void setup() {
  Serial.begin(9600);
  Serial.print("Attempting to connect to Network named: ");
  Serial.println(ssid);

  WiFi.config(ip);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nConnected to WiFi");
  Serial.println("Waiting for an IP address");

  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  printWifiStatus();
  server.begin();

  // Set LED Pins as Output
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Turn off all LEDs initially
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
}

void loop() {
  WiFiClient client = server.available();

  // **NEW: Server can input commands via Serial Monitor**
  if (Serial.available()) {
    char inputCommand = Serial.read();  // Read single character from Serial Monitor

    // Ignore newline characters
    if (inputCommand == '\n' || inputCommand == '\r') {
      return;
    }

    Serial.print("Server Input Command: ");
    Serial.println(inputCommand);

    // **LED Control for Server Input**
    if (inputCommand == 'R') {
      Serial.println("Turning ON Green LED (Conveyor)");
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, LOW);
    } 
    else if (inputCommand == 'W') {
      Serial.println("Turning ON Blue LED (Drilling)");
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
      digitalWrite(RED_LED, LOW);
    } 
    else if (inputCommand == 'X') {
      Serial.println("Turning ON Red LED (Emergency STOP)");
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, HIGH);
    } 
    else {
      Serial.println("Invalid Command! Use R, W, or X.");
    }
  }

  // **Check for Client Input**
  if (client) {
    if (client.available()) {
      memset(b, 0, sizeof(b));  // Clear buffer
      client.readBytesUntil('\n', b, sizeof(b) - 1);

      Serial.print("Received Command from Client: ");
      Serial.println(b);

      // **LED Control for Client Input**
      if (b[0] == 'R') {
        Serial.println("Turning ON Green LED (Conveyor)");
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
      } 
      else if (b[0] == 'W') {
        Serial.println("Turning ON Blue LED (Drilling)");
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
      } 
      else if (b[0] == 'X') {
        Serial.println("Turning ON Red LED (Emergency STOP)");
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
      } 
      else {
        Serial.println("Invalid Command from Client.");
      }
    }
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress IP = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(IP);
}
