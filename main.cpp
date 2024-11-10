#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ESP32Servo.h>  // Use standard Servo library
#include <WiFi.h>
#include <PubSubClient.h>

#define SERVO_PIN 15
#define LCD_ADDR 0x27
#define ROWS 4
#define COLS 4

// Ultrasonic sensor pins (Check for correct pin assignments in Wokwi)
#define TRIG_PIN 13
#define ECHO_PIN 12

// Push button pin (acting as fingerprint sensor)
#define BUTTON_PIN 14

// Initialize LCD
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// Keypad configuration
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {23, 19, 18, 5};   // Check that these match your schematic
byte colPins[COLS] = {17, 16, 4, 2};    // Check that these match your schematic

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Servo myServo;

// Valid PIN and variables
String validPin = "1234";
String enteredPin = "";
static int attempt = 0;
bool useFingerprint = false;
bool doorOpen = false;

// WiFi and MQTT configurations
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;  // MQTT default port

// MQTT Topics
const char* objectDetectionTopic = "home/door/notification";
const char* doorStatusTopic = "home/door/status";

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Function declaration for unlockDoor
void unlockDoor();

void setupWiFi() {
  lcd.clear();
  lcd.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  lcd.clear();
  lcd.print("WiFi connected");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  myServo.attach(SERVO_PIN);
  myServo.write(0);  // Make sure the door starts locked

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize button (fingerprint sensor)
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Assumes the button is using an internal pull-up resistor

  lcd.clear();
  lcd.print("Select Option:");
  lcd.setCursor(0, 1);
  lcd.print("B: FP, C: PIN");

  // Set up WiFi and MQTT
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);  // Specify MQTT server and port
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Ultrasonic sensor detection
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  if (distance <= 15) {
    delay(2000);
    lcd.clear();
    lcd.print("Enter pin code:");
    Serial.println("Someone is near the door");
    client.publish(objectDetectionTopic, "Object detected within 15 cm");  // Send object detection message
  }

  // Keypad input handling
  char key = keypad.getKey();

  if (key) {
    if (key == 'B') {
      useFingerprint = true;
      lcd.clear();
      lcd.print("Input fingerprint");
      lcd.setCursor(0, 1);
      lcd.print("that is registered");
    } else if (key == 'C') {
      useFingerprint = false;
      lcd.clear();
      lcd.print("Enter pin code:");
    } else if (!useFingerprint && key != '*' && key != '#') {
      enteredPin += key;
      lcd.clear();
      lcd.print("Pin code: ");
      lcd.print(key);
    }

    if (!useFingerprint && enteredPin.length() == 4) {
      if (enteredPin == validPin) {
        lcd.clear();
        lcd.print("Correct PIN");
        lcd.setCursor(0, 1);
        lcd.print("Press 'D' to open");
        enteredPin = "";
        attempt = 0;
      } else {
        attempt++;
        lcd.clear();
        lcd.print("Invalid pin code");
        delay(2000);
        lcd.clear();
        lcd.print("Enter pin code:");

        if (attempt >= 3) {
          lcd.clear();
          lcd.print("System locked");
          delay(5000);
          attempt = 0;
          lcd.clear();
          lcd.print("Enter pin code:");
        }
      }
      enteredPin = "";
    }

    // Open door if 'D' is pressed after correct PIN
    if (!useFingerprint && key == 'D') {
      unlockDoor();
    }
  }

  // Check if button (fingerprint sensor) is pressed when in fingerprint mode
  if (useFingerprint && digitalRead(BUTTON_PIN) == LOW && !doorOpen) {
    unlockDoor();
    doorOpen = true;  // Set door as open to prevent repeated unlocking
    delay(500);       // Short debounce delay
  } else if (digitalRead(BUTTON_PIN) == HIGH && doorOpen) {
    doorOpen = false;  // Reset door state when button is released
  }
}

void unlockDoor() {
  lcd.clear();
  lcd.print("Door opened...");
  myServo.write(90);  // Unlock door by rotating servo to 90 degrees
  delay(2000);        // Keep door unlocked for 2 seconds
  myServo.write(0);   // Lock door again by returning servo to 0 degrees
  lcd.clear();
  lcd.print("Door locked");
  client.publish(doorStatusTopic, "Door Opened");  // Send door status to MQTT
  delay(1000);
  client.publish(doorStatusTopic, "Door Locked");  // Send door locked status to MQTT
}

void lockDoor() {
  lcd.clear();
  lcd.print("Door locked");
  myServo.write(0);   // Ensure servo is in locked position
}
