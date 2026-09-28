// Paste in your Blynk information from the dashboard

#define BLYNK_TEMPLATE_ID "TMPL2iZJ2_b0p"
#define BLYNK_TEMPLATE_NAME "ESE 1110 Final Proj"
#define BLYNK_AUTH_TOKEN "dK7OwK1ERGOB3uFu-pUbSkmMGrtuzA2L"

#include <Blynk.h>
#include <SPI.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

/* 
 * Combined Code - FSR and IMU Integration with Serial Acceleration Output
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 *   ends to +5V and ground
 *   wiper to LCD VO pin (pin 3)
 *
 * ESE 1900 Spring 2024
 * Lab 2 Week 2 Part 4b
 * FSR Sensor, MPU6050 IMU, and LCD
 */

#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define fsrPin A1  // FSR pin

// LCD pin definitions
const int rs = 12;
const int en = 11;
const int d4 = 5;
const int d5 = 4;
const int d6 = 3;
const int d7 = 2;

// Metronome
const int bpm = 110;
const int metronomeInterval = 60000 / bpm;
// Speaker pin definition
const int speakerPin = 13; // Connect your speaker to digital pin D13

// Initialize the LiquidCrystal library
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Initialize MPU6050
Adafruit_MPU6050 mpu;

// FSR variables
int fsrReading; // The current reading from the FSR

// Calibration Constants for FlexiForce A201 with 10kΩ resistor
const float FSR_A = 0.04;  // N per unit (Calibration based on 10kΩ resistor)
const float FSR_B = 0.0;    // N (No offset)

// WiFi credentials
// IMPORTANT: Replace these with your actual WiFi credentials.
// For security reasons, avoid hardcoding sensitive information in production code.
const char ssid[] = "esesyno";    // Your WiFi SSID
const char pass[] = "eniacpenn";  // Your WiFi Password

// Variables for timing
unsigned long lastAccelTime = 0;       // Time of the last accelerometer event
unsigned long currentTime = 0;         // Current time in milliseconds
unsigned long timeBetweenAccel = 0;    // Time elapsed between accelerometer events
unsigned long lastMetronomeTime = 0;   // To store the last metronome beep time

// For accelerometer sign change detection
double lastAccelSign = 1;
int signChangeCounter = 0;

// Function to convert raw FSR reading to force in Newtons
float convertFSRReadingToForce(int raw) {
    // Linear mapping: F = a * raw + b
    float force = FSR_A * raw + FSR_B;
    return force;
}

void setup(void) {
    // Initialize Serial Monitor for debugging
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    // Initialize the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Clear the LCD and display initial message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");

    // Initialize speaker pin as output
    pinMode(speakerPin, OUTPUT);

    // Initialize FSR pin
    pinMode(fsrPin, INPUT);

    // Initialize MPU6050 sensor
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        lcd.setCursor(0, 1);
        lcd.print("MPU6050 Error");
        while (1) {
            delay(10);
        }
    }

    // Configure accelerometer range
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    // Set filter bandwidth to reduce noise
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100); // Allow sensor to stabilize

    // Initialize last event times
    lastAccelTime = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    Serial.println("Setup complete. Starting main loop.");
}

void loop() {
    // Run Blynk
    Blynk.run();

    // Read sensor events
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // ------------------------------
    // Removed: Acceleration data print to Serial Monitor
    /*
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x, 2);
    Serial.print(" g, Y: ");
    Serial.print(a.acceleration.y, 2);
    Serial.print(" g, Z: ");
    Serial.print(a.acceleration.z, 2);
    Serial.println(" g");
    */

    // ------------------------------
    // Metronome Implementation
    // ------------------------------
    unsigned long currentMillis = millis();

    if (currentMillis - lastMetronomeTime >= metronomeInterval) {
        tone(speakerPin, 1000);
        delay(100);            
        noTone(speakerPin);    
        lastMetronomeTime = currentMillis;
    }

    // ------------------------------
    // Accelerometer Sign Change Logic and Force Sensor Logic
    // ------------------------------
    // Detect zero crossing in acceleration x-axis
    if ((lastAccelSign * a.acceleration.x) < 0) {
        lastAccelSign = a.acceleration.x;
        signChangeCounter += 1;
    }

    if (signChangeCounter >= 2) { // Changed from == 2 to >= 2 to ensure robustness
        signChangeCounter = 0;

        // Read FSR Reading
        fsrReading = analogRead(fsrPin);

        // Debugging: Print raw FSR reading
        Serial.print("Raw FSR Reading: ");
        Serial.println(fsrReading);

        // Convert FSR reading to force (Newtons)
        float force = convertFSRReadingToForce(fsrReading);

        // Debugging: Print calculated force
        Serial.print("Force: ");
        Serial.print(force, 2);
        Serial.println(" N");

        // Send force to Blynk virtual pin V3
        Blynk.virtualWrite(V3, force);

        // Removed: Display force on LCD
        /*
        lcd.setCursor(0, 0);
        lcd.print("Force: ");
        lcd.print(force, 1);
        lcd.print(" N     "); // Extra spaces to clear any residual characters
        */

        // Update LCD and Blynk based on force thresholds
        if (force > 600.0) { // Adjusted threshold near max (445 N)
            Blynk.virtualWrite(V4, 1);
            lcd.setCursor(0, 0);
            lcd.print("Lighter        ");
        } 
        else if (force < 400.0) { // Lower threshold (100 N)
            Blynk.virtualWrite(V4, 1);
            lcd.setCursor(0, 0);
            lcd.print("Harder         ");
        } 
        else {
            Blynk.virtualWrite(V4, 0);
            lcd.setCursor(0, 0);
            lcd.print("Good           ");
        }

        // Calculate time between accelerometer events
        currentTime = millis();
        timeBetweenAccel = currentTime - lastAccelTime;
        lastAccelTime = currentTime;
        Blynk.virtualWrite(V2, 60000 / timeBetweenAccel); // BPM

        // ------------------------------
        // Removed: Time between accel events print to Serial Monitor
        /*
        Serial.print("Time between accel events: ");
        Serial.print(timeBetweenAccel);
        Serial.print(" ms (");
        Serial.print(timeBetweenAccel / 1000.0, 3); // Convert to seconds with 3 decimal places
        Serial.println(" s)");
        */

        // Update Pace based on time between accelerometer events
        if (timeBetweenAccel > 700) { // If more than 700 ms
            Blynk.virtualWrite(V4, 1);
            lcd.setCursor(0, 1);
            lcd.print("Pace: Slow     "); // Clear rest of line
            Serial.println("Pace: Slow");
        }
        else if (timeBetweenAccel < 400) { // If less than 400 ms
            Blynk.virtualWrite(V4, 1);
            lcd.setCursor(0, 1);
            lcd.print("Pace: Fast     ");
            Serial.println("Pace: Fast");
        }
        else {
            Blynk.virtualWrite(V4, 0);
            lcd.setCursor(0, 1);
            lcd.print("Pace: Good     ");
            Serial.println("Pace: Good");
        }
    }
}

// Helper function to map float values (similar to Arduino's map() for integers)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


