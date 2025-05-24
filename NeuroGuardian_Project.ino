//Required Libraries
#include <Wire.h>                          // For I2C communication (MPU6050)
#include <Adafruit_MPU6050.h>             // MPU6050 library
#include <Adafruit_Sensor.h>              // Unified sensor library
#include <TinyGPSPlus.h>                  // TinyGPSPlus library for GPS parsing

// Pins for Sensors
#define GSR_PIN 34                         // Analog pin connected to GSR sensor
#define PULSE_PIN 35                       // Analog pin connected to pulse sensor
#define TEMP_PIN 32                        // Analog pin connected to skin temperature sensor
#define GPS_RX_PIN 16                     // GPS TX → ESP32 RX2 (GPIO16)
#define GPS_TX_PIN 17                     // GPS RX → ESP32 TX2 (GPIO17)

// Configuration
#define SIMULATE_MODE true                // Set to true to use fallback GPS values

// Create Sensor Objects
Adafruit_MPU6050 mpu;                     // IMU sensor object
HardwareSerial gpsSerial(2);              // Use UART2
TinyGPSPlus gps;

// Reference location (e.g., guardian's location)
const float guardianLat = 18.553456;
const float guardianLon = 73.845123;

// Distance threshold in meters
const float allowedDistance = 100.0;

// Setup Function - Runs Once on Boot
void setup() {
  Serial.begin(115200);                   // Start serial communication with PC
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);


  // Initialize the MPU6050 IMU sensor
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);                            // Stop execution if IMU fails to initialize
  }

  // Configure IMU
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("All sensors initialized.");
}

// Estimate Emotion Based on Sensor Data
String estimateEmotion(int gsr, int pulse, int temp) {
  if (gsr > 70 && pulse > 70) return "Stressed";
  if (gsr < 40 && pulse < 60) return "Calm";
  return "Neutral";
}

// Normalize Analog Sensor Values to 0–100 Scale
int normalizeSensorValue(int raw, int minVal, int maxVal) {
  return map(raw, minVal, maxVal, 0, 100);
}

// Haversine Formula to Calculate Distance
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000; // Earth radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Format Data as JSON for App/Cloud Communication
String formatDataAsJSON(float lat, float lon, float dist, String emotion) {
  String json = "{";
  json += "\"latitude\":" + String(lat, 6) + ",";
  json += "\"longitude\":" + String(lon, 6) + ",";
  json += "\"distance\":" + String(dist, 2) + ",";
  json += "\"emotion\":\"" + emotion + "\"";
  json += "}";
  return json;
}

// Simulate Sending Data to App (future: Bluetooth/Wi-Fi)
void sendDataToApp(String json) {
  Serial.println("Sending to app: " + json);
}

// Main Loop
void loop() {
  // Read Raw Analog Sensor Values
  int gsrValue = analogRead(GSR_PIN);
  int pulseValue = analogRead(PULSE_PIN);
  int tempValue = analogRead(TEMP_PIN);

  // Normalize for Emotion Estimation
  int normGSR = normalizeSensorValue(gsrValue, 0, 1023);
  int normPulse = normalizeSensorValue(pulseValue, 0, 1023);
  int normTemp = normalizeSensorValue(tempValue, 0, 1023);

  // Estimate Emotional State
  String emotion = estimateEmotion(normGSR, normPulse, normTemp);

  // Read IMU Data
  sensors_event_t a, g, imuTemp;
  mpu.getEvent(&a, &g, &imuTemp);

  // Parse GPS Data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  float latitude = 0.0;
  float longitude = 0.0;

  if (SIMULATE_MODE) {
    latitude = 18.55;
    longitude = 73.85;
  } else if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  // Calculate Distance from Guardian
  float distance = calculateDistance(latitude, longitude, guardianLat, guardianLon);

  // Format and Send JSON Data
  String jsonData = formatDataAsJSON(latitude, longitude, distance, emotion);
  sendDataToApp(jsonData);

  // Display All Sensor Data on Serial Monitor
  Serial.println("\n=== Sensor Output ===");
  Serial.print("GSR: "); Serial.print(gsrValue);
  Serial.print(" | Pulse: "); Serial.print(pulseValue);
  Serial.print(" | Temp: "); Serial.print(tempValue);

  Serial.print("\nEmotion: "); Serial.println(emotion);

  Serial.print("IMU Accel - X: "); Serial.print(a.acceleration.x);
  Serial.print(" | Y: "); Serial.print(a.acceleration.y);
  Serial.print(" | Z: "); Serial.println(a.acceleration.z);

  Serial.print("Location: "); Serial.print(latitude, 6); Serial.print(", "); Serial.print(longitude, 6);
  Serial.print(" | Distance from Guardian: "); Serial.print(distance); Serial.print(" m");
  Serial.print(" | Status: "); Serial.println(distance > allowedDistance ? "ALERT - Out of Zone" : "Safe");

  delay(1000); // Wait before next reading
}
