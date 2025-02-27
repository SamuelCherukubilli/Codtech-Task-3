# Codtech-Task-3

IoT Security System Prototype
The goal of this project is to build an IoT-based security system that detects motion, captures images, and sends alerts to a mobile app in real-time. The system can be used to monitor areas such as a home, office, or property for any unauthorized access or suspicious activity.

This project can be implemented using a Raspberry Pi or an ESP32/ESP8266 with a camera module and motion detection sensor. For real-time notifications, we can use a platform like Blynk, Firebase, or MQTT.

Let's break down the project into hardware setup, software setup, and system flow.

Components Required:
Microcontroller:

Raspberry Pi (with Raspbian OS for ease of camera and motion detection integration) or ESP32 for a smaller, simpler solution.
Camera Module:

Raspberry Pi Camera Module (for Raspberry Pi) or an ESP32-CAM for ESP32 boards.
Motion Sensor:

PIR (Passive Infrared) Motion Sensor for detecting motion.
Internet Connection:

Wi-Fi module for the microcontroller (ESP32 has built-in Wi-Fi, while Raspberry Pi requires a Wi-Fi adapter or ethernet connection).
Mobile App:

Use Blynk or Firebase for receiving alerts, or you can develop a custom app using Flutter or React Native.
Power Supply:

Ensure the power supply is suitable for your microcontroller and camera module.
System Overview:
Motion Detection:

The PIR sensor detects motion when there is movement in the monitored area.
Image Capture:

Once motion is detected, the camera captures an image of the scene.
Real-Time Alerts:

The system sends an alert (via a mobile app) with the image captured, notifying the user of suspicious activity.
Cloud Storage (Optional):

If desired, store captured images in the cloud (e.g., Google Drive, Firebase) for later review.
Step-by-Step Design:
Step 1: Hardware Setup
Connect the PIR Sensor:

VCC to 5V or 3.3V (depending on your microcontroller).
GND to Ground.
OUT to a GPIO pin (e.g., GPIO 17 on Raspberry Pi or GPIO 13 on ESP32).
Connect the Camera:

For Raspberry Pi, connect the Raspberry Pi Camera Module to the camera port (CSI port).
For ESP32, use the built-in camera module (like the ESP32-CAM) and connect the camera according to the specific module’s configuration.
Power Supply:

Ensure a stable power supply for the Raspberry Pi (5V micro-USB) or the ESP32 (via USB or Li-ion battery for portability).
Step 2: Software Setup
Option 1: Using Raspberry Pi
Install Raspbian OS:

Download and install Raspbian OS on your Raspberry Pi.
Connect the Raspberry Pi to a monitor, keyboard, and mouse for the initial setup.
Install Required Libraries:

Install libraries for the PIR sensor and camera on Raspberry Pi. You can use Python with libraries like RPi.GPIO and picamera.
bash
Copy
sudo apt-get update
sudo apt-get install python3-picamera python3-rpi.gpio
Python Code to Detect Motion & Capture Image:

The following Python code detects motion and captures an image:
python
Copy
import RPi.GPIO as GPIO
import time
import picamera
import requests

# Setup GPIO for PIR sensor
PIR_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

# Camera setup
camera = picamera.PICamera()

# Function to capture and send an image
def capture_and_send_image():
    filename = "/home/pi/motion_image.jpg"
    camera.capture(filename)
    
    # Send image to cloud service (e.g., Firebase, Google Drive, or MQTT)
    send_alert(filename)

def send_alert(image_path):
    # Example using Firebase to upload the image
    url = "https://your-firebase-url.com/upload"
    files = {'file': open(image_path, 'rb')}
    response = requests.post(url, files=files)
    print(response.status_code)

# Main loop
try:
    print("System Ready")
    while True:
        if GPIO.input(PIR_PIN):
            print("Motion Detected!")
            capture_and_send_image()
            time.sleep(10)  # Wait 10 seconds before checking again to avoid multiple triggers
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("System Exited")
Set Up Mobile Notifications:
Use Firebase Cloud Messaging (FCM) to send push notifications to the mobile app when motion is detected and an image is captured.
Option 2: Using ESP32-CAM
Install ESP32 Board in Arduino IDE:

Go to File > Preferences in the Arduino IDE.
Add the following URL to the Additional Boards Manager URLs:
arduino
Copy
https://dl.espressif.com/dl/package_esp32_index.json
Install the ESP32 board from the Boards Manager.
ESP32-CAM Setup:

Connect the ESP32-CAM to your computer using a USB-to-Serial adapter for programming.
Code for Motion Detection and Image Capture:

Here is an example Arduino code for ESP32-CAM that uses the PIR sensor to trigger the camera:
cpp
Copy
#include <WiFi.h>
#include <esp_camera.h>
#include <FirebaseESP32.h>

// Wi-Fi credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// Firebase credentials
#define FIREBASE_HOST "your_firebase_project_id.firebaseio.com"
#define FIREBASE_AUTH "your_firebase_database_secret"

// PIR motion sensor pin
#define PIR_PIN 13

// Initialize Firebase
FirebaseData firebaseData;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // Initialize PIR sensor
  pinMode(PIR_PIN, INPUT);
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL;
  config.ledc_timer = LEDC_TIMER;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_camera_init(&config);
}

void loop() {
  if (digitalRead(PIR_PIN) == HIGH) {
    Serial.println("Motion detected!");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to capture image");
      return;
    }

    // Send image to Firebase
    Firebase.pushBlob(firebaseData, "/motion_images", fb->buf, fb->len);
    
    // Free the frame buffer
    esp_camera_fb_return(fb);
    
    delay(5000); // Wait before next detection
  }
}
Set Up Mobile App:
You can use Firebase Cloud Messaging (FCM) to send real-time notifications or Blynk to send alerts to the app when motion is detected.
Use Firebase Storage to store the image and display it in the app.
Step 3: Mobile App Setup
You can build a mobile app using Blynk or Firebase:

Blynk: Use Blynk's push notification feature to send alerts to the mobile app with the image URL or Firebase storage reference.
Firebase: Develop a custom app using Flutter or React Native to show live alerts with the captured images and enable the user to view them.
Conclusion:
This IoT security system prototype allows you to detect motion, capture images, and send alerts in real-time. It uses a PIR sensor for motion detection, a camera module to capture images, and a cloud service (like Firebase) to store images and send alerts to a mobile app. You can expand this project to include features like email alerts, video capture, or integration with other IoT devices for more complex security systems.
