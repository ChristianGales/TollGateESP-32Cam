// Tollgate System with Esp32 Cam
// Created By Christian Charles Gales
// Based on https://randomnerdtutorials.com/telegram-esp32-cam-photo-arduino/
// https://github.com/ChristianGales

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

//Network Cridentials Make (must have an internet connection)
// const char* ssid = "GlobeAtHome_C3424";
// const char* password = "Galesfamily#09";

const char* ssid = "RedNote12Pro5G";
const char* password = "12345678";


// Initialize Telegram BOT
// String BOTtoken = "7514510568:AAFw8pPUPt6AO8PZ9Cl_uW2wC1lVgsEKcro"; 
// String CHAT_ID = "7931352728";

String BOTtoken = "7800075174:AAHmN_dJ3q51gmykfTA_pwVX9IPvva6E-6U"; 
String CHAT_ID = "7800075174";


bool sendPhoto = false;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

#define FLASH_LED_PIN 4
bool flashState = LOW;

// Ultrasonic sensor pins
#define TRIG_PIN 13   // GPIO 13
#define ECHO_PIN 12   // GPIO 12

// Servo pin
#define SERVO_PIN 2   // GPIO 2
Servo securityServo;  // Create servo object

// Ultrasonic detection settings
const int triggerDistance = 50; // Distance in cm to trigger alerts
unsigned long lastDetectionTime = 0;
const int detectionCooldown = 10000; // 10 seconds cooldown between detections

//Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
}

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);
    
    String from_name = bot.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      welcome += "/scan : performs a servo sweep and reports distances\n";
      welcome += "/status : reports current security status\n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      Serial.println("Change flash LED state");
    }
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
    if (text == "/scan") {
      performSecurityScan();
    }
    if (text == "/status") {
      float distance = getDistance();
      String status = "Current status:\n";
      status += "Distance: " + String(distance) + " cm\n";
      status += "Last detection: " + String((millis() - lastDetectionTime) / 1000) + " seconds ago\n";
      status += "Security mode: Active";
      bot.sendMessage(CHAT_ID, status, "");
    }
  }
}


// Function to measure distance using ultrasonic sensor
float getDistance() {
    // Clear the trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Set the trigger pin high for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read the echo pin, and calculate the distance based on the speed of sound
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout at 30ms to prevent hangs
    if (duration == 0) {
        Serial.println("Ultrasonic sensor not responding!");
        return -1; // Indicate an invalid reading
    }

    float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    // Validate distance (typical range for ultrasonic sensors is 2cm - 400cm)
    if (distance < 2 || distance > 400) {
        Serial.println("Invalid distance reading: " + String(distance) + " cm");
        return -1; // Return -1 if reading is out of range
    }

    return distance;
}


// Scan area using servo motor and ultrasonic sensor
//test servo 
void performSecurityScan() {
  String scanResults = "Security Scan Results:\n";
  
  // Sweep servo from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle += 45) {
    securityServo.write(angle);
    delay(500); // Allow servo to reach position
    
    float distance = getDistance();
    scanResults += "Angle " + String(angle) + "°: " + String(distance) + " cm\n";
  }
  
  // Return to center position
  securityServo.write(90);
  bot.sendMessage(CHAT_ID, scanResults, "");
}


// Function to check for motion using ultrasonic sensor
void checkForMotion() {
  float distance = getDistance();

  // Ignore invalid readings
  if (distance == -1) {
    Serial.println("Skipping motion check due to invalid sensor reading.");
    return;
  }

  // If something is closer than the trigger distance and cooldown period has passed
  if (distance < triggerDistance && (millis() - lastDetectionTime > detectionCooldown)) {
    Serial.println("Motion detected at distance: " + String(distance) + " cm");

    // Turn on flash for better photo quality
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(100);

    // Send alert message with photo
    bot.sendMessage(CHAT_ID, "⚠️ ALERT: Motion detected at " + String(distance) + " cm!", "");
    sendPhotoTelegram();

    // Tollgate-style servo movement
    securityServo.write(0); // Tollgate up position 
    delay(3000);           // Wait for 3 seconds 

    securityServo.write(90); // Tollgate down position 

    // Turn off flash
    digitalWrite(FLASH_LED_PIN, LOW);

    // Update last detection time
    lastDetectionTime = millis();
  }
}


String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  //Dispose first picture because of bad quality
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb); // dispose the buffered image
  
  // Take a new photo
  fb = NULL;  
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }  
  
  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--Chrlsed\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--Chrlsed\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Chrlsed--\r\n";

    size_t imageLen = fb->len;
    size_t extraLen = head.length() + tail.length();
    size_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=Chrlsed");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  // Init Serial Monitor
  Serial.begin(115200);

  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);
  
  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize servo
  securityServo.attach(SERVO_PIN);
  securityServo.write(90); // Center position to start

  // Config and init the camera
  configInitCamera();

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Send startup notification
  bot.sendMessage(CHAT_ID, "TollGateSystem is now online!", "");
}

void loop() {
  // Check for motion using the ultrasonic sensor
  checkForMotion();
  
  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }
  
  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}
