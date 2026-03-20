// ================================================================================
// SELF-BALANCING CAR - ESP32 với MQTT + Firebase (v5 - Legacy Token)
// ================================================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <Kalman.h>
#include <ArduinoJson.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ================== WIFI ==================
const char* ssid = "NTSS31AD8-L5-4";
const char* password = "ntss31ad8l54";

// ================== FIREBASE - Dùng Database Secret ==================
// Lấy Database Secret từ: Firebase Console -> Project Settings -> Service accounts -> Database secrets
#define DATABASE_URL "balance-bot-56bb0-default-rtdb.firebaseio.com"
#define DATABASE_SECRET "bboziXJmWrWx7rpMUkMsySU7OWzHEyIBylGCZj2t"  // <-- Thay bằng Database Secret

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool firebaseReady = false;
unsigned long lastFirebaseUpdate = 0;

// ================== MQTT ==================
const char* mqtt_broker = "98538d1071034b5085a76c0f2371ce74.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "balancebot1";
const char* mqtt_pass = "Huy@2024Secure";
const char* mqtt_client_id = "BalanceBot_001";

#define TOPIC_TELEMETRY   "balancebot1/001/telemetry"
#define TOPIC_COMMAND     "balancebot1/001/command"
#define TOPIC_STATUS      "balancebot1/001/status"
#define TOPIC_CONFIG      "balancebot1/001/config"

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

unsigned long lastMqttReconnect = 0;
unsigned long lastMqttPublish = 0;
bool mqttConnected = false;

// ================== PINS ==================
#define ENA 25
#define IN1 27
#define IN2 26
#define ENB 14
#define IN3 12
#define IN4 13
#define ENC_L_A 34
#define ENC_L_B 35
#define ENC_R_A 32
#define ENC_R_B 33
#define SDA_PIN 21
#define SCL_PIN 22

// ================== CONSTANTS ==================
#define SPEED_LEFT      40
#define SPEED_RIGHT     40
#define MAX_SAFE_ANGLE  15.0f
#define PWM_MODE_SPEED  50
#define CMD_TIMEOUT_MS  300

#define PI_VAL 3.14159265359f
#define ToDeg (180.0f / PI_VAL)
#define ToRad (PI_VAL / 180.0f)

float angle_offset = -2.0f;
const float THETA_FACTOR = 0.3636f;

Kalman kalman_h;

// ================== SHARED VARIABLES ==================
SemaphoreHandle_t xMutex;

volatile long leftencoder = 0;
volatile long righencoder = 0;

float K1 = 0.0f, K2 = 0.0f, K3 = 800.0f;
float K4 = 1.5f, K5 = 0.0f, K6 = 0.0f;

long PWML = 0, PWMR = 0;
int min_pwm_L = 150, min_pwm_R = 150;
int motorTrim = 0;

float ForwardBack = 0;
int LeftRight = 0;
int ForwardPWM = 0;

bool falldown = false;
bool cmdActive = false;

uint32_t timer = 0;
double mpudata = 0;
float gyroRate = 0;
uint8_t i2cData[14];

float theta, psi, phi;
float thetadot, psidot, phidot;
float thetaold = 0, phiold = 0;

unsigned long lastCmdTime = 0;

// ================== MQTT CALLBACK ==================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';
  
  Serial.printf("[MQTT] %s: %s\n", topic, msg);
  
  if (strcmp(topic, TOPIC_COMMAND) == 0) {
    char c = toupper(msg[0]);
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      switch(c) {
        case 'F': ForwardPWM = PWM_MODE_SPEED; lastCmdTime = millis(); cmdActive = true; Serial.println("-> FORWARD"); break;
        case 'B': ForwardPWM = -PWM_MODE_SPEED; lastCmdTime = millis(); cmdActive = true; Serial.println("-> BACKWARD"); break;
        case 'L': LeftRight = SPEED_LEFT; lastCmdTime = millis(); cmdActive = true; Serial.println("-> LEFT"); break;
        case 'R': LeftRight = -SPEED_RIGHT; lastCmdTime = millis(); cmdActive = true; Serial.println("-> RIGHT"); break;
        case 'S': ForwardBack = 0; LeftRight = 0; ForwardPWM = 0; cmdActive = false; Serial.println("-> STOP"); break;
      }
      xSemaphoreGive(xMutex);
    }
  } 
  else if (strcmp(topic, TOPIC_CONFIG) == 0) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (doc.containsKey("K3")) K3 = doc["K3"].as<float>();
        if (doc.containsKey("K4")) K4 = doc["K4"].as<float>();
        if (doc.containsKey("angle_offset")) angle_offset = doc["angle_offset"].as<float>();
        xSemaphoreGive(xMutex);
        Serial.printf("[Config] K3=%.1f K4=%.2f offset=%.2f\n", K3, K4, angle_offset);
      }
    }
  }
}

// ================== I2C ==================
uint8_t i2cWrite(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(data, len);
  return Wire.endTransmission();
}

uint8_t i2cWrite(uint8_t reg, uint8_t data) {
  return i2cWrite(reg, &data, 1);
}

uint8_t i2cRead(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  if (Wire.endTransmission(false)) return 1;
  Wire.requestFrom((uint8_t)0x68, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) data[i++] = Wire.read();
  return 0;
}

// ================== ENCODER ISR ==================
void IRAM_ATTR left_isr() {
  leftencoder += digitalRead(ENC_L_B) ? 1 : -1;
}

void IRAM_ATTR righ_isr() {
  righencoder += digitalRead(ENC_R_B) ? -1 : 1;
}

// ================== SENSOR ==================
void readmpu() {
  i2cRead(0x3B, i2cData, 14);
  int16_t accX = (i2cData[0] << 8) | i2cData[1];
  int16_t accZ = (i2cData[4] << 8) | i2cData[5];
  int16_t Gyro = (i2cData[10] << 8) | i2cData[11];

  double dt = (micros() - timer) / 1000000.0;
  timer = micros();
  
  mpudata = kalman_h.getAngle(atan2(-accX, accZ) * 57.2958, Gyro / 131.0, dt);
  gyroRate = (Gyro / 131.0) * ToRad;
  falldown = abs(mpudata) > 60.0f;
}

// ================== LQR ==================
void getlqr(float th, float thd, float ps, float psd, float ph, float phd) {
  float c = K1 * th + K2 * thd + K3 * ps + K4 * psd;
  float d = K5 * ph + K6 * phd;
  PWML = constrain((long)(c - d), -255L, 255L);
  PWMR = constrain((long)(c + d), -255L, 255L);
}

// ================== MOTOR ==================
void motorcontrol(long l, long r, bool stop) {
  if (stop) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  l += motorTrim;
  if (abs(l) < 20) l = 0;
  if (abs(r) < 20) r = 0;
  if (l != 0 && abs(l) < min_pwm_L) l = l > 0 ? min_pwm_L : -min_pwm_L;
  if (r != 0 && abs(r) < min_pwm_R) r = r > 0 ? min_pwm_R : -min_pwm_R;

  analogWrite(ENA, abs(l));
  digitalWrite(IN1, l > 0 ? LOW : HIGH);
  digitalWrite(IN2, l > 0 ? HIGH : LOW);
  analogWrite(ENB, abs(r));
  digitalWrite(IN3, r > 0 ? LOW : HIGH);
  digitalWrite(IN4, r > 0 ? HIGH : LOW);
}

// ================== SERIAL COMMAND ==================
void processSerialCmd() {
  while (Serial.available()) {
    char c = toupper(Serial.read());
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      switch(c) {
        case 'F': ForwardPWM = PWM_MODE_SPEED; lastCmdTime = millis(); cmdActive = true; break;
        case 'B': ForwardPWM = -PWM_MODE_SPEED; lastCmdTime = millis(); cmdActive = true; break;
        case 'L': LeftRight = SPEED_LEFT; lastCmdTime = millis(); cmdActive = true; break;
        case 'R': LeftRight = -SPEED_RIGHT; lastCmdTime = millis(); cmdActive = true; break;
        case 'S': ForwardBack = 0; LeftRight = 0; ForwardPWM = 0; cmdActive = false; break;
      }
      xSemaphoreGive(xMutex);
    }
  }
}

// ================== TASK: BALANCE (Core 1) ==================
void TaskBalance(void *p) {
  TickType_t xLastWake = xTaskGetTickCount();
  uint32_t tloop = micros();

  for (;;) {
    readmpu();

    if ((micros() - tloop) > 3000) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        long le = leftencoder;
        long re = righencoder;
        int lr = LeftRight;
        int fp = ForwardPWM;
        xSemaphoreGive(xMutex);

        theta = THETA_FACTOR * (le + re) * ToRad;
        psi = (mpudata + angle_offset) * ToRad;
        phi = THETA_FACTOR * (le - re) * ToRad;

        double dt = (micros() - tloop) / 1000000.0;
        tloop = micros();

        thetadot = (theta - thetaold) / dt;
        psidot = gyroRate;
        phidot = (phi - phiold) / dt;

        thetaold = theta;
        phiold = phi;

        getlqr(theta, thetadot, psi, psidot, phi, phidot);

        int safePWM = fp;
        if (fabs(psi * ToDeg) > MAX_SAFE_ANGLE) safePWM = 0;

        motorcontrol(PWML - lr + safePWM, PWMR + lr + safePWM, falldown);
      }
    }
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(3));
  }
}

// ================== TASK: CONTROL (Core 0) ==================
void TaskControl(void *p) {
  for (;;) {
    processSerialCmd();

    if (cmdActive && (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ForwardBack = 0;
        LeftRight = 0;
        ForwardPWM = 0;
        cmdActive = false;
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ================== TASK: MQTT (Core 0) ==================
void TaskMQTT(void *p) {
  TickType_t xLastWake = xTaskGetTickCount();
  
  for (;;) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected) {
      if (!mqtt.connected()) {
        if (millis() - lastMqttReconnect > 5000) {
          lastMqttReconnect = millis();
          Serial.print("[MQTT] Reconnecting...");
          if (mqtt.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
            Serial.println(" OK!");
            mqtt.subscribe(TOPIC_COMMAND);
            mqtt.subscribe(TOPIC_CONFIG);
          } else {
            Serial.printf(" FAILED rc=%d\n", mqtt.state());
          }
        }
      }
      
      mqtt.loop();
      
      if (mqtt.connected() && (millis() - lastMqttPublish >= 100)) {
        lastMqttPublish = millis();
        
        StaticJsonDocument<200> doc;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          doc["ang"] = roundf((mpudata + angle_offset) * 10) / 10;
          doc["pwmL"] = PWML;
          doc["pwmR"] = PWMR;
          doc["encL"] = leftencoder;
          doc["encR"] = righencoder;
          doc["fall"] = falldown;
          doc["bal"] = abs(mpudata + angle_offset) < 5.0;
          xSemaphoreGive(xMutex);
        }
        
        char buffer[200];
        serializeJson(doc, buffer, sizeof(buffer));
        mqtt.publish(TOPIC_TELEMETRY, buffer);
      }
    }
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));
  }
}

// ================== TASK: FIREBASE (Core 0) ==================
void TaskFirebase(void *p) {
  for (;;) {
    if (firebaseReady && WiFi.status() == WL_CONNECTED) {
      if (millis() - lastFirebaseUpdate >= 500) {
        lastFirebaseUpdate = millis();
        
        FirebaseJson json;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          json.set("angle", roundf((mpudata + angle_offset) * 10) / 10);
          json.set("pwm_l", (int)PWML);
          json.set("pwm_r", (int)PWMR);
          json.set("enc_l", (int)leftencoder);
          json.set("enc_r", (int)righencoder);
          json.set("falldown", falldown);
          json.set("balanced", abs(mpudata + angle_offset) < 5.0);
          json.set("uptime", (int)(millis() / 1000));
          xSemaphoreGive(xMutex);
        }
        
        if (!Firebase.RTDB.setJSON(&fbdo, "/robots/bot001/telemetry", &json)) {
          Serial.printf("[Firebase] Error: %s\n", fbdo.errorReason().c_str());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n================================");
  Serial.println("  BALANCE BOT - MQTT + Firebase");
  Serial.println("  Version 5.0 (Legacy Token)");
  Serial.println("================================\n");

  // Mutex
  xMutex = xSemaphoreCreateMutex();

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(ENC_L_A, left_isr, RISING);
  attachInterrupt(ENC_R_A, righ_isr, RISING);

  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  i2cData[0] = 7;
  i2cWrite(0x19, i2cData, 4);
  i2cWrite(0x6B, 0x01);
  delay(100);
  
  i2cRead(0x3B, i2cData, 6);
  int16_t ax = (i2cData[0] << 8) | i2cData[1];
  int16_t az = (i2cData[4] << 8) | i2cData[5];
  kalman_h.setAngle(atan2(-ax, az) * 57.2958);
  kalman_h.setQangle(0.0000085);
  kalman_h.setQbias(0.000005);
  kalman_h.setRmeasure(0.0009);
  timer = micros();
  Serial.println("[MPU6050] OK");

  // WiFi
  Serial.printf("[WiFi] %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf(" OK! IP: %s\n", WiFi.localIP().toString().c_str());
    
    // NTP
    Serial.print("[NTP] Syncing");
    configTime(7 * 3600, 0, "pool.ntp.org");
    delay(3000);
    Serial.println(" OK!");
    
    // ===== FIREBASE SETUP (Legacy Token) =====
    Serial.print("[Firebase] Connecting");
    
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;
    
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    
    // Test connection
    delay(1000);
    if (Firebase.RTDB.setString(&fbdo, "/robots/bot001/status", "online")) {
      firebaseReady = true;
      Serial.println(" OK!");
      Serial.println("[Firebase] Ready!");
    } else {
      Serial.println(" FAILED!");
      Serial.printf("[Firebase] %s\n", fbdo.errorReason().c_str());
    }
    
    // ===== MQTT SETUP =====
    Serial.print("[MQTT] Connecting");
    
    espClient.setInsecure();
    espClient.setTimeout(15);
    
    mqtt.setServer(mqtt_broker, mqtt_port);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    mqtt.setKeepAlive(60);
    mqtt.setSocketTimeout(15);
    
    timeout = 0;
    while (!mqtt.connect(mqtt_client_id, mqtt_user, mqtt_pass) && timeout < 5) {
      Serial.print(".");
      delay(1000);
      timeout++;
    }
    
    if (mqtt.connected()) {
      mqttConnected = true;
      Serial.println(" OK!");
      mqtt.subscribe(TOPIC_COMMAND);
      mqtt.subscribe(TOPIC_CONFIG);
      
      char statusMsg[100];
      snprintf(statusMsg, sizeof(statusMsg), "{\"online\":true,\"ip\":\"%s\"}", WiFi.localIP().toString().c_str());
      mqtt.publish(TOPIC_STATUS, statusMsg, true);
      Serial.println("[MQTT] Ready!");
    } else {
      Serial.printf(" FAILED rc=%d\n", mqtt.state());
    }
    
  } else {
    Serial.println(" FAILED!");
  }

  // Create tasks
  xTaskCreatePinnedToCore(TaskBalance, "Balance", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskFirebase, "Firebase", 8192, NULL, 1, NULL, 0);

  Serial.println("\n================================");
  Serial.printf("  Firebase: %s\n", firebaseReady ? "OK" : "FAILED");
  Serial.printf("  MQTT: %s\n", mqttConnected ? "OK" : "FAILED");
  Serial.println("================================");
  Serial.println("[OK] System Ready!\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
