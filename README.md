#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <ArduinoJson.h>

// --- Config WiFi & MQTT ---
// [SECURITY WARNING] Do not upload real passwords to GitHub!
const char ssid[] = "pakkapon2006";  // แก้เป็นชื่อ WiFi ของคุณตอนใช้งานจริง
const char pass[] = "33333333";  // แก้เป็นรหัสผ่าน WiFi ของคุณตอนใช้งานจริง

const char* mqtt_server = "broker.hivemq.com"; 
const int mqtt_port = 1883;

const char* TOPIC_CMD = "myrobot/command";    
const char* TOPIC_STATUS = "myrobot/status"; 

WiFiClient espClient;
PubSubClient client(espClient);

LiquidCrystal_I2C lcd(0x27, 16, 2); 
#define SDA_PIN 16
#define SCL_PIN 17

// --- SENSORS & PINS ---
const int TRIG_PIN = 21, ECHO_PIN = 22, DISTANCE_STOP = 25; 
const int TRIG_LOAD = 15, ECHO_LOAD = 34;
const int LOAD_THRESHOLD = 10; 

const int LED_RED = 2, LED_YELLOW = 5, LED_GREEN = 23; 
const int S1 = 13, S2 = 12, S3 = 14, S4 = 27, S5 = 4;
const int LINE = 0; 
const int IN1 = 32, IN2 = 33, IN3 = 25, IN4 = 26, ENA = 19, ENB = 18; 

// --- SPEEDS ---
const int SPD_NORMAL = 1800;
const int SPD_LOW    = 1000;
const int SPD_BOOST  = 2047;
const int SPD_SPIN   = 2047; 

int targetRoom = 0;       
bool isDelivering = false, isReturning = false, readyToCount = true;
int intersectionCount = 0, muteMode = 0;
bool hasCargo = false;
unsigned long muteTimer = 0, lastMsgTime = 0;

// --- Function Prototypes ---
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void updateStatus();
void checkCargoStatus();
long getDistance();
long getLoadDistance();
void motorControl(int speedL, int speedR, bool fwdL, bool fwdR);
void stopCar();
void spinBack170();
void executeTurn(bool isLeft);
void handleLineFollowing();

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_GREEN, OUTPUT);
  pinMode(TRIG_LOAD, OUTPUT); pinMode(ECHO_LOAD, INPUT);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init(); lcd.backlight(); lcd.print("System Booting");

  analogWriteResolution(ENA, 11); analogWriteResolution(ENB, 11);
  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT); pinMode(S5, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop(); 

  if (millis() - lastMsgTime > 1000) {
    updateStatus(); 
    lastMsgTime = millis();
  }

  int v1 = digitalRead(S1); int v2 = digitalRead(S2); int v3 = digitalRead(S3);
  int v4 = digitalRead(S4); int v5 = digitalRead(S5);

  // --- Check 5-Point Intersection ---
  if (v1 == LINE && v2 == LINE && v3 == LINE && v4 == LINE && v5 == LINE) {
      
      // CASE: Arrived at Target Room
      if (isDelivering) {
          stopCar(); 
          lcd.setCursor(0, 1); lcd.print("Arrived Room   ");
          while (getLoadDistance() < LOAD_THRESHOLD) { // Wait for unload
             stopCar();
             delay(100);
             digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW)); 
          }
          digitalWrite(LED_YELLOW, LOW);
          lcd.setCursor(0, 1); lcd.print("Unloaded! Back ");
          delay(1000); 
          spinBack170();
          isDelivering = false;
          isReturning = true;
          readyToCount = true;
          intersectionCount = 0; 
      }
      
      // CASE: Arrived Home
      else if (isReturning) {
          bool arrivedHome = false;
          if (targetRoom == 2 && intersectionCount >= 3) arrivedHome = true;
          else if (targetRoom == 3 && intersectionCount >= 3) arrivedHome = true;
          else if (targetRoom == 1 && intersectionCount >= 2) arrivedHome = true;

          if (arrivedHome) {
              stopCar();
              lcd.setCursor(0, 1); lcd.print("Home! Spin 170 ");
              delay(500);
              spinBack170();   
              stopCar();        
              isReturning = false;
              targetRoom = 0;
              lcd.setCursor(0, 1); lcd.print("Ready for Cmd  ");
              return;
          } else {
              handleLineFollowing();
              return;
          }
      }
      return; 
  }

  // Safety Stop: Off-track
  if (v1 != LINE && v2 != LINE && v3 != LINE && v4 != LINE && v5 != LINE) {
      stopCar();
      return; 
  }

  // Obstacle Avoidance
  if (getDistance() < DISTANCE_STOP) {
    stopCar();
    digitalWrite(LED_RED, HIGH);
    return; 
  } else {
    digitalWrite(LED_RED, LOW);
  }

  if (!isDelivering && !isReturning) { 
    stopCar(); 
    return; 
  }

  handleLineFollowing();
}

void handleLineFollowing() {
  int v1 = digitalRead(S1); int v2 = digitalRead(S2); int v3 = digitalRead(S3);
  int v4 = digitalRead(S4); int v5 = digitalRead(S5);
  bool is5Point = (v1==LINE && v2==LINE && v3==LINE && v4==LINE && v5==LINE);

  if (millis() - muteTimer > 1600) muteMode = 0;
  if (muteMode == 1) { v1 = !LINE; v2 = !LINE; } 
  if (muteMode == 2) { v4 = !LINE; v5 = !LINE; } 

  // Delivering Logic
  if (isDelivering && muteMode == 0 && readyToCount) {
    if (v1 == LINE || v5 == LINE) {
        if (targetRoom == 2) {
            intersectionCount++; readyToCount = false;
            if (intersectionCount == 1) { if(v1 == LINE) muteMode = 1; else muteMode = 2; muteTimer = millis(); }
            else if (intersectionCount == 2) executeTurn(true); 
        }
        else if (targetRoom == 3) {
             intersectionCount++; readyToCount = false;
             if (intersectionCount <= 2) { if(v1 == LINE) muteMode = 1; else muteMode = 2; muteTimer = millis(); }
             else if (intersectionCount == 3) executeTurn(false); 
        }
        else if (targetRoom == 1) {
            intersectionCount++; readyToCount = false;
            if (intersectionCount == 1) executeTurn(false); 
            else { muteMode = 2; muteTimer = millis(); }
        }
    }
  }
  
  // Returning Logic
  if (isReturning && muteMode == 0 && readyToCount) {
    if (v1 == LINE || v5 == LINE || is5Point) {
        if (targetRoom == 2) {
            intersectionCount++; readyToCount = false;
            if (intersectionCount == 1) executeTurn(false); 
            else if (intersectionCount == 2) { if(v1 == LINE) muteMode = 1; else muteMode = 2; muteTimer = millis(); }
        }
        else if (targetRoom == 3) {
            intersectionCount++; readyToCount = false;
            if (intersectionCount == 1) executeTurn(true); 
            else if (intersectionCount == 2) { if(v1 == LINE) muteMode = 1; else muteMode = 2; muteTimer = millis(); }
        }
        else if (targetRoom == 1) {
            intersectionCount++; readyToCount = false;
            if (intersectionCount == 1) executeTurn(true); 
        }
    }
  }

  if (v1 != LINE && v5 != LINE && !is5Point) readyToCount = true;

  if (v2 == LINE) motorControl(SPD_LOW, SPD_BOOST, true, true);
  else if (v4 == LINE) motorControl(SPD_BOOST, SPD_LOW, true, true);
  else motorControl(SPD_NORMAL, SPD_NORMAL, true, true);
}

void spinBack170() { 
    stopCar(); delay(200);
    motorControl(SPD_SPIN, SPD_SPIN, false, false); delay(300); 
    stopCar(); delay(200);
    motorControl(SPD_SPIN, SPD_SPIN, true, false); 
    delay(550); 
    unsigned long startSearch = millis();
    while(digitalRead(S3) != LINE) {
        if (millis() - startSearch > 3000) break; 
    }
    stopCar(); delay(200);
}

void executeTurn(bool isLeft) {
  motorControl(SPD_NORMAL, SPD_NORMAL, true, true); delay(500); 
  if (isLeft) motorControl(SPD_SPIN, SPD_SPIN, false, true); 
  else motorControl(SPD_SPIN, SPD_SPIN, true, false);       
  delay(400); 
  while(digitalRead(S3) != LINE); 
  stopCar(); delay(100);
}

long getDistance() { digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2); digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW); long duration = pulseIn(ECHO_PIN, HIGH, 30000); long cm = duration * 0.034 / 2; return (cm <= 0) ? 999 : cm; }
long getLoadDistance() { digitalWrite(TRIG_LOAD, LOW); delayMicroseconds(2); digitalWrite(TRIG_LOAD, HIGH); delayMicroseconds(10); digitalWrite(TRIG_LOAD, LOW); long duration = pulseIn(ECHO_LOAD, HIGH, 10000); long cm = duration * 0.034 / 2; return (cm <= 0) ? 999 : cm; }
void motorControl(int speedL, int speedR, bool fwdL, bool fwdR) { analogWrite(ENA, speedL); analogWrite(ENB, speedR); digitalWrite(IN1, fwdL ? HIGH : LOW); digitalWrite(IN2, fwdL ? LOW : HIGH); digitalWrite(IN3, fwdR ? HIGH : LOW); digitalWrite(IN4, fwdR ? LOW : HIGH); }
void stopCar() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENA, 0); analogWrite(ENB, 0); }

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload, length);
  const char* cmd = doc["cmd"];
  String msg = String(cmd);
  if (msg == "room1") { targetRoom = 1; isDelivering = true; isReturning = false; intersectionCount = 0; readyToCount = true; } 
  else if (msg == "room2") { targetRoom = 2; isDelivering = true; isReturning = false; intersectionCount = 0; readyToCount = true; } 
  else if (msg == "room3") { targetRoom = 3; isDelivering = true; isReturning = false; intersectionCount = 0; readyToCount = true; } 
}

void checkCargoStatus() { long dist = getLoadDistance(); hasCargo = (dist > 0 && dist < LOAD_THRESHOLD); }
void updateStatus() {
  checkCargoStatus();
  if (isDelivering || isReturning) { digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_YELLOW, LOW); }
  else { digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_GREEN, LOW); }
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("WF:OK "); lcd.print(hasCargo ? "Ld:YES" : "Ld:NO ");
  lcd.setCursor(0, 1);
  if (isDelivering) { lcd.print("Go Rm: "); lcd.print(targetRoom); }
  else if (isReturning) lcd.print("Home Coming"); else lcd.print("Ready...");
  StaticJsonDocument<200> doc; 
  doc["cargo"] = hasCargo; doc["status"] = isDelivering ? "delivering" : "idle"; doc["room"] = targetRoom;
  char jsonBuffer[512]; serializeJson(doc, jsonBuffer); client.publish(TOPIC_STATUS, jsonBuffer);
}
void setup_wifi() { WiFi.begin(ssid, pass); while (WiFi.status() != WL_CONNECTED) delay(500); }
void reconnect() { while (!client.connected()) { String clientId = "ESP32-"; clientId += String(random(0xffff), HEX); if (client.connect(clientId.c_str())) client.subscribe(TOPIC_CMD); else delay(5000); } }
