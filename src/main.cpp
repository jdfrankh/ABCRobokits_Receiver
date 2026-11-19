#include <Arduino.h>
#include <WiFi.h>
#include "EspNowEZ.h"
#include <esp_now.h>
#include <esp_system.h>
#include "esp_wifi.h"
#include <ESP32Servo.h>
#include <vector>
#include "ESC.h" // RC_ESP library installed by Library Manager

const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int timeout = 0;
bool receivedRemoteSignal = false; 
float MotorSpeed = .99f;

//For newer boards
//#define Mot1 0
//#define Mot2 3

//For older boards
#define Mot1 2
#define Mot2 3

//For newer boards
//#define Mot3 8
//#define Mot4 4

//For older boards
#define Mot3 8
#define Mot4 9

//For my newer boards
//#define ESC1 2
//#define ESC2 1

//For my older boards
#define ESC1 1
#define ESC2 10 // #define ES2 0

// ---------------------------------------------------------------------------
// Battery / motor scaling
// ---------------------------------------------------------------------------
// BLHeli ESCs translate the 1000-2000 us pulse linearly into average motor
// voltage:  Vmotor_avg ~= throttle_fraction * Vbattery.  That means the same
// PWM pulse drives a motor harder on a 4S pack than on a 3S pack.  To keep
// the 5010 within a safe operating envelope regardless of pack size, we cap
// the maximum pulse width based on the configured cell count so that the
// motor never sees more than MOTOR_MAX_VOLTAGE on average.
//
// Tune these two numbers for your setup:
//   BATTERY_CELLS     - number of LiPo cells in series (3 for 3S, 4 for 4S)
//   MOTOR_MAX_VOLTAGE - the highest average voltage you want the motor to see
//                       at full throttle.  11.1 V (3S nominal) is a
//                       conservative default for a generic 5010; raise it if
//                       your motor is rated for higher voltage.
#define BATTERY_CELLS         2
#define CELL_NOMINAL_VOLTAGE  3.7f
// 5010 motors are typically rated for 4S-6S.  14.8 V = full 4S nominal,
// which lets the ESC reach full 2000 us throttle on a 4S pack.  Drop this
// back toward 11.1 V if you ever swap in a motor with a lower voltage limit.
#define MOTOR_MAX_VOLTAGE     8.1f


#define ESCSMINDISTANCE       900  // 900 to 1100
#define ESC_ABSOLUTE_MAX_US   2200  // BLHeli full-throttle pulse

// throttle_cap = MOTOR_MAX_VOLTAGE / Vbattery_nominal, clamped to [0,1]
static constexpr float _battery_nominal_v =
    BATTERY_CELLS * CELL_NOMINAL_VOLTAGE;
static constexpr float _throttle_cap = _battery_nominal_v;
    // (MOTOR_MAX_VOLTAGE >= _battery_nominal_v)
    //     ? 1.0f
    //     : (MOTOR_MAX_VOLTAGE / _battery_nominal_v);

// Maximum pulse width we will ever send to the ESC for this pack.
#define ESCMAXDISTANCE                                                         \
    ((int)(ESCSMINDISTANCE +                                                   \
           (ESC_ABSOLUTE_MAX_US - ESCSMINDISTANCE) * _throttle_cap))

//For newer boards
//#define SERVO1 10
//#define SERVO2 9

//For older boards
#define SERVO1 0 // #define SERVO1 10
#define SERVO2 7

#define SERVOIDLEANGLE 0
#define SERVOARMANGLE 90
#define SERVOATTACKANGLE 180

#define PWM1 20
#define PWM2 21

#define LED1 6
#define LED2 5

const float ESCADJUST = 500.0f;

Servo FirstESC;
float escSpeed1 = 0;
bool escArmed = false;
//ESC SecondESC(ESC2, 1000, 2000, 1000); 
uint16_t escSpeed2 = 0;





//struct __attribute__((packed)) Message{
  const char id[20] = "Receiver 14";
//};



  // Layout must stay byte-identical to sendData in ESPRemote/src/main.cpp.
  struct __attribute__((packed)) sendData{
  uint8_t macHandshake[6];
  bool buttons[4]; // Array to store the state of the buttons
  uint16_t joysticksDegrees[2]; // Array to store the state of the joysticks -- polar coordinates
  uint16_t joysticksMagnitude[2]; // Array to store the magnitude of the joysticks

  bool connectCheck = false;
};

// Drive scaling now lives here instead of riding along in the packet.
#define DC_MAX 255
#define DC_MIN 0
#define JOYSTICK_DEADZONE 600


// Remote button map
#define BTN_LEFTTRIGGER  0
#define BTN_RIGHTTRIGGER 1
#define BTN_BOTTOM       2
#define BTN_TOP          3

// Weapon controls: the right trigger cycles the preset levels, the left one
// is an instant kill.
#define BTN_THROTTLE_CYCLE BTN_RIGHTTRIGGER
#define BTN_THROTTLE_STOP  BTN_LEFTTRIGGER
#define BTN_ARM            BTN_TOP

// A button must read the same for this many consecutive packets before an edge
// counts, so a single noisy sample cannot disarm or spin up mid-fight.
#define ARM_DEBOUNCE_SAMPLES 5

struct EdgeButton {
  bool stable = false;
  uint8_t sameCount = 0;

  // True exactly once per debounced press.
  bool pressed(bool raw){
    if(raw == stable){ sameCount = 0; return false; }
    if(++sameCount < ARM_DEBOUNCE_SAMPLES) return false;
    sameCount = 0;
    stable = raw;
    return stable;
  }
};

EdgeButton armBtn, cycleBtn, stopBtn;

// Throttle presets in cycle order; index 0 is off. No ramping - each press
// jumps straight to the next level.
static const uint8_t escLevels[] = {0, 100, 75, 50};
#define ESC_LEVEL_COUNT (sizeof(escLevels) / sizeof(escLevels[0]))
uint8_t escLevelIndex = 0;

static void escAttachIdle(){
  FirstESC.setPeriodHertz(50);
  FirstESC.attach(ESC2, ESCSMINDISTANCE, ESC_ABSOLUTE_MAX_US);
  FirstESC.writeMicroseconds(ESCSMINDISTANCE);
}

// ledcDetachPin leaves the pad floating, and a floating signal wire reads as
// noise to the ESC, so park it low.
static void escDetachIdle(){
  FirstESC.writeMicroseconds(ESCSMINDISTANCE);
  FirstESC.detach();
  pinMode(ESC2, OUTPUT);
  digitalWrite(ESC2, LOW);
}

// Cycles of ~10 ms without a packet before the failsafe cuts the weapon.
#define LINK_TIMEOUT_CYCLES 50

enum Direction {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
};

static uint8_t joystickToDirection(uint16_t degrees, uint16_t magnitude) {
  if (magnitude < JOYSTICK_DEADZONE) return STOP;
  if (degrees >= 45 && degrees < 135)  return FORWARD;
  if (degrees >= 135 && degrees < 225) return LEFT;
  if (degrees >= 225 && degrees < 315) return BACKWARD;
  return RIGHT;
}

static uint8_t magnitudePercent(uint16_t magnitude) {
  return (uint8_t)constrain((magnitude * 100) / 2048, 0, 100);
}

Container message;
uint8_t macAddress[6];
//Message msg;
sendData sD;

// ---------------------------------------------------------------------------
// Telemetry sent back to the remote.
// This layout must stay byte-identical to the copy in ESPRemote/src/mainMenu.h.
// ---------------------------------------------------------------------------
#define TELEMETRY_MAGIC       0xA7
#define TELEMETRY_MAX_ITEMS   6
#define TELEMETRY_KEY_LEN     10
#define TELEMETRY_VALUE_LEN   10
#define TELEMETRY_PERIOD_MS   150

struct __attribute__((packed)) TelemetryItem {
  char key[TELEMETRY_KEY_LEN];
  char value[TELEMETRY_VALUE_LEN];
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t magic;
  uint8_t count;
  TelemetryItem items[TELEMETRY_MAX_ITEMS];
};

static uint32_t lastTelemetryMs = 0;

static void addTelemetry(TelemetryPacket &p, const char *key, const char *value) {
  if (p.count >= TELEMETRY_MAX_ITEMS) return;
  strncpy(p.items[p.count].key, key, TELEMETRY_KEY_LEN - 1);
  strncpy(p.items[p.count].value, value, TELEMETRY_VALUE_LEN - 1);
  p.count++;
}

static const char *directionName(uint8_t d) {
  switch (d) {
    case 1:  return "FWD";
    case 2:  return "BACK";
    case 3:  return "LEFT";
    case 4:  return "RIGHT";
    default: return "STOP";
  }
}

void sendTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = millis();

  // No remote MAC yet means nothing has been received, so nowhere to reply to.
  if (!esp_now_is_peer_exist(sD.macHandshake)) return;

  TelemetryPacket p = {};
  p.magic = TELEMETRY_MAGIC;

  uint8_t dir[2] = {
      joystickToDirection(sD.joysticksDegrees[0], sD.joysticksMagnitude[0]),
      joystickToDirection(sD.joysticksDegrees[1], sD.joysticksMagnitude[1])};

  bool moving = (dir[0] != STOP) || (dir[1] != STOP);
  int throttlePct = (int)((escSpeed1 * 100.0f) /
                          (float)(ESCMAXDISTANCE - ESCSMINDISTANCE));

  char buf[TELEMETRY_VALUE_LEN];

  addTelemetry(p, "ARMED", escArmed ? "YES" : "NO");
  addTelemetry(p, "WEAPON", escSpeed1 > 0.0f ? "SPIN" : "IDLE");
  addTelemetry(p, "DRIVE", directionName(dir[0]));
  addTelemetry(p, "MOVING", moving ? "YES" : "NO");

  snprintf(buf, sizeof(buf), "%d%%", constrain(throttlePct, 0, 100));
  addTelemetry(p, "THROT", buf);

  snprintf(buf, sizeof(buf), "%u",
           (unsigned)magnitudePercent(sD.joysticksMagnitude[0]));
  addTelemetry(p, "SPEED", buf);

  esp_now_send(sD.macHandshake, (uint8_t *)&p, sizeof(p));
}

bool waitingForResponse(){

 

  //message.store(msg); 
  //message.printBytes();
  int result = esp_now_send(broadcastAddress, (uint8_t*) &id, sizeof(id) );
  //int result = ESPNow.broadcastMessage(message);
  result == ESP_OK ? digitalWrite(LED1, !digitalRead(LED1)) : digitalWrite(LED1, LOW);

  //result == ESP_OK ? Serial.println("Sent successfully"): Serial.printf("Broadcast send failed: %d\n", result);

  delay(250); // Send every 2 seconds


  return(true);

}

// Optional: callback for send status
//void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {

//}

void onReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  const uint8_t *recv_info = mac_addr;
  
  if (len == sizeof(sendData)) {
   // Serial.println("Data received from remote!"); 
    memcpy(&sD, incomingData, sizeof(sendData));

    //Serial.printf("%02x ,%02x,%02x,%20x: ", sD.Direction[0], sD.Direction[1], sD.objectRun[0], sD.objectArm[0]);
  //  for(size_t i= 0; i < len; i++){
  //    Serial.printf("%02X,", incomingData[i]);
  //  }
  //  Serial.printf("Length: %d", len);
  //  Serial.printf(": %d", sD.Direction[0]);
  //  Serial.println();

  //Serial.printf("Arm 0: %d, Arm 1: %d, Interact 0: %d, Interact 1: %d" , sD.objectArm[0],sD.objectArm[1], sD.objectRun[0], sD.objectRun[1] );
  //Serial.println();

    if (!esp_now_is_peer_exist(sD.macHandshake)) {
  //  Serial.println("Adding peer");
    ESPNow.add_peer(sD.macHandshake);
    }
    
    digitalWrite(LED2, HIGH);
    timeout = LINK_TIMEOUT_CYCLES;



    if(sD.macHandshake[5] == macAddress[5] && sD.macHandshake[4] == macAddress[4] && sD.macHandshake[3] == macAddress[3] ){
     
    }

  } else {
   // Serial.print("Unexpected message length: ");
    //Serial.println(len);
  }


}




void mapFunction(){
//  Serial.println("Mapping");

  // Sampled every cycle so the edge state stays in sync even while disarmed.
  bool cyclePressed = cycleBtn.pressed(sD.buttons[BTN_THROTTLE_CYCLE]);
  bool stopPressed  = stopBtn.pressed(sD.buttons[BTN_THROTTLE_STOP]);

  // Arm/disarm toggles on a debounced rising edge; boots disarmed and off.
  // The ESC is only attached while armed, so the signal line is dead otherwise.
  if(armBtn.pressed(sD.buttons[BTN_ARM])){
    escArmed = !escArmed;
    escLevelIndex = 0;
    escArmed ? escAttachIdle() : escDetachIdle();
  }

  if(!escArmed || stopPressed){
    escLevelIndex = 0;
  }
  else if(cyclePressed){
    escLevelIndex = (escLevelIndex + 1) % ESC_LEVEL_COUNT;
  }

  escSpeed1 = (escLevels[escLevelIndex] / 100.0f) *
              (float)(ESCMAXDISTANCE - ESCSMINDISTANCE);
  if(escArmed){
    FirstESC.writeMicroseconds(ESCSMINDISTANCE + (int)escSpeed1);
  }
  digitalWrite(LED2, escLevelIndex > 0 ? HIGH : LOW);

  uint8_t dir = joystickToDirection(sD.joysticksDegrees[0],
                                    sD.joysticksMagnitude[0]);

  switch(dir){
    case STOP:
      analogWrite(Mot1, DC_MIN * MotorSpeed);
      analogWrite(Mot2, DC_MIN * MotorSpeed);
      analogWrite(Mot3, DC_MIN * MotorSpeed);
      analogWrite(Mot4, DC_MIN * MotorSpeed);
    break;
    case FORWARD:
      analogWrite(Mot1, DC_MAX * MotorSpeed);
      analogWrite(Mot2, DC_MIN * MotorSpeed);
      analogWrite(Mot3, DC_MAX * MotorSpeed);
      analogWrite(Mot4, DC_MIN * MotorSpeed);
    break;
    case BACKWARD:
      analogWrite(Mot1, DC_MIN * MotorSpeed);
      analogWrite(Mot2, DC_MAX * MotorSpeed);
      analogWrite(Mot3, DC_MIN * MotorSpeed);
      analogWrite(Mot4, DC_MAX * MotorSpeed);
    break;
    case LEFT:
      analogWrite(Mot1, DC_MIN * MotorSpeed);
      analogWrite(Mot2, DC_MAX * MotorSpeed);
      analogWrite(Mot3, DC_MAX * MotorSpeed);
      analogWrite(Mot4, DC_MIN * MotorSpeed);
    break;
    case RIGHT:
      analogWrite(Mot1, DC_MAX * MotorSpeed);
      analogWrite(Mot2, DC_MIN * MotorSpeed);
      analogWrite(Mot3, DC_MIN * MotorSpeed);
      analogWrite(Mot4, DC_MAX * MotorSpeed);
    break;
    default:
      analogWrite(Mot1, DC_MIN * MotorSpeed);
      analogWrite(Mot2, DC_MIN * MotorSpeed);
      analogWrite(Mot3, DC_MIN * MotorSpeed);
      analogWrite(Mot4, DC_MIN * MotorSpeed);
    break;

  };


}

void setup() {
//  delay(10000);
//  Serial.begin(115200);
 // delay(1000);

  //delay(250);

  pinMode(Mot1, OUTPUT);
  pinMode(Mot2, OUTPUT);
  pinMode(Mot3, OUTPUT);
  pinMode(Mot4, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);


//  pinMode(ESC1, OUTPUT);
 // pinMode(ESC2, OUTPUT);   

  digitalWrite(LED1,LOW);
  digitalWrite(LED2, LOW);

  digitalWrite(Mot1, LOW);
  digitalWrite(Mot2, LOW);
  digitalWrite(Mot3, LOW);
  digitalWrite(Mot4, LOW);

  // Reserve a timer for the ESC, but leave the signal pin dead until armed.
  ESP32PWM::allocateTimer(0);
  pinMode(ESC2, OUTPUT);
  digitalWrite(ESC2, LOW);

  //delay(250);



  //ESP_ERROR_CHECK(esp_wifi_set_channel(chan,WIFI_SECOND_CHAN_NONE));

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  if (ESPNow.init() != ESP_OK) {
  //  Serial.println("Error initializing ESP-NOW");
    return;
  }
  WiFi.macAddress(macAddress); 
  //WiFi.setTxPower(WIFI_POWER_19dBm);
  ESPNow.reg_recv_cb(onReceive);
  // Optional: set send callback if you want to track status
  //ESPNow.reg_send_cb(onSend);

  
//  Serial.print("My MAC Address: ");
 // Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],macAddress[1],macAddress[2],macAddress[3],macAddress[4],macAddress[5]);
 // Serial.println();
 // Serial.printf("ESC throttle cap: %dS pack, Vnom=%.1fV, motor cap=%.1fV -> "
 //               "max pulse = %d us (%.0f%% throttle)\n",
 //               BATTERY_CELLS, _battery_nominal_v, MOTOR_MAX_VOLTAGE,
 //               ESCMAXDISTANCE, _throttle_cap * 100.0f);

  int peerErr = ESPNow.add_peer((uint8_t*)broadcastAddress, 0);
  //if (peerErr != ESP_OK) Serial.printf("add_peer(broadcast) failed: %d\n", peerErr);

}

void loop() {
  if(timeout > 0){
    digitalWrite(LED1, LOW);
    mapFunction();
    sendTelemetry();
    timeout--;
    delay(10);
  }
  else{
    digitalWrite(LED1,LOW);
    digitalWrite(LED2, LOW);
    // Link lost: fall back to idle instead of holding the last throttle.
    if(escArmed){
      escArmed = false;
      escDetachIdle();
    }
    escSpeed1 = 0;
    escLevelIndex = 0;
    waitingForResponse();
  }

}

