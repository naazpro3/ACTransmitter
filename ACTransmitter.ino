#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>


Adafruit_MPU6050 mpu;

// ================= ESP-NOW (ROBO SOCCER) =================
typedef struct {
  int ch1;  // Steering
  int ch2;
  int ch3;  // Throttle
  int ch4;
} ChannelData;

ChannelData espData;

// >>> REPLACE WITH YOUR ROBOSOCCER BOT MAC <<<
uint8_t soccerBotMAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ================= POTENTIOMETERS =================
#define POT1_PIN 36
#define POT2_PIN 39   // THROTTLE
#define POT3_PIN 34   // STEERING
#define POT4_PIN 35

constexpr int JOY_MIN = 1000;
constexpr int JOY_MAX = 2000;
constexpr int JOY_CENTER = 1500;

// ================= BUTTONS =================
#define BTN_DOWNSHIFT 18
#define BTN_UPSHIFT   19
#define BTN_JS_LEFT   23
#define BTN_JS_RIGHT  25

// ================= TRIM / DEADZONE =================
constexpr int MODE1_TRIM = 30;      // Mode 1 joystick trim
constexpr int MODE2_DEADZONE = 15;  // Mode 2 gyro deadzone

// ================= VIBRATION (L293D BRIDGED) =================
#define VIB_PIN_A 33
#define VIB_PIN_B 27
#define VIB_PWM_CH 0
#define VIB_PWM_FREQ 20000
#define VIB_PWM_RES 8

// ================= NEOPIXEL =================
#define NP_PIN    13
#define NP_COUNT  8
#define NP_BRIGHT 120
Adafruit_NeoPixel strip(NP_COUNT, NP_PIN, NEO_GRB + NEO_KHZ800);

// ================= STATE =================
unsigned long lastBlink = 0;
bool blinkState = false;

unsigned long vibKickUntil = 0;

bool downshiftAnimActive = false;
unsigned long animStartTime = 0;

bool prevDownshift = false;
bool prevUpshift   = false;

// ================= DOWNSHIFT ANIMATION =================
constexpr unsigned long DOWNSHIFT_ANIM_TIME = 400;
constexpr uint8_t PURPLE_R = 180;
constexpr uint8_t PURPLE_G = 0;
constexpr uint8_t PURPLE_B = 255;

// ================= GYRO STEERING =================
constexpr float GYRO_STEER_GAIN = 15.0f; // adjusted for 1000–2000 scale
float gyroAngle = 0.0f;                 // integrated angle
unsigned long lastLoopTime = 0;

// ================= COMPLEMENTARY FILTER =================
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

float fusedAngle = 0.0f;   // fused gyro + accel angle

// ================= STEERING MODE =================
enum SteeringMode { MODE_JOYSTICK = 1, MODE_GYRO = 2 };
SteeringMode currentMode = MODE_JOYSTICK;

unsigned long bothPaddlePressedTime = 0;
bool bothPaddlePrevState = false;
bool modeSwitchDone = false; // prevent continuous toggling

// ================= MODE LED INDICATION =================
bool modeIndicationActive = false;
unsigned long modeIndicationStart = 0;

// ================= GYRO MODE SWITCH =================
unsigned long gyroModeSwitchTime = 0;

// ================= iBUS / VJOY =================
constexpr uint8_t IBUS_CHANNEL_COUNT = 14;
constexpr uint16_t NEUTRAL = 1500;
uint16_t chVals[IBUS_CHANNEL_COUNT] = {NEUTRAL};
constexpr uint8_t IBUS_FRAME_SIZE = 32;

// ================= GLOBALS FOR iBUS =================
int finalSteering = JOY_CENTER;
int throttleRaw = 0;
bool downshift = false;
bool upshift = false;

// ================= FUNCTIONS =================
void triggerShiftVibration() {
  vibKickUntil = millis() + 180;
}

void startDownshiftAnimation() {
  downshiftAnimActive = true;
  animStartTime = millis();
}

// Map throttle 0–4095 to 1000–2000, center 1850 → 1500 (for sending only)
int mappedThrottle() {
  int raw = throttleRaw; // use raw ADC
  if (raw < 1850) {
    return 1000 + ((long)raw * 500) / 1850;
  } else {
    return 1500 + ((long)(raw - 1850) * 500) / (4095 - 1850);
  }
}

void sendToPC() {
  // Map transmitter values to iBus channels
  chVals[0] = finalSteering;                        // CH1 → Steering
  chVals[1] = mappedThrottle();                     // CH2 → Throttle (mapped)
  chVals[2] = downshift ? 2000 : 1000;             // CH3 → Downshift paddle
  chVals[3] = upshift   ? 2000 : 1000;             // CH4 → Upshift paddle
  chVals[4] = !digitalRead(BTN_JS_LEFT)  ? 2000 : 1000;  // CH5 → Left joystick button
  chVals[5] = !digitalRead(BTN_JS_RIGHT) ? 2000 : 1000;  // CH6 → Right joystick button

  // Fill rest with neutral
  for (uint8_t i = 6; i < IBUS_CHANNEL_COUNT; i++) {
    chVals[i] = NEUTRAL;
  }

  // Construct iBus frame
  uint8_t frame[IBUS_FRAME_SIZE];
  frame[0] = 0x20;
  frame[1] = 0x40;

  for (uint8_t i = 0; i < IBUS_CHANNEL_COUNT; i++) {
    frame[2 + i * 2]     = chVals[i] & 0xFF;
    frame[2 + i * 2 + 1] = (chVals[i] >> 8) & 0xFF;
  }

  uint16_t checksum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) checksum -= frame[i];

  frame[30] = checksum & 0xFF;
  frame[31] = (checksum >> 8) & 0xFF;

  Serial.write(frame, IBUS_FRAME_SIZE);
}

//esp32 espnow signal sender function
void sendToRoboSoccer() {
  espData.ch1 = finalSteering;        // Steering 1000–2000
  espData.ch2 = 1500;                 // unused
  espData.ch3 = mappedThrottle();     // Throttle 1000–2000
  espData.ch4 = 1500;                 // unused

  esp_now_send(soccerBotMAC, (uint8_t*)&espData, sizeof(espData));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // -------- MPU6050 --------
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // -------- BUTTONS --------
  pinMode(BTN_DOWNSHIFT, INPUT_PULLUP);
  pinMode(BTN_UPSHIFT,  INPUT_PULLUP);
  pinMode(BTN_JS_LEFT,  INPUT_PULLUP);
  pinMode(BTN_JS_RIGHT, INPUT_PULLUP);

  // -------- VIBRATION PWM --------
  ledcSetup(VIB_PWM_CH, VIB_PWM_FREQ, VIB_PWM_RES);
  ledcAttachPin(VIB_PIN_A, VIB_PWM_CH);
  pinMode(VIB_PIN_B, OUTPUT);
  digitalWrite(VIB_PIN_B, LOW);

  // -------- NEOPIXEL --------
  strip.begin();
  strip.setBrightness(NP_BRIGHT);
  strip.show();

  lastLoopTime = millis();

    // -------- ESP-NOW INIT --------
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, soccerBotMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("ESP-NOW peer add failed");
    }
  }
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0; // delta time in seconds
  lastLoopTime = now;

  // ================= MPU READ =================
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ================= INPUTS =================
  throttleRaw = analogRead(POT2_PIN); // raw for LEDs/vibration
  int steeringRaw = analogRead(POT3_PIN);

  downshift = !digitalRead(BTN_DOWNSHIFT);
  upshift   = !digitalRead(BTN_UPSHIFT);

  // ================= EDGE DETECT =================
  if (downshift && !prevDownshift) {
    triggerShiftVibration();
    startDownshiftAnimation();
  }
  if (upshift && !prevUpshift) {
    triggerShiftVibration();
  }

  prevDownshift = downshift;
  prevUpshift   = upshift;

  // ================= BOTH PADDLE PRESS MODE SWITCH =================
  bool bothPressed = downshift && upshift;

  if (bothPressed) {
    if (!modeSwitchDone) { // only allow toggle once per hold
      bothPaddlePressedTime += dt * 1000;
      if (bothPaddlePressedTime >= 3000) {
        SteeringMode previousMode = currentMode;
        currentMode = (currentMode == MODE_JOYSTICK) ? MODE_GYRO : MODE_JOYSTICK;

        // if switched to gyro mode, reset for 1s
        if (currentMode == MODE_GYRO) {
          gyroModeSwitchTime = millis();
          fusedAngle = 0;
        }

        modeIndicationActive = true;
        modeIndicationStart = millis();
        modeSwitchDone = true;
        bothPaddlePressedTime = 0;
      }
    }
  } else {
    bothPaddlePressedTime = 0;
    modeSwitchDone = false;
  }

  // ================= GYRO STEERING =================
  float gyroRate = -g.gyro.z * RAD_TO_DEG;

  if (currentMode == MODE_GYRO) {
    if (millis() - gyroModeSwitchTime < 1000) {
      finalSteering = JOY_CENTER;
    } else {
    // Absolute angle from accelerometer (gravity reference)
    float accelAngle =
      atan2(a.acceleration.y, a.acceleration.x) * RAD_TO_DEG;

    // Complementary filter
    fusedAngle =
      COMPLEMENTARY_ALPHA * (fusedAngle + gyroRate * dt) +
      (1.0f - COMPLEMENTARY_ALPHA) * accelAngle;

    int steer = JOY_CENTER + fusedAngle * GYRO_STEER_GAIN;

    if (abs(steer - JOY_CENTER) < MODE2_DEADZONE)
      steer = JOY_CENTER;

      finalSteering = constrain(steer, JOY_MIN, JOY_MAX);
    }
  } else {
    finalSteering = map(steeringRaw, 0, 4095, JOY_MIN, JOY_MAX) + MODE1_TRIM;
    finalSteering = constrain(finalSteering, JOY_MIN, JOY_MAX);
  }

  // ================= LED LOGIC =================
  strip.clear();
  if (modeIndicationActive && millis() - modeIndicationStart <= 2000) {
    if (currentMode == MODE_JOYSTICK) {
      for (int i = 0; i < 4; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
    } else {
      for (int i = 4; i < 8; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
    }
    strip.show();
  } else {
    modeIndicationActive = false;

    if (downshiftAnimActive) {
      unsigned long elapsed = millis() - animStartTime;
      float progress = elapsed / float(DOWNSHIFT_ANIM_TIME);
      progress = constrain(progress, 0.0, 1.0);

      uint8_t intensity = uint8_t(progress * 255);
      int maxPairs = NP_COUNT / 2;
      int activePairs = max(1, int(progress * maxPairs));

      int centerL = (NP_COUNT / 2) - 1;
      int centerR = (NP_COUNT / 2);

      uint8_t r = (PURPLE_R * intensity) / 255;
      uint8_t gCol = (PURPLE_G * intensity) / 255;
      uint8_t b = (PURPLE_B * intensity) / 255;

      for (int i = 0; i < activePairs; i++) {
        if (centerL - i >= 0)
          strip.setPixelColor(centerL - i, strip.Color(r, gCol, b));
        if (centerR + i < NP_COUNT)
          strip.setPixelColor(centerR + i, strip.Color(r, gCol, b));
      }

      if (progress >= 1.0) {
        downshiftAnimActive = false;
      }
    } else {
      if (throttleRaw < 1840) {
        strip.clear();
      } else {
        int leds = map(throttleRaw, 1840, 4095, 1, NP_COUNT);
        leds = constrain(leds, 1, NP_COUNT);

        float t = float(throttleRaw - 1840) / (4095 - 1840);
        uint8_t r, gCol;
        if (throttleRaw >= 4095) {
          r = 255; gCol = 0;
        } else {
          r = uint8_t(t * 255);
          gCol = uint8_t((1.0 - t) * 255);
        }

        for (int i = 0; i < leds; i++) {
          strip.setPixelColor((NP_COUNT - 1) - i, strip.Color(r, gCol, 0));
        }
      }
    }
    strip.show();
  }

  // ================= VIBRATION =================
  int vibPWM = 0;
  if (millis() < vibKickUntil) {
    // shift vibration unchanged
    vibPWM = 230;
  }
  else if (throttleRaw > 1840 && throttleRaw < 4095) {
    // throttle/RPM vibration halved
    vibPWM = map(throttleRaw, 1840, 4095, 40, 160);
  }
  else if (throttleRaw >= 4095) {
    vibPWM = 170; // 50% of max
  }

  vibPWM = constrain(vibPWM, 0, 255);
  ledcWrite(VIB_PWM_CH, vibPWM);

  // ================= SEND TO PC =================
  sendToPC();

  sendToRoboSoccer();

  delay(7); // approximate iBus frame rate
}
