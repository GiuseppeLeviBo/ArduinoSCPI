#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
/*
#if !defined(ARDUINO_ARCH_ESP32)
#error "This sketch targets ESP32 / WEMOS D1 R32."
#endif
*/
#define BAUDRATE 115200

enum ScpiError : int8_t {
  ERR_NONE = 0,
  ERR_CMD_UNKNOWN = -1,
  ERR_PARAM_RANGE = -2,
  ERR_EXECUTION = -3,
  ERR_TIMEOUT = -4,
  ERR_MODE = -5
};

enum GpioMode : uint8_t {
  MODE_IN = 0,
  MODE_OUT,
  MODE_PULLUP,
  MODE_ANA
};

enum AnalogRef : uint8_t {
  REF_DEF = 0,
  REF_INT,
  REF_EXT
};

enum TriggerMode : uint8_t {
  TRIG_IMM = 0,
  TRIG_ANA,
  TRIG_DIG
};

enum TrigSlope : uint8_t {
  SLOPE_POS = 0,
  SLOPE_NEG
};

enum AcqState : uint8_t {
  ACQ_IDLE = 0,
  ACQ_PREFILL,
  ACQ_ARMED,
  ACQ_POST,
  ACQ_DONE
};

enum RadioMode : uint8_t {
  RADIO_OFF = 0,
  RADIO_WIFI,
  RADIO_BT,
  RADIO_COEX,
  RADIO_UNKNOWN
};

enum PinCapability : uint8_t {
  CAP_IN = 1 << 0,
  CAP_OUT = 1 << 1,
  CAP_PULLUP = 1 << 2,
  CAP_ANA = 1 << 3
};

struct PhysicalPinDef {
  uint8_t gpio;
  const char *label;
  int8_t digitalChannel;
  int8_t analogChannel;
  uint8_t caps;
  bool adc2;
};

static const PhysicalPinDef kPhysicalPins[] = {
  {26, "D2", 0, 6, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {25, "D3", 1, 7, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {17, "D4", 2, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {16, "D5", 3, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {27, "D6", 4, 8, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {14, "D7", 5, 9, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {12, "D8", 6, 10, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {13, "D9", 7, 11, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {5, "D10", 8, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {23, "D11", 9, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {19, "D12", 10, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {18, "D13", 11, -1, CAP_IN | CAP_OUT | CAP_PULLUP, false},
  {2, "A0", -1, 0, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {4, "A1", -1, 1, CAP_IN | CAP_OUT | CAP_PULLUP | CAP_ANA, true},
  {35, "A2", -1, 2, CAP_IN | CAP_ANA, false},
  {34, "A3", -1, 3, CAP_IN | CAP_ANA, false},
  {36, "A4", -1, 4, CAP_IN | CAP_ANA, false},
  {39, "A5", -1, 5, CAP_IN | CAP_ANA, false}
};

static const uint8_t kDigitalPins[] = {26, 25, 17, 16, 27, 14, 12, 13, 5, 23, 19, 18};
static const uint8_t kAnalogPins[] = {2, 4, 35, 34, 36, 39, 26, 25, 27, 14, 12, 13};
static const uint8_t kPwmPins[] = {13, 5};
static const uint8_t kDacPins[] = {25, 26};

static const uint8_t kPhysicalPinCount = sizeof(kPhysicalPins) / sizeof(kPhysicalPins[0]);
static const uint8_t kDigitalPinCount = sizeof(kDigitalPins) / sizeof(kDigitalPins[0]);
static const uint8_t kAnalogPinCount = sizeof(kAnalogPins) / sizeof(kAnalogPins[0]);
static const uint8_t kPwmPinCount = sizeof(kPwmPins) / sizeof(kPwmPins[0]);
static const uint8_t kDacPinCount = sizeof(kDacPins) / sizeof(kDacPins[0]);

static const uint16_t MAX_TOTAL_POINTS = 4096;
static const uint32_t PWM_FREQ_HZ = 5000;
static const uint8_t PWM_RES_BITS = 8;
static const uint32_t SERVO_FREQ_HZ = 50;
static const uint8_t SERVO_RES_BITS = 16;
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2500;

ScpiError lastError = ERR_NONE;
bool ackEnabled = true;
AnalogRef refMode = REF_DEF;
float vRef = 3.3f;
uint8_t adcResolutionBits = 12;

GpioMode pinModes[kPhysicalPinCount];
bool pinDigitalStates[kPhysicalPinCount];

uint8_t currentAnalogChannel = 0;
uint8_t scanList[kAnalogPinCount];
uint8_t scanCount = 0;

TriggerMode trigMode = TRIG_IMM;
TrigSlope trigSlope = SLOPE_POS;
float trigLevel = 1.65f;
unsigned long trigTimeout = 1000;
int trigAnalogChannel = 0;
int trigDigitalPinIndex = 0;
float lastTrigVolt = 0.0f;
int lastTrigDig = LOW;

AcqState acqState = ACQ_IDLE;
uint16_t acqBuffer[MAX_TOTAL_POINTS];
uint16_t acqPoints = 64;
uint16_t acqHead = 0;
uint16_t prefillCount = 0;
uint16_t postCount = 0;
unsigned long lastSampleTime = 0;
uint32_t acqTStep = 1000;

uint8_t pwmValue[kPwmPinCount];
uint8_t servoAngle[kPwmPinCount];
bool pwmAttached[kPwmPinCount];
bool servoAttached[kPwmPinCount];
uint8_t pinPwmValue[kPhysicalPinCount];
uint32_t pinPwmFreq[kPhysicalPinCount];
bool pinPwmAttached[kPhysicalPinCount];
uint8_t dacValue[kDacPinCount];

RadioMode radioMode = RADIO_OFF;
bool adc2Locked = false;
bool wifiScanInProgress = false;
int lastWifiScanCount = -1;
wl_status_t lastWifiStatusCode = WL_IDLE_STATUS;
String lastWifiTargetSsid;
String lastWifiFailure;
String lastRadioReason;

String serialBuffer;

void sendAck();
void setError(ScpiError err);
bool parseIntStrict(const String &input, long &value);
String normalizeToken(const String &input);
int findPhysicalPinIndexByGpio(uint8_t gpio);
int findAnalogChannelByGpio(uint8_t gpio);
int findPwmChannelByPin(uint8_t gpio);
int findDacChannelByPin(uint8_t gpio);
bool resolvePhysicalPin(const String &token, int &physicalIndex);
bool resolveAnalogChannel(const String &token, int &analogChannel);
bool resolveDigitalPin(const String &token, int &physicalIndex);
bool resolvePwmChannel(const String &token, int &pwmChannel);
bool resolvePwmPin(const String &token, int &physicalIndex);
bool resolveDacChannel(const String &token, int &dacChannel);
bool setScanList(const String &cmd);
bool pinSupports(uint8_t physicalIndex, uint8_t capability);
bool pinUsesAdc2(uint8_t physicalIndex);
uint16_t rawMaxValue();
const char *modeToText(GpioMode mode);
void printCapabilities(uint8_t physicalIndex);
void onGpioModeChange(uint8_t gpioNum, GpioMode mode);
void onAnalogOperationRequested(int analogChannel, const char *operation);
void wifiBtArbiterStubRequestAdc2(const String &resourceId);
bool wifiBtArbiterStubIsAdc2Locked();
bool isAnalogChannelAccessible(int analogChannel);
float readVolt(uint8_t analogChannel);
uint16_t readRaw(uint8_t analogChannel);
uint32_t readMilliVolts(uint8_t analogChannel);
bool checkTriggerInstant();
bool waitTrigger();
void runAcquisitionEngine();
void readScan(Stream &interface);
void measAll(Stream &interface);
void resetDevice();
void processCommand(String &cmd);
void detachWaveformGeneratorsOnPin(uint8_t gpio);
bool ensurePwmAttached(uint8_t pwmChannel);
bool ensureServoAttached(uint8_t pwmChannel);
bool ensurePinPwmAttached(uint8_t physicalIndex);
uint32_t servoDutyFromAngle(uint8_t angle);
bool setWifiEnabled(bool enabled);
const char *wifiStatusText();
const char *wifiAuthModeText(wifi_auth_mode_t authMode);
const char *wifiLinkStatusText(wl_status_t status);
bool parseWifiJoinArgs(const String &input, String &ssid, String &password);

void sendAck() {
  if (ackEnabled) {
    Serial.println(F("OK"));
  }
}

void setError(ScpiError err) {
  lastError = err;
  if (ackEnabled) {
    Serial.println(F("ERR"));
  }
}

bool parseIntStrict(const String &input, long &value) {
  String tmp = input;
  tmp.trim();
  if (tmp.length() == 0) {
    return false;
  }

  uint16_t start = 0;
  if (tmp.charAt(0) == '+' || tmp.charAt(0) == '-') {
    start = 1;
  }
  if (start >= tmp.length()) {
    return false;
  }

  for (uint16_t i = start; i < tmp.length(); ++i) {
    if (!isDigit(static_cast<unsigned char>(tmp.charAt(i)))) {
      return false;
    }
  }

  value = tmp.toInt();
  return true;
}

String normalizeToken(const String &input) {
  String tmp = input;
  tmp.trim();
  tmp.toUpperCase();
  return tmp;
}

int findPhysicalPinIndexByGpio(uint8_t gpio) {
  for (uint8_t i = 0; i < kPhysicalPinCount; ++i) {
    if (kPhysicalPins[i].gpio == gpio) {
      return i;
    }
  }
  return -1;
}

int findAnalogChannelByGpio(uint8_t gpio) {
  for (uint8_t i = 0; i < kAnalogPinCount; ++i) {
    if (kAnalogPins[i] == gpio) {
      return i;
    }
  }
  return -1;
}

int findPwmChannelByPin(uint8_t gpio) {
  for (uint8_t i = 0; i < kPwmPinCount; ++i) {
    if (kPwmPins[i] == gpio) {
      return i;
    }
  }
  return -1;
}

int findDacChannelByPin(uint8_t gpio) {
  for (uint8_t i = 0; i < kDacPinCount; ++i) {
    if (kDacPins[i] == gpio) {
      return i;
    }
  }
  return -1;
}

bool pinSupports(uint8_t physicalIndex, uint8_t capability) {
  if (physicalIndex >= kPhysicalPinCount) {
    return false;
  }
  return (kPhysicalPins[physicalIndex].caps & capability) != 0;
}

bool pinUsesAdc2(uint8_t physicalIndex) {
  if (physicalIndex >= kPhysicalPinCount) {
    return false;
  }
  return kPhysicalPins[physicalIndex].adc2;
}

uint16_t rawMaxValue() {
  return (1U << adcResolutionBits) - 1U;
}

const char *modeToText(GpioMode mode) {
  switch (mode) {
    case MODE_IN: return "IN";
    case MODE_OUT: return "OUT";
    case MODE_PULLUP: return "PULLUP";
    case MODE_ANA: return "ANA";
    default: return "UNKNOWN";
  }
}

bool isModeBlockedByRadio(uint8_t physicalIndex, GpioMode mode) {
  if (physicalIndex >= kPhysicalPinCount) {
    return false;
  }
  return mode == MODE_ANA &&
         pinUsesAdc2(physicalIndex) &&
         wifiBtArbiterStubIsAdc2Locked();
}

void printModeStatus(uint8_t physicalIndex) {
  Serial.print(modeToText(pinModes[physicalIndex]));
  if (isModeBlockedByRadio(physicalIndex, pinModes[physicalIndex])) {
    Serial.print(F(",NAVAIL,RADIO"));
  }
  Serial.println();
}

void printCapabilities(uint8_t physicalIndex) {
  bool first = true;
  if (pinSupports(physicalIndex, CAP_IN)) {
    Serial.print(F("IN"));
    first = false;
  }
  if (pinSupports(physicalIndex, CAP_OUT)) {
    if (!first) Serial.print(",");
    Serial.print(F("OUT"));
    first = false;
  }
  if (pinSupports(physicalIndex, CAP_PULLUP)) {
    if (!first) Serial.print(",");
    Serial.print(F("PULLUP"));
    first = false;
  }
  if (pinSupports(physicalIndex, CAP_ANA)) {
    if (!first) Serial.print(",");
    Serial.print(F("ANA"));
  }
  Serial.println();
}

void onGpioModeChange(uint8_t gpioNum, GpioMode mode) {
  if (mode != MODE_ANA) {
    return;
  }

  int physicalIndex = findPhysicalPinIndexByGpio(gpioNum);
  if (physicalIndex >= 0 && pinUsesAdc2(static_cast<uint8_t>(physicalIndex))) {
    wifiBtArbiterStubRequestAdc2(String("GPIO") + String(gpioNum));
  }
}

void onAnalogOperationRequested(int analogChannel, const char *operation) {
  if (analogChannel < 0 || analogChannel >= kAnalogPinCount) {
    return;
  }

  int physicalIndex = findPhysicalPinIndexByGpio(kAnalogPins[analogChannel]);
  if (physicalIndex >= 0 && pinUsesAdc2(static_cast<uint8_t>(physicalIndex))) {
    wifiBtArbiterStubRequestAdc2(String(operation) + ":AN" + String(analogChannel));
  }
}

void wifiBtArbiterStubRequestAdc2(const String &resourceId) {
  lastRadioReason = resourceId;
}

bool wifiBtArbiterStubIsAdc2Locked() {
  return adc2Locked;
}

void detachWaveformGeneratorsOnPin(uint8_t gpio) {
  int physicalIndex = findPhysicalPinIndexByGpio(gpio);
  if (physicalIndex >= 0) {
    if (pinPwmAttached[physicalIndex]) {
      ledcDetach(gpio);
    }
    pinPwmAttached[physicalIndex] = false;
    pinPwmValue[physicalIndex] = 0;
  }

  int dacChannel = findDacChannelByPin(gpio);
  if (dacChannel >= 0) {
    dacWrite(gpio, 0);
    dacValue[dacChannel] = 0;
  }

  int pwmChannel = findPwmChannelByPin(gpio);
  if (pwmChannel < 0) {
    return;
  }

  if (pwmAttached[pwmChannel] || servoAttached[pwmChannel]) {
    ledcDetach(gpio);
  }
  pwmAttached[pwmChannel] = false;
  servoAttached[pwmChannel] = false;
}

bool ensurePwmAttached(uint8_t pwmChannel) {
  if (pwmChannel >= kPwmPinCount) {
    return false;
  }

  uint8_t gpio = kPwmPins[pwmChannel];
  int physicalIndex = findPhysicalPinIndexByGpio(gpio);
  if (servoAttached[pwmChannel]) {
    ledcDetach(gpio);
    servoAttached[pwmChannel] = false;
  }
  if (physicalIndex >= 0 && pinPwmAttached[physicalIndex]) {
    ledcDetach(gpio);
    pinPwmAttached[physicalIndex] = false;
    pinPwmValue[physicalIndex] = 0;
  }
  if (!pwmAttached[pwmChannel]) {
    if (!ledcAttach(gpio, PWM_FREQ_HZ, PWM_RES_BITS)) {
      return false;
    }
    pwmAttached[pwmChannel] = true;
  }
  return true;
}

bool ensurePinPwmAttached(uint8_t physicalIndex) {
  if (physicalIndex < 0 || physicalIndex >= kPhysicalPinCount) {
    return false;
  }

  uint8_t gpio = kPhysicalPins[physicalIndex].gpio;
  int legacyPwmChannel = findPwmChannelByPin(gpio);
  if (legacyPwmChannel >= 0 && servoAttached[legacyPwmChannel]) {
    ledcDetach(gpio);
    servoAttached[legacyPwmChannel] = false;
    pwmAttached[legacyPwmChannel] = false;
  }

  if (!pinPwmAttached[physicalIndex]) {
    if (!ledcAttach(gpio, pinPwmFreq[physicalIndex], PWM_RES_BITS)) {
      return false;
    }
    pinPwmAttached[physicalIndex] = true;
  }
  return true;
}

bool ensureServoAttached(uint8_t pwmChannel) {
  if (pwmChannel >= kPwmPinCount) {
    return false;
  }

  uint8_t gpio = kPwmPins[pwmChannel];
  int physicalIndex = findPhysicalPinIndexByGpio(gpio);
  if (pwmAttached[pwmChannel]) {
    ledcDetach(gpio);
    pwmAttached[pwmChannel] = false;
  }
  if (physicalIndex >= 0 && pinPwmAttached[physicalIndex]) {
    ledcDetach(gpio);
    pinPwmAttached[physicalIndex] = false;
    pinPwmValue[physicalIndex] = 0;
  }
  if (!servoAttached[pwmChannel]) {
    if (!ledcAttach(gpio, SERVO_FREQ_HZ, SERVO_RES_BITS)) {
      return false;
    }
    servoAttached[pwmChannel] = true;
  }
  return true;
}

uint32_t servoDutyFromAngle(uint8_t angle) {
  uint32_t pulseUs = SERVO_MIN_US + ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180U;
  const uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1UL;
  return (pulseUs * maxDuty) / periodUs;
}

bool setWifiEnabled(bool enabled) {
  if (enabled) {
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    if (!WiFi.mode(WIFI_STA)) {
      return false;
    }
    WiFi.disconnect(false, false);
    radioMode = RADIO_WIFI;
    adc2Locked = true;
    lastWifiStatusCode = WiFi.status();
    lastWifiFailure = "";
    return true;
  }

  WiFi.scanDelete();
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  radioMode = RADIO_OFF;
  adc2Locked = false;
  wifiScanInProgress = false;
  lastWifiStatusCode = WL_IDLE_STATUS;
  lastWifiFailure = "";
  return true;
}

const char *wifiStatusText() {
  if (radioMode == RADIO_OFF) {
    return "OFF";
  }
  if (wifiScanInProgress) {
    return "SCANNING";
  }
  if (WiFi.isConnected()) {
    return "CONNECTED";
  }
  return "IDLE";
}

const char *wifiAuthModeText(wifi_auth_mode_t authMode) {
  switch (authMode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    case WIFI_AUTH_WAPI_PSK: return "WAPI";
    default: return "UNKNOWN";
  }
}

const char *wifiLinkStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "WL_UNKNOWN";
  }
}

bool parseWifiJoinArgs(const String &input, String &ssid, String &password) {
  int comma = input.indexOf(',');
  if (comma < 0) {
    return false;
  }

  ssid = input.substring(0, comma);
  password = input.substring(comma + 1);
  ssid.trim();
  password.trim();

  if (ssid.length() == 0) {
    return false;
  }

  if (ssid.startsWith("\"") && ssid.endsWith("\"") && ssid.length() >= 2) {
    ssid = ssid.substring(1, ssid.length() - 1);
  }
  if (password.startsWith("\"") && password.endsWith("\"") && password.length() >= 2) {
    password = password.substring(1, password.length() - 1);
  }

  return ssid.length() > 0;
}

bool resolvePhysicalPin(const String &token, int &physicalIndex) {
  String tmp = normalizeToken(token);
  long value = 0;

  if (tmp.startsWith(F("GPIO"))) {
    if (!parseIntStrict(tmp.substring(4), value)) {
      return false;
    }
    physicalIndex = findPhysicalPinIndexByGpio((uint8_t)value);
    return physicalIndex >= 0;
  }

  if (tmp.startsWith(F("DCH"))) {
    if (!parseIntStrict(tmp.substring(3), value)) {
      return false;
    }
    if (value < 0 || value >= kDigitalPinCount) {
      return false;
    }
    physicalIndex = findPhysicalPinIndexByGpio(kDigitalPins[value]);
    return physicalIndex >= 0;
  }

  if (tmp.startsWith(F("AN"))) {
    if (!parseIntStrict(tmp.substring(2), value)) {
      return false;
    }
    if (value < 0 || value >= kAnalogPinCount) {
      return false;
    }
    physicalIndex = findPhysicalPinIndexByGpio(kAnalogPins[value]);
    return physicalIndex >= 0;
  }

  if (tmp.startsWith(F("D"))) {
    if (!parseIntStrict(tmp.substring(1), value)) {
      return false;
    }
    if (value < 2 || value > 13) {
      return false;
    }
    physicalIndex = findPhysicalPinIndexByGpio(kDigitalPins[value - 2]);
    return physicalIndex >= 0;
  }

  if (tmp.startsWith(F("A"))) {
    if (!parseIntStrict(tmp.substring(1), value)) {
      return false;
    }
    if (value < 0 || value > 5) {
      return false;
    }
    physicalIndex = findPhysicalPinIndexByGpio(kAnalogPins[value]);
    return physicalIndex >= 0;
  }

  if (!parseIntStrict(tmp, value)) {
    return false;
  }
  if (value < 0 || value >= kDigitalPinCount) {
    return false;
  }
  physicalIndex = findPhysicalPinIndexByGpio(kDigitalPins[value]);
  return physicalIndex >= 0;
}

bool resolveAnalogChannel(const String &token, int &analogChannel) {
  String tmp = normalizeToken(token);
  long value = 0;
  int physicalIndex = -1;

  if (tmp.startsWith(F("AN"))) {
    if (!parseIntStrict(tmp.substring(2), value)) {
      return false;
    }
    if (value < 0 || value >= kAnalogPinCount) {
      return false;
    }
    analogChannel = (int)value;
    return true;
  }

  if (tmp.startsWith(F("GPIO"))) {
    if (!parseIntStrict(tmp.substring(4), value)) {
      return false;
    }
    analogChannel = findAnalogChannelByGpio((uint8_t)value);
    return analogChannel >= 0;
  }

  if (tmp.startsWith(F("DCH"))) {
    if (!parseIntStrict(tmp.substring(3), value)) {
      return false;
    }
    if (value < 0 || value >= kDigitalPinCount) {
      return false;
    }
    analogChannel = findAnalogChannelByGpio(kDigitalPins[value]);
    return analogChannel >= 0;
  }

  if (tmp.startsWith(F("A"))) {
    if (!parseIntStrict(tmp.substring(1), value)) {
      return false;
    }
    if (value < 0 || value > 5) {
      return false;
    }
    analogChannel = (int)value;
    return true;
  }

  if (tmp.startsWith(F("D"))) {
    if (!parseIntStrict(tmp.substring(1), value)) {
      return false;
    }
    if (value < 2 || value > 13) {
      return false;
    }
    analogChannel = findAnalogChannelByGpio(kDigitalPins[value - 2]);
    return analogChannel >= 0;
  }

  if (!parseIntStrict(tmp, value)) {
    return false;
  }
  if (value < 0 || value >= kAnalogPinCount) {
    return false;
  }
  physicalIndex = findPhysicalPinIndexByGpio(kAnalogPins[value]);
  if (physicalIndex < 0 || !pinSupports((uint8_t)physicalIndex, CAP_ANA)) {
    return false;
  }
  analogChannel = (int)value;
  return true;
}

bool resolveDigitalPin(const String &token, int &physicalIndex) {
  if (!resolvePhysicalPin(token, physicalIndex)) {
    return false;
  }
  return pinSupports((uint8_t)physicalIndex, CAP_IN) || pinSupports((uint8_t)physicalIndex, CAP_OUT);
}

bool resolvePwmChannel(const String &token, int &pwmChannel) {
  String tmp = normalizeToken(token);
  long value = 0;
  if (!parseIntStrict(tmp, value)) {
    return false;
  }
  if (value < 0 || value >= kPwmPinCount) {
    return false;
  }
  pwmChannel = (int)value;
  return true;
}

bool resolvePwmPin(const String &token, int &physicalIndex) {
  if (resolvePhysicalPin(token, physicalIndex)) {
    return pinSupports((uint8_t)physicalIndex, CAP_OUT);
  }

  int pwmChannel = -1;
  if (!resolvePwmChannel(token, pwmChannel)) {
    return false;
  }

  physicalIndex = findPhysicalPinIndexByGpio(kPwmPins[pwmChannel]);
  return physicalIndex >= 0;
}

bool resolveDacChannel(const String &token, int &dacChannel) {
  String tmp = normalizeToken(token);
  long value = 0;

  if (tmp == F("DAC1")) {
    dacChannel = 0;
    return true;
  }
  if (tmp == F("DAC2")) {
    dacChannel = 1;
    return true;
  }
  if (tmp.startsWith(F("GPIO"))) {
    if (!parseIntStrict(tmp.substring(4), value)) {
      return false;
    }
    dacChannel = findDacChannelByPin((uint8_t)value);
    return dacChannel >= 0;
  }
  if (tmp.startsWith(F("DCH"))) {
    if (!parseIntStrict(tmp.substring(3), value) || value < 0 || value >= kDigitalPinCount) {
      return false;
    }
    dacChannel = findDacChannelByPin(kDigitalPins[value]);
    return dacChannel >= 0;
  }
  if (tmp.startsWith(F("D"))) {
    if (!parseIntStrict(tmp.substring(1), value) || value < 2 || value > 13) {
      return false;
    }
    dacChannel = findDacChannelByPin(kDigitalPins[value - 2]);
    return dacChannel >= 0;
  }
  if (!parseIntStrict(tmp, value)) {
    return false;
  }
  if (value < 0 || value >= kDacPinCount) {
    return false;
  }
  dacChannel = (int)value;
  return true;
}

bool isAnalogChannelAccessible(int analogChannel) {
  if (analogChannel < 0 || analogChannel >= kAnalogPinCount) {
    setError(ERR_PARAM_RANGE);
    return false;
  }

  int physicalIndex = findPhysicalPinIndexByGpio(kAnalogPins[analogChannel]);
  if (physicalIndex < 0) {
    setError(ERR_EXECUTION);
    return false;
  }

  if (!pinSupports((uint8_t)physicalIndex, CAP_ANA)) {
    setError(ERR_MODE);
    return false;
  }

  if (pinModes[physicalIndex] != MODE_ANA) {
    setError(ERR_MODE);
    return false;
  }

  onAnalogOperationRequested(analogChannel, "ANA");
  if (pinUsesAdc2((uint8_t)physicalIndex) && wifiBtArbiterStubIsAdc2Locked()) {
    setError(ERR_MODE);
    return false;
  }

  return true;
}

uint16_t readRaw(uint8_t analogChannel) {
  return analogRead(kAnalogPins[analogChannel]);
}

uint32_t readMilliVolts(uint8_t analogChannel) {
  return analogReadMilliVolts(kAnalogPins[analogChannel]);
}

float readVolt(uint8_t analogChannel) {
  uint16_t raw = readRaw(analogChannel);
  return ((float)raw * vRef) / (float)rawMaxValue();
}

bool setScanList(const String &cmd) {
  scanCount = 0;

  int start = cmd.indexOf('@');
  int end = cmd.indexOf(')');
  if (start < 0 || end < 0 || end <= start) {
    return false;
  }

  String list = cmd.substring(start + 1, end);
  int pos = 0;

  while (pos < list.length()) {
    int comma = list.indexOf(',', pos);
    if (comma < 0) {
      comma = list.length();
    }

    String item = list.substring(pos, comma);
    item.trim();
    if (item.length() == 0) {
      return false;
    }

    int colon = item.indexOf(':');
    if (colon >= 0) {
      int startChannel = -1;
      int endChannel = -1;
      if (!resolveAnalogChannel(item.substring(0, colon), startChannel) ||
          !resolveAnalogChannel(item.substring(colon + 1), endChannel) ||
          startChannel > endChannel) {
        return false;
      }
      for (int ch = startChannel; ch <= endChannel; ++ch) {
        if (scanCount >= kAnalogPinCount) {
          return false;
        }
        scanList[scanCount++] = (uint8_t)ch;
      }
    } else {
      int channel = -1;
      if (!resolveAnalogChannel(item, channel) || scanCount >= kAnalogPinCount) {
        return false;
      }
      scanList[scanCount++] = (uint8_t)channel;
    }

    pos = comma + 1;
  }

  return scanCount > 0;
}

bool checkTriggerInstant() {
  if (trigMode == TRIG_IMM) {
    return true;
  }

  if (trigMode == TRIG_ANA) {
    if (!isAnalogChannelAccessible(trigAnalogChannel)) {
      return false;
    }
    float value = readVolt((uint8_t)trigAnalogChannel);
    bool triggered = false;
    if (trigSlope == SLOPE_POS && lastTrigVolt < trigLevel && value >= trigLevel) {
      triggered = true;
    }
    if (trigSlope == SLOPE_NEG && lastTrigVolt > trigLevel && value <= trigLevel) {
      triggered = true;
    }
    lastTrigVolt = value;
    return triggered;
  }

  int physicalIndex = trigDigitalPinIndex;
  if (physicalIndex < 0 || physicalIndex >= kPhysicalPinCount) {
    setError(ERR_EXECUTION);
    return false;
  }
  if (!(pinModes[physicalIndex] == MODE_IN || pinModes[physicalIndex] == MODE_PULLUP)) {
    setError(ERR_MODE);
    return false;
  }

  int state = digitalRead(kPhysicalPins[physicalIndex].gpio);
  bool triggered = false;
  if (trigSlope == SLOPE_POS && lastTrigDig == LOW && state == HIGH) {
    triggered = true;
  }
  if (trigSlope == SLOPE_NEG && lastTrigDig == HIGH && state == LOW) {
    triggered = true;
  }
  lastTrigDig = state;
  return triggered;
}

bool waitTrigger() {
  if (trigMode == TRIG_IMM) {
    return true;
  }

  ScpiError errorSnapshot = lastError;

  if (trigMode == TRIG_ANA) {
    if (!isAnalogChannelAccessible(trigAnalogChannel)) {
      return false;
    }
    lastTrigVolt = readVolt((uint8_t)trigAnalogChannel);
  } else {
    if (trigDigitalPinIndex < 0 || trigDigitalPinIndex >= kPhysicalPinCount) {
      setError(ERR_EXECUTION);
      return false;
    }
    if (!(pinModes[trigDigitalPinIndex] == MODE_IN || pinModes[trigDigitalPinIndex] == MODE_PULLUP)) {
      setError(ERR_MODE);
      return false;
    }
    lastTrigDig = digitalRead(kPhysicalPins[trigDigitalPinIndex].gpio);
  }

  unsigned long startMillis = millis();
  while (millis() - startMillis < trigTimeout) {
    if (checkTriggerInstant()) {
      return true;
    }
    if (lastError != errorSnapshot) {
      return false;
    }
  }

  setError(ERR_TIMEOUT);
  return false;
}

void measAll(Stream &interface) {
  for (uint8_t i = 0; i < kAnalogPinCount; ++i) {
    if (!isAnalogChannelAccessible(i)) {
      return;
    }
    interface.print(readVolt(i), 4);
    if (i + 1 < kAnalogPinCount) {
      interface.print(",");
    }
  }
  interface.println();
}

void readScan(Stream &interface) {
  if (!waitTrigger()) {
    return;
  }

  for (uint8_t i = 0; i < scanCount; ++i) {
    if (!isAnalogChannelAccessible(scanList[i])) {
      return;
    }
    interface.print(readVolt(scanList[i]), 4);
    if (i + 1 < scanCount) {
      interface.print(",");
    }
  }
  interface.println();
}

void runAcquisitionEngine() {
  if (acqState == ACQ_IDLE || acqState == ACQ_DONE) {
    return;
  }

  unsigned long now = micros();
  if (now - lastSampleTime < acqTStep) {
    return;
  }
  lastSampleTime = now;

  for (uint8_t i = 0; i < scanCount; ++i) {
    if (!isAnalogChannelAccessible(scanList[i])) {
      acqState = ACQ_IDLE;
      return;
    }
    acqBuffer[(acqHead * scanCount) + i] = readRaw(scanList[i]);
  }

  if (acqState == ACQ_PREFILL) {
    ++prefillCount;
    acqHead = (acqHead + 1) % acqPoints;
    if (prefillCount >= acqPoints / 2) {
      acqState = ACQ_ARMED;
      if (trigMode == TRIG_ANA) {
        if (!isAnalogChannelAccessible(trigAnalogChannel)) {
          acqState = ACQ_IDLE;
          return;
        }
        lastTrigVolt = readVolt((uint8_t)trigAnalogChannel);
      } else if (trigMode == TRIG_DIG) {
        lastTrigDig = digitalRead(kPhysicalPins[trigDigitalPinIndex].gpio);
      }
    }
    return;
  }

  if (acqState == ACQ_ARMED) {
    bool triggered = checkTriggerInstant();
    if (lastError != ERR_NONE) {
      acqState = ACQ_IDLE;
      return;
    }
    acqHead = (acqHead + 1) % acqPoints;
    if (triggered) {
      postCount = 0;
      acqState = ACQ_POST;
    }
    return;
  }

  if (acqState == ACQ_POST) {
    ++postCount;
    acqHead = (acqHead + 1) % acqPoints;
    if (postCount >= acqPoints / 2) {
      acqState = ACQ_DONE;
    }
  }
}

void resetDevice() {
  refMode = REF_DEF;
  vRef = 3.3f;
  adcResolutionBits = 12;
  analogReadResolution(adcResolutionBits);

  lastError = ERR_NONE;
  ackEnabled = true;
  setWifiEnabled(false);
  lastWifiScanCount = -1;
  lastWifiTargetSsid = "";
  lastWifiFailure = "";
  lastRadioReason = "";

  currentAnalogChannel = 0;
  scanCount = 0;

  trigMode = TRIG_IMM;
  trigSlope = SLOPE_POS;
  trigLevel = 1.65f;
  trigTimeout = 1000;
  trigAnalogChannel = 0;
  trigDigitalPinIndex = findPhysicalPinIndexByGpio(kDigitalPins[0]);
  lastTrigVolt = 0.0f;
  lastTrigDig = LOW;

  acqState = ACQ_IDLE;
  acqPoints = 64;
  acqHead = 0;
  prefillCount = 0;
  postCount = 0;
  acqTStep = 1000;
  lastSampleTime = 0;

  for (uint8_t i = 0; i < kPwmPinCount; ++i) {
    pwmValue[i] = 0;
    servoAngle[i] = 0;
    pwmAttached[i] = false;
    servoAttached[i] = false;
    ledcDetach(kPwmPins[i]);
  }

  for (uint8_t i = 0; i < kDacPinCount; ++i) {
    dacValue[i] = 0;
    dacWrite(kDacPins[i], 0);
  }

  for (uint8_t i = 0; i < kPhysicalPinCount; ++i) {
    pinDigitalStates[i] = LOW;
    pinPwmValue[i] = 0;
    pinPwmFreq[i] = PWM_FREQ_HZ;
    pinPwmAttached[i] = false;
    if (pinSupports(i, CAP_ANA)) {
      pinModes[i] = MODE_ANA;
      pinMode(kPhysicalPins[i].gpio, INPUT);
      onGpioModeChange(kPhysicalPins[i].gpio, MODE_ANA);
    } else if (pinSupports(i, CAP_OUT)) {
      pinModes[i] = MODE_OUT;
      pinMode(kPhysicalPins[i].gpio, OUTPUT);
      digitalWrite(kPhysicalPins[i].gpio, LOW);
    } else {
      pinModes[i] = MODE_IN;
      pinMode(kPhysicalPins[i].gpio, INPUT);
    }
  }
}

void processCommand(String &cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == F("*IDN?")) {
    Serial.println(F("OpenSCPI-Lab,WEMOS-D1-R32,0.1-ALPHA"));
    return;
  }
  if (cmd == F("*RST")) {
    resetDevice();
    sendAck();
    return;
  }
  if (cmd == F("*OPC?")) {
    Serial.println((acqState == ACQ_IDLE || acqState == ACQ_DONE) ? F("1") : F("0"));
    return;
  }
  if (cmd == F("*CLS")) {
    lastError = ERR_NONE;
    sendAck();
    return;
  }

  if (cmd == F("SYST:ERR?")) {
    switch (lastError) {
      case ERR_NONE: Serial.println(F("0,\"No error\"")); break;
      case ERR_CMD_UNKNOWN: Serial.println(F("-100,\"Command error\"")); break;
      case ERR_PARAM_RANGE: Serial.println(F("-222,\"Data out of range\"")); break;
      case ERR_EXECUTION: Serial.println(F("-200,\"Execution error\"")); break;
      case ERR_TIMEOUT: Serial.println(F("-250,\"Timeout error\"")); break;
      case ERR_MODE: Serial.println(F("-221,\"Settings conflict\"")); break;
      default: Serial.println(F("-300,\"Device-specific error\"")); break;
    }
    lastError = ERR_NONE;
    return;
  }

  if (cmd.startsWith(F("SYST:ACK "))) {
    String token = normalizeToken(cmd.substring(9));
    if (token == F("ON") || token == F("1")) {
      ackEnabled = true;
      sendAck();
      return;
    }
    if (token == F("OFF") || token == F("0")) {
      ackEnabled = false;
      sendAck();
      return;
    }
    setError(ERR_PARAM_RANGE);
    return;
  }
  if (cmd == F("SYST:ACK?")) {
    Serial.println(ackEnabled ? F("1") : F("0"));
    return;
  }

  if (cmd == F("SYST:CAP?")) {
    Serial.println(F("ESP32,GPIO_MODE,GPIO_CAP,ADC12,ADC_MV,PWM_LEDC_ANY,PWM_FREQ,DAC2,SERVO_LEDC,TRIG,ACQ,ADC2_GUARD,WIFI_CTRL,WIFI_SCAN"));
    return;
  }

  if (cmd == F("SYST:PINMAP?")) {
    Serial.println(F("DCH0=GPIO26,DCH1=GPIO25,DCH2=GPIO17,DCH3=GPIO16,DCH4=GPIO27,DCH5=GPIO14,DCH6=GPIO12,DCH7=GPIO13,DCH8=GPIO5,DCH9=GPIO23,DCH10=GPIO19,DCH11=GPIO18;AN0=GPIO2,AN1=GPIO4,AN2=GPIO35,AN3=GPIO34,AN4=GPIO36,AN5=GPIO39,AN6=GPIO26,AN7=GPIO25,AN8=GPIO27,AN9=GPIO14,AN10=GPIO12,AN11=GPIO13"));
    return;
  }

  if (cmd == F("SYST:WIFI:ON")) {
    if (!setWifiEnabled(true)) {
      setError(ERR_EXECUTION);
      return;
    }
    sendAck();
    return;
  }

  if (cmd == F("SYST:WIFI:OFF")) {
    if (!setWifiEnabled(false)) {
      setError(ERR_EXECUTION);
      return;
    }
    sendAck();
    return;
  }

  if (cmd == F("SYST:WIFI:STAT?")) {
    Serial.println(wifiStatusText());
    return;
  }

  if (cmd == F("SYST:WIFI:SCAN?")) {
    if (radioMode == RADIO_OFF) {
      setError(ERR_MODE);
      return;
    }

    wifiScanInProgress = true;
    int networkCount = WiFi.scanNetworks();
    wifiScanInProgress = false;
    lastWifiScanCount = networkCount;
    lastWifiStatusCode = WiFi.status();

    if (networkCount < 0) {
      lastWifiFailure = "SCAN_FAILED";
      setError(ERR_EXECUTION);
      return;
    }

    lastWifiFailure = "";
    Serial.println(F("SSID,RSSI,AUTH,CHAN"));
    for (int i = 0; i < networkCount; ++i) {
      Serial.print(WiFi.SSID(i));
      Serial.print(",");
      Serial.print(WiFi.RSSI(i));
      Serial.print(",");
      Serial.print(wifiAuthModeText(WiFi.encryptionType(i)));
      Serial.print(",");
      Serial.println(WiFi.channel(i));
    }
    WiFi.scanDelete();
    return;
  }

  if (cmd.startsWith(F("SYST:WIFI:JOIN "))) {
    String ssid;
    String password;
    if (!parseWifiJoinArgs(cmd.substring(15), ssid, password)) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    lastWifiTargetSsid = ssid;
    lastWifiFailure = "";

    if (radioMode == RADIO_OFF && !setWifiEnabled(true)) {
      lastWifiFailure = "RADIO_ENABLE_FAILED";
      setError(ERR_EXECUTION);
      return;
    }

    WiFi.begin(ssid.c_str(), password.c_str());
    unsigned long start = millis();
    while (millis() - start < 15000) {
      wl_status_t status = WiFi.status();
      lastWifiStatusCode = status;
      if (status == WL_CONNECTED) {
        radioMode = RADIO_WIFI;
        adc2Locked = true;
        lastWifiFailure = "";
        sendAck();
        return;
      }
      if (status == WL_CONNECT_FAILED) {
        lastWifiFailure = "CONNECT_FAILED";
        setError(ERR_EXECUTION);
        return;
      }
      if (status == WL_NO_SSID_AVAIL) {
        lastWifiFailure = "NO_SSID";
        setError(ERR_EXECUTION);
        return;
      }
      if (status == WL_CONNECTION_LOST) {
        lastWifiFailure = "CONNECTION_LOST";
        setError(ERR_EXECUTION);
        return;
      }
      delay(200);
    }
    lastWifiStatusCode = WiFi.status();
    lastWifiFailure = "JOIN_TIMEOUT";
    setError(ERR_TIMEOUT);
    return;
  }

  if (cmd == F("SYST:WIFI:DISC")) {
    if (radioMode == RADIO_OFF) {
      sendAck();
      return;
    }
    WiFi.disconnect(false, false);
    lastWifiStatusCode = WiFi.status();
    lastWifiFailure = "";
    sendAck();
    return;
  }

  if (cmd == F("SYST:WIFI:IP?")) {
    if (radioMode == RADIO_OFF || !WiFi.isConnected()) {
      Serial.println(F("NONE"));
      return;
    }
    Serial.println(WiFi.localIP());
    return;
  }

  if (cmd == F("SYST:WIFI:RSSI?")) {
    if (radioMode == RADIO_OFF || !WiFi.isConnected()) {
      Serial.println(F("NONE"));
      return;
    }
    Serial.println(WiFi.RSSI());
    return;
  }

  if (cmd == F("SYST:WIFI:DBG:STAT?")) {
    Serial.print(F("RADIO="));
    Serial.print(radioMode == RADIO_OFF ? F("OFF") : F("ON"));
    Serial.print(F(",STAT="));
    Serial.print(wifiStatusText());
    Serial.print(F(",WIFI_STATUS="));
    Serial.print(wifiLinkStatusText(lastWifiStatusCode));
    Serial.print(F(",CODE="));
    Serial.println((int)lastWifiStatusCode);
    return;
  }

  if (cmd == F("SYST:WIFI:DBG:SCAN:LAST?")) {
    Serial.println(lastWifiScanCount);
    return;
  }

  if (cmd == F("SYST:WIFI:DBG:SSID?")) {
    if (lastWifiTargetSsid.length() == 0) {
      Serial.println(F("NONE"));
    } else {
      Serial.println(lastWifiTargetSsid);
    }
    return;
  }

  if (cmd == F("SYST:WIFI:DBG:FAIL?")) {
    if (lastWifiFailure.length() == 0) {
      Serial.println(F("NONE"));
    } else {
      Serial.println(lastWifiFailure);
    }
    return;
  }

  if (cmd == F("SYST:WIFI:DBG:DIAG?")) {
    Serial.print(F("RADIO="));
    Serial.print(radioMode == RADIO_OFF ? F("OFF") : F("ON"));
    Serial.print(F(",STAT="));
    Serial.print(wifiStatusText());
    Serial.print(F(",WIFI_STATUS="));
    Serial.print(wifiLinkStatusText(lastWifiStatusCode));
    Serial.print(F(",TARGET="));
    if (lastWifiTargetSsid.length() == 0) {
      Serial.print(F("NONE"));
    } else {
      Serial.print(lastWifiTargetSsid);
    }
    Serial.print(F(",IP="));
    if (WiFi.isConnected()) Serial.print(WiFi.localIP());
    else Serial.print(F("NONE"));
    Serial.print(F(",RSSI="));
    if (WiFi.isConnected()) Serial.print(WiFi.RSSI());
    else Serial.print(F("NONE"));
    Serial.print(F(",SCAN_LAST="));
    Serial.print(lastWifiScanCount);
    Serial.print(F(",LAST_FAIL="));
    if (lastWifiFailure.length() == 0) {
      Serial.println(F("NONE"));
    } else {
      Serial.println(lastWifiFailure);
    }
    return;
  }

  if (cmd.startsWith(F("CAL:REF "))) {
    String token = normalizeToken(cmd.substring(8));
    if (token == F("DEF")) {
      refMode = REF_DEF;
      vRef = 3.3f;
      sendAck();
      return;
    }
    if (token == F("INT")) {
      refMode = REF_INT;
      vRef = 1.1f;
      sendAck();
      return;
    }
    if (token == F("EXT")) {
      refMode = REF_EXT;
      sendAck();
      return;
    }
    setError(ERR_PARAM_RANGE);
    return;
  }
  if (cmd == F("CAL:REF?")) {
    if (refMode == REF_DEF) Serial.println(F("DEF"));
    else if (refMode == REF_INT) Serial.println(F("INT"));
    else Serial.println(F("EXT"));
    return;
  }

  if (cmd.startsWith(F("CAL:VREF "))) {
    float value = cmd.substring(9).toFloat();
    if (value >= 0.5f && value <= 3.6f) {
      vRef = value;
      sendAck();
      return;
    }
    setError(ERR_PARAM_RANGE);
    return;
  }
  if (cmd == F("CAL:VREF?")) {
    Serial.println(vRef, 3);
    return;
  }

  if (cmd.startsWith(F("CONF:ADC:RES "))) {
    long bits = 0;
    if (!parseIntStrict(cmd.substring(13), bits) || bits < 9 || bits > 12) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    adcResolutionBits = (uint8_t)bits;
    analogReadResolution(adcResolutionBits);
    sendAck();
    return;
  }
  if (cmd == F("CONF:ADC:RES?")) {
    Serial.println(adcResolutionBits);
    return;
  }

  if (cmd.startsWith(F("CONF:VOLT "))) {
    int analogChannel = -1;
    if (!resolveAnalogChannel(cmd.substring(10), analogChannel)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    currentAnalogChannel = (uint8_t)analogChannel;
    sendAck();
    return;
  }
  if (cmd == F("CONF:VOLT?")) {
    Serial.print(F("AN"));
    Serial.println(currentAnalogChannel);
    return;
  }

  if (cmd.startsWith(F("MEAS:RAW? "))) {
    int analogChannel = -1;
    if (!resolveAnalogChannel(cmd.substring(10), analogChannel)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (!isAnalogChannelAccessible(analogChannel)) {
      return;
    }
    Serial.println(readRaw((uint8_t)analogChannel));
    return;
  }

  if (cmd.startsWith(F("MEAS:MVOLT?"))) {
    int analogChannel = currentAnalogChannel;
    if (cmd.length() > 11) {
      if (!resolveAnalogChannel(cmd.substring(11), analogChannel)) {
        setError(ERR_PARAM_RANGE);
        return;
      }
    }
    if (!isAnalogChannelAccessible(analogChannel)) {
      return;
    }
    Serial.println(readMilliVolts((uint8_t)analogChannel));
    return;
  }

  if (cmd.startsWith(F("MEAS:VOLT?"))) {
    int analogChannel = currentAnalogChannel;
    if (cmd.length() > 10) {
      if (!resolveAnalogChannel(cmd.substring(10), analogChannel)) {
        setError(ERR_PARAM_RANGE);
        return;
      }
    }
    if (!isAnalogChannelAccessible(analogChannel)) {
      return;
    }
    Serial.println(readVolt((uint8_t)analogChannel), 4);
    return;
  }

  if (cmd == F("MEAS:VOLT:ALL?")) {
    measAll(Serial);
    return;
  }

  if (cmd.startsWith(F("ROUT:SCAN "))) {
    if (setScanList(cmd)) {
      sendAck();
    } else {
      setError(ERR_PARAM_RANGE);
    }
    return;
  }
  if (cmd == F("ROUT:SCAN?")) {
    for (uint8_t i = 0; i < scanCount; ++i) {
      Serial.print(F("AN"));
      Serial.print(scanList[i]);
      if (i + 1 < scanCount) {
        Serial.print(",");
      }
    }
    Serial.println();
    return;
  }

  if (cmd == F("READ?")) {
    if (scanCount == 0) {
      setError(ERR_EXECUTION);
      return;
    }
    for (uint8_t i = 0; i < scanCount; ++i) {
      if (!isAnalogChannelAccessible(scanList[i])) {
        return;
      }
    }
    if (trigMode == TRIG_ANA && !isAnalogChannelAccessible(trigAnalogChannel)) {
      return;
    }
    readScan(Serial);
    return;
  }

  if (cmd.startsWith(F("GPIO:CAP? ")) || cmd.startsWith(F("GPIO:CAP?"))) {
    int physicalIndex = -1;
    if (!resolvePhysicalPin(cmd.substring(10), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    printCapabilities((uint8_t)physicalIndex);
    return;
  }

  if (cmd.startsWith(F("GPIO:MODE ")) || cmd.startsWith(F("DIG:MODE "))) {
    int prefixLen = cmd.startsWith(F("GPIO:MODE ")) ? 10 : 9;
    int comma = cmd.indexOf(',');
    int physicalIndex = -1;
    if (comma < 0 || !resolvePhysicalPin(cmd.substring(prefixLen, comma), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    String modeToken = normalizeToken(cmd.substring(comma + 1));
    GpioMode newMode;
    if (modeToken == F("IN")) newMode = MODE_IN;
    else if (modeToken == F("OUT")) newMode = MODE_OUT;
    else if (modeToken == F("PULLUP")) newMode = MODE_PULLUP;
    else if (modeToken == F("ANA")) newMode = MODE_ANA;
    else {
      setError(ERR_PARAM_RANGE);
      return;
    }

    uint8_t caps = kPhysicalPins[physicalIndex].caps;
    bool allowed = false;
    if (newMode == MODE_IN) allowed = (caps & CAP_IN) != 0;
    if (newMode == MODE_OUT) allowed = (caps & CAP_OUT) != 0;
    if (newMode == MODE_PULLUP) allowed = (caps & CAP_PULLUP) != 0;
    if (newMode == MODE_ANA) allowed = (caps & CAP_ANA) != 0;
    if (!allowed) {
      setError(ERR_MODE);
      return;
    }
    if (isModeBlockedByRadio((uint8_t)physicalIndex, newMode)) {
      setError(ERR_MODE);
      return;
    }

    detachWaveformGeneratorsOnPin(kPhysicalPins[physicalIndex].gpio);
    if (newMode == MODE_IN) pinMode(kPhysicalPins[physicalIndex].gpio, INPUT);
    else if (newMode == MODE_OUT) {
      pinMode(kPhysicalPins[physicalIndex].gpio, OUTPUT);
      digitalWrite(kPhysicalPins[physicalIndex].gpio, pinDigitalStates[physicalIndex] ? HIGH : LOW);
    }
    else if (newMode == MODE_PULLUP) pinMode(kPhysicalPins[physicalIndex].gpio, INPUT_PULLUP);
    else pinMode(kPhysicalPins[physicalIndex].gpio, INPUT);

    pinModes[physicalIndex] = newMode;
    onGpioModeChange(kPhysicalPins[physicalIndex].gpio, newMode);
    sendAck();
    return;
  }

  if (cmd.startsWith(F("GPIO:MODE? ")) || cmd.startsWith(F("DIG:MODE? "))) {
    int prefixLen = cmd.startsWith(F("GPIO:MODE? ")) ? 11 : 10;
    int physicalIndex = -1;
    if (!resolvePhysicalPin(cmd.substring(prefixLen), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    printModeStatus((uint8_t)physicalIndex);
    return;
  }

  if (cmd.startsWith(F("DIG:IN? "))) {
    int physicalIndex = -1;
    if (!resolveDigitalPin(cmd.substring(8), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (!(pinModes[physicalIndex] == MODE_IN || pinModes[physicalIndex] == MODE_PULLUP)) {
      setError(ERR_MODE);
      return;
    }
    Serial.println(digitalRead(kPhysicalPins[physicalIndex].gpio));
    return;
  }

  if (cmd.startsWith(F("DIG:OUT "))) {
    int comma = cmd.indexOf(',');
    int physicalIndex = -1;
    long value = 0;
    if (comma < 0 ||
        !resolveDigitalPin(cmd.substring(8, comma), physicalIndex) ||
        !parseIntStrict(cmd.substring(comma + 1), value)) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    if (value != 0 && value != 1) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (pinModes[physicalIndex] != MODE_OUT || !pinSupports((uint8_t)physicalIndex, CAP_OUT)) {
      setError(ERR_MODE);
      return;
    }

    detachWaveformGeneratorsOnPin(kPhysicalPins[physicalIndex].gpio);
    pinDigitalStates[physicalIndex] = (value == 1);
    digitalWrite(kPhysicalPins[physicalIndex].gpio, value ? HIGH : LOW);
    sendAck();
    return;
  }

  if (cmd.startsWith(F("DIG:OUT? "))) {
    int physicalIndex = -1;
    if (!resolveDigitalPin(cmd.substring(9), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (pinModes[physicalIndex] != MODE_OUT || !pinSupports((uint8_t)physicalIndex, CAP_OUT)) {
      setError(ERR_MODE);
      return;
    }
    Serial.println(pinDigitalStates[physicalIndex] ? F("1") : F("0"));
    return;
  }

  if (cmd.startsWith(F("SOUR:PWM "))) {
    int comma = cmd.indexOf(',');
    int physicalIndex = -1;
    long value = 0;
    if (comma < 0 ||
        !resolvePwmPin(cmd.substring(9, comma), physicalIndex) ||
        !parseIntStrict(cmd.substring(comma + 1), value)) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    if (value < 0 || value > 255) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    if (physicalIndex < 0 || pinModes[physicalIndex] != MODE_OUT || !pinSupports((uint8_t)physicalIndex, CAP_OUT)) {
      setError(ERR_MODE);
      return;
    }

    if (!ensurePinPwmAttached((uint8_t)physicalIndex)) {
      setError(ERR_EXECUTION);
      return;
    }

    pinPwmValue[physicalIndex] = (uint8_t)value;
    ledcWrite(kPhysicalPins[physicalIndex].gpio, (uint32_t)value);

    int legacyPwmChannel = findPwmChannelByPin(kPhysicalPins[physicalIndex].gpio);
    if (legacyPwmChannel >= 0) {
      pwmValue[legacyPwmChannel] = (uint8_t)value;
      pwmAttached[legacyPwmChannel] = true;
    }
    sendAck();
    return;
  }

  if (cmd.startsWith(F("SOUR:PWM? "))) {
    int physicalIndex = -1;
    if (!resolvePwmPin(cmd.substring(10), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    Serial.println(pinPwmValue[physicalIndex]);
    return;
  }

  if (cmd.startsWith(F("SOUR:PWM:FREQ "))) {
    int comma = cmd.indexOf(',');
    int physicalIndex = -1;
    long value = 0;
    if (comma < 0 ||
        !resolvePwmPin(cmd.substring(14, comma), physicalIndex) ||
        !parseIntStrict(cmd.substring(comma + 1), value)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (value <= 0 || value > 40000000L) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (pinModes[physicalIndex] != MODE_OUT || !pinSupports((uint8_t)physicalIndex, CAP_OUT)) {
      setError(ERR_MODE);
      return;
    }

    bool wasAttached = pinPwmAttached[physicalIndex];
    if (wasAttached) {
      ledcDetach(kPhysicalPins[physicalIndex].gpio);
      pinPwmAttached[physicalIndex] = false;
    }
    pinPwmFreq[physicalIndex] = (uint32_t)value;
    if (wasAttached && !ensurePinPwmAttached((uint8_t)physicalIndex)) {
      setError(ERR_EXECUTION);
      return;
    }
    if (wasAttached) {
      ledcWrite(kPhysicalPins[physicalIndex].gpio, (uint32_t)pinPwmValue[physicalIndex]);
    }
    sendAck();
    return;
  }

  if (cmd.startsWith(F("SOUR:PWM:FREQ? "))) {
    int physicalIndex = -1;
    if (!resolvePwmPin(cmd.substring(15), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    Serial.println(pinPwmFreq[physicalIndex]);
    return;
  }

  if (cmd.startsWith(F("SOUR:DAC "))) {
    int comma = cmd.indexOf(',');
    int dacChannel = -1;
    long value = 0;
    if (comma < 0 ||
        !resolveDacChannel(cmd.substring(9, comma), dacChannel) ||
        !parseIntStrict(cmd.substring(comma + 1), value)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    if (value < 0 || value > 255) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    int physicalIndex = findPhysicalPinIndexByGpio(kDacPins[dacChannel]);
    if (physicalIndex < 0 || !pinSupports((uint8_t)physicalIndex, CAP_OUT)) {
      setError(ERR_MODE);
      return;
    }

    detachWaveformGeneratorsOnPin(kDacPins[dacChannel]);
    pinModes[physicalIndex] = MODE_OUT;
    pinMode(kDacPins[dacChannel], OUTPUT);
    dacValue[dacChannel] = (uint8_t)value;
    dacWrite(kDacPins[dacChannel], (uint8_t)value);
    sendAck();
    return;
  }

  if (cmd.startsWith(F("SOUR:DAC? "))) {
    int dacChannel = -1;
    if (!resolveDacChannel(cmd.substring(10), dacChannel)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    Serial.println(dacValue[dacChannel]);
    return;
  }

  if (cmd.startsWith(F("SOUR:SERVO "))) {
    int comma = cmd.indexOf(',');
    int pwmChannel = -1;
    long angle = 0;
    if (comma < 0 ||
        !resolvePwmChannel(cmd.substring(11, comma), pwmChannel) ||
        !parseIntStrict(cmd.substring(comma + 1), angle)) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    if (angle < 0 || angle > 180) {
      setError(ERR_PARAM_RANGE);
      return;
    }

    int physicalIndex = findPhysicalPinIndexByGpio(kPwmPins[pwmChannel]);
    if (physicalIndex < 0 || pinModes[physicalIndex] != MODE_OUT) {
      setError(ERR_MODE);
      return;
    }

    if (!ensureServoAttached((uint8_t)pwmChannel)) {
      setError(ERR_EXECUTION);
      return;
    }

    servoAngle[pwmChannel] = (uint8_t)angle;
    ledcWrite(kPwmPins[pwmChannel], servoDutyFromAngle((uint8_t)angle));
    sendAck();
    return;
  }

  if (cmd.startsWith(F("SOUR:SERVO? "))) {
    int pwmChannel = -1;
    if (!resolvePwmChannel(cmd.substring(12), pwmChannel)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    Serial.println(servoAngle[pwmChannel]);
    return;
  }

  if (cmd.startsWith(F("SOUR:SERVO:ATT? "))) {
    int pwmChannel = -1;
    if (!resolvePwmChannel(cmd.substring(16), pwmChannel)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    Serial.println(servoAttached[pwmChannel] ? F("1") : F("0"));
    return;
  }

  if (cmd.startsWith(F("TRIG:SOUR "))) {
    String token = normalizeToken(cmd.substring(10));
    if (token == F("IMM")) trigMode = TRIG_IMM;
    else if (token == F("ANA")) trigMode = TRIG_ANA;
    else if (token == F("DIG")) trigMode = TRIG_DIG;
    else {
      setError(ERR_PARAM_RANGE);
      return;
    }
    sendAck();
    return;
  }
  if (cmd == F("TRIG:SOUR?")) {
    if (trigMode == TRIG_IMM) Serial.println(F("IMM"));
    else if (trigMode == TRIG_ANA) Serial.println(F("ANA"));
    else Serial.println(F("DIG"));
    return;
  }

  if (cmd.startsWith(F("TRIG:SLOP "))) {
    String token = normalizeToken(cmd.substring(10));
    if (token == F("POS")) trigSlope = SLOPE_POS;
    else if (token == F("NEG")) trigSlope = SLOPE_NEG;
    else {
      setError(ERR_PARAM_RANGE);
      return;
    }
    sendAck();
    return;
  }
  if (cmd == F("TRIG:SLOP?")) {
    Serial.println(trigSlope == SLOPE_POS ? F("POS") : F("NEG"));
    return;
  }

  if (cmd.startsWith(F("TRIG:CHAN "))) {
    if (trigMode == TRIG_IMM) {
      setError(ERR_MODE);
      return;
    }

    if (trigMode == TRIG_ANA) {
      int analogChannel = -1;
      if (!resolveAnalogChannel(cmd.substring(10), analogChannel)) {
        setError(ERR_PARAM_RANGE);
        return;
      }
      trigAnalogChannel = analogChannel;
      sendAck();
      return;
    }

    int physicalIndex = -1;
    if (!resolveDigitalPin(cmd.substring(10), physicalIndex)) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    trigDigitalPinIndex = physicalIndex;
    sendAck();
    return;
  }

  if (cmd == F("TRIG:CHAN?")) {
    if (trigMode == TRIG_ANA) {
      Serial.print(F("AN"));
      Serial.println(trigAnalogChannel);
      return;
    }
    if (trigMode == TRIG_DIG) {
      Serial.print(F("GPIO"));
      Serial.println(kPhysicalPins[trigDigitalPinIndex].gpio);
      return;
    }
    Serial.println(F("NONE"));
    return;
  }

  if (cmd.startsWith(F("TRIG:LEV "))) {
    if (trigMode != TRIG_ANA) {
      setError(ERR_MODE);
      return;
    }
    float level = cmd.substring(9).toFloat();
    if (level < 0.0f || level > vRef) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    trigLevel = level;
    sendAck();
    return;
  }
  if (cmd == F("TRIG:LEV?")) {
    Serial.println(trigLevel, 3);
    return;
  }

  if (cmd.startsWith(F("TRIG:TOUT "))) {
    long value = 0;
    if (!parseIntStrict(cmd.substring(10), value) || value <= 0) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    trigTimeout = (unsigned long)value;
    sendAck();
    return;
  }
  if (cmd == F("TRIG:TOUT?")) {
    Serial.println(trigTimeout);
    return;
  }

  if (cmd.startsWith(F("ACQ:POIN "))) {
    long value = 0;
    uint16_t channels = (scanCount > 0) ? scanCount : 1;
    if (!parseIntStrict(cmd.substring(9), value) || value <= 0 || (value * channels) > MAX_TOTAL_POINTS) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    acqPoints = (uint16_t)value;
    sendAck();
    return;
  }
  if (cmd == F("ACQ:POIN?")) {
    Serial.println(acqPoints);
    return;
  }

  if (cmd.startsWith(F("ACQ:TINT "))) {
    long value = 0;
    if (!parseIntStrict(cmd.substring(9), value) || value <= 0) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    acqTStep = (uint32_t)value;
    sendAck();
    return;
  }
  if (cmd == F("ACQ:TINT?")) {
    Serial.println(acqTStep);
    return;
  }

  if (cmd == F("ACQ:STAT?")) {
    if (acqState == ACQ_IDLE) Serial.println(F("IDLE"));
    else if (acqState == ACQ_PREFILL) Serial.println(F("PREFILL"));
    else if (acqState == ACQ_ARMED) Serial.println(F("ARMED"));
    else if (acqState == ACQ_POST) Serial.println(F("POST"));
    else Serial.println(F("DONE"));
    return;
  }

  if (cmd == F("INIT")) {
    if (scanCount == 0) {
      setError(ERR_EXECUTION);
      return;
    }
    if ((uint32_t)scanCount * (uint32_t)acqPoints > MAX_TOTAL_POINTS) {
      setError(ERR_PARAM_RANGE);
      return;
    }
    for (uint8_t i = 0; i < scanCount; ++i) {
      if (!isAnalogChannelAccessible(scanList[i])) {
        return;
      }
    }
    if (trigMode == TRIG_ANA && !isAnalogChannelAccessible(trigAnalogChannel)) {
      return;
    }
    if (trigMode == TRIG_DIG &&
        !(pinModes[trigDigitalPinIndex] == MODE_IN || pinModes[trigDigitalPinIndex] == MODE_PULLUP)) {
      setError(ERR_MODE);
      return;
    }

    acqState = ACQ_PREFILL;
    acqHead = 0;
    prefillCount = 0;
    postCount = 0;
    lastSampleTime = micros();
    sendAck();
    return;
  }

  if (cmd == F("ABOR")) {
    acqState = ACQ_IDLE;
    sendAck();
    return;
  }

  if (cmd == F("FETC?")) {
    if (scanCount == 0 || acqState == ACQ_IDLE) {
      setError(ERR_EXECUTION);
      return;
    }

    ScpiError errorSnapshot = lastError;
    unsigned long start = millis();
    while (acqState != ACQ_DONE) {
      runAcquisitionEngine();
      if (lastError != errorSnapshot) {
        acqState = ACQ_IDLE;
        return;
      }
      if (millis() - start > 2000) {
        setError(ERR_TIMEOUT);
        acqState = ACQ_IDLE;
        return;
      }
    }

    uint16_t index = acqHead;
    for (uint16_t row = 0; row < acqPoints; ++row) {
      for (uint8_t ch = 0; ch < scanCount; ++ch) {
        float value = ((float)acqBuffer[(index * scanCount) + ch] * vRef) / (float)rawMaxValue();
        Serial.print(value, 4);
        if (ch + 1 < scanCount) {
          Serial.print(",");
        }
      }
      Serial.println();
      index = (index + 1) % acqPoints;
    }
    acqState = ACQ_IDLE;
    return;
  }

  setError(ERR_CMD_UNKNOWN);
}

void setup() {
  Serial.begin(BAUDRATE);
  resetDevice();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      if (serialBuffer.length() < 180) {
        serialBuffer += c;
      }
    }
  }

  runAcquisitionEngine();
}
