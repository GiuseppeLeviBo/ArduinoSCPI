#include <Servo.h>

#define BAUDRATE 115200

/* ---------- Hardware mapping ---------- */
const uint8_t analogPins[6] = {A0,A1,A2,A3,A4,A5};
const uint8_t digitalPins[] = {2,3,4,5,6,7,8,9,10,11,12,LED_BUILTIN};
const uint8_t pwmPins[2] = {9,10};

/* ---------- Gestione Errori SCPI ---------- */
enum ScpiError : int8_t {
  ERR_NONE = 0,
  ERR_CMD_UNKNOWN = -1,    
  ERR_PARAM_RANGE = -2,    
  ERR_EXECUTION = -3,      
  ERR_TIMEOUT = -4,        
  ERR_MODE = -5            
};

ScpiError lastError = ERR_NONE;
bool ackEnabled = true; 

void sendAck() {
  if(ackEnabled) Serial.println(F("OK")); 
}

void setError(ScpiError err) {
  lastError = err;
  if(ackEnabled) Serial.println(F("ERR")); 
}

/* ---------- Prototipi Funzioni ---------- */
float readVolt(uint8_t ch);
bool checkTriggerInstant();
bool isValidPwmChannel(int ch);
bool isValidAnalogChannel(int ch);
bool isValidDigitalChannel(int ch);
bool resolveDigitalChannel(const String &token, int &channel);
int parseChannel(const String &value);
int findPwmChannelByPin(uint8_t pin);
bool setScanList(const String &cmd);
bool waitTrigger();
void runAcquisitionEngine();
void processCommand(String &cmd);

/* ---------- Stato ---------- */
enum AnalogRef { REF_DEF, REF_INT, REF_EXT };
AnalogRef refMode = REF_DEF;
float vRef = 5.0; 

uint8_t currentAnalogChannel = 0;
const uint8_t digitalPinCount = sizeof(digitalPins) / sizeof(digitalPins[0]);

enum DigMode : uint8_t { MODE_OUT, MODE_IN, MODE_PULLUP };
DigMode digitalMode[digitalPinCount];
bool digitalState[digitalPinCount]; 

uint8_t pwmValue[2];
uint8_t servoAngle[2]; 
bool servoAttached[2];
const uint8_t pwmPinCount = sizeof(pwmPins) / sizeof(pwmPins[0]);

Servo servos[2];

uint8_t scanList[6];
uint8_t scanCount = 0;

enum TriggerMode {TRIG_IMM, TRIG_ANA, TRIG_DIG}; 
TriggerMode trigMode = TRIG_IMM;
enum TrigSlope {SLOP_POS, SLOP_NEG}; 
TrigSlope trigSlope = SLOP_POS;

float trigLevel = 2.5;
unsigned long trigTimeout = 1000;
int trigAnalogChannel = 0;   
int trigDigitalChannel = 0;  

// Variabili per l'Edge Trigger
float lastTrigVolt = 0.0;
int lastTrigDig = LOW;

/* ---------- Acquisizione Bufferizzata ---------- */
enum AcqState {ACQ_IDLE, ACQ_PREFILL, ACQ_ARMED, ACQ_POST, ACQ_DONE};
AcqState acqState = ACQ_IDLE;

const uint16_t MAX_TOTAL_POINTS = 300; 
uint16_t acqBuffer[MAX_TOTAL_POINTS]; 

uint16_t acqPoints = 50;  
uint16_t acqHead = 0;     
uint16_t prefillCount = 0;
uint16_t postCount = 0;
unsigned long lastSampleTime = 0;
uint32_t acqTStep = 1000; 

/* ---------- Utility ---------- */
bool checkTriggerInstant() {
  if(trigMode == TRIG_IMM) return true;
  
  if(trigMode == TRIG_ANA) {
    float v = readVolt(trigAnalogChannel); 
    bool triggered = false;
    if(trigSlope == SLOP_POS && lastTrigVolt < trigLevel && v >= trigLevel) triggered = true;
    if(trigSlope == SLOP_NEG && lastTrigVolt > trigLevel && v <= trigLevel) triggered = true;
    lastTrigVolt = v;
    return triggered;
  }
  
  if(trigMode == TRIG_DIG) {
    int st = digitalRead(digitalPins[trigDigitalChannel]);
    bool triggered = false;
    if(trigSlope == SLOP_POS && lastTrigDig == LOW && st == HIGH) triggered = true;
    if(trigSlope == SLOP_NEG && lastTrigDig == HIGH && st == LOW) triggered = true;
    lastTrigDig = st;
    return triggered;
  }
  return false;
}

bool isValidAnalogChannel(int ch) { return ch >= 0 && ch < 6; }
bool isValidDigitalChannel(int ch) { return ch >= 0 && ch < digitalPinCount; }
bool isValidPwmChannel(int ch) { return ch >= 0 && ch < pwmPinCount; }

int findPwmChannelByPin(uint8_t pin) {
  for(uint8_t i = 0; i < pwmPinCount; i++)
    if(pwmPins[i] == pin) return i;
  return -1;
}

int parseChannel(const String &value) {
  String tmp = value;
  tmp.trim();
  return tmp.toInt();
}

bool resolveDigitalChannel(const String &token, int &channel) {
  String tmp = token;
  tmp.trim();
  if(tmp.length() == 0) return false;

  if(tmp.charAt(0) == 'D' || tmp.charAt(0) == 'd') {
    int parsedPin = parseChannel(tmp.substring(1));
    for(uint8_t i = 0; i < digitalPinCount; i++) {
      if(parsedPin == digitalPins[i]) { channel = i; return true; }
    }
    return false;
  }

  int parsed = parseChannel(tmp);
  if(isValidDigitalChannel(parsed)) { channel = parsed; return true; }
  return false;
}

float readVolt(uint8_t ch) {
  int raw = analogRead(analogPins[ch]);
  return raw * (vRef / 1024.0); 
}

/* ---------- Reset ---------- */
void resetDevice() {
  analogReference(DEFAULT);
  refMode = REF_DEF;
  vRef = 5.0;

  currentAnalogChannel = 0;
  trigMode = TRIG_IMM;   
  trigSlope = SLOP_POS;  
  trigLevel = 2.5;       
  trigTimeout = 1000;    
  trigAnalogChannel = 0;     
  trigDigitalChannel = 0;    
  
  acqState = ACQ_IDLE;
  scanCount = 0;
  acqPoints = 50;
  acqTStep = 1000;

  lastError = ERR_NONE;
  ackEnabled = true;
  
  for(uint8_t i = 0; i < pwmPinCount; i++) {
    if(servoAttached[i]) { servos[i].detach(); servoAttached[i] = false; }
    pwmValue[i] = 0;
    servoAngle[i] = 0; 
    analogWrite(pwmPins[i], 0);
  }

  for(uint8_t i = 0; i < digitalPinCount; i++) {
    pinMode(digitalPins[i], OUTPUT); 
    digitalMode[i] = MODE_OUT;
    digitalState[i] = LOW;
    digitalWrite(digitalPins[i], LOW);
  }
}

bool setScanList(const String &cmd) {
  scanCount = 0;
  int start = cmd.indexOf('@');
  int end = cmd.indexOf(')');
  if(start == -1 || end == -1) return false; 

  String list = cmd.substring(start+1,end);

  if(list.indexOf(':') != -1) {
    int colon = list.indexOf(':');
    int ch_start = parseChannel(list.substring(0,colon));
    int ch_end = parseChannel(list.substring(colon+1));
    for(int ch = ch_start; ch <= ch_end; ch++) {
      if(isValidAnalogChannel(ch)) scanList[scanCount++] = ch;
    }
  }
  else {
    int i = 0;
    while(i < list.length()) {
      int comma = list.indexOf(',',i);
      if(comma == -1) comma = list.length();
      int ch = parseChannel(list.substring(i,comma));
      if(isValidAnalogChannel(ch)) scanList[scanCount++] = ch;
      i = comma+1;
    }
  }
  return scanCount > 0;
}

bool waitTrigger() {
  if(trigMode == TRIG_IMM) return true;
  
  if(trigMode == TRIG_ANA) lastTrigVolt = readVolt(trigAnalogChannel);
  else if(trigMode == TRIG_DIG) lastTrigDig = digitalRead(digitalPins[trigDigitalChannel]);

  unsigned long startMillis = millis();

  while(millis() - startMillis < trigTimeout) {
    if(checkTriggerInstant()) return true;
  }
  return false; 
}

void measAll(Stream &interface) {
  for(int i=0;i<6;i++) {
    interface.print(readVolt(i),4);
    if(i<5) interface.print(",");
  }
  interface.println();
}

void readScan(Stream &interface) {
  if(!waitTrigger()) { setError(ERR_TIMEOUT); return; }

  for(int i=0;i<scanCount;i++) {
    interface.print(readVolt(scanList[i]),4);
    if(i<scanCount-1) interface.print(",");
  }
  interface.println();
}

/* ---------- Macchina a Stati ---------- */
void runAcquisitionEngine() {
  if(acqState == ACQ_IDLE || acqState == ACQ_DONE) return;

  unsigned long now = micros();
  
  if(now - lastSampleTime >= acqTStep) {
    lastSampleTime = now;

    for(int i = 0; i < scanCount; i++) {
       acqBuffer[(acqHead * scanCount) + i] = analogRead(analogPins[scanList[i]]);
    }

    if(acqState == ACQ_PREFILL) {
      prefillCount++;
      acqHead = (acqHead + 1) % acqPoints; 
      if(prefillCount >= acqPoints / 2) {
        acqState = ACQ_ARMED;
        if(trigMode == TRIG_ANA) lastTrigVolt = readVolt(trigAnalogChannel);
        else if(trigMode == TRIG_DIG) lastTrigDig = digitalRead(digitalPins[trigDigitalChannel]);
      }
    }
    else if(acqState == ACQ_ARMED) {
      bool triggered = checkTriggerInstant();
      acqHead = (acqHead + 1) % acqPoints;
      if(triggered) {
        postCount = 0;
        acqState = ACQ_POST; 
      }
    }
    else if(acqState == ACQ_POST) {
      postCount++;
      acqHead = (acqHead + 1) % acqPoints;
      if(postCount >= acqPoints / 2) acqState = ACQ_DONE; 
    }
  }
}

/* ---------- Command parser ---------- */
void processCommand(String &cmd) 
{
  cmd.trim();

  /* SYSTEM & IEEE-488.2 COMPATIBILITY */
  if(cmd == F("*IDN?")) { Serial.println(F("OpenSCPI-Lab,Arduino-UNO,1.0-RC2")); return; }
  if(cmd == F("*RST")) { resetDevice(); sendAck(); return; }
  
  if(cmd == F("*OPC?")) { 
    if(acqState == ACQ_IDLE || acqState == ACQ_DONE) Serial.println(F("1")); 
    else Serial.println(F("0")); 
    return; 
  }
  
  if(cmd == F("*CLS")) { lastError = ERR_NONE; sendAck(); return; }

  if(cmd == F("SYST:ERR?")) {
    switch(lastError) {
      case ERR_NONE:         Serial.println(F("0,\"No error\"")); break;
      case ERR_CMD_UNKNOWN:  Serial.println(F("-100,\"Command error\"")); break;
      case ERR_PARAM_RANGE:  Serial.println(F("-222,\"Data out of range\"")); break;
      case ERR_EXECUTION:    Serial.println(F("-200,\"Execution error\"")); break;
      case ERR_TIMEOUT:      Serial.println(F("-250,\"Timeout error\"")); break;
      case ERR_MODE:         Serial.println(F("-221,\"Settings conflict\"")); break;
      default:               Serial.println(F("-300,\"Device-specific error\"")); break;
    }
    lastError = ERR_NONE; 
    return;
  }

  /* ACK */
  if(cmd.startsWith(F("SYST:ACK "))) {
    if(cmd.endsWith(F("ON")) || cmd.endsWith(F("1"))) ackEnabled = true;
    else if(cmd.endsWith(F("OFF")) || cmd.endsWith(F("0"))) ackEnabled = false;
    else { setError(ERR_PARAM_RANGE); return; }
    sendAck(); return;
  }
  if(cmd == F("SYST:ACK?")) { Serial.println(ackEnabled ? F("1") : F("0")); return; }

  /* CALIBRATION / REFERENCE */
  if(cmd.startsWith(F("CAL:REF "))) {
    if(cmd.endsWith(F("DEF"))) { analogReference(DEFAULT); vRef = 5.0; refMode = REF_DEF; sendAck(); return; }
    else if(cmd.endsWith(F("INT"))) { analogReference(INTERNAL); vRef = 1.1; refMode = REF_INT; sendAck(); return; }
    else if(cmd.endsWith(F("EXT"))) { analogReference(EXTERNAL); refMode = REF_EXT; sendAck(); return; }
    else { setError(ERR_PARAM_RANGE); return; }
  }
  if(cmd == F("CAL:REF?")) {
    if(refMode == REF_DEF) Serial.println(F("DEF"));
    else if(refMode == REF_INT) Serial.println(F("INT"));
    else Serial.println(F("EXT"));
    return;
  }

  if(cmd.startsWith(F("CAL:VREF "))) {
    float v = cmd.substring(9).toFloat();
    if(v > 0.5 && v <= 6.0) { vRef = v; sendAck(); }
    else setError(ERR_PARAM_RANGE);
    return;
  }
  if(cmd == F("CAL:VREF?")) { Serial.println(vRef, 3); return; }

  /* ANALOG READINGS */
  if(cmd.startsWith(F("CONF:VOLT"))) {
    int ch = parseChannel(cmd.substring(9));
    if(isValidAnalogChannel(ch)) { currentAnalogChannel = ch; sendAck(); return; }
    setError(ERR_PARAM_RANGE); return;
  }
  if(cmd == F("CONF:VOLT?")) { Serial.println(currentAnalogChannel); return; }

  if(cmd.startsWith(F("MEAS:VOLT?"))) {
    int ch = currentAnalogChannel;
    if(cmd.length()>10) { ch = parseChannel(cmd.substring(10)); }
    if(!isValidAnalogChannel(ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(readVolt(ch),4); return;
  }

  if(cmd.startsWith(F("MEAS:RAW?"))) {
    int ch = parseChannel(cmd.substring(9));
    if(!isValidAnalogChannel(ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(analogRead(analogPins[ch])); return;
  }

  if(cmd == F("MEAS:VOLT:ALL?")) { measAll(Serial); return; }

  /* ROUTING */
  if(cmd.startsWith(F("ROUT:SCAN "))) { 
    if(setScanList(cmd)) sendAck(); 
    else setError(ERR_PARAM_RANGE); 
    return; 
  }
  if(cmd == F("ROUT:SCAN?")) {
    for(int i = 0; i < scanCount; i++) {
      Serial.print(scanList[i]);
      if(i < scanCount - 1) Serial.print(",");
    }
    Serial.println(); return;
  }

  if(cmd == F("READ?")) {
    if(scanCount == 0) { setError(ERR_EXECUTION); return; }
    readScan(Serial); return;
  }

  /* GPIO EVOLUTO */
  if(cmd.startsWith(F("DIG:MODE "))) {
    int comma = cmd.indexOf(',');
    int ch = -1;
    if(comma == -1 || !resolveDigitalChannel(cmd.substring(9,comma), ch)) { setError(ERR_PARAM_RANGE); return; }
    
    String modeStr = cmd.substring(comma+1);
    modeStr.trim();
    if(modeStr == F("IN")) { pinMode(digitalPins[ch], INPUT); digitalMode[ch] = MODE_IN; }
    else if(modeStr == F("OUT")) { pinMode(digitalPins[ch], OUTPUT); digitalMode[ch] = MODE_OUT; }
    else if(modeStr == F("PULLUP")) { pinMode(digitalPins[ch], INPUT_PULLUP); digitalMode[ch] = MODE_PULLUP; }
    else { setError(ERR_PARAM_RANGE); return; }
    sendAck(); return;
  }

  if(cmd.startsWith(F("DIG:MODE? "))) {
    int ch = -1;
    if(!resolveDigitalChannel(cmd.substring(10), ch)) { setError(ERR_PARAM_RANGE); return; }
    if(digitalMode[ch] == MODE_IN) Serial.println(F("IN"));
    else if(digitalMode[ch] == MODE_OUT) Serial.println(F("OUT"));
    else Serial.println(F("PULLUP"));
    return;
  }

  if(cmd.startsWith(F("DIG:IN? "))) {
    int ch = -1;
    if(!resolveDigitalChannel(cmd.substring(8), ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(digitalRead(digitalPins[ch])); return;
  }

  if(cmd.startsWith(F("DIG:OUT "))) {
    int comma = cmd.indexOf(',');
    int ch = -1;
    if(comma == -1 || !resolveDigitalChannel(cmd.substring(8,comma), ch)) { setError(ERR_PARAM_RANGE); return; }

    if(digitalMode[ch] != MODE_OUT) { setError(ERR_MODE); return; }

    int val = parseChannel(cmd.substring(comma+1));
    if(val == 0 || val == 1) {
      int pwmChannel = findPwmChannelByPin(digitalPins[ch]);
      if(pwmChannel >= 0 && servoAttached[pwmChannel]) {
        servos[pwmChannel].detach(); servoAttached[pwmChannel] = false;
      }
      digitalState[ch] = val;
      digitalWrite(digitalPins[ch], val ? HIGH : LOW);
      sendAck(); return;
    }
    setError(ERR_PARAM_RANGE); return;
  }

  if(cmd.startsWith(F("DIG:OUT? "))) {
    int ch = -1;
    if(!resolveDigitalChannel(cmd.substring(9), ch)) { setError(ERR_PARAM_RANGE); return; }
    if(digitalMode[ch] != MODE_OUT) { setError(ERR_MODE); return; }
    Serial.println(digitalState[ch]); return;
  }

  /* SOUR (PWM & SERVO) */
  if(cmd.startsWith(F("SOUR:PWM "))) {
    int comma = cmd.indexOf(',');
    if(comma == -1) { setError(ERR_PARAM_RANGE); return; }
    
    int ch = parseChannel(cmd.substring(9,comma));
    
    if(!isValidPwmChannel(ch)) { setError(ERR_PARAM_RANGE); return; }

    int pinIndex = -1;
    for(uint8_t i = 0; i < digitalPinCount; i++) {
      if(digitalPins[i] == pwmPins[ch]) { pinIndex = i; break; }
    }
    if(pinIndex >= 0 && digitalMode[pinIndex] != MODE_OUT) { setError(ERR_MODE); return; }

    int val = parseChannel(cmd.substring(comma+1));

    if(val >= 0 && val <= 255) {
      if(servoAttached[ch]) { servos[ch].detach(); servoAttached[ch] = false; }
      pwmValue[ch] = val;
      analogWrite(pwmPins[ch], val);
      sendAck(); return;
    }
    setError(ERR_PARAM_RANGE); return;
  }

  if(cmd.startsWith(F("SOUR:PWM? "))) {
    int ch = parseChannel(cmd.substring(10));
    if(!isValidPwmChannel(ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(pwmValue[ch]); return;
  }

  if(cmd.startsWith(F("SOUR:SERVO "))) {
    int comma = cmd.indexOf(',');
    if(comma == -1) { setError(ERR_PARAM_RANGE); return; }

    int ch = parseChannel(cmd.substring(11,comma));
    
    if(!isValidPwmChannel(ch)) { setError(ERR_PARAM_RANGE); return; }

    int pinIndex = -1;
    for(uint8_t i = 0; i < digitalPinCount; i++) {
      if(digitalPins[i] == pwmPins[ch]) { pinIndex = i; break; }
    }
    if(pinIndex >= 0 && digitalMode[pinIndex] != MODE_OUT) { setError(ERR_MODE); return; }

    int angle = parseChannel(cmd.substring(comma+1));

    if(angle >= 0 && angle <= 180) {
      if(!servoAttached[ch]) { servos[ch].attach(pwmPins[ch]); servoAttached[ch] = true; }
      servos[ch].write(angle);
      servoAngle[ch] = angle; 
      sendAck(); return;
    }
    setError(ERR_PARAM_RANGE); return;
  }

  if(cmd.startsWith(F("SOUR:SERVO? "))) {
    int ch = parseChannel(cmd.substring(12));
    if(!isValidPwmChannel(ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(servoAngle[ch]); return;
  }

  if(cmd.startsWith(F("SOUR:SERVO:ATT? "))) {
    int ch = parseChannel(cmd.substring(16));
    if(!isValidPwmChannel(ch)) { setError(ERR_PARAM_RANGE); return; }
    Serial.println(servoAttached[ch] ? F("1") : F("0")); return;
  }

  /* TRIGGER */
  if(cmd.startsWith(F("TRIG:SOUR "))) {
    if(cmd.endsWith(F("IMM"))) trigMode = TRIG_IMM;
    else if(cmd.endsWith(F("ANA"))) trigMode = TRIG_ANA;
    else if(cmd.endsWith(F("DIG"))) trigMode = TRIG_DIG;
    else { setError(ERR_PARAM_RANGE); return; }
    sendAck(); return;
  }
  if(cmd == F("TRIG:SOUR?")) {
    if(trigMode == TRIG_IMM) Serial.println(F("IMM"));
    else if(trigMode == TRIG_ANA) Serial.println(F("ANA"));
    else Serial.println(F("DIG")); 
    return;
  }

  if(cmd.startsWith(F("TRIG:SLOP "))) {
    if(cmd.endsWith(F("POS"))) trigSlope = SLOP_POS;
    else if(cmd.endsWith(F("NEG"))) trigSlope = SLOP_NEG;
    else { setError(ERR_PARAM_RANGE); return; }
    sendAck(); return;
  }
  if(cmd == F("TRIG:SLOP?")) { Serial.println(trigSlope == SLOP_POS ? F("POS") : F("NEG")); return; }

  if(cmd.startsWith(F("TRIG:CHAN "))) {
    String token = cmd.substring(10);
    token.trim();
    if(trigMode == TRIG_ANA) {
      int ch = parseChannel(token);
      if(isValidAnalogChannel(ch)) { trigAnalogChannel = ch; sendAck(); return; }
    }
    else if(trigMode == TRIG_DIG) {
      int ch = -1;
      if(resolveDigitalChannel(token, ch)) { 
        trigDigitalChannel = ch; 
        pinMode(digitalPins[ch], INPUT); 
        digitalMode[ch] = MODE_IN;
        sendAck(); return; 
      }
    }
    else { setError(ERR_MODE); return; }
    setError(ERR_PARAM_RANGE); return;
  }
  if(cmd == F("TRIG:CHAN?")) {
    if(trigMode == TRIG_ANA) Serial.println(trigAnalogChannel);
    else if(trigMode == TRIG_DIG) Serial.println(trigDigitalChannel);
    else Serial.println(F("NONE"));
    return;
  }
  
  if(cmd.startsWith(F("TRIG:LEV "))) { 
    if(trigMode == TRIG_DIG) { setError(ERR_MODE); return; } 
    
    float lev = cmd.substring(9).toFloat(); 
    if(trigMode == TRIG_ANA && (lev < 0.0 || lev > vRef)) { setError(ERR_PARAM_RANGE); return; }
    
    trigLevel = lev;
    sendAck(); return; 
  }
  if(cmd == F("TRIG:LEV?")) { Serial.println(trigLevel,3); return; }

  if(cmd.startsWith(F("TRIG:TOUT "))) {
    long parsedTout = cmd.substring(10).toInt();
    if (parsedTout > 0) { trigTimeout = (unsigned long)parsedTout; sendAck(); }
    else setError(ERR_PARAM_RANGE);
    return;
  }
  if(cmd == F("TRIG:TOUT?")) { Serial.println(trigTimeout); return; }

  /* ACQUISITION BUFFER */
  if(cmd.startsWith(F("ACQ:POIN "))) {
    int pts = cmd.substring(9).toInt();
    uint16_t currentChans = (scanCount > 0) ? scanCount : 1;
    if(pts > 0 && (pts * currentChans) <= MAX_TOTAL_POINTS) { acqPoints = pts; sendAck(); }
    else setError(ERR_PARAM_RANGE);
    return;
  }
  if(cmd == F("ACQ:POIN?")) { Serial.println(acqPoints); return; }

  if(cmd.startsWith(F("ACQ:TINT "))) {
    long us = cmd.substring(9).toInt(); 
    if(us > 0) { acqTStep = (uint32_t)us; sendAck(); }
    else setError(ERR_PARAM_RANGE);
    return;
  }
  if(cmd == F("ACQ:TINT?")) { Serial.println(acqTStep); return; }

  if(cmd == F("ACQ:STAT?")) {
    switch(acqState) {
      case ACQ_IDLE: Serial.println(F("IDLE")); break;
      case ACQ_PREFILL: Serial.println(F("PREFILL")); break;
      case ACQ_ARMED: Serial.println(F("ARMED")); break;
      case ACQ_POST: Serial.println(F("POST")); break;
      case ACQ_DONE: Serial.println(F("DONE")); break;
    }
    return;
  }

  if(cmd == F("INIT")) {
    if(scanCount == 0) { setError(ERR_EXECUTION); return; }
    if((uint16_t)acqPoints * scanCount > MAX_TOTAL_POINTS) { setError(ERR_PARAM_RANGE); return; }

    acqState = ACQ_PREFILL;
    acqHead = 0;
    prefillCount = 0;
    postCount = 0; 
    lastSampleTime = micros();
    sendAck(); return;
  }

  if(cmd == F("ABOR")) { acqState = ACQ_IDLE; sendAck(); return; }

  if(cmd == F("FETC?")) {
    if(scanCount == 0) { setError(ERR_EXECUTION); return; } 
    if(acqState == ACQ_IDLE) { setError(ERR_EXECUTION); return; } 
    
    unsigned long waitStart = millis();
    while(acqState != ACQ_DONE) {
      runAcquisitionEngine(); 
      if(millis() - waitStart > 2000) {
        setError(ERR_TIMEOUT);
        acqState = ACQ_IDLE; 
        return;
      }
    }
    
    uint16_t index = acqHead; 
    for(uint16_t i = 0; i < acqPoints; i++) {
      for(uint8_t ch = 0; ch < scanCount; ch++) {
        float v = acqBuffer[(index * scanCount) + ch] * (vRef / 1024.0);
        Serial.print(v, 4);
        if(ch < scanCount - 1) Serial.print(",");
      }
      Serial.println(); 
      index = (index + 1) % acqPoints;
    }
    acqState = ACQ_IDLE; 
    return;
  }

  setError(ERR_CMD_UNKNOWN);
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(BAUDRATE);
  resetDevice();
}

/* ---------- Loop ---------- */
String serialBuffer = "";

void loop() {
  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
    }
    else {
      if(serialBuffer.length() < 100) serialBuffer += c;
    }
  }
  runAcquisitionEngine();
}