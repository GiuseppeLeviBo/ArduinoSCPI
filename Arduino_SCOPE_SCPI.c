#include <Servo.h>

#define BAUDRATE 115200

/* ---------- Hardware mapping ---------- */

const uint8_t analogPins[6] = {A0,A1,A2,A3,A4,A5};
const uint8_t digitalPins[] = {2,3,4,5,6,7,8,9,10,11,12,LED_BUILTIN};
const uint8_t pwmPins[2] = {9,10};

/* ---------- Stato ---------- */

uint8_t currentAnalogChannel = 0;
const uint8_t digitalPinCount = sizeof(digitalPins) / sizeof(digitalPins[0]);
bool digitalState[digitalPinCount];
uint8_t pwmValue[2];
bool servoAttached[2];
bool ackEnabled = true; 
const uint8_t pwmPinCount = sizeof(pwmPins) / sizeof(pwmPins[0]);

Servo servos[2];

uint8_t scanList[6];
uint8_t scanCount = 0;

enum TriggerMode {TRIG_IMM, TRIG_ANA, TRIG_DIG}; 
TriggerMode trigMode = TRIG_IMM;

enum TrigSlope {SLOP_POS, SLOP_NEG}; // Nuovo: Fronte di Trigger
TrigSlope trigSlope = SLOP_POS;

float trigLevel = 2.5;
unsigned long trigTimeout = 1000;
int trigAnalogChannel = 0;   
int trigDigitalChannel = 0;  

/* ---------- Acquisizione Bufferizzata Dinamica ---------- */
enum AcqState {ACQ_IDLE, ACQ_PREFILL, ACQ_ARMED, ACQ_POST, ACQ_DONE};
AcqState acqState = ACQ_IDLE;

// BUFFER LINEARE: 300 punti totali disponibili (600 byte di RAM)
const uint16_t MAX_TOTAL_POINTS = 300; 
uint16_t acqBuffer[MAX_TOTAL_POINTS]; 

uint16_t acqPoints = 50;  
uint16_t acqHead = 0;     
uint16_t prefillCount = 0;
uint16_t postCount = 0;
unsigned long lastSampleTime = 0;
uint32_t acqTStep = 1000;  

/* ---------- Utility ---------- */
void sendAck()
{
  if(ackEnabled) Serial.println(F("OK")); 
}

bool checkTriggerInstant()
{
  if(trigMode == TRIG_IMM) return true;
  
  if(trigMode == TRIG_ANA)
  {
    int raw = analogRead(analogPins[trigAnalogChannel]);
    float v = raw * (5.0 / 1024.0); 
    if(trigSlope == SLOP_POS) return (v >= trigLevel);
    else return (v <= trigLevel); // Scatta sul fronte di discesa
  }
  
  if(trigMode == TRIG_DIG)
  {
    int expectedState = (trigLevel >= 0.5) ? HIGH : LOW;
    // Nel digitale, ignoriamo momentaneamente lo slope e scattiamo sullo stato logico atteso
    return (digitalRead(digitalPins[trigDigitalChannel]) == expectedState);
  }
  
  return false;
}

bool isValidAnalogChannel(int ch) { return ch >= 0 && ch < 6; }
bool isValidDigitalChannel(int ch) { return ch >= 0 && ch < digitalPinCount; }
bool isValidPwmChannel(int ch) { return ch >= 0 && ch < pwmPinCount; }

int findPwmChannelByPin(uint8_t pin)
{
  for(uint8_t i = 0; i < pwmPinCount; i++)
    if(pwmPins[i] == pin) return i;
  return -1;
}

int parseChannel(String &value)
{
  value.trim();
  return value.toInt();
}

bool resolveDigitalChannel(String token, int &channel)
{
  token.trim();
  if(token.length() == 0) return false;

  if(token.charAt(0) == 'D' || token.charAt(0) == 'd')
  {
    String numPart = token.substring(1);
    int parsedPin = parseChannel(numPart);
    for(uint8_t i = 0; i < digitalPinCount; i++)
    {
      if(parsedPin == digitalPins[i]) { channel = i; return true; }
    }
    return false;
  }

  int parsed = parseChannel(token);
  if(isValidDigitalChannel(parsed)) { channel = parsed; return true; }
  return false;
}

float readVolt(uint8_t ch)
{
  int raw = analogRead(analogPins[ch]);
  return raw * (5.0 / 1024.0); 
}

/* ---------- Reset ---------- */

void resetDevice()
{
  currentAnalogChannel = 0;
  trigMode = TRIG_IMM;   
  trigSlope = SLOP_POS;  // Reset Slope
  trigLevel = 2.5;       
  trigTimeout = 1000;    
  trigAnalogChannel = 0;     
  trigDigitalChannel = 0;    
  
  for(uint8_t i = 0; i < pwmPinCount; i++)
  {
    if(servoAttached[i]) { servos[i].detach(); servoAttached[i] = false; }
    pwmValue[i] = 0;
    analogWrite(pwmPins[i], 0);
  }

  for(uint8_t i = 0; i < digitalPinCount; i++)
  {
    pinMode(digitalPins[i], OUTPUT); 
    digitalState[i] = LOW;
    digitalWrite(digitalPins[i], LOW);
  }
}

void setScanList(String &cmd)
{
  scanCount = 0;
  int start = cmd.indexOf('@');
  int end = cmd.indexOf(')');
  if(start == -1 || end == -1) return; 

  String list = cmd.substring(start+1,end);

  if(list.indexOf(':') != -1)
  {
    int colon = list.indexOf(':');
    String sStart = list.substring(0,colon);
    String sEnd = list.substring(colon+1);
    for(int ch = sStart.toInt(); ch <= sEnd.toInt(); ch++)
      if(isValidAnalogChannel(ch)) scanList[scanCount++] = ch;
  }
  else
  {
    int i = 0;
    while(i < list.length())
    {
      int comma = list.indexOf(',',i);
      if(comma == -1) comma = list.length();
      String sCh = list.substring(i,comma);
      int ch = sCh.toInt();
      if(isValidAnalogChannel(ch)) scanList[scanCount++] = ch;
      i = comma+1;
    }
  }
}

bool waitTrigger()
{
  if(trigMode == TRIG_IMM) return true;
  unsigned long startMillis = millis();

  if(trigMode == TRIG_ANA)
  {
    while(millis() - startMillis < trigTimeout)
    {
      float v = readVolt(trigAnalogChannel);
      if(trigSlope == SLOP_POS && v >= trigLevel) return true;
      if(trigSlope == SLOP_NEG && v <= trigLevel) return true;
    }
  }
  else if(trigMode == TRIG_DIG)
  {
    int expectedState = (trigLevel >= 0.5) ? HIGH : LOW;
    while(millis() - startMillis < trigTimeout)
      if(digitalRead(digitalPins[trigDigitalChannel]) == expectedState) return true;
  }
  return false; 
}

void measAll(Stream &interface)
{
  for(int i=0;i<6;i++)
  {
    interface.print(readVolt(i),4);
    if(i<5) interface.print(",");
  }
  interface.println();
}

void readScan(Stream &interface)
{
  if(!waitTrigger()) { interface.println(F("ERR:TIMEOUT")); return; }

  for(int i=0;i<scanCount;i++)
  {
    interface.print(readVolt(scanList[i]),4);
    if(i<scanCount-1) interface.print(",");
  }
  interface.println();
}

/* ---------- Macchina a Stati (Estratta per permettere il Fetch bloccante) ---------- */
void runAcquisitionEngine()
{
  if(acqState == ACQ_IDLE || acqState == ACQ_DONE) return;

  unsigned long now = micros();
  
  if(now - lastSampleTime >= acqTStep)
  {
    lastSampleTime = now;

    // Scrittura nel buffer lineare
    for(int i = 0; i < scanCount; i++)
    {
       acqBuffer[(acqHead * scanCount) + i] = analogRead(analogPins[scanList[i]]);
    }

    if(acqState == ACQ_PREFILL)
    {
      prefillCount++;
      acqHead = (acqHead + 1) % acqPoints; 
      
      if(prefillCount >= acqPoints / 2) 
        acqState = ACQ_ARMED;
    }
    else if(acqState == ACQ_ARMED)
    {
      bool triggered = checkTriggerInstant();
      acqHead = (acqHead + 1) % acqPoints;
      
      if(triggered)
      {
        postCount = 0;
        acqState = ACQ_POST; 
      }
    }
    else if(acqState == ACQ_POST)
    {
      postCount++;
      acqHead = (acqHead + 1) % acqPoints;
      
      if(postCount >= acqPoints / 2) 
        acqState = ACQ_DONE; 
    }
  }
}

/* ---------- Command parser ---------- */
void processCommand(String &cmd) 
{
  cmd.trim();

  if(cmd == F("*IDN?")) { Serial.println(F("OpenSCPI-Lab,Arduino-UNO,1.2")); return; }
  if(cmd == F("*RST")) { resetDevice(); sendAck(); return; }
  if(cmd == F("CONF:VOLT?")) { Serial.println(currentAnalogChannel); return; }
  
  if(cmd.startsWith(F("SYST:ACK ")))
  {
    if(cmd.endsWith(F("ON")) || cmd.endsWith(F("1"))) ackEnabled = true;
    else if(cmd.endsWith(F("OFF")) || cmd.endsWith(F("0"))) ackEnabled = false;
    else { Serial.println(F("ERR")); return; }
    sendAck(); return;
  }

  if(cmd == F("SYST:ACK?")) { Serial.println(ackEnabled ? F("1") : F("0")); return; }
  
  if(cmd.startsWith(F("CONF:VOLT")))
  {
    String num = cmd.substring(9);
    int ch = parseChannel(num);
    if(isValidAnalogChannel(ch)) { currentAnalogChannel = ch; sendAck(); return; }
    Serial.println(F("ERR")); return;
  }

  if(cmd.startsWith(F("MEAS:VOLT?")))
  {
    int ch = currentAnalogChannel;
    if(cmd.length()>10) { String num = cmd.substring(10); ch = parseChannel(num); }
    if(!isValidAnalogChannel(ch)) { Serial.println(F("ERR")); return; }
    Serial.println(readVolt(ch),4); return;
  }

  if(cmd.startsWith(F("MEAS:RAW?")))
  {
    String num = cmd.substring(9);
    int ch = parseChannel(num);
    if(!isValidAnalogChannel(ch)) { Serial.println(F("ERR")); return; }
    Serial.println(analogRead(analogPins[ch])); return;
  }

  if(cmd == F("MEAS:VOLT:ALL?")) { measAll(Serial); return; }

  if(cmd == F("ROUT:SCAN?"))
  {
    for(int i = 0; i < scanCount; i++)
    {
      Serial.print(scanList[i]);
      if(i < scanCount - 1) Serial.print(",");
    }
    Serial.println(); return;
  }

  if(cmd.startsWith(F("ROUT:SCAN"))) { setScanList(cmd); sendAck(); return; }

  if(cmd == F("READ?"))
  {
    if(scanCount == 0) { Serial.println(F("ERR:NOSCAN")); return; }
    readScan(Serial); return;
  }

  if(cmd.startsWith(F("DIG:OUT ")))
  {
    int comma = cmd.indexOf(',');
    int ch = -1;
    if(comma == -1 || !resolveDigitalChannel(cmd.substring(8,comma), ch)) { Serial.println(F("ERR")); return; }

    String valStr = cmd.substring(comma+1);
    int val = parseChannel(valStr);

    if(val == 0 || val == 1)
    {
      int pwmChannel = findPwmChannelByPin(digitalPins[ch]);
      if(pwmChannel >= 0)
      {
        pwmValue[pwmChannel] = val ? 255 : 0;
        if(servoAttached[pwmChannel]) { servos[pwmChannel].detach(); servoAttached[pwmChannel] = false; }
      }
      digitalState[ch] = val;
      digitalWrite(digitalPins[ch], val ? HIGH : LOW);
      sendAck(); return;
    }
    Serial.println(F("ERR")); return;
  }

  if(cmd.startsWith(F("DIG:OUT?")))
  {
    int ch = -1;
    if(!resolveDigitalChannel(cmd.substring(8), ch)) { Serial.println(F("ERR")); return; }
    Serial.println(digitalState[ch]); return;
  }

  if(cmd.startsWith(F("SOUR:PWM ")))
  {
    int comma = cmd.indexOf(',');
    if(comma == -1) { Serial.println(F("ERR")); return; }
    
    String chStr = cmd.substring(9,comma);
    String valStr = cmd.substring(comma+1);
    int ch = parseChannel(chStr);
    int val = parseChannel(valStr);

    if(isValidPwmChannel(ch) && val >= 0 && val <= 255)
    {
      if(servoAttached[ch]) { servos[ch].detach(); servoAttached[ch] = false; }
      pwmValue[ch] = val;
      analogWrite(pwmPins[ch], val);
      sendAck(); return;
    }
    Serial.println(F("ERR")); return;
  }

  if(cmd.startsWith(F("SOUR:PWM?")))
  {
    String num = cmd.substring(9);
    int ch = parseChannel(num);
    if(!isValidPwmChannel(ch)) { Serial.println(F("ERR")); return; }
    Serial.println(pwmValue[ch]); return;
  }

  if(cmd.startsWith(F("SOUR:SERVO")))
  {
    int comma = cmd.indexOf(',');
    if(comma == -1) { Serial.println(F("ERR")); return; }

    String chStr = cmd.substring(11,comma);
    String valStr = cmd.substring(comma+1);
    int ch = parseChannel(chStr);
    int angle = parseChannel(valStr);

    if(isValidPwmChannel(ch) && angle >= 0 && angle <= 180)
    {
      if(!servoAttached[ch]) { servos[ch].attach(pwmPins[ch]); servoAttached[ch] = true; }
      servos[ch].write(angle);
      sendAck(); return;
    }
    Serial.println(F("ERR")); return;
  }

  if(cmd.startsWith(F("TRIG:SOUR ")))
  {
    if(cmd.endsWith(F("IMM"))) trigMode = TRIG_IMM;
    else if(cmd.endsWith(F("ANA"))) trigMode = TRIG_ANA;
    else if(cmd.endsWith(F("DIG"))) trigMode = TRIG_DIG;
    else { Serial.println(F("ERR")); return; }
    sendAck(); return;
  }

  if(cmd == F("TRIG:SOUR?"))
  {
    if(trigMode == TRIG_IMM) Serial.println(F("IMM"));
    else if(trigMode == TRIG_ANA) Serial.println(F("ANA"));
    else Serial.println(F("DIG")); 
    return;
  }

  /* NUOVO: TRIG:SLOP (Fronte salita/discesa) */
  if(cmd.startsWith(F("TRIG:SLOP ")))
  {
    if(cmd.endsWith(F("POS"))) trigSlope = SLOP_POS;
    else if(cmd.endsWith(F("NEG"))) trigSlope = SLOP_NEG;
    else { Serial.println(F("ERR")); return; }
    sendAck(); return;
  }

  if(cmd == F("TRIG:SLOP?"))
  {
    Serial.println(trigSlope == SLOP_POS ? F("POS") : F("NEG")); 
    return;
  }

  if(cmd.startsWith(F("TRIG:CHAN ")))
  {
    String token = cmd.substring(10);
    token.trim();

    if(trigMode == TRIG_ANA)
    {
      int ch = parseChannel(token);
      if(isValidAnalogChannel(ch)) { trigAnalogChannel = ch; sendAck(); return; }
    }
    else if(trigMode == TRIG_DIG)
    {
      int ch = -1;
      if(resolveDigitalChannel(token, ch)) { trigDigitalChannel = ch; pinMode(digitalPins[ch], INPUT); sendAck(); return; }
    }
    else { Serial.println(F("ERR:MODE")); return; }

    Serial.println(F("ERR")); return;
  }

  if(cmd == F("TRIG:CHAN?"))
  {
    if(trigMode == TRIG_ANA) Serial.println(trigAnalogChannel);
    else if(trigMode == TRIG_DIG) Serial.println(trigDigitalChannel);
    else Serial.println(F("NONE"));
    return;
  }
  
  if(cmd.startsWith(F("TRIG:LEV ")))
  {
    trigLevel = cmd.substring(9).toFloat();
    sendAck(); return;
  }

  if(cmd == F("TRIG:LEV?")) { Serial.println(trigLevel,3); return; }

  if(cmd.startsWith(F("TRIG:TOUT ")))
  {
    long parsedTout = cmd.substring(10).toInt();
    if (parsedTout > 0) { trigTimeout = (unsigned long)parsedTout; sendAck(); }
    else Serial.println(F("ERR"));
    return;
  }

  if(cmd == F("TRIG:TOUT?")) { Serial.println(trigTimeout); return; }

  if(cmd.startsWith(F("ACQ:POIN ")))
  {
    int pts = cmd.substring(9).toInt();
    if(pts > 0 && pts <= MAX_TOTAL_POINTS) { acqPoints = pts; sendAck(); }
    else Serial.println(F("ERR:VAL"));
    return;
  }

  if(cmd.startsWith(F("ACQ:TINT ")))
  {
    int ms = cmd.substring(9).toInt();
    if(ms > 0) { acqTStep = (uint32_t)ms; sendAck(); }
    else Serial.println(F("ERR:VAL"));
    return;
  }

  if(cmd == F("INIT"))
  {
    if(scanCount == 0) { Serial.println(F("ERR:NOSCAN")); return; }
    
    if((uint16_t)acqPoints * scanCount > MAX_TOTAL_POINTS) 
    { 
      Serial.println(F("ERR:MEM")); 
      return; 
    }

    acqState = ACQ_PREFILL;
    acqHead = 0;
    prefillCount = 0;
    lastSampleTime = micros();
    sendAck();
    return;
  }

  if(cmd == F("ABOR")) { acqState = ACQ_IDLE; sendAck(); return; }

  /* FETCH MODIFICATO: Ora aspetta e fa girare la macchina a stati! */
  if(cmd == F("FETC?"))
  {
    if(acqState == ACQ_IDLE) { Serial.println(F("ERR:NOTARMED")); return; }
    
    unsigned long waitStart = millis();

    // Loop bloccante finché non ha finito, ma con un timeout di 2 SECONDI
    while(acqState != ACQ_DONE)
    {
      runAcquisitionEngine(); // Continua a campionare in background!
      
      if(millis() - waitStart > 2000) // Timeout di 2 secondi
      {
        Serial.println(F("ERR:TIMEOUT"));
        acqState = ACQ_IDLE; // Disarma l'oscilloscopio
        return;
      }
    }
    
    // Se siamo usciti dal while senza timeout, vuol dire che acqState == ACQ_DONE !
    uint16_t index = acqHead; 
    
    for(uint16_t i = 0; i < acqPoints; i++)
    {
      for(uint8_t ch = 0; ch < scanCount; ch++)
      {
        float v = acqBuffer[(index * scanCount) + ch] * (5.0 / 1024.0);
        Serial.print(v, 4);
        if(ch < scanCount - 1) Serial.print(",");
      }
      Serial.println(); 
      index = (index + 1) % acqPoints;
    }
    
    acqState = ACQ_IDLE; 
    return;
  }

  Serial.println(F("ERR"));
}

/* ---------- Setup ---------- */

void setup()
{
  Serial.begin(BAUDRATE);

  for(uint8_t i = 0; i < digitalPinCount; i++) pinMode(digitalPins[i],OUTPUT);
  for(uint8_t i = 0; i < pwmPinCount; i++) pinMode(pwmPins[i],OUTPUT);

  resetDevice();
}

/* ---------- Loop ---------- */

String serialBuffer = "";

void loop()
{
  while(Serial.available() > 0)
  {
    char c = Serial.read();
    if(c == '\n')
    {
      processCommand(serialBuffer);
      serialBuffer = "";
    }
    else
    {
      if(serialBuffer.length() < 100) serialBuffer += c;
    }
  }

  // Chiamata alla macchina a stati
  runAcquisitionEngine();
}