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

const uint8_t pwmPinCount = sizeof(pwmPins) / sizeof(pwmPins[0]);

Servo servos[2];

uint8_t scanList[6];
uint8_t scanCount = 0;

enum TriggerMode {TRIG_IMM, TRIG_ANA};
TriggerMode trigMode = TRIG_IMM;

float trigLevel = 2.5;
unsigned long trigTimeout = 1000;

/* ---------- Utility ---------- */

bool isValidAnalogChannel(int ch)
{
  return ch >= 0 && ch < 6;
}

bool isValidDigitalChannel(int ch)
{
  return ch >= 0 && ch < digitalPinCount;
}

bool isValidPwmChannel(int ch)
{
  return ch >= 0 && ch < pwmPinCount;
}

int findPwmChannelByPin(uint8_t pin)
{
  for(uint8_t i = 0; i < pwmPinCount; i++)
  {
    if(pwmPins[i] == pin)
      return i;
  }

  return -1;
}

int parseChannel(String value)
{
  value.trim();
  return value.toInt();
}

bool resolveDigitalChannel(String token, int &channel)
{
  token.trim();

  if(token.length() == 0)
    return false;

  if(token.charAt(0) == 'D' || token.charAt(0) == 'd')
  {
    int parsedPin = parseChannel(token.substring(1));

    for(uint8_t i = 0; i < digitalPinCount; i++)
    {
      if(parsedPin == digitalPins[i])
      {
        channel = i;
        return true;
      }
    }

    return false;
  }

  int parsed = parseChannel(token);

  if(isValidDigitalChannel(parsed))
  {
    channel = parsed;
    return true;
  }

  return false;
}

float readVolt(uint8_t ch)
{
  int raw = analogRead(analogPins[ch]);
  return raw * (5.0 / 1023.0);
}

/* ---------- Reset ---------- */

void resetDevice()
{
  currentAnalogChannel = 0;

  for(uint8_t i = 0; i < pwmPinCount; i++)
  {
    if(servoAttached[i])
    {
      servos[i].detach();
      servoAttached[i] = false;
    }

    pwmValue[i] = 0;
    analogWrite(pwmPins[i], 0);
  }

  for(uint8_t i = 0; i < digitalPinCount; i++)
  {
    digitalState[i] = LOW;
    digitalWrite(digitalPins[i], LOW);
  }
}


void setScanList(String cmd)
{
   scanCount = 0;

  int start = cmd.indexOf('@');
  int end = cmd.indexOf(')');

  if(start == -1 || end == -1)
    return; // protezione

  String list = cmd.substring(start+1,end);

  if(list.indexOf(':') != -1)
  {
    int colon = list.indexOf(':');

    int ch_start = list.substring(0,colon).toInt();
    int ch_end   = list.substring(colon+1).toInt();

    for(int ch = ch_start; ch <= ch_end; ch++)
    {
      if(isValidAnalogChannel(ch))
        scanList[scanCount++] = ch;
    }
  }
  else
  {
    int i = 0;
    while(i < list.length())
    {
      int comma = list.indexOf(',',i);
      if(comma == -1) comma = list.length();

      int ch = list.substring(i,comma).toInt();

      if(isValidAnalogChannel(ch))
        scanList[scanCount++] = ch;

      i = comma+1;
    }
  }
}


bool waitTrigger()
{
  // Se è immediato, restituisci subito true
  if(trigMode == TRIG_IMM) return true;

  if(trigMode == TRIG_ANA)
  {
    unsigned long startMillis = millis(); // Salva il tempo di inizio

    // Continua finché non scade il timeout
    while(millis() - startMillis < trigTimeout)
    {
      float v = readVolt(0); // trigger su A0

      if(v >= trigLevel)
        return true; // Trigger superato, usciamo con successo
    }
    
    return false; // Il ciclo è finito senza superare il livello = Timeout!
  }
  
  return false; // Fallback di sicurezza
}

void measAll(Stream &interface)
{
  for(int i=0;i<6;i++)
  {
    interface.print(readVolt(i),4);

    if(i<5)
      interface.print(",");
  }

  interface.println();
}

void readScan(Stream &interface)
{
  // Se il trigger fallisce per timeout, stampa un errore e interrompi
  if(!waitTrigger()) 
  {
    interface.println("ERR:TIMEOUT");
    return;
  }

  // Se siamo qui, il trigger è scattato correttamente
  for(int i=0;i<scanCount;i++)
  {
    float v = readVolt(scanList[i]);

    interface.print(v,4);

    if(i<scanCount-1)
      interface.print(",");
  }

  interface.println();
}


/* ---------- Command parser ---------- */

void processCommand(String cmd)
{
  cmd.trim();

  /* *IDN? */

  if(cmd == "*IDN?")
  {
    Serial.println("OpenSCPI-Lab,Arduino-UNO,1.2");
    return;
  }

  /* *RST */

  if(cmd == "*RST")
  {
    resetDevice();
    Serial.println("OK");
    return;
  }

  if(cmd == "CONF:VOLT?")
  {
    Serial.println(currentAnalogChannel);
    return;
  }

  /* CONF:VOLT */

  if(cmd.startsWith("CONF:VOLT"))
  {
    int ch = parseChannel(cmd.substring(9));

    if(isValidAnalogChannel(ch))
    {
      currentAnalogChannel = ch;
      Serial.println("OK");
      return;
    }

    Serial.println("ERR");
    return;
  }

  /* MEAS:VOLT? */

  if(cmd.startsWith("MEAS:VOLT?"))
  {
    int ch = currentAnalogChannel;

    if(cmd.length()>10)
      ch = parseChannel(cmd.substring(10));

    if(!isValidAnalogChannel(ch))
    {
      Serial.println("ERR");
      return;
    }

    Serial.println(readVolt(ch),4);
    return;
  }

  /* MEAS:RAW? */

  if(cmd.startsWith("MEAS:RAW?"))
  {
    int ch = parseChannel(cmd.substring(9));

    if(!isValidAnalogChannel(ch))
    {
      Serial.println("ERR");
      return;
    }

    int raw = analogRead(analogPins[ch]);
    Serial.println(raw);
    return;
  }

  if(cmd == "MEAS:VOLT:ALL?")
  {
    measAll(Serial);
    return;
  }

  if(cmd == "ROUT:SCAN?")
  {
    for(int i = 0; i < scanCount; i++)
    {
      Serial.print(scanList[i]);
      if(i < scanCount - 1)
        Serial.print(",");
    }

    Serial.println();
    return;
  }

  if(cmd.startsWith("ROUT:SCAN"))
  {
    setScanList(cmd);
    Serial.println("OK");
    return;
  }

  if(cmd == "READ?")
  {
    if(scanCount == 0)
    {
      Serial.println("ERR"); // nessuna scan list
      return;
    }

    readScan(Serial);
    return;
  }

  /* DIG:OUT */

  if(cmd.startsWith("DIG:OUT "))
  {
    int comma = cmd.indexOf(',');
    int ch = -1;

    if(comma == -1 || !resolveDigitalChannel(cmd.substring(8,comma), ch))
    {
      Serial.println("ERR");
      return;
    }

    int val = parseChannel(cmd.substring(comma+1));

    if(val == 0 || val == 1)
    {
      int pwmChannel = findPwmChannelByPin(digitalPins[ch]);
      if(pwmChannel >= 0)
      {
        pwmValue[pwmChannel] = val ? 255 : 0;

        if(servoAttached[pwmChannel])
        {
          servos[pwmChannel].detach();
          servoAttached[pwmChannel] = false;
        }
      }

      digitalState[ch] = val;
      digitalWrite(digitalPins[ch], val ? HIGH : LOW);
      Serial.println("OK");
      return;
    }

    Serial.println("ERR");
    return;
  }

  /* DIG:OUT? */

  if(cmd.startsWith("DIG:OUT?"))
  {
    int ch = -1;

    if(!resolveDigitalChannel(cmd.substring(8), ch))
    {
      Serial.println("ERR");
      return;
    }

    Serial.println(digitalState[ch]);
    return;
  }

  /* SOUR:PWM */

  if(cmd.startsWith("SOUR:PWM "))
  {
    int comma = cmd.indexOf(',');

    if(comma == -1)
    {
      Serial.println("ERR");
      return;
    }

    int ch = parseChannel(cmd.substring(9,comma));
    int val = parseChannel(cmd.substring(comma+1));

    if(isValidPwmChannel(ch) && val >= 0 && val <= 255)
    {
      if(servoAttached[ch])
      {
        servos[ch].detach();
        servoAttached[ch] = false;
      }

      pwmValue[ch] = val;
      analogWrite(pwmPins[ch], val);
      Serial.println("OK");
      return;
    }

    Serial.println("ERR");
    return;
  }

  /* SOUR:PWM? */

  if(cmd.startsWith("SOUR:PWM?"))
  {
    int ch = parseChannel(cmd.substring(9));

    if(!isValidPwmChannel(ch))
    {
      Serial.println("ERR");
      return;
    }

    Serial.println(pwmValue[ch]);
    return;
  }

  /* SOUR:SERVO */

  if(cmd.startsWith("SOUR:SERVO"))
  {
    int comma = cmd.indexOf(',');

    if(comma == -1)
    {
      Serial.println("ERR");
      return;
    }

    int ch = parseChannel(cmd.substring(11,comma));
    int angle = parseChannel(cmd.substring(comma+1));

    if(isValidPwmChannel(ch) && angle >= 0 && angle <= 180)
    {
      if(!servoAttached[ch])
      {
        servos[ch].attach(pwmPins[ch]);
        servoAttached[ch] = true;
      }

      servos[ch].write(angle);
      Serial.println("OK");
      return;
    }

    Serial.println("ERR");
    return;
  }

  if(cmd.startsWith("TRIG:SOUR "))
  {
    if(cmd.endsWith("IMM"))
      trigMode = TRIG_IMM;
    else if(cmd.endsWith("ANA"))
      trigMode = TRIG_ANA;
    else
    {
      Serial.println("ERR");
      return;
    }

    Serial.println("OK");
    return;
  }

  if(cmd == "TRIG:SOUR?")
  {
    if(trigMode == TRIG_IMM)
      Serial.println("IMM");
    else
      Serial.println("ANA");

    return;
  }

  if(cmd.startsWith("TRIG:LEV "))
  {
    trigLevel = cmd.substring(9).toFloat();
    Serial.println("OK");
    return;
  }

  if(cmd == "TRIG:LEV?")
  {
    Serial.println(trigLevel,3);
    return;
  }
/* TRIG:TOUT (Imposta il timeout del trigger in millisecondi) */
  if(cmd.startsWith("TRIG:TOUT "))
  {
    // Il comando "TRIG:TOUT " è lungo 10 caratteri. Prendiamo ciò che segue.
    long parsedTout = cmd.substring(10).toInt();
    
    if (parsedTout > 0) // Il timeout deve essere maggiore di zero
    {
      trigTimeout = (unsigned long)parsedTout;
      Serial.println("OK");
    }
    else
    {
      Serial.println("ERR");
    }
    return;
  }

  /* TRIG:TOUT? (Richiede il timeout attuale) */
  if(cmd == "TRIG:TOUT?")
  {
    Serial.println(trigTimeout);
    return;
  }
  Serial.println("ERR");
}

/* ---------- Setup ---------- */

void setup()
{
  Serial.begin(BAUDRATE);

  for(uint8_t i = 0; i < digitalPinCount; i++)
    pinMode(digitalPins[i],OUTPUT);

  for(uint8_t i = 0; i < pwmPinCount; i++)
    pinMode(pwmPins[i],OUTPUT);

  resetDevice();
}

/* ---------- Loop ---------- */

String buffer="";

void loop()
{
  while(Serial.available())
  {
    char c = Serial.read();

    if(c=='\n')
    {
      processCommand(buffer);
      buffer="";
    }
    else
      buffer += c;
  }
}
