#include <Servo.h>

#define BAUDRATE 115200

/* ---------- Hardware mapping ---------- */

const uint8_t analogPins[6] = {A0,A1,A2,A3,A4,A5};
const uint8_t digitalPins[7] = {2,3,4,5,6,7,8};
const uint8_t pwmPins[2] = {9,10};

/* ---------- Stato ---------- */

uint8_t currentAnalogChannel = 0;
bool digitalState[7];
uint8_t pwmValue[2];

Servo servos[2];

uint8_t scanList[6];
uint8_t scanCount = 0;

enum TriggerMode {TRIG_IMM, TRIG_ANA};
TriggerMode trigMode = TRIG_IMM;

float trigLevel = 2.5;

/* ---------- Utility ---------- */

float readVolt(uint8_t ch)
{
  int raw = analogRead(analogPins[ch]);
  return raw * (5.0 / 1023.0);
}

/* ---------- Reset ---------- */

void resetDevice()
{
  currentAnalogChannel = 0;

  for(int i=0;i<7;i++)
  {
    digitalState[i]=0;
    digitalWrite(digitalPins[i],LOW);
  }

  for(int i=0;i<2;i++)
  {
    pwmValue[i]=0;
    analogWrite(pwmPins[i],0);
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
      if(ch>=0 && ch<=5)
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

      if(ch>=0 && ch<=5)
        scanList[scanCount++] = ch;

      i = comma+1;
    }
  }
}


void waitTrigger()
{
  if(trigMode == TRIG_IMM) return;

  if(trigMode == TRIG_ANA)
  {
    while(true)
    {
      float v = readVolt(0); // trigger su A0

      if(v >= trigLevel)
        break;
    }
  }
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
  waitTrigger();

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
    Serial.println("OpenSCPI-Lab,Arduino-UNO,1.0");
    return;
  }

  /* *RST */

  if(cmd == "*RST")
  {
    resetDevice();
    Serial.println("OK");
    return;
  }

  /* CONF:VOLT */

  if(cmd.startsWith("CONF:VOLT"))
  {
    int ch = cmd.substring(9).toInt();

    if(ch>=0 && ch<=5)
      currentAnalogChannel = ch;

    Serial.println("OK");
    return;
  }

  /* MEAS:VOLT? */

  if(cmd.startsWith("MEAS:VOLT?"))
  {
    int ch = currentAnalogChannel;

    if(cmd.length()>10)
      ch = cmd.substring(10).toInt();

    Serial.println(readVolt(ch),4);
    return;
  }

  /* MEAS:RAW? */

  if(cmd.startsWith("MEAS:RAW?"))
  {
    int ch = cmd.substring(9).toInt();
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
  for(int i=0;i<scanCount;i++)
  {
    Serial.print(scanList[i]);
    if(i<scanCount-1) Serial.print(",");
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

    int ch = cmd.substring(8,comma).toInt();
    int val = cmd.substring(comma+1).toInt();

    if(ch>=0 && ch<=6)
    {
      digitalState[ch]=val;
      digitalWrite(digitalPins[ch],val);
    }

    Serial.println("OK");
    return;
  }

  /* DIG:OUT? */

  if(cmd.startsWith("DIG:OUT?"))
  {
    int ch = cmd.substring(8).toInt();
    Serial.println(digitalState[ch]);
    return;
  }

  /* SOUR:PWM */

  if(cmd.startsWith("SOUR:PWM "))
  {
    int comma = cmd.indexOf(',');

    int ch = cmd.substring(9,comma).toInt();
    int val = cmd.substring(comma+1).toInt();

    if(ch>=0 && ch<=1)
    {
      pwmValue[ch]=val;
      analogWrite(pwmPins[ch],val);
    }

    Serial.println("OK");
    return;
  }

  /* SOUR:PWM? */

  if(cmd.startsWith("SOUR:PWM?"))
  {
    int ch = cmd.substring(9).toInt();
    Serial.println(pwmValue[ch]);
    return;
  }

  /* SOUR:SERVO */

  if(cmd.startsWith("SOUR:SERVO"))
  {
    int comma = cmd.indexOf(',');

    int ch = cmd.substring(11,comma).toInt();
    int angle = cmd.substring(comma+1).toInt();

    if(ch>=0 && ch<=1)
      servos[ch].write(angle);

    Serial.println("OK");
    return;
  }

  if(cmd.startsWith("TRIG:SOUR "))
{
  if(cmd.endsWith("IMM")) trigMode = TRIG_IMM;
  if(cmd.endsWith("ANA")) trigMode = TRIG_ANA;

  Serial.println("OK");
  return;
}

if(cmd == "TRIG:SOUR?")
{
  if(trigMode==TRIG_IMM) Serial.println("IMM");
  else Serial.println("ANA");
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

  Serial.println("ERR");
}

/* ---------- Setup ---------- */

void setup()
{
  Serial.begin(BAUDRATE);

  for(int i=0;i<7;i++)
    pinMode(digitalPins[i],OUTPUT);

  for(int i=0;i<2;i++)
  {
    pinMode(pwmPins[i],OUTPUT);
    servos[i].attach(pwmPins[i]);
  }

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
