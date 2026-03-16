# ArduinoSCPI
A simple SCPI Library for Arduino


---

# 📘 **OpenSCPI-Lab UNO – User Manual v1.2**

---

## 1. Introduzione

**OpenSCPI-Lab UNO** è uno strumento didattico basato su Arduino UNO che implementa un sottoinsieme del protocollo SCPI (Standard Commands for Programmable Instruments), lo stesso utilizzato da strumenti professionali come quelli di Keysight Technologies e Tektronix.

Permette di:

* effettuare misure analogiche
* controllare uscite digitali
* generare segnali PWM
* controllare servo motori
* eseguire acquisizioni multi-canale
* introdurre il concetto di **trigger**

---

## 2. Connessione

Lo strumento comunica via:

* USB (Seriale CDC)
* Baudrate: **115200**

Terminazione comandi:

```
\n
```

---

## 3. Modello concettuale

Lo strumento segue questo flusso:

```
Configurazione → Trigger → Acquisizione → Lettura
```

---

## 4. Identificazione

### `*IDN?`

Restituisce l’identità dello strumento:

```
OpenSCPI-Lab,Arduino-UNO,1.2
```

---

### `*RST`

Reset completo dello strumento:

* reset canali
* reset uscite
* reset configurazioni

---

## 5. Misure analogiche

### Canali disponibili

| Canale | Pin |
| ------ | --- |
| 0      | A0  |
| 1      | A1  |
| 2      | A2  |
| 3      | A3  |
| 4      | A4  |
| 5      | A5  |

---

### `CONF:VOLT <ch>`

Seleziona il canale corrente.

Esempio:

```
CONF:VOLT 2
```

---

### `CONF:VOLT?`

Restituisce il canale attivo.

---

### `MEAS:VOLT?`

Misura sul canale configurato.

---

### `MEAS:VOLT? <ch>`

Misura diretta su un canale.

---

### `MEAS:RAW? <ch>`

Restituisce valore ADC (0–1023).

---

### `MEAS:VOLT:ALL?`

Misura tutti i canali:

```
1.02,0.98,0.12,4.50,0.00,0.33
```

---

## 6. Uscite digitali

### Canali

| SCPI | Pin   |
| ---- | ----- |
| 0–6  | D2–D8 |

---

### `DIG:OUT <ch>,<val>`

Imposta uscita:

```
DIG:OUT 3,1
DIG:OUT 3,0
```

---

### `DIG:OUT? <ch>`

Legge stato uscita.

---

## 7. PWM

### Canali

| SCPI | Pin |
| ---- | --- |
| 0    | D9  |
| 1    | D10 |

---

### `SOUR:PWM <ch>,<val>`

Valore:

```
0–255
```

---

### `SOUR:PWM? <ch>`

Restituisce duty cycle.

---

## 8. Servo

### `SOUR:SERVO <ch>,<angle>`

Angolo:

```
0–180°
```

---

## 9. Acquisizione multi-canale

---

### `ROUT:SCAN (@list)`

Definisce i canali da acquisire.

Esempi:

```
ROUT:SCAN (@0,1,2)
ROUT:SCAN (@0:5)
```

---

### `ROUT:SCAN?`

Restituisce lista attiva.

---

### `READ?`

Esegue acquisizione sui canali configurati.

Esempio:

```
1.23,0.98,3.45
```

---

## 10. Trigger (concetto chiave)

Il trigger definisce **quando iniziare la misura**.

---

### Modalità disponibili

| Comando | Significato      |
| ------- | ---------------- |
| `IMM`   | immediato        |
| `ANA`   | soglia analogica |

---

### `TRIG:SOUR IMM`

Misura immediata.

---

### `TRIG:SOUR ANA`

Attende evento analogico.

---

### `TRIG:LEV <val>`

Imposta soglia (Volt).

Esempio:

```
TRIG:LEV 2.5
```

---

### `TRIG:SOUR?`

Restituisce modalità trigger.

---

### `TRIG:LEV?`

Restituisce soglia.

---

## 11. Come funziona il trigger analogico

Quando attivo:

```
TRIG:SOUR ANA
```

lo strumento:

1. legge continuamente il canale A0
2. confronta con la soglia
3. quando:

```
V >= soglia
```

→ esegue la misura

---

## 12. Esempi

---

### Misura singola

```
MEAS:VOLT? 1
```

---

### Tutti i canali

```
MEAS:VOLT:ALL?
```

---

### Scan list

```
ROUT:SCAN (@0,1,2)
READ?
```

---

### Trigger analogico

```
TRIG:SOUR ANA
TRIG:LEV 2.0
READ?
```

---

## 13. Esempio Python

```python
import serial
import time

ser = serial.Serial("/dev/ttyACM0",115200,timeout=1)
time.sleep(2)

def query(cmd):
    ser.write((cmd+"\n").encode())
    return ser.readline().decode().strip()

print(query("*IDN?"))

print(query("MEAS:VOLT? 0"))

query("ROUT:SCAN (@0,1,2)")
print(query("READ?"))

query("TRIG:SOUR ANA")
query("TRIG:LEV 2.5")

print(query("READ?"))
```

---

## 14. Limitazioni hardware

Arduino UNO:

* ADC singolo (non simultaneo)
* conversione sequenziale (~100 µs per canale)
* risoluzione 10 bit

---

## 15. Obiettivi didattici

Questo progetto introduce:

* SCPI
* automazione di misura
* acquisizione dati
* sistemi reattivi (trigger)
* interazione Python-hardware

---

## 16. Licenza e contributi

Progetto open source pensato per:

* scuole
* università
* autoapprendimento


