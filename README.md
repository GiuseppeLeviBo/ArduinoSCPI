# ArduinoSCPI

Libreria/firmware SCPI per Arduino UNO che espone un piccolo strumento programmabile via seriale USB. Il firmware implementa misure analogiche, uscite digitali, PWM, servo, acquisizione multi-canale e trigger analogico/digitale con sintassi ispirata a SCPI. 

---

# 📘 OpenSCPI-Lab UNO – Manuale utente

## 1. Panoramica

**OpenSCPI-Lab UNO** trasforma un Arduino UNO in uno strumento controllabile via SCPI (**S**tandard **C**ommands for **P**rogrammable **I**nstruments).

Funzioni principali attualmente implementate:

- identificazione e reset strumento
- selezione e lettura ingressi analogici `A0..A5`
- lettura ADC grezza oppure tensione convertita in volt
- scansione di più canali analogici con `ROUT:SCAN` + `READ?`
- controllo uscite digitali sui pin `D2..D13`
- controllo PWM sui pin `D9` e `D10`
- controllo servo sui pin `D9` e `D10`
- query servo: ultimo angolo impostato e stato attach
- trigger immediato, analogico o digitale
- gestione pendenza trigger (`TRIG:SLOP POS|NEG`) anche su trigger digitale (edge)
- abilitazione/disabilitazione delle risposte di conferma `OK`
- gestione errori SCPI tramite `SYST:ERR?` e pulizia stato con `*CLS`
- selezione riferimento ADC (`CAL:REF DEF|INT|EXT`) e valore Vref software (`CAL:VREF`)
- gestione avanzata GPIO con direzione/modalità (`DIG:MODE`) e lettura ingresso (`DIG:IN?`)
- **(nuovo, firmware scope)** acquisizione bufferizzata tipo oscilloscopio con pre-trigger e post-trigger (`INIT` + `FETC?`)
- **(nuovo, firmware scope)** stato acquisizione interrogabile (`ACQ:STAT?`)

### Varianti firmware nel repository

Il repository include ora **due sketch**:

- `Arduino_SCPI.c`: versione base/leggera (comandi SCPI essenziali)
- `Arduino_SCOPE_SCPI.c`: versione estesa con funzionalità scope (acquisizione bufferizzata)

> Questa documentazione mantiene i comandi comuni e aggiunge, in sezioni dedicate, le estensioni specifiche della versione `Arduino_SCOPE_SCPI.c`.

### ⚠️ Compatibilità rapida tra firmware (importante)

Per evitare ambiguità:

- `Arduino_SCPI.c` è **davvero una versione base**.
- molte estensioni documentate sotto sono **solo** per `Arduino_SCOPE_SCPI.c`.

| Comando/famiglia | `Arduino_SCPI.c` (base) | `Arduino_SCOPE_SCPI.c` (esteso) |
| --- | --- | --- |
| `*IDN?`, `*RST`, `SYST:ACK` | ✅ | ✅ |
| `*OPC?`, `*CLS`, `SYST:ERR?` | ❌ | ✅ |
| `CAL:REF`, `CAL:VREF` | ❌ | ✅ |
| `DIG:MODE`, `DIG:IN?` | ❌ | ✅ |
| `TRIG:SLOP` | ❌ | ✅ |
| `SOUR:SERVO?`, `SOUR:SERVO:ATT?` | ❌ | ✅ |
| `ACQ:*`, `INIT`, `ABOR`, `FETC?` | ❌ | ✅ |

> Se usi il firmware base `Arduino_SCPI.c`, i comandi non supportati rispondono con errore.

---

## 2. Comunicazione seriale

Lo strumento comunica tramite porta seriale USB.

- **Baudrate:** `115200`
- **Terminazione comando:** newline `\n`
- **Formato generale risposte:**
  - valori numerici o stringhe per le query `...?`
  - `OK` per i comandi di configurazione, se gli ACK sono abilitati
  - `ERR` in caso di errore di comando/esecuzione

> Per il dettaglio errore usare sempre `SYST:ERR?` (stile SCPI): il firmware mantiene l'ultimo errore e lo azzera dopo la lettura.

### Esempio

```text
*IDN?\n
```

---

## 3. Modello operativo

Per le acquisizioni multi-canale il flusso concettuale è:

```text
Configurazione -> Trigger -> Acquisizione -> Lettura
```

In pratica:

1. si definisce la lista di canali con `ROUT:SCAN`
2. si configura l'eventuale trigger con `TRIG:*`
3. si avvia la lettura con `READ?`

Per la variante scope (`Arduino_SCOPE_SCPI.c`) è disponibile anche il flusso avanzato:

```text
Configurazione -> Arm (INIT) -> Trigger + campionamento -> Fetch (FETC?)
```

---

## 4. Mappa hardware

### Ingressi analogici

| Canale SCPI | Pin Arduino |
| --- | --- |
| 0 | A0 |
| 1 | A1 |
| 2 | A2 |
| 3 | A3 |
| 4 | A4 |
| 5 | A5 |

### Uscite digitali

| Canale SCPI | Pin Arduino |
| --- | --- |
| 0 | D2 |
| 1 | D3 |
| 2 | D4 |
| 3 | D5 |
| 4 | D6 |
| 5 | D7 |
| 6 | D8 |
| 7 | D9 |
| 8 | D10 |
| 9 | D11 |
| 10 | D12 |
| 11 | D13 / LED_BUILTIN |

> Nota: i pin `D9` e `D10` possono essere usati sia come uscite digitali sia come uscite PWM/servo.

### PWM / Servo

| Canale SCPI | Pin Arduino |
| --- | --- |
| 0 | D9 |
| 1 | D10 |

---

## 5. Comandi standard

> Nota compatibilità: in questa sezione `*OPC?`, `*CLS` e `SYST:ERR?` sono disponibili **solo** nel firmware `Arduino_SCOPE_SCPI.c`.

### `*IDN?`

Restituisce l'identità dello strumento.

**Risposta:**

```text
OpenSCPI-Lab,Arduino-UNO,1.1_SCOPE
```

### `*RST`

Ripristina lo stato iniziale dello strumento:

- canale analogico corrente = `0`
- trigger = `IMM`
- livello trigger = `2.5`
- timeout trigger = `1000 ms`
- uscite digitali = `LOW`
- PWM = `0`
- servo sganciati (`detach`)
- *(firmware scope)* fronte trigger = `POS`
- riferimento ADC = `DEFAULT`, `vRef = 5.0`
- modalità digitali riportate a `OUT`

**Risposta:** `OK` se gli ACK sono attivi.

### `*OPC?`

Query di stato operazione completa:

- `1` se non ci sono acquisizioni in corso (`ACQ_IDLE` o `ACQ_DONE`)
- `0` se acquisizione armata/in corso

### `*CLS`

Azzera lo stato errore SCPI interno.

### `SYST:ERR?`

Restituisce e consuma l'ultimo errore SCPI in formato numerico+testo, ad esempio:

```text
0,"No error"
-222,"Data out of range"
-221,"Settings conflict"
```

---

## 6. Gestione ACK

Il firmware può rispondere con `OK` dopo i comandi di configurazione. Per retrocompatibilità gli ACK sono attivi di default.

### `SYST:ACK ON`
### `SYST:ACK 1`

Abilita le risposte `OK`.

### `SYST:ACK OFF`
### `SYST:ACK 0`

Disabilita le risposte `OK`.

### `SYST:ACK?`

Restituisce:

- `1` = ACK attivi
- `0` = ACK disattivi

---

## 7. Misure analogiche

## ⚠️ Warning importante su riferimento ADC e precisione

> `CAL:REF` e `CAL:VREF` sono comandi **solo** di `Arduino_SCOPE_SCPI.c`.  
> La versione base `Arduino_SCPI.c` usa conversione fissa a 5V (`raw * (5.0 / 1023.0)`).

Nel firmware `Arduino_SCOPE_SCPI.c` la conversione in volt usa:

```text
V = raw * (vRef / 1024.0)
```

dove `vRef` dipende dalla configurazione SCPI:

- `CAL:REF DEF` -> riferimento ADC default Arduino (tipicamente 5V su UNO alimentato USB)
- `CAL:REF INT` -> riferimento interno nominale 1.1V
- `CAL:REF EXT` -> riferimento esterno sul pin AREF/Vref (hardware esterno)
- `CAL:VREF <val>` -> imposta il valore numerico usato dal firmware per la conversione in volt

Se si usa `INT` o `EXT` **senza** aggiornare correttamente `CAL:VREF`, le misure in volt e le soglie trigger analogiche possono risultare errate.

> In particolare con riferimento interno a 1.1V, segnali oltre il fondo scala saturano il convertitore: verificare cablaggio/attenuazione prima delle misure.

### Comandi calibrazione/riferimento

### `CAL:REF <DEF|INT|EXT>`

Imposta la sorgente di riferimento ADC:

- `DEF` = default Arduino
- `INT` = riferimento interno 1.1V
- `EXT` = riferimento esterno su pin Vref/AREF

### `CAL:REF?`

Restituisce `DEF`, `INT` oppure `EXT`.

### `CAL:VREF <value>`

Imposta il valore di riferimento usato nei calcoli in volt.

- range ammesso firmware scope: `0.5 < value <= 6.0`

### `CAL:VREF?`

Restituisce `vRef` con 3 decimali.

### Mappa canali analogici

| Canale | Pin |
| --- | --- |
| 0 | A0 |
| 1 | A1 |
| 2 | A2 |
| 3 | A3 |
| 4 | A4 |
| 5 | A5 |

### `CONF:VOLT <ch>`

Seleziona il canale analogico corrente (`0..5`).

**Esempio:**

```text
CONF:VOLT 2
```

### `CONF:VOLT?`

Restituisce il canale analogico attualmente selezionato.

### `MEAS:VOLT?`

Legge la tensione sul canale correntemente configurato.

### `MEAS:VOLT? <ch>`

Legge direttamente il canale specificato (`0..5`) senza cambiare la configurazione corrente.

### `MEAS:RAW? <ch>`

Restituisce il valore ADC grezzo del canale specificato (`0..5`), nel range:

```text
0..1023
```

### `MEAS:VOLT:ALL?`

Restituisce in una singola riga le tensioni dei sei canali analogici.

**Esempio:**

```text
1.0215,0.9785,0.1173,4.5015,0.0000,0.3324
```

> Nota: nel firmware `Arduino_SCOPE_SCPI.c` la conversione usa sempre `raw * (vRef / 1024.0)`.

---

## 8. GPIO digitali (IN/OUT/PULLUP, input e output)

> `DIG:MODE`, `DIG:MODE?` e `DIG:IN?` sono disponibili **solo** nel firmware `Arduino_SCOPE_SCPI.c`.  
> Nel firmware base restano `DIG:OUT` e `DIG:OUT?`.

### Mappa canali digitali

| Canale | Pin |
| --- | --- |
| 0 | D2 |
| 1 | D3 |
| 2 | D4 |
| 3 | D5 |
| 4 | D6 |
| 5 | D7 |
| 6 | D8 |
| 7 | D9 |
| 8 | D10 |
| 9 | D11 |
| 10 | D12 |
| 11 | D13 |

### `DIG:OUT <ch>,<val>`

Imposta un'uscita digitale.

**Prerequisito (solo firmware scope):** il canale deve essere in modalità `OUT` (vedi `DIG:MODE`), altrimenti il firmware restituisce errore di modalità (`SYST:ERR?` -> `-221`).

- `<val>` può essere `0` oppure `1`
- `<ch>` può essere indicato in due modi:
  - **indice SCPI**: `0..11`
  - **pin fisico Arduino**: `D2`, `D3`, ..., `D13`

**Esempi:**

```text
DIG:OUT 3,1
DIG:OUT 3,0
DIG:OUT D13,1
DIG:OUT D13,0
```

### Regola importante per il canale

- `DIG:OUT 9,1` significa **canale SCPI 9** -> pin `D11`
- `DIG:OUT D9,1` significa **pin fisico `D9`**

Questa distinzione evita ambiguità fra indice logico e numero del pin Arduino.

### Interazione con PWM/Servo

Se si usa `DIG:OUT` su un pin PWM (`D9` o `D10`):

- il corrispondente valore PWM viene aggiornato internamente a `255` oppure `0`
- un eventuale servo collegato su quel canale viene disattivato (`detach`)

### `DIG:OUT? <ch>`

Legge lo stato logico dell'uscita specificata.

Sono accettati sia gli indici SCPI sia i nomi pin `D<n>`.

Anche qui il canale deve essere in modalità `OUT`.

### `DIG:MODE <ch>,<IN|OUT|PULLUP>`

Configura la modalità del pin digitale:

- `IN` input floating
- `OUT` uscita digitale
- `PULLUP` input con pull-up interna

`<ch>` accetta sia indice SCPI (`0..11`) sia pin (`D2..D13`).

### `DIG:MODE? <ch>`

Restituisce la modalità corrente: `IN`, `OUT` o `PULLUP`.

### `DIG:IN? <ch>`

Legge direttamente il livello logico del pin (`0`/`1`) in modalità input.

---

## 9. PWM

### Mappa canali PWM

| Canale | Pin |
| --- | --- |
| 0 | D9 |
| 1 | D10 |

### `SOUR:PWM <ch>,<val>`

Imposta il duty cycle PWM.

- `<ch>`: `0..1`
- `<val>`: `0..255`

**Esempio:**

```text
SOUR:PWM 0,128
```

### `SOUR:PWM? <ch>`

Restituisce il valore PWM corrente del canale richiesto.

### Nota operativa

Se sullo stesso canale era stato precedentemente attivato un servo, il firmware esegue automaticamente il `detach()` prima di applicare il PWM.

---

## 10. Servo

### Mappa canali servo

| Canale | Pin |
| --- | --- |
| 0 | D9 |
| 1 | D10 |

### `SOUR:SERVO <ch>,<angle>`

Imposta la posizione del servo.

- `<ch>`: `0..1`
- `<angle>`: `0..180`

**Esempio:**

```text
SOUR:SERVO 1,90
```

Quando il canale non è ancora associato a un servo, il firmware esegue automaticamente `attach()` sul pin corrispondente.

### `SOUR:SERVO? <ch>` *(firmware scope)*

Restituisce l'ultimo angolo impostato sul canale servo richiesto.

- `<ch>`: `0..1`
- valore tipico restituito: `0..180`

### `SOUR:SERVO:ATT? <ch>` *(firmware scope)*

Restituisce lo stato attuale di attach del servo:

- `1` = servo attaccato
- `0` = servo non attaccato (detach)

---

## 11. Scansione multi-canale

### `ROUT:SCAN (@list)`

Definisce la lista di canali analogici da acquisire con `READ?`.

Formati supportati:

- elenco esplicito: `(@0,1,2)`
- intervallo: `(@0:5)`

**Esempi:**

```text
ROUT:SCAN (@0,1,2)
ROUT:SCAN (@0:5)
```

### `ROUT:SCAN?`

Restituisce la lista canali attualmente configurata come sequenza separata da virgole.

**Esempio:**

```text
0,1,2
```

### `READ?`

Esegue l'acquisizione dei canali definiti in `ROUT:SCAN`.

- se il trigger è soddisfatto, restituisce le tensioni dei canali selezionati
- se la lista è vuota o c'è errore di esecuzione, restituisce `ERR` (dettaglio con `SYST:ERR?`)
- se il trigger va in timeout, restituisce `ERR` (dettaglio `-250,"Timeout error"` via `SYST:ERR?`)

**Esempio risposta:**

```text
1.2307,0.9814,3.4487
```

---

## 12. Trigger

Il firmware supporta tre modalità trigger:

- `IMM` = trigger immediato
- `ANA` = trigger analogico
- `DIG` = trigger digitale

### `TRIG:SOUR <mode>`

Seleziona la sorgente di trigger.

**Valori ammessi:**

```text
IMM
ANA
DIG
```

**Esempi:**

```text
TRIG:SOUR IMM
TRIG:SOUR ANA
TRIG:SOUR DIG
```

### `TRIG:SOUR?`

Restituisce la modalità trigger corrente.

### `TRIG:SLOP <POS|NEG>` *(solo firmware scope)*

Imposta il fronte di trigger nella modalità scope:

- `POS` = fronte di salita
- `NEG` = fronte di discesa

In `ANA` la condizione diventa:

- `POS`: trigger quando `V >= level`
- `NEG`: trigger quando `V <= level`

Per trigger digitali (`TRIG:SOUR DIG`) il firmware usa trigger **edge**:

- `TRIG:SLOP POS` = fronte LOW→HIGH
- `TRIG:SLOP NEG` = fronte HIGH→LOW

### `TRIG:SLOP?` *(solo firmware scope)*

Restituisce `POS` oppure `NEG`.

### `TRIG:CHAN <ch>`

Imposta il canale sorgente del trigger.

Il significato dipende dalla modalità corrente:

- con `TRIG:SOUR ANA`, `<ch>` deve essere un canale analogico `0..5`
- con `TRIG:SOUR DIG`, `<ch>` può essere:
  - un indice digitale SCPI `0..11`
  - un pin nel formato `D<n>`

#### Mappa rapida per `TRIG:CHAN`

**Trigger analogico**

| Canale | Pin |
| --- | --- |
| 0 | A0 |
| 1 | A1 |
| 2 | A2 |
| 3 | A3 |
| 4 | A4 |
| 5 | A5 |

**Trigger digitale**

| Canale | Pin |
| --- | --- |
| 0 | D2 |
| 1 | D3 |
| 2 | D4 |
| 3 | D5 |
| 4 | D6 |
| 5 | D7 |
| 6 | D8 |
| 7 | D9 |
| 8 | D10 |
| 9 | D11 |
| 10 | D12 |
| 11 | D13 |

Quando si configura un trigger digitale, il pin selezionato viene messo in `INPUT` per permettere la lettura del segnale esterno.

Se si tenta di usare `TRIG:CHAN` mentre la modalità è `IMM`, il firmware risponde:

```text
ERR
```

(dettaglio: `-221,"Settings conflict"` da `SYST:ERR?`)

### `TRIG:CHAN?`

Restituisce:

- il canale analogico di trigger se il modo è `ANA`
- il canale digitale di trigger se il modo è `DIG`
- `NONE` se il modo è `IMM`

### `TRIG:LEV <value>`

Imposta il livello di trigger.

- in modalità `ANA`, la condizione di trigger è `V >= level` (oppure `V <= level` se `TRIG:SLOP NEG` nella versione scope)
- in modalità `DIG`, il comando **non è ammesso** (`Settings conflict`, errore `-221`)

### `TRIG:LEV?`

Restituisce il livello trigger corrente.

> Nota: nelle revisioni più recenti il comportamento di `TRIG:LEV?` è stato corretto per essere coerente con la modalità trigger attiva; se la modalità non usa `TRIG:LEV`, il comando può restituire errore.

### `TRIG:TOUT <ms>`

Imposta il timeout del trigger in millisecondi.

- deve essere maggiore di `0`
- il default dopo reset è `1000`

### `TRIG:TOUT?`

Restituisce il timeout trigger corrente in millisecondi.

---

## 13. Acquisizione scope bufferizzata *(solo `Arduino_SCOPE_SCPI.c`)*

La variante scope introduce una macchina a stati di acquisizione con buffer circolare logico su array lineare:

- stati: `ACQ_IDLE`, `ACQ_PREFILL`, `ACQ_ARMED`, `ACQ_POST`, `ACQ_DONE`
- buffer ADC raw massimo: `MAX_TOTAL_POINTS = 300` campioni totali (distribuiti su tutti i canali in scan)
- pre-trigger e post-trigger al 50% dei punti richiesti

### `ACQ:POIN <n>`

Imposta il numero di punti per acquisizione.

- ammessi: `1 .. 300`
- errore su valore non valido: `ERR:VAL`

### `ACQ:TINT <us>`

Imposta il passo temporale di campionamento interno in **microsecondi** (variabile firmware `acqTStep`).

- valore ammesso: `> 0`
- errore su valore non valido: `ERR:VAL`

> Nota: nel codice la variabile locale si chiama `ms`, ma viene confrontata/assegnata direttamente a `micros()` come unità di microsecondi.

### `ACQ:POIN?`

Restituisce il numero di punti configurato.

### `ACQ:TINT?`

Restituisce il passo temporale in microsecondi.

### `ACQ:STAT?`

Restituisce lo stato corrente della macchina acquisizione:

- `IDLE`
- `PREFILL`
- `ARMED`
- `POST`
- `DONE`

### `INIT`

Arma l'acquisizione scope.

Controlli effettuati:

- se `ROUT:SCAN` non è configurato -> `ERR` (`-200,"Execution error"`)
- se `ACQ:POIN * numero_canali_scan > 300` -> `ERR` (`-222,"Data out of range"`)

Se valido, entra in stato `ACQ_PREFILL`.

### `ABOR`

Interrompe/disarma l'acquisizione e riporta lo stato a `ACQ_IDLE`.

### `FETC?`

Attende il completamento dell'acquisizione e poi restituisce i dati acquisiti.

Comportamento:

- se non armato (`ACQ_IDLE`) -> `ERR` (`-200,"Execution error"`)
- timeout interno fetch di 2 secondi -> `ERR` (`-250,"Timeout error"`)
- in caso positivo, stampa `acqPoints` righe
- ogni riga contiene i canali della scan corrente separati da virgola
- valori convertiti in volt con `raw * (vRef / 1024.0)`

Formato output (esempio con 2 canali in scan):

```text
1.2344,0.1025
1.2451,0.1030
...
```

---

## 14. Esempi rapidi

### Misura singolo canale

```text
CONF:VOLT 0
MEAS:VOLT?
```

### Misura diretta senza riconfigurare

```text
MEAS:VOLT? 4
MEAS:RAW? 4
```

### Accensione LED integrato

```text
DIG:OUT D13,1
```

### PWM su D9

```text
SOUR:PWM 0,200
```

### Servo a 45° su D10

```text
SOUR:SERVO 1,45
```

### Acquisizione con trigger analogico

```text
ROUT:SCAN (@0,1,2)
TRIG:SOUR ANA
TRIG:CHAN 0
TRIG:LEV 2.500
TRIG:TOUT 3000
READ?
```

### Acquisizione con trigger digitale su D2 alto

```text
ROUT:SCAN (@0:2)
TRIG:SOUR DIG
TRIG:SLOP POS
TRIG:CHAN D2
TRIG:TOUT 5000
READ?
```

### Sequenza scope completa (firmware `Arduino_SCOPE_SCPI.c`)

```text
ROUT:SCAN (@0,1)
TRIG:SOUR ANA
TRIG:SLOP POS
TRIG:CHAN 0
TRIG:LEV 2.300
ACQ:POIN 120
ACQ:TINT 250
INIT
FETC?
```

---

## 15. Limiti attuali

- firmware pensato per **Arduino UNO / ATmega328P**
- tensioni calcolate usando `vRef` configurabile via `CAL:REF`/`CAL:VREF` (default 5V)
- il trigger si applica alla lettura `READ?` e, nella versione scope, anche al ciclo `INIT`/`FETC?`
- il firmware non implementa un parser SCPI completo, ma un sottoinsieme pratico
- nella variante `Arduino_SCOPE_SCPI.c` la memoria totale dei campioni è limitata a `300` valori ADC complessivi

---

## 16. Licenza

Distribuito secondo la licenza riportata nel file `LICENSE`.
