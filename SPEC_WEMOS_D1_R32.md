# Specifiche firmware SCPI per WEMOS D1 R32 (ESP32)

## 1. Obiettivo
Definire una variante SCPI per WEMOS D1 R32 che sfrutti pienamente ESP32, senza vincolo di compatibilita' completa con Arduino UNO.

Obiettivi v1:
- usare tutti i canali analogici disponibili nel perimetro pin scelto (`A0..A5` + pin digitali analog-capable);
- introdurre un modello GPIO esplicito con `GPIO:MODE <ch>,IN|OUT|ANA|PULLUP` e validazione capability per pin;
- definire fin da subito il comportamento coerente di trigger e acquisizione in presenza di canali ADC2;
- predisporre uno stub radio (Wi-Fi/BT) richiamato dai cambi di modalita', pronto per integrazione reale.

## 2. Modello canali e pin

### 2.1 Tabella unica canali/pin (vincolante)
Tabella unica pin-centrica: stessa sorgente fisica, possibili alias SCPI digitali e analogici.

| Label pin | GPIO | Alias digitale SCPI | Alias analogico SCPI | ADC (se ANA) | IN | OUT | PULLUP | ANA | Rischio interferenza Wi-Fi/BT (ANA) | Note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| D2 | GPIO26 | `DCH0` / `D2` | `AN6` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D3 | GPIO25 | `DCH1` / `D3` | `AN7` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D4 | GPIO17 | `DCH2` / `D4` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| D5 | GPIO16 | `DCH3` / `D5` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| D6 | GPIO27 | `DCH4` / `D6` | `AN8` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D7 | GPIO14 | `DCH5` / `D7` | `AN9` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D8 | GPIO12 | `DCH6` / `D8` | `AN10` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D9 | GPIO13 | `DCH7` / `D9` | `AN11` | ADC2 | SI | SI | SI | SI | SI | Dual-role digitale/analogico |
| D10 | GPIO5 | `DCH8` / `D10` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| D11 | GPIO23 | `DCH9` / `D11` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| D12 | GPIO19 | `DCH10` / `D12` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| D13 | GPIO18 | `DCH11` / `D13` | N/A | N/A | SI | SI | SI | NO | NO | Solo digitale |
| A0 | GPIO2 | N/A | `AN0` / `A0` | ADC2 | SI | SI | SI | SI | SI | Analogico dedicato; anche GPIO nativo |
| A1 | GPIO4 | N/A | `AN1` / `A1` | ADC2 | SI | SI | SI | SI | SI | Analogico dedicato; anche GPIO nativo |
| A2 | GPIO35 | N/A | `AN2` / `A2` | ADC1 | SI | NO | NO | SI | NO | Input-only |
| A3 | GPIO34 | N/A | `AN3` / `A3` | ADC1 | SI | NO | NO | SI | NO | Input-only |
| A4 | GPIO36 | N/A | `AN4` / `A4` | ADC1 | SI | NO | NO | SI | NO | Input-only |
| A5 | GPIO39 | N/A | `AN5` / `A5` | ADC1 | SI | NO | NO | SI | NO | Input-only |

### 2.2 Regole di lettura della tabella unica
- Se un pin ha sia alias `DCH*` sia alias `AN*`, e' utilizzabile sia in comandi digitali sia analogici.
- `AN0..AN11` sono i 12 canali analogici effettivi del firmware v1.
- `DCH0..DCH11` restano i 12 canali digitali.
- Tutti i pin in tabella sono indirizzabili nel namespace `GPIO:*` tramite token fisico `GPIO<n>`.
- I canali analogici su ADC2 hanno rischio di conflitto con Wi-Fi/BT attivo (policy in sezione 5).
- Livello massimo ammesso sugli ingressi analogici: 3.3V.

### 2.3 PWM / servo v1
Compatibilita' minima:
- PWM/servo CH0 -> D9 -> GPIO13
- PWM/servo CH1 -> D10 -> GPIO5

## 3. Contratto comandi SCPI (v1 R32)

### 3.1 GPIO
Comando primario:
- `GPIO:MODE <ch>,<IN|OUT|ANA|PULLUP>`
- `GPIO:MODE? <ch>`
- `GPIO:CAP? <ch>`

Alias legacy opzionali:
- `DIG:MODE` -> alias di `GPIO:MODE`
- `DIG:MODE?` -> alias di `GPIO:MODE?`

Parsing `<ch>` per GPIO:
- token fisico `GPIO<n>` (primario, consigliato)
- label `D2..D13`
- label `A0..A5`
- alias `DCH0..DCH11`
- alias `AN0..AN11`
- indice digitale legacy `0..11` (solo alias di `DCH*`)

Nota:
- in v1 `GPIO:*` indirizza il pin fisico (namespace GPIO reale), non solo il namespace digitale.
- esempio normativo: `GPIO:CAP? GPIO35` deve rispondere `IN,ANA`.
- esempio normativo: `GPIO:MODE GPIO35,ANA` e' valido; `GPIO:MODE GPIO35,OUT` -> `-221`.

Regole:
- modalita' non supportata per quel pin -> `-221`.
- parametro fuori range / sintassi errata -> `-222`.
- `PULLUP` su R32 e' supportato solo sui pin che hanno `PULLUP=SI` in tabella 2.1.
- quando `mode=ANA` su pin ADC2, invocare stub radio (sezione 4).

### 3.2 Misure analogiche
Comandi analogici operano su `AN0..AN11`.

- `CONF:VOLT <ach>` / `CONF:VOLT?`
- `MEAS:VOLT? [ach]`
- `MEAS:RAW? <ach>`
- `MEAS:VOLT:ALL?` (ritorna 12 valori in ordine `AN0..AN11`)
- `ROUT:SCAN (@...)` / `ROUT:SCAN?`
- `READ?`

Parsing `<ach>`:
- indice `0..11`
- token espliciti: `AN0..AN11`
- token pin analogici: `A0..A5`
- token pin digitali analogici: `D2,D3,D6,D7,D8,D9`
- token fisici `GPIO<n>` (solo se il pin ha `ANA=SI`)

Formato `ROUT:SCAN`:
- lista: `(@0,2,5,8)`
- range: `(@0:11)`
- misto token: `(@A2,D8,AN11)`

### 3.3 Trigger
- `TRIG:SOUR <IMM|ANA|DIG>`
- `TRIG:CHAN <ch>`

Regole `TRIG:CHAN`:
- con `TRIG:SOUR ANA`, `<ch>` deve essere un canale analogico valido (`AN0..AN11` o token equivalente).
- con `TRIG:SOUR DIG`, `<ch>` deve essere canale digitale (`0..11` o `D2..D13`).
- con `TRIG:SOUR IMM`, `TRIG:CHAN` non applicabile -> `-221`.

### 3.4 Acquisizione (`INIT`/`FETC?`)
L'acquisizione usa la lista `ROUT:SCAN` su canali analogici `AN*`.

Regole preflight obbligatorie in `INIT` (e anche in `READ?`):
- ogni canale della scan list deve essere analogicamente disponibile;
- se radio lock su ADC2 attivo e la scan list contiene canali ADC2 -> `-221`;
- se trigger analogico configurato su canale ADC2 con radio lock attivo -> `-221`.

Comportamento scelto: fail-fast deterministico.
- Nessun downsample automatico.
- Nessun "skip" silenzioso dei canali.

## 4. Stub radio Wi-Fi/BT (v1)

### 4.1 Scopo
Lo stub non abilita/disabilita realmente radio in v1. Definisce pero' il contratto per la gestione futura di contese ADC2.

### 4.2 Hook richiesti
- `onGpioModeChange(gpioNum, mode)`
- `onAnalogOperationRequested(ach, operation)`
- `wifiBtArbiterStubRequestAdc2(resourceId)`
- `wifiBtArbiterStubIsAdc2Locked()` -> bool

`resourceId` puo' essere:
- singolo canale (`ANx`)
- contesto scan (`SCAN`)
- contesto trigger (`TRIG_ANA`)

### 4.3 Stato minimo stub
- `radioMode`: `OFF|WIFI|BT|COEX|UNKNOWN` (v1 default: `OFF`)
- `adc2Locked`: `false|true` (v1 default: `false`)
- ultimo motivo lock (debug interno)

## 5. Politica errori (coerente su tutto il firmware)

- `-222 Data out of range`:
  - canale non valido
  - valore numerico fuori range
  - token malformato
- `-221 Settings conflict`:
  - modalita' GPIO non supportata sul pin
  - `DIG:OUT` su pin non in `OUT`
  - uso ADC2 quando `adc2Locked=true` (misura, trigger analogico, scan/acquisizione)
- `-200 Execution error`:
  - operazione non possibile per stato interno non coerente
- `-250 Timeout error`:
  - timeout trigger/acquisizione

## 6. Impatti su architettura software

File proposti:
- `src/main.cpp`
- `src/scpi_parser.cpp/.h`
- `src/scpi_commands_core.cpp`
- `src/scpi_commands_gpio.cpp`
- `src/scpi_commands_analog.cpp`
- `src/scpi_commands_trigger.cpp`
- `src/scpi_commands_acq.cpp`
- `src/hal_board_d1_uno32.h`
- `src/hal_adc_esp32.cpp`
- `src/hal_pwm_esp32.cpp`
- `src/radio_arbiter_stub_esp32.cpp/.h`

Principio chiave:
- parser SCPI separato dalla logica hardware/radio;
- tutte le verifiche capability e ADC2 lock in un layer unico, riusato da MEAS/READ/TRIG/INIT.

## 7. Test di accettazione

### 7.1 GPIO
- `GPIO:MODE` e `GPIO:CAP?` su tutti i pin fisici in tabella (`GPIO2,4,5,12,13,14,16,17,18,19,23,25,26,27,34,35,36,39`).
- tentativi invalidi (`ANA` su pin non analogico) -> `-221`.
- verifica `GPIO:CAP?` coerente con tabella 2.1.
- caso esplicito: `GPIO:CAP? GPIO35` -> `IN,ANA`.

### 7.2 Analogico
- `MEAS:*` e `READ?` su tutti i `AN0..AN11`.
- `MEAS:VOLT:ALL?` restituisce 12 campi.
- `ROUT:SCAN (@0:11)` funziona con radio lock disattivo.

### 7.3 Trigger/acquisizione con policy ADC2
- `TRIG:SOUR ANA` su canale ADC1 con `adc2Locked=true` -> consentito.
- `TRIG:SOUR ANA` su canale ADC2 con `adc2Locked=true` -> `-221`.
- `INIT` con scan mista ADC1+ADC2 e `adc2Locked=true` -> `-221`.
- `INIT` con sola scan ADC1 e `adc2Locked=true` -> consentito.

### 7.4 PWM/servo
- regressione PWM/servo su D9/D10.

## 8. Decisioni aperte

1. Alias legacy:
   - opzione A: mantenere `DIG:MODE` come alias indefinito.
   - opzione B: mantenerlo per una release e poi deprecarlo.
2. `CAL:REF` su ESP32:
   - opzione A: alias software.
   - opzione B: comando deprecato con errore esplicito e sostituzione con `CONF:ADC:*`.
3. Limite iniziale `MAX_TOTAL_POINTS` su ESP32 (proposta: 4096 raw totali).
4. Fase 2: verifica mapping e note elettriche contro documentazione ufficiale WEMOS D1 R32 + core `d1_uno32`.
