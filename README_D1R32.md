# WEMOS D1 R32 SCPI

Manuale operativo per lo sketch [WEMOS_D1_R32_SCPI.ino](WEMOS_D1_R32_SCPI/WEMOS_D1_R32_SCPI.ino).

Documenti collegati:

- [SPEC_WEMOS_D1_R32.md](SPEC_WEMOS_D1_R32.md)
- [SPEC_WIFI_D1_R32.md](SPEC_WIFI_D1_R32.md)
- [TEST_PLAN_WIFI_D1_R32.md](TEST_PLAN_WIFI_D1_R32.md)

## 1. Panoramica

Questo firmware porta OpenSCPI-Lab su **WEMOS D1 R32 / ESP32** sfruttando funzioni native della piattaforma:

- GPIO indirizzabili per nome fisico (`GPIO25`, `GPIO35`, ...)
- ingressi analogici estesi (`AN0..AN11`)
- uscite PWM su molti pin di output
- DAC hardware su `GPIO25` e `GPIO26`
- trigger analogico e digitale
- acquisizione bufferizzata tipo oscilloscopio

Identificazione firmware:

```text
*IDN?
OpenSCPI-Lab,WEMOS-D1-R32,0.1-ALPHA
```

## 2. Pin importanti

### LED integrato

- `LED_BUILTIN = GPIO2`
- sulla board `GPIO2` coincide anche con `A0` / `AN0`

Esempio:

```text
GPIO:MODE GPIO2,OUT
DIG:OUT GPIO2,1
DIG:OUT GPIO2,0
```

### DAC hardware

- `DAC1 = GPIO25`
- `DAC2 = GPIO26`
- risoluzione DAC: `8 bit`
- range comando: `0..255`
- tensione massima pratica: circa `0..3.3 V`

Formula pratica:

```text
Vout ~= 3.3 * value / 255
```

### ADC analogici

Il firmware espone 12 canali analogici:

| Canale | Pin |
| --- | --- |
| `AN0` | `GPIO2` / `A0` |
| `AN1` | `GPIO4` / `A1` |
| `AN2` | `GPIO35` / `A2` |
| `AN3` | `GPIO34` / `A3` |
| `AN4` | `GPIO36` / `A4` |
| `AN5` | `GPIO39` / `A5` |
| `AN6` | `GPIO26` / `D2` |
| `AN7` | `GPIO25` / `D3` |
| `AN8` | `GPIO27` / `D6` |
| `AN9` | `GPIO14` / `D7` |
| `AN10` | `GPIO12` / `D8` |
| `AN11` | `GPIO13` / `D9` |

Nota:

- `GPIO34`, `GPIO35`, `GPIO36`, `GPIO39` sono input-only
- per misure più tranquille conviene usare `GPIO34/35/36/39` perché sono `ADC1`
- i pin `ADC2` possono entrare in conflitto con Wi-Fi

## 3. Sintassi canali

Il firmware accetta più forme di indirizzamento.

Per i GPIO:

- `GPIO25`
- `D2..D13`
- `A0..A5`
- `DCH0..DCH11`
- `AN0..AN11`

Per i canali analogici:

- `AN0..AN11`
- `A0..A5`
- `GPIO<n>` se il pin è analogico

Esempio:

```text
GPIO:CAP? GPIO35
GPIO:MODE GPIO35,ANA
MEAS:MVOLT? GPIO35
```

## 4. Comandi principali

### Sistema

```text
*IDN?
*RST
*OPC?
*CLS
SYST:ERR?
SYST:ACK ON
SYST:ACK OFF
SYST:ACK?
SYST:CAP?
SYST:PINMAP?
```

### GPIO

Capability e modalità:

```text
GPIO:CAP? GPIO35
GPIO:MODE GPIO35,ANA
GPIO:MODE? GPIO35
```

Modalità disponibili:

- `IN`
- `OUT`
- `PULLUP`
- `ANA`

I comandi legacy `DIG:MODE` e `DIG:MODE?` sono accettati come alias.

I/O digitale:

```text
DIG:OUT GPIO2,1
DIG:OUT GPIO2,0
DIG:OUT? GPIO2
DIG:IN? GPIO35
```

### Analogico

```text
CONF:VOLT AN2
CONF:VOLT?
MEAS:RAW? GPIO35
MEAS:MVOLT? GPIO35
MEAS:VOLT? GPIO35
MEAS:VOLT:ALL?
```

ADC resolution:

```text
CONF:ADC:RES 12
CONF:ADC:RES?
```

### DAC

```text
SOUR:DAC DAC1,128
SOUR:DAC? DAC1
SOUR:DAC GPIO25,255
SOUR:DAC? GPIO25
```

Token validi:

- `DAC1`, `DAC2`
- `GPIO25`, `GPIO26`
- `D2`, `D3`

### PWM

PWM esteso su pin di output:

```text
GPIO:MODE GPIO26,OUT
SOUR:PWM:FREQ GPIO26,500
SOUR:PWM GPIO26,128
SOUR:PWM? GPIO26
SOUR:PWM:FREQ? GPIO26
```

Nota:

- il firmware supporta anche i canali legacy `0` e `1`, corrispondenti a `GPIO13` e `GPIO5`
- per uso nuovo conviene indirizzare direttamente il pin fisico

### Servo

Compatibilità minima sui due canali legacy:

```text
GPIO:MODE GPIO13,OUT
SOUR:SERVO 0,90
SOUR:SERVO? 0
SOUR:SERVO:ATT? 0
```

### Trigger

```text
TRIG:SOUR IMM
TRIG:SOUR ANA
TRIG:SOUR DIG
TRIG:SOUR?
TRIG:SLOP POS
TRIG:SLOP NEG
TRIG:SLOP?
TRIG:CHAN GPIO35
TRIG:CHAN?
TRIG:LEV 1.65
TRIG:LEV?
TRIG:TOUT 2000
TRIG:TOUT?
```

### Acquisizione

```text
ROUT:SCAN (@GPIO35)
ROUT:SCAN (@AN2,AN3)
ROUT:SCAN?
READ?

ACQ:POIN 500
ACQ:POIN?
ACQ:TINT 100
ACQ:TINT?
ACQ:STAT?
INIT
ABOR
FETC?
```

Valori attuali nel firmware:

- buffer massimo totale: `4096` campioni raw
- default `ACQ:POIN = 64`
- default `ACQ:TINT = 1000 us`

### Wi-Fi

Comandi attualmente disponibili:

```text
SYST:WIFI:ON
SYST:WIFI:OFF
SYST:WIFI:STAT?
SYST:WIFI:SCAN?
SYST:WIFI:JOIN "iPhone (8)","testR32."
SYST:WIFI:DISC
SYST:WIFI:IP?
SYST:WIFI:RSSI?
SYST:WIFI:DBG:STAT?
SYST:WIFI:DBG:SSID?
SYST:WIFI:DBG:FAIL?
SYST:WIFI:DBG:DIAG?
```

Stati osservabili:

- `OFF`: radio spenta
- `IDLE`: radio attiva, non connessa
- `SCANNING`: scansione in corso
- `CONNECTED`: rete associata e IP valido

Note:

- SSID e password con spazi o parentesi sono supportati se quotati
- `SYST:WIFI:OFF` libera subito i canali `ADC2`

Comportamento GPIO con `ADC2` e radio attiva:

```text
GPIO:MODE GPIO26,ANA
SYST:ERR?
GPIO:MODE? GPIO26
```

Risposte attese:

- `GPIO:MODE GPIO26,ANA` -> `ERR`
- `SYST:ERR?` -> `-221,"Settings conflict"`
- se il pin era gia' in analogico quando il Wi-Fi viene acceso: `GPIO:MODE? GPIO26` -> `ANA,NAVAIL,RADIO`

## 5. Esempi pratici

### LED integrato

```text
GPIO:MODE GPIO2,OUT
DIG:OUT GPIO2,1
DIG:OUT GPIO2,0
```

### Rampa DAC misurata da ADC

Collegamento:

```text
GPIO25 -> GPIO35
```

Sequenza:

```text
GPIO:MODE GPIO25,OUT
GPIO:MODE GPIO35,ANA
SOUR:DAC GPIO25,0
MEAS:MVOLT? GPIO35
SOUR:DAC GPIO25,128
MEAS:MVOLT? GPIO35
SOUR:DAC GPIO25,255
MEAS:MVOLT? GPIO35
```

### PWM campionato come onda quadra

Collegamento:

```text
GPIO26 -> GPIO35
```

Sequenza:

```text
GPIO:MODE GPIO26,OUT
GPIO:MODE GPIO35,ANA
SOUR:PWM:FREQ GPIO26,100
SOUR:PWM GPIO26,128
ROUT:SCAN (@GPIO35)
ACQ:POIN 500
ACQ:TINT 100
TRIG:SOUR ANA
TRIG:CHAN GPIO35
TRIG:LEV 1.65
TRIG:SLOP POS
TRIG:TOUT 2000
INIT
FETC?
```

### Join Wi-Fi e diagnostica

```text
SYST:WIFI:ON
SYST:WIFI:SCAN?
SYST:WIFI:JOIN "iPhone (8)","testR32."
SYST:WIFI:STAT?
SYST:WIFI:IP?
SYST:WIFI:RSSI?
SYST:WIFI:DBG:DIAG?
SYST:WIFI:DISC
SYST:WIFI:OFF
```

## 6. Sicurezza e note hardware

- tutti i pin ESP32 sono da usare a `3.3 V`
- non applicare `5 V` diretti agli ingressi
- `GPIO34/35/36/39` sono solo input
- `GPIO25 -> GPIO35` e' un collegamento sicuro per test, perche' `GPIO35` e' solo ingresso
- non collegare mai due uscite pilotate una contro l'altra
- `GPIO2` e' anche LED integrato e anche `A0`
- con `SYST:WIFI:ON` i pin `ADC2` sono bloccati per uso analogico
- con `SYST:WIFI:OFF` i pin `ADC2` tornano disponibili

## 7. Stato attuale

Firmware testato con successo in modo pratico su board:

- identificazione SCPI
- accensione/spegnimento LED integrato
- DAC su `GPIO25`
- misura analogica della rampa DAC su `GPIO35`
- acquisizione di un PWM campionato su ingresso analogico
- scansione Wi-Fi, join/disconnect e diagnostica base
- blocco `ADC2` con Wi-Fi attivo e rilascio immediato con `SYST:WIFI:OFF`

Questo documento descrive lo stato reale dello sketch corrente, non una roadmap futura.
