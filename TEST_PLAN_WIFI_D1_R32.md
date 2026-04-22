# Piano test Wi-Fi per WEMOS D1 R32

## 1. Obiettivo

Definire una batteria di test ripetibile per il futuro firmware Wi-Fi della WEMOS D1 R32, con attenzione a:

- recovery via seriale
- affidabilita' del boot
- correttezza della policy ADC2
- controllo remoto SCPI via TCP

## 2. Setup minimo

Hardware:

- WEMOS D1 R32
- cavo USB
- PC con `arduino-cli`
- rete Wi-Fi di test

Setup opzionale per test analogici:

- ponticello `GPIO26 -> GPIO35`
- ponticello `GPIO25 -> GPIO35`

## 3. Macro di test da automatizzare

### T01 - Boot locale

Passi:

- alimenta la board
- invia `*IDN?` via seriale

Atteso:

- risposta entro timeout
- nessun blocco dovuto al Wi-Fi

### T02 - WIFI ON/OFF base

Passi:

- invia `SYST:WIFI:ON`
- interroga `SYST:WIFI:STAT?`
- invia `SYST:WIFI:OFF`
- interroga `SYST:WIFI:STAT?`

Atteso:

- risposta `IDLE` o `CONNECTED` dopo `SYST:WIFI:ON`
- risposta `OFF` dopo `SYST:WIFI:OFF`
- seriale sempre responsiva

### T03 - Scan reti

Passi:

- invia `SYST:WIFI:ON`
- invia `SYST:WIFI:SCAN?`

Atteso:

- elenco reti
- formato coerente
- nessun riavvio o freeze

### T04 - Join rete valida

Passi:

- invia `SYST:WIFI:ON`
- invia `SYST:WIFI:JOIN <ssid>,<pw>`
- interroga `SYST:WIFI:STAT?`
- interroga `SYST:WIFI:IP?`

Atteso:

- stato finale `CONNECTED`
- IP valido

### T05 - Join rete invalida

Passi:

- `SYST:WIFI:ON`
- invia join con password errata

Atteso:

- stato `ERROR` o equivalente
- seriale ancora responsiva

### T06 - Persistenza credenziali

Passi:

- `SYST:WIFI:ON`
- `SYST:WIFI:SAVE ON`
- join rete valida
- reboot
- interroga stato via seriale

Atteso:

- reconnect automatico
- seriale disponibile anche durante il reconnect

### T07 - Forget credenziali

Passi:

- `SYST:WIFI:FORGET`
- reboot

Atteso:

- nessun auto-connect
- boot locale pulito

### T08 - Recovery da seriale

Passi:

- Wi-Fi connesso
- server TCP attivo
- invia via seriale `SYST:WIFI:OFF`
- interroga `SYST:WIFI:STAT?`
- invia via seriale `SYST:NET:SCPI:STOP`
- invia via seriale `SYST:WIFI:DISC`

Atteso:

- `SYST:WIFI:OFF` ha effetto immediato
- entrambe le azioni eseguite subito
- seriale sempre funzionante

### T09 - ADC1 consentito con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- `GPIO:MODE GPIO35,ANA`
- `MEAS:MVOLT? GPIO35`

Atteso:

- misura valida

### T10 - ADC2 vietato con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- `GPIO:MODE GPIO26,ANA`
- `SYST:ERR?`

Atteso:

- `GPIO:MODE GPIO26,ANA` -> `ERR`
- `SYST:ERR?` -> `-221`

### T11 - ADC2 riabilitato con WIFI OFF

Passi:

- `SYST:WIFI:OFF`
- `GPIO:MODE GPIO26,ANA`
- `MEAS:MVOLT? GPIO26`

Atteso:

- misura valida

### T12 - Trigger analogico ADC1 con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- imposta trigger analogico su `GPIO35`

Atteso:

- configurazione valida

### T13 - Trigger analogico ADC2 con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- prova trigger analogico su `GPIO26`

Atteso:

- errore `-221`

### T14 - Acquisizione ADC1 con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- scan su `GPIO35`
- `INIT`
- `FETC?`

Atteso:

- acquisizione valida

### T15 - Acquisizione ADC2 con Wi-Fi attivo

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- scan su `GPIO26`
- `INIT`

Atteso:

- errore `-221`

### T16 - Server TCP SCPI

Passi:

- `SYST:WIFI:ON`
- connetti Wi-Fi
- avvia server TCP
- apri client da PC
- invia `*IDN?`

Atteso:

- risposta corretta
- stesso comportamento SCPI della seriale

### T17 - Arbitraggio seriale > TCP

Passi:

- client TCP connesso
- invia comando da seriale che modifica stato rete

Atteso:

- il comando seriale ha effetto
- la scheda non resta “presa” dal client remoto

## 4. Matrice rapida atteso/fallimento

| Caso | SYST:WIFI:OFF | SYST:WIFI:ON |
| --- | --- | --- |
| ADC1 | OK | OK |
| ADC2 | OK | FAIL `-221` |
| Seriale | OK | OK |
| TCP SCPI | OFF/STOPPED o opzionale | OK |

## 5. Automazione consigliata

Script da preparare:

- `scripts/build_upload.ps1`
- `scripts/scpi_serial_send.ps1`
- `scripts/scpi_tcp_send.ps1`
- `scripts/test_wifi_smoke.ps1`
- `scripts/test_adc2_guard.ps1`

Output atteso:

- log test in file testo
- esito `PASS/FAIL`
- tempi principali

## 6. Criteri di accettazione v1

La v1 Wi-Fi e' accettata se:

- la seriale resta sempre utilizzabile
- il boot non si blocca
- il join/disconnect funzionano
- il server SCPI TCP risponde
- la policy ADC2 e' coerente e sempre rispettata
- `FORGET` e `DISC` permettono sempre il recupero del device
