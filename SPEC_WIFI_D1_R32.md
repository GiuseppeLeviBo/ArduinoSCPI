# Specifica Wi-Fi v1 per WEMOS D1 R32

## 1. Obiettivo

Definire una prima architettura Wi-Fi sensata per un device edge SCPI basato su ESP32, con priorita' assoluta a:

- recuperabilita' via seriale USB
- comportamento non bloccante al boot
- controllo remoto semplice e robusto
- separazione chiara tra rete, SCPI e risorse hardware locali

## 2. Principi non negoziabili

### 2.1 Priorita' canali di controllo

Ordine di priorita':

1. seriale USB
2. SCPI via TCP
3. canali futuri come MQTT

Regola:

- un comando ricevuto via seriale deve sempre essere accettato ed eseguito
- la seriale deve poter sempre spegnere o riconfigurare il Wi-Fi
- una sessione remota non deve mai rendere inaccessibile la scheda

### 2.2 Boot e recovery

Il boot deve essere sempre non bloccante:

- il parser seriale deve essere attivo subito
- il Wi-Fi deve connettersi in background
- un fallimento di connessione non deve impedire l'uso locale
- deve esistere sempre un percorso di recovery completo via seriale

### 2.3 Politica ADC2

Con Wi-Fi attivo:

- i canali ADC2 sono vietati
- misure, trigger e acquisizioni su ADC2 devono fallire con errore chiaro
- nessun fallback implicito, nessun comportamento ambiguo

Scelta di progetto:

- `ADC2 forbidden while Wi-Fi active`

## 3. Scope funzionale v1

### 3.1 Funzioni incluse

- accensione/spegnimento esplicito del Wi-Fi
- scansione reti Wi-Fi
- join a rete `STA`
- disconnessione
- stato rete
- hostname configurabile
- persistenza credenziali
- reconnect automatico non bloccante
- server TCP SCPI
- arresto del server TCP via seriale
- forget delle credenziali via seriale

### 3.2 Funzioni escluse da v1

- captive portal
- modalità AP di provisioning
- MQTT
- Telegram bot
- OTA
- TLS complesso

Queste restano successive.

## 4. Modello operativo

### 4.1 Stati runtime

Stati Wi-Fi:

- `OFF`
- `IDLE`
- `SCANNING`
- `CONNECTING`
- `CONNECTED`
- `ERROR`

Stati server SCPI TCP:

- `STOPPED`
- `LISTEN`
- `CLIENT_CONNECTED`

### 4.2 Risorse persistenti

Da salvare in NVS:

- SSID
- password
- flag auto-connect
- hostname
- porta server SCPI TCP

Da non salvare:

- stato client TCP corrente
- IP ottenuto da DHCP
- errori runtime temporanei

## 5. Contratto SCPI proposto

### 5.1 Stato e scansione

- `SYST:WIFI:ON`
- `SYST:WIFI:OFF`
- `SYST:WIFI:STAT?`
- `SYST:WIFI:SCAN?`
- `SYST:WIFI:RSSI?`
- `SYST:WIFI:IP?`
- `SYST:WIFI:MAC?`
- `SYST:WIFI:HOST?`

Formati:

- `SYST:WIFI:STAT?` -> `OFF|IDLE|SCANNING|CONNECTED`
- `SYST:WIFI:SCAN?` -> elenco reti in formato CSV o una riga per rete

Formato consigliato per `SCAN?`:

```text
SSID,RSSI,AUTH,CHAN
```

Semantica di base:

- `SYST:WIFI:ON` abilita lo stack Wi-Fi ma non implica da solo una connessione riuscita
- `SYST:WIFI:OFF` spegne il Wi-Fi runtime e libera immediatamente i canali ADC2
- `SYST:WIFI:STAT?` espone lo stato runtime del sottosistema Wi-Fi

### 5.2 Configurazione e connessione

- `SYST:WIFI:HOST <name>`
- `SYST:WIFI:JOIN <ssid>,<password>`
- `SYST:WIFI:DISC`
- `SYST:WIFI:SAVE <ON|OFF>`
- `SYST:WIFI:FORGET`

Semantica:

- `SYST:WIFI:ON` e' il prerequisito logico per `SCAN?`, `JOIN` e server TCP
- `SYST:WIFI:OFF` forza:
  - disconnessione
  - stop del server TCP SCPI
  - rilascio del vincolo ADC2
- `JOIN` tenta la connessione immediata
- se `SAVE=ON`, le credenziali vengono memorizzate
- `FORGET` cancella le credenziali memorizzate
- `DISC` disconnette ma non implica necessariamente `FORGET`

### 5.3 Diagnostica minima

- `SYST:WIFI:DBG:STAT?`
- `SYST:WIFI:DBG:SCAN:LAST?`
- `SYST:WIFI:DBG:SSID?`
- `SYST:WIFI:DBG:FAIL?`
- `SYST:WIFI:DBG:DIAG?`

### 5.4 Server SCPI su TCP

- `SYST:NET:SCPI:PORT <n>`
- `SYST:NET:SCPI:PORT?`
- `SYST:NET:SCPI:STAT?`
- `SYST:NET:SCPI:START`
- `SYST:NET:SCPI:STOP`

Scelte v1:

- una sola sessione client attiva alla volta
- protocollo TCP plain text
- stessa sintassi SCPI della seriale

### 5.4 Recovery e safe mode

- `SYST:SAFE ON`
- `SYST:SAFE OFF`
- `SYST:FACT:NET`

Semantica:

- `SYST:SAFE ON` spegne radio e servizi remoti, lasciando intatta la seriale
- `SYST:SAFE OFF` riabilita il comportamento normale
- `SYST:FACT:NET` resetta solo la configurazione di rete

## 6. Regole di arbitraggio

### 6.1 Seriale vs TCP

La seriale ha sempre precedenza.

Conseguenze:

- un comando seriale `SYST:WIFI:OFF` deve essere sempre eseguito
- un comando seriale puo' fermare il server TCP
- un comando seriale puo' forzare `DISC` o `FORGET`
- il parser seriale non deve aspettare lock detenuti dal layer Wi-Fi

### 6.2 Sessioni TCP

In v1:

- un solo client TCP alla volta
- nuovi client vengono rifiutati se un altro e' connesso
- timeout inattivita' configurabile in release successive

## 7. Interazione con analogico, trigger e acquisizione

### 7.1 Regola globale

Se `SYST:WIFI:ON` e il Wi-Fi runtime e' attivo:

- tutti i canali ADC2 sono vietati

Questo si applica a:

- `MEAS:*`
- `READ?`
- `ROUT:SCAN`
- `TRIG:SOUR ANA`
- `TRIG:CHAN`
- `INIT`
- `FETC?`

Se `SYST:WIFI:OFF`:

- i canali ADC2 tornano utilizzabili
- il server TCP SCPI non deve essere disponibile

### 7.2 Canali consentiti con Wi-Fi attivo

Consentiti:

- `GPIO34`, `GPIO35`, `GPIO36`, `GPIO39`
- quindi `AN2`, `AN3`, `AN4`, `AN5`

Vietati:

- `AN0`, `AN1`, `AN6..AN11`

### 7.3 Politica errori

Se viene richiesto un canale ADC2 con Wi-Fi attivo:

- `GPIO:MODE GPIO<n>,ANA` deve fallire immediatamente con `-221`
- risposta errore `-221,"Settings conflict"`

Se un pin ADC2 era gia' in `ANA` e poi la radio viene accesa:

- `GPIO:MODE? GPIO<n>` deve indicare `ANA,NAVAIL,RADIO`

Nessun tentativo automatico di:

- cambiare pin
- mascherare il canale
- continuare una scan parziale

## 8. Architettura software proposta

File/moduli suggeriti:

- `wifi_manager_esp32.cpp/.h`
- `scpi_commands_wifi.cpp`
- `scpi_commands_net.cpp`
- `scpi_transport_serial.cpp`
- `scpi_transport_tcp.cpp`
- `settings_store_nvs.cpp/.h`
- `radio_guard_adc2.cpp/.h`

Principi:

- parser SCPI unico
- trasporti separati
- stato rete centralizzato
- policy ADC2 centralizzata

## 9. Sequenza di boot desiderata

1. init seriale
2. init parser SCPI
3. carico configurazione da NVS
4. se auto-connect attivo, avvio connessione Wi-Fi in background
5. se configurato, avvio server TCP quando IP disponibile
6. in ogni caso la seriale resta sempre operativa

## 10. Test di accettazione v1

### 10.1 Rete base

- `SYST:WIFI:ON` porta la radio in stato attivo senza bloccare la seriale
- `SYST:WIFI:OFF` spegne la radio senza bloccare la seriale
- scan reti disponibile
- join a rete valida
- join a rete invalida con errore controllato
- disconnect
- reconnect dopo reboot con credenziali salvate

### 10.2 Recovery

- mentre il Wi-Fi e' attivo, un comando seriale `SYST:WIFI:OFF` deve funzionare sempre
- mentre il Wi-Fi e' attivo, un comando seriale `DISC` deve funzionare sempre
- mentre il server TCP e' attivo, `SYST:NET:SCPI:STOP` via seriale deve funzionare sempre
- `FORGET` via seriale deve ripristinare il boot locale senza auto-connect

### 10.3 ADC2 policy

- con Wi-Fi attivo, `MEAS:MVOLT? GPIO35` deve funzionare
- con Wi-Fi attivo, `GPIO:MODE GPIO26,ANA` deve fallire con `-221`
- con Wi-Fi attivo, `MEAS:MVOLT? GPIO25` deve fallire con `-221`
- con Wi-Fi attivo, `INIT` su scan con `GPIO35` deve funzionare
- con Wi-Fi attivo, `INIT` su scan con `GPIO26` deve fallire con `-221`

## 11. Roadmap oltre v1

### P2

- AP fallback
- captive portal
- MQTT publish/subscribe
- NTP

### P3

- OTA
- webhook
- bridge Telegram esterno

Nota di prodotto:

- Telegram bot diretto on-device non e' priorita' v1
- MQTT ha piu' senso di Telegram come prima integrazione remota
