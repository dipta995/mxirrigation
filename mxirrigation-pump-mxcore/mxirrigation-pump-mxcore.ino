/**
 * ============================================================================
 * Progetto:    MxIrrigation - Pump Controller
 * File:        mxirrigation-pump-mxcore.ino
 * Autore:      Nicola Deboni - MxSolutions
 * Versione:    1.6.0
 * Ultima mod.: 2026-09-19
 * Repository:  vault "03 - Elettronica/Progetti/mxirrigation/firmware/
 *              mxirrigation-pump-mxcore" -> vedi [[pump-controller]]
 * ----------------------------------------------------------------------------
 * Descrizione:
 *   Controllore della stazione di pompaggio: master + 2 pompe + relè di
 *   pre-stadio, sensore di pressione analogico, 4 modalità di avvio
 *   sequenziate non bloccanti, protezioni di alta e bassa pressione, web UI.
 *
 *   Porting della 1.3.0 su MxSolutionCore 0.1.4, con quattro aggiunte
 *   richieste: pagina di configurazione, ping non bloccante, registrazione
 *   della pressione solo a pompe avviate, IP fisso impostabile dal portale.
 *
 *   La logica pompe (sequenziatore, blocco comandi, trip) è quella della
 *   1.3.0, invariata nei tempi e nell'ordine dei relè.
 *
 * Hardware:
 *   Scheda:      ESP32 classico
 *   Periferiche: scheda 4 relè, sensore di pressione analogico, LED di stato
 *
 *   Pin:
 *     GPIO   direzione   funzione                   note
 *     21     OUT         Relè 1 - MASTER            attivo alto
 *     19     OUT         Relè 2 - POMPA 1           attivo alto
 *     18     OUT         Relè 3 - POMPA 2           attivo alto
 *     5      OUT         Relè 4 - pre-stadio        attivo alto; pin di strapping
 *     25     OUT         LED di stato               acceso = rete su,
 *                                                   lampeggio = rete giù
 *     36     IN (ADC1)   Sensore di pressione       solo ingresso, nessuna
 *                                                   pull-up. ADC1: continua a
 *                                                   funzionare con il WiFi
 *                                                   acceso (ADC2 no)
 *     0      IN pull-up  Reset di fabbrica (BOOT)   a massa >= 3 s all'avvio;
 *                                                   cancella "mxnet", non "mxid"
 *
 * Arduino IDE / arduino-cli:
 *   Board:            ESP32 Dev Module / esp32:esp32:esp32
 *   Core version:     esp32 3.3.11
 *   Partition Scheme: Minimal SPIFFS (min_spiffs) - obbligatorio
 *   Flash:            QIO 80MHz, 4MB
 *
 *   Librerie (nome - versione esatta testata):
 *     MxSolutionCore  - 0.1.4
 *     ESPping         - (quella già in uso dalla 1.3.0)
 *
 * Note di versione:
 *   2026-09-20 1.6.0 - Le CINQUE temporizzazioni della sequenza di avvio non
 *                      sono piu' costanti scritte nel codice: si regolano da
 *                      /config e stanno in NVS. Erano numeri magici inline
 *                      (5000/2000/4000/5000/40000) e il tempo giusto fra due
 *                      pompe si tara sul campo, non a tavolino. Rinominati gli
 *                      stati PSEQ_*_WAIT4S ecc., che incorporavano il valore
 *                      nel nome e sarebbero diventati bugiardi.
 *   2026-09-19 1.5.1 - Il topic ntfy non e' piu' una costante: si ricava dal
 *                      NOME DELLA PISTA (MxIdentity.site(), normalizzato con
 *                      le stesse regole dello slug della libreria). Una pista
 *                      = un canale di notifiche, per tutti i suoi device.
 *                      Senza identita' impostata si ripiega sul topic storico
 *                      e lo si dichiara in /info e nel log.
 *   2026-09-19 1.5.0 - NUOVO: notifica ntfy all'avvio. A boot concluso manda
 *                      data/ora, CAUSA DEL RIAVVIO (esp_reset_reason) e stato
 *                      completo: pompe, pressione, soglie, ping, IP, identita'.
 *                      Non parte da setup() ma da loop(), perche' al termine
 *                      del setup l'ora NTP non c'e' ancora e la notifica
 *                      direbbe 01/01/1970. Attende rete+ora fino a 90 s, poi
 *                      manda comunque dichiarando l'ora non disponibile.
 *                      Attivabile da /config (default ON, chiave NVS bootNtfy)
 *                      e provabile da seriale con "boot ntfy" senza riavviare.
 *   2026-09-19 1.4.2 - FIX: il grafico di /pressure non si disegnava MAI. La
 *                      riga ctx.fillText('piu' vecchio', ...) era un errore di
 *                      sintassi JavaScript (l'apostrofo chiudeva la stringa),
 *                      quindi il browser scartava tutto il blocco <script> e
 *                      draw() non veniva mai chiamata: canvas bianco con i dati
 *                      regolarmente presenti nella pagina. Ora la stringa usa
 *                      le virgolette doppie.
 *   2026-09-18 1.4.1 - Aggiunto /status JSON (senza auth, sola lettura) per il
 *                      pannello ESP32-2432S028. Le chiavi comuni sono le stesse
 *                      della centralina valvole, così un consumatore legge i due
 *                      nodi con lo stesso codice e distingue dal campo "type".
 *   2026-09-18 1.4.0 - Porting su MxSolutionCore 0.1.4 + quattro aggiunte.
 *                      [1] Pagina /config: soglie di allarme alta/bassa in bar,
 *                          tempi di attesa prima del trip, taratura del sensore
 *                          (coppia valore-sensore / pressione), host e
 *                          abilitazione del ping. Tutto in NVS "pumpcfg", con
 *                          clamp dei valori fuori range invece di rifiuto.
 *                      [2] Ping non bloccante: era Ping.ping() chiamata dal
 *                          loop(), che lo fermava per tutto il timeout ICMP.
 *                          Ora gira in un task FreeRTOS dedicato e il loop()
 *                          legge solo il risultato.
 *                      [3] Registrazione della pressione SOLO a pompe avviate.
 *                          Il sensore continua a essere letto ogni secondo (i
 *                          trip e il valore a video servono sempre), ma nello
 *                          storico entrano solo i campioni con le pompe in
 *                          moto, separati da un marcatore di interruzione.
 *                      [4] IP fisso impostabile dal captive portal; il
 *                          192.168.5.43 qui sotto è solo il default di fabbrica.
 *                      Dalla libreria arrivano anche watchdog, identità in NVS
 *                      e OTA (LAN + pull mxota), che la 1.3.0 non aveva.
 *                      Tolti: setupWiFi/ensureWiFiConnected, la gestione NTP a
 *                      mano e le credenziali WiFi nel sorgente.
 *   2026-08-30 1.3.0 - /info separata, blocco comandi a pompe accese, banner
 *                      seriale corretto. Mai flashata.
 *              1.2.2 - baseline. Sul dispositivo gira ancora la 1.2.1, di cui
 *                      non esiste il sorgente.
 * ============================================================================
 */

#include <MxCore.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <ESPping.h>
#include <esp_system.h>   // esp_reset_reason(): perche' e' ripartito

const char* FW_VERSION = "1.6.0";

// =========================================================================
// CONFIGURAZIONE DI FABBRICA
// =========================================================================

// --- rete ----------------------------------------------------------------
// Le credenziali WiFi NON stanno qui: captive portal "Mx-Setup-XXXX".
// Anche l'IP si cambia dal portale, e quello che si imposta lì VINCE su
// questo, che vale solo finché nessuno ha configurato la rete.
IPAddress STATIC_IP(192, 168, 5, 43);
IPAddress GATEWAY  (192, 168, 5, 1);
IPAddress SUBNET   (255, 255, 255, 0);
IPAddress DNS1     (192, 168, 5, 1);
IPAddress DNS2     (1, 1, 1, 1);

// --- OTA: DA RIEMPIRE PRIMA DEL FLASH ------------------------------------
// --- OTA: endpoint standard della flotta ---------------------------------
// L'URL NON si scrive a mano intero. Il campo "fw" sceglie QUALE binario il
// server consegna: sbagliarlo significa farsi installare il firmware di un
// altro nodo (successo il 20/09/2026 sulle valvole, che chiedevano
// "mxirrigation-pump"). Il campo "id" serve al server solo per sapere chi ha
// chiamato, e viene preso dall'identita' del device A RUNTIME, cosi' lo stesso
// sorgente puo' stare su piu' device senza che si confondano in manage.php.
//
// ATTENZIONE alla password ArduinoOTA: qui va una password NORMALE. NON
// l'hash bcrypt di config.php del server mxota, che e' il segreto del pannello
// di amministrazione e non deve finire nel binario di un device.
const char* OTA_PASSWORD = "$2y$10$2U7O4S/degdXcsmHVfDPY.0cxqgMCBLYDIP3UNWaaU86.sO2Lk.iO";
const char* OTA_HTTP_BASE = "https://mxsolutions.it/mxota/api.php";
const char* OTA_FW_NAME   = "mxirrigation-pump";
const char* OTA_TOKEN     = "wmpUpdate";

// Composto in setup() da buildOtaUrl(): base + fw + id + token.
// MxOta rifiuta un URL oltre i 224 caratteri, quindi si controlla qui.
char otaHttpUrl[224] = "";


// --- auth delle pagine web ----------------------------------------------
#define ENABLE_WEB_PASSWORD 0
const char* WEB_PASSWORD = "1234";
const char* WEB_PW_PARAM = "pw";

// --- ntfy ----------------------------------------------------------------
const char* NTFY_SERVER = "https://ntfy.sh";

// Il topic NON e' fisso: si ricava dal nome della pista (vedi ntfyTopic()).
// Una pista = un canale di notifiche, condiviso da tutti i suoi device.
// Questo resta solo come rete di sicurezza per un device non ancora
// provisionato: e' il topic usato fino alla 1.5.0, cosi' un controller gia'
// installato continua a farsi sentire anche prima di ricevere l'identita'.
const char* NTFY_TOPIC_FALLBACK = "wmp-irrigation";

// --- reset di fabbrica ---------------------------------------------------
#define FACTORY_RESET_PIN 0

// --- hardware ------------------------------------------------------------
const int relayPins[] = {21, 19, 18, 5};
const int numRelays   = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin      = 25;
const int analogPin   = 36;

// --- ping ----------------------------------------------------------------
const unsigned long PING_START_DELAY_MS     = 120000;  // parte 120 s dopo il boot
const unsigned long PING_PERIOD_MS          = 30000;   // ogni 30 s, a pompe spente
const int           PING_FAIL_REBOOT_COUNT  = 8;
const unsigned long PING_REBOOT_COOLDOWN_MS = 300000;  // 5 minuti

// =========================================================================
// TIPI
// =========================================================================
//
// NOTA: enum e struct stanno QUI, prima di qualunque definizione di funzione.
// L'IDE Arduino genera i prototipi automatici e li inserisce subito prima
// della prima funzione del file: se i tipi fossero definiti più in basso, i
// prototipi li userebbero prima che esistano e la compilazione fallirebbe
// con "'StartMode' was not declared in this scope".

enum PumpSequenceState {
  PSEQ_IDLE = 0,
  PSEQ_RELAY4_WAIT,          // relè4 acceso, si aspetta prima del master
  PSEQ_MASTER_WAIT,          // master acceso, si aspetta prima di pompa1
  PSEQ_PUMP1_WAIT,           // pompa1 accesa, si aspetta prima di pompa2
  PSEQ_STAGGERED_PUMP2_WAIT,
  PSEQ_STAGGERED_PUMP2_HOLD
};

enum StartMode {
  START_NONE = 0,
  START_MASTER_BOTH,
  START_MASTER_SOLAR_BOTH,
  START_SINGLE_ONLY,
  START_STAGGERED_AUTO_OFF
};

// =========================================================================
// [1] CONFIGURAZIONE UTENTE (pagina /config, persistita in NVS "pumpcfg")
// =========================================================================
//
// Le soglie sono in CENTIBAR (bar * 100) e non più in valore grezzo del
// sensore: con la taratura configurabile, "5,72 bar" resta la stessa
// pressione anche se il sensore viene ritarato, mentre "raw 1200" no.
//
// I default riproducono esattamente la 1.3.0:
//   raw 1200 con taratura 2095->10,00 bar = 5,72 bar
//   raw  150 con la stessa taratura       = 0,71 bar
//
struct PumpConfig {
  uint16_t maxCb;      // soglia di allarme alta pressione [centibar]
  uint32_t maxDelayS;  // attesa sopra soglia prima del trip [s]
  uint16_t minCb;      // soglia di allarme bassa pressione [centibar]
  uint32_t minDelayS;  // attesa sotto soglia prima del trip [s]
  uint16_t calRaw;     // valore del sensore alla pressione di taratura
  uint16_t calCb;      // pressione corrispondente [centibar]
  bool     pingEnabled;
  char     pingHost[40];
  bool     bootNotify;   // ntfy all'avvio, con ora, causa del reset e stato

  // Temporizzazioni della sequenza di avvio [ms]. Erano numeri scritti nel
  // codice: il tempo giusto fra due pompe dipende dall'impianto (portata,
  // battente, spunto del motore) e si tara sul campo.
  uint32_t seqRelay4Ms;  // relè4 ON -> master ON          (modo "entrambe")
  uint32_t seqMasterMs;  // master ON -> pompa1 ON
  uint32_t seqPump2Ms;   // pompa1 ON -> pompa2 ON         <- fra le due pompe
  uint32_t seqStag2Ms;   // pompa1 ON -> pompa2 ON         (modo sfalsato)
  uint32_t seqStagHoldMs;// quanto resta accesa pompa2     (modo sfalsato)
};

PumpConfig cfgp;

static const PumpConfig CFG_DEFAULT = {
  572,            // 5,72 bar
  20,             // 20 s
  71,             // 0,71 bar
  260,            // 260 s
  2095, 1000,     // taratura: raw 2095 = 10,00 bar
  true,
  "192.168.5.40",
  true,           // notifica di avvio attiva di fabbrica
  5000, 2000, 4000, 5000, 40000   // i valori cablati fino alla 1.5.1
};

// Limiti di clamp. Fuori range si limita, non si rifiuta.
static const uint16_t CFG_CB_MIN     = 10;     // 0,10 bar
static const uint16_t CFG_CB_MAX     = 6000;   // 60,00 bar
static const uint32_t CFG_DELAY_MIN  = 1;      // s
static const uint32_t CFG_DELAY_MAX  = 3600;   // s
// Temporizzazioni: lo zero e' ammesso (relè insieme), il tetto serve solo a
// impedire che una manovra resti appesa per errore di battitura.
static const uint32_t CFG_SEQ_MS_MAX      = 300000;    // 5 min
static const uint32_t CFG_SEQ_HOLD_MS_MIN = 1000;      // 1 s
static const uint32_t CFG_SEQ_HOLD_MS_MAX = 3600000;   // 1 h

static const uint16_t CFG_CALRAW_MIN = 100;
static const uint16_t CFG_CALRAW_MAX = 4095;

Preferences prefs;
static const char* CFG_NS = "pumpcfg";

// Niente template: i prototipi automatici dell'IDE Arduino non ne riportano
// la riga "template <...>" e la compilazione fallisce con "'T' does not name
// a type". Due funzioni normali costano nulla e non hanno il problema.
static uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void cfgClamp(PumpConfig& c) {
  c.calRaw    = clampU16(c.calRaw, CFG_CALRAW_MIN, CFG_CALRAW_MAX);
  c.calCb     = clampU16(c.calCb, CFG_CB_MIN, CFG_CB_MAX);
  c.maxCb     = clampU16(c.maxCb, CFG_CB_MIN, CFG_CB_MAX);
  c.minCb     = clampU16(c.minCb, 0, CFG_CB_MAX);
  c.maxDelayS = clampU32(c.maxDelayS, CFG_DELAY_MIN, CFG_DELAY_MAX);
  c.minDelayS = clampU32(c.minDelayS, CFG_DELAY_MIN, CFG_DELAY_MAX);

  // La soglia bassa deve restare sotto quella alta, altrimenti le due
  // protezioni si contendono lo stesso intervallo e la pompa non parte mai.
  if (c.minCb >= c.maxCb) {
    c.minCb = (c.maxCb > CFG_CB_MIN + 10) ? (uint16_t)(c.maxCb - 10) : 0;
  }
  c.seqRelay4Ms   = clampU32(c.seqRelay4Ms,   0, CFG_SEQ_MS_MAX);
  c.seqMasterMs   = clampU32(c.seqMasterMs,   0, CFG_SEQ_MS_MAX);
  c.seqPump2Ms    = clampU32(c.seqPump2Ms,    0, CFG_SEQ_MS_MAX);
  c.seqStag2Ms    = clampU32(c.seqStag2Ms,    0, CFG_SEQ_MS_MAX);
  c.seqStagHoldMs = clampU32(c.seqStagHoldMs, CFG_SEQ_HOLD_MS_MIN, CFG_SEQ_HOLD_MS_MAX);

  if (c.pingHost[0] == '\0') {
    strncpy(c.pingHost, CFG_DEFAULT.pingHost, sizeof(c.pingHost) - 1);
    c.pingHost[sizeof(c.pingHost) - 1] = '\0';
  }
}

void cfgLoad() {
  cfgp = CFG_DEFAULT;
  prefs.begin(CFG_NS, true);
  cfgp.maxCb       = prefs.getUShort("maxCb",    CFG_DEFAULT.maxCb);
  cfgp.maxDelayS   = prefs.getULong ("maxDelay", CFG_DEFAULT.maxDelayS);
  cfgp.minCb       = prefs.getUShort("minCb",    CFG_DEFAULT.minCb);
  cfgp.minDelayS   = prefs.getULong ("minDelay", CFG_DEFAULT.minDelayS);
  cfgp.calRaw      = prefs.getUShort("calRaw",   CFG_DEFAULT.calRaw);
  cfgp.calCb       = prefs.getUShort("calCb",    CFG_DEFAULT.calCb);
  cfgp.pingEnabled = prefs.getBool  ("pingOn",   CFG_DEFAULT.pingEnabled);
  prefs.getString("pingHost", cfgp.pingHost, sizeof(cfgp.pingHost));
  cfgp.bootNotify  = prefs.getBool  ("bootNtfy", CFG_DEFAULT.bootNotify);
  cfgp.seqRelay4Ms   = prefs.getULong("seqRelay4",  CFG_DEFAULT.seqRelay4Ms);
  cfgp.seqMasterMs   = prefs.getULong("seqMaster",  CFG_DEFAULT.seqMasterMs);
  cfgp.seqPump2Ms    = prefs.getULong("seqPump2",   CFG_DEFAULT.seqPump2Ms);
  cfgp.seqStag2Ms    = prefs.getULong("seqStag2",   CFG_DEFAULT.seqStag2Ms);
  cfgp.seqStagHoldMs = prefs.getULong("seqStagHold",CFG_DEFAULT.seqStagHoldMs);
  prefs.end();
  cfgClamp(cfgp);
}

void cfgSave() {
  cfgClamp(cfgp);
  prefs.begin(CFG_NS, false);
  prefs.putUShort("maxCb",    cfgp.maxCb);
  prefs.putULong ("maxDelay", cfgp.maxDelayS);
  prefs.putUShort("minCb",    cfgp.minCb);
  prefs.putULong ("minDelay", cfgp.minDelayS);
  prefs.putUShort("calRaw",   cfgp.calRaw);
  prefs.putUShort("calCb",    cfgp.calCb);
  prefs.putBool  ("pingOn",   cfgp.pingEnabled);
  prefs.putString("pingHost", cfgp.pingHost);
  prefs.putBool  ("bootNtfy", cfgp.bootNotify);
  prefs.putULong ("seqRelay4",  cfgp.seqRelay4Ms);
  prefs.putULong ("seqMaster",  cfgp.seqMasterMs);
  prefs.putULong ("seqPump2",   cfgp.seqPump2Ms);
  prefs.putULong ("seqStag2",   cfgp.seqStag2Ms);
  prefs.putULong ("seqStagHold",cfgp.seqStagHoldMs);
  prefs.end();
}

// =========================================================================
// STATO
// =========================================================================

WebServer server(80);
MxCoreConfig cfg;

int   raw = 0;
float valore = 0;               // pressione corrente in bar
unsigned long pressureTimer = 0;

bool masterOn = false;
unsigned long bootMs = 0;

// --- notifica ntfy di avvio ----------------------------------------------
// Non si spedisce dentro setup(): l'ora arriva da NTP e al termine del setup
// non e' ancora sincronizzata, quindi la notifica direbbe 01/01/1970. Si
// aspetta in loop() che ci siano rete E ora; oltre BOOT_NOTIFY_TIMEOUT_MS si
// manda comunque, dichiarando che l'ora non e' disponibile: meglio una
// notifica monca che nessuna notifica.
static const unsigned long BOOT_NOTIFY_TIMEOUT_MS = 90000UL;   // 90 s
bool bootNotifySent = false;

unsigned long overStartMs = 0;  // da quando siamo sopra soglia
unsigned long underStartMs = 0; // da quando siamo sotto soglia

String pressureTripLog = "";
String lowPressureTripLog = "";

// --- log eventi -----------------------------------------------------------
static const int EVENT_LOG_CAPACITY = 10;
String eventLogs[EVENT_LOG_CAPACITY];
int eventLogHead = 0;
int eventLogCount = 0;

// --- [3] storico pressione: solo a pompe avviate --------------------------
// PRESS_GAP segna l'interruzione fra una sessione di lavoro e la successiva,
// così il grafico non unisce con una riga due accensioni distinte.
static const int      PRESSURE_HISTORY_SAMPLES = 3600;   // 1 h di moto continuo
static const uint16_t PRESS_GAP = 0xFFFF;
uint16_t pressureHistoryCb[PRESSURE_HISTORY_SAMPLES];
int  pressHistHead = 0;
int  pressHistCount = 0;
bool pressWasRecording = false;
unsigned long lastRecordMs = 0;

// --- sequenziatore pompe (tipi definiti in testa al file) ------------------
PumpSequenceState pumpSeqState = PSEQ_IDLE;
unsigned long pumpSeqMs = 0;
StartMode pendingStartMode = START_NONE;
StartMode activeStartMode = START_NONE;
bool pendingStop = false;

// --- [2] ping non bloccante ------------------------------------------------
TaskHandle_t pingTaskHandle = nullptr;
volatile bool pingBusy = false;
volatile bool pingResultReady = false;
volatile bool pingResultOk = false;
char pingHostInFlight[40] = "";

unsigned long lastPingMs = 0;
int pingFailCount = 0;
unsigned long lastPingRestartMs = 0;

// =========================================================================
// HELPER
// =========================================================================

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

String nowString() {
  char buf[32];
  MxTime.local(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S");
  return String(buf);
}

// "4.0 s" — per i log e la pagina di configurazione, che ora mostrano valori
// scelti dall'utente e non piu' numeri noti a priori.
String msText(uint32_t ms) {
  return String(ms / 1000.0f, 1) + " s";
}

String formatUptime(unsigned long uptimeMs) {
  unsigned long s = uptimeMs / 1000UL;
  unsigned long sec = s % 60UL;  s /= 60UL;
  unsigned long min = s % 60UL;  s /= 60UL;
  unsigned long hou = s % 24UL;  s /= 24UL;
  unsigned long days = s % 30UL;
  unsigned long mon = s / 30UL;

  String out;
  if (mon > 0)  out += String(mon) + " mesi ";
  if (days > 0 || mon > 0) out += String(days) + " g ";
  out += String(hou) + " h " + String(min) + " m " + String(sec) + " s";
  return out;
}

String htmlEscape(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;";  break;
      case '<': o += "&lt;";   break;
      case '>': o += "&gt;";   break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += c;
    }
  }
  return o;
}

String urlEncode(const String& s) {
  String o;
  char buf[5];
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      o += c;
    } else if (c == ' ') {
      o += '+';
    } else {
      snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
      o += buf;
    }
  }
  return o;
}

void addEventLog(const String& msg) {
  String line = (MxTime.valid() ? nowString() : String("uptime ") + formatUptime(millis()));
  eventLogs[eventLogHead] = line + " - " + msg;
  eventLogHead = (eventLogHead + 1) % EVENT_LOG_CAPACITY;
  if (eventLogCount < EVENT_LOG_CAPACITY) eventLogCount++;
  Serial.println("[EVT] " + msg);
}

// --- taratura -------------------------------------------------------------
// Lineare da zero passando per il punto di taratura (calRaw -> calCb).
// NOTA: la 1.3.0 saturava a 10,00 bar per raw >= 2095. Qui NON si satura:
// una sovrapressione oltre il punto di taratura va vista, non nascosta —
// è proprio la condizione che deve far scattare l'allarme.
uint16_t rawToCentibar(int rawValue) {
  if (rawValue <= 0) return 0;
  if (cfgp.calRaw == 0) return 0;
  long cb = ((long)rawValue * (long)cfgp.calCb + (cfgp.calRaw / 2)) / (long)cfgp.calRaw;
  if (cb < 0) cb = 0;
  if (cb > 65000) cb = 65000;
  return (uint16_t)cb;
}

void clearPressureAlertLogs() {
  pressureTripLog = "";
  lowPressureTripLog = "";
  overStartMs = 0;
  underStartMs = 0;
}

// --- [3] storico ----------------------------------------------------------
void pressPush(uint16_t v) {
  pressureHistoryCb[pressHistHead] = v;
  pressHistHead = (pressHistHead + 1) % PRESSURE_HISTORY_SAMPLES;
  if (pressHistCount < PRESSURE_HISTORY_SAMPLES) pressHistCount++;
}

void pressClear() {
  pressHistHead = 0;
  pressHistCount = 0;
  pressWasRecording = false;
}

// =========================================================================
// AUTH
// =========================================================================

bool isWebAuthorized() {
#if ENABLE_WEB_PASSWORD
  if (!server.hasArg(WEB_PW_PARAM)) return false;
  return server.arg(WEB_PW_PARAM) == WEB_PASSWORD;
#else
  return true;
#endif
}

bool ensureAuthorized() {
  if (isWebAuthorized()) return true;
  server.send(403, "text/plain", "Forbidden");
  return false;
}

String pwQuery() {
#if ENABLE_WEB_PASSWORD
  return String("?") + WEB_PW_PARAM + "=" + WEB_PASSWORD;
#else
  return "";
#endif
}

// =========================================================================
// POMPE
// =========================================================================

bool pumpsBusy() {
  return masterOn
      || pumpSeqState != PSEQ_IDLE
      || pendingStartMode != START_NONE
      || activeStartMode != START_NONE;
}

bool rejectIfPumpsBusy() {
  if (!pumpsBusy()) return false;
  server.send(409, "text/html",
              "<font color=orange size=5>Pompe gia' in funzione o avvio in corso - spegnere prima</font>");
  return true;
}

void resetSequencer() {
  pumpSeqState = PSEQ_IDLE;
  pumpSeqMs = 0;
  pendingStartMode = START_NONE;
  activeStartMode = START_NONE;
  pendingStop = false;
}

void stopAllPumps(const String& reason) {
  for (int i = 0; i < numRelays; i++) digitalWrite(relayPins[i], LOW);
  masterOn = false;
  resetSequencer();
  addEventLog(reason);
}

void queueStartMode(StartMode mode, const String& actionLabel) {
  clearPressureAlertLogs();
  pendingStartMode = mode;
  pendingStop = false;
  addEventLog("RICHIESTA: " + actionLabel);
}

// Il topic ntfy si costruisce dal NOME DELLA PISTA. Una pista = un canale:
// chi segue "wafna-motopark" riceve gli avvisi di tutti i device di quella
// pista, senza iscriversi a un topic per macchina.
//
// La normalizzazione e' la stessa che MxIdentity usa per lo slug (minuscole,
// [a-z0-9] tenuti, tutto il resto '-', niente trattini doppi o ai bordi), cosi'
// "Wafna MotoPark" da' "wafna-motopark" sia qui sia nel device_id. La funzione
// della libreria e' static dentro MxIdentity.cpp e non e' esposta: se un
// giorno serve ad altri firmware, il posto giusto e' un MxIdentity::siteSlug().
String ntfySlug(const char* in) {
  String out;
  bool lastDash = true;              // parte true: niente '-' iniziale
  for (size_t i = 0; in && in[i]; i++) {
    char c = in[i];
    if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) { out += c; lastDash = false; }
    else if (!lastDash) { out += '-'; lastDash = true; }
  }
  while (out.length() > 0 && out.charAt(out.length() - 1) == '-') out.remove(out.length() - 1);
  return out;
}

// true se il topic e' quello vero della pista, false se si sta ripiegando.
bool ntfyTopicFromSite() {
  return MxIdentity.provisioned() && ntfySlug(MxIdentity.site()).length() > 0;
}

// Calcolato una volta sola: l'identita' non cambia senza un riavvio.
String ntfyTopic() {
  static String topic;
  if (topic.length()) return topic;
  if (ntfyTopicFromSite()) topic = ntfySlug(MxIdentity.site());
  else                     topic = String(NTFY_TOPIC_FALLBACK);
  return topic;
}

void sendNtfyNotification(const String& title, const String& message,
                          const String& priority, const String& tags) {
  if (!MxNet.connected()) {
    addEventLog("NTFY: saltata, WiFi assente");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  // Timeout espliciti: senza, una rete che non risponde terrebbe fermo il
  // loop() abbastanza da far scattare il watchdog della libreria (15 s).
  http.setConnectTimeout(4000);
  http.setTimeout(5000);

  String url = String(NTFY_SERVER) + "/" + ntfyTopic() + "/publish?";
  url += "title=" + urlEncode(title);
  url += "&message=" + urlEncode(message);
  url += "&priority=" + urlEncode(priority);
  if (tags.length() > 0) url += "&tags=" + urlEncode(tags);

  if (!http.begin(client, url)) {
    addEventLog("NTFY: begin fallita");
    return;
  }

  int code = http.GET();
  if (code > 0) addEventLog("NTFY: inviata (" + String(code) + ") " + title);
  else          addEventLog("NTFY: invio fallito " + title + " err=" + String(code));
  http.end();
}

// Perche' il controller e' ripartito. E' l'informazione che mancava di piu':
// un uptime corto visto per caso non dice se c'e' stato un calo di tensione,
// un watchdog o un aggiornamento OTA.
String resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "accensione (tensione tolta e rimessa)";
    case ESP_RST_EXT:      return "reset esterno";
    case ESP_RST_SW:       return "riavvio software (comando o OTA)";
    case ESP_RST_PANIC:    return "CRASH del firmware (panic)";
    case ESP_RST_INT_WDT:  return "WATCHDOG di interrupt";
    case ESP_RST_TASK_WDT: return "WATCHDOG di task";
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT (tensione insufficiente)";
    case ESP_RST_DEEPSLEEP:return "risveglio da deep sleep";
    case ESP_RST_SDIO:     return "reset SDIO";
    default:               return "sconosciuta";
  }
}

// Il corpo della notifica di avvio: ora, causa del reset e stato completo.
String buildBootMessage() {
  String m;
  m += "Ora: " + (MxTime.valid() ? nowString() : String("NON disponibile (NTP non sincronizzato)")) + "\n";
  m += "Causa del riavvio: " + resetReasonText() + "\n";
  m += "Firmware: " + String(FW_VERSION) + " su MxSolutionCore " + String(MXCORE_VERSION) + "\n";

  if (MxIdentity.provisioned())
    m += "Device: " + String(MxIdentity.site()) + " / " + String(MxIdentity.device()) + "\n";
  else
    m += "Device: IDENTITA' NON IMPOSTATA\n";

  m += "IP: " + WiFi.localIP().toString() + "  (RSSI " + String(WiFi.RSSI()) + " dBm)\n";
  m += "--- stato ---\n";
  m += "Pompe: " + String(masterOn ? "IN MOTO" : "ferme") + "\n";
  m += "Pressione: " + String(valore, 2) + " bar (sensore " + String(raw) + ")\n";
  m += "Soglie: max " + String(cfgp.maxCb / 100.0f, 2) + " bar dopo " + String(cfgp.maxDelayS) + " s"
       ", min " + String(cfgp.minCb / 100.0f, 2) + " bar dopo " + String(cfgp.minDelayS) + " s\n";
  m += "Ping di rete: " + String(cfgp.pingEnabled ? "attivo verso " + String(cfgp.pingHost) : String("disattivato")) + "\n";
  m += "Canale ntfy: " + ntfyTopic()
     + String(ntfyTopicFromSite() ? "" : "  (RIPIEGO: identita' non impostata)") + "\n";
  m += "Pronto dopo " + formatUptime(millis() - bootMs) + " dall'avvio";
  return m;
}

// Chiamata a ogni giro: decide se e' il momento di spedire. Spedisce UNA volta
// sola per accensione.
void serviceBootNotification() {
  if (bootNotifySent) return;

  if (!cfgp.bootNotify) {           // disattivata da /config
    bootNotifySent = true;
    return;
  }

  bool scaduto = (millis() - bootMs) >= BOOT_NOTIFY_TIMEOUT_MS;

  if (!MxNet.connected()) {
    // Senza rete non si puo' spedire. Oltre il timeout si rinuncia, invece di
    // restare in attesa per sempre e spedire a mezza giornata di distanza una
    // notifica che parla di un avvio ormai vecchio.
    if (scaduto) {
      bootNotifySent = true;
      addEventLog("BOOT: notifica ntfy saltata, WiFi assente entro "
                  + String(BOOT_NOTIFY_TIMEOUT_MS / 1000) + " s");
    }
    return;
  }

  // C'e' rete: si aspetta l'ora, ma non oltre il timeout.
  if (!MxTime.valid() && !scaduto) return;

  bootNotifySent = true;
  sendNtfyNotification(
    "MxIrrigation - pompe: avvio",
    buildBootMessage(),
    "default", "electric_plug,arrows_counterclockwise");
  addEventLog("BOOT: notifica ntfy inviata");
}

void checkHighPressureTrip() {
  uint16_t cb = rawToCentibar(raw);

  if (cb > cfgp.maxCb) {
    if (overStartMs == 0) { overStartMs = millis(); return; }

    if (millis() - overStartMs >= cfgp.maxDelayS * 1000UL) {
      stopAllPumps("ALLARME: alta pressione, pompe spente (" + String(cb / 100.0f, 2) + " bar)");
      overStartMs = 0;

      pressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid red;color:red;'>";
      pressureTripLog += "<b>ALLARME: POMPE SPENTE - ALTA PRESSIONE</b><br>";
      pressureTripLog += "Ora: " + htmlEscape(nowString()) + "<br>";
      pressureTripLog += "Pressione: " + String(cb / 100.0f, 2) + " bar (soglia "
                       + String(cfgp.maxCb / 100.0f, 2) + ", raw " + String(raw) + ")<br>";
      pressureTripLog += "</div>";

      sendNtfyNotification(
        "MxIrrigation - allarme alta pressione",
        "Pompe spente per alta pressione alle " + nowString() +
        ". " + String(cb / 100.0f, 2) + " bar, soglia " + String(cfgp.maxCb / 100.0f, 2) + " bar",
        "high", "warning,pressure");
    }
  } else {
    overStartMs = 0;
  }
}

void checkLowPressureTripWhilePumpsOn() {
  if (!masterOn) { underStartMs = 0; return; }

  uint16_t cb = rawToCentibar(raw);

  if (cb < cfgp.minCb) {
    if (underStartMs == 0) { underStartMs = millis(); return; }

    if (millis() - underStartMs >= cfgp.minDelayS * 1000UL) {
      stopAllPumps("ALLARME: bassa pressione, pompe spente (" + String(cb / 100.0f, 2) + " bar)");
      underStartMs = 0;

      lowPressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid orange;color:orange;'>";
      lowPressureTripLog += "<b>ALLARME: POMPE SPENTE - BASSA PRESSIONE</b><br>";
      lowPressureTripLog += "Ora: " + htmlEscape(nowString()) + "<br>";
      lowPressureTripLog += "Pressione: " + String(cb / 100.0f, 2) + " bar (minimo "
                          + String(cfgp.minCb / 100.0f, 2) + ", raw " + String(raw) + ")<br>";
      lowPressureTripLog += "</div>";

      sendNtfyNotification(
        "MxIrrigation - allarme bassa pressione",
        "Pompe spente per bassa pressione alle " + nowString() +
        ". " + String(cb / 100.0f, 2) + " bar, minimo " + String(cfgp.minCb / 100.0f, 2) + " bar",
        "high", "warning,pressure");
    }
  } else {
    underStartMs = 0;
  }
}

void runPumpSequencer() {
  unsigned long now = millis();

  if (pendingStop) {
    stopAllPumps("POMPE SPENTE (richiesta di stop)");
    return;
  }

  if (pumpSeqState == PSEQ_IDLE && pendingStartMode != START_NONE) {
    activeStartMode = pendingStartMode;
    pendingStartMode = START_NONE;

    switch (activeStartMode) {
      case START_MASTER_BOTH:
        digitalWrite(relayPins[3], HIGH);   // relè 4 per primo
        digitalWrite(relayPins[0], LOW);
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = false;
        pumpSeqState = PSEQ_RELAY4_WAIT;
        pumpSeqMs = now;
        addEventLog("AVVIO: entrambe (relè4 ON, sequenza)");
        break;

      case START_MASTER_SOLAR_BOTH:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH);   // master ON
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT;
        pumpSeqMs = now;
        addEventLog("AVVIO: master solar (master ON, sequenza)");
        break;

      case START_SINGLE_ONLY:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH);
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT;
        pumpSeqMs = now;
        addEventLog("AVVIO: singola (master ON, solo pompa 1)");
        break;

      case START_STAGGERED_AUTO_OFF:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH);
        digitalWrite(relayPins[1], HIGH);   // pompa 1 subito
        digitalWrite(relayPins[2], LOW);
        masterOn = true;
        pumpSeqState = PSEQ_STAGGERED_PUMP2_WAIT;
        pumpSeqMs = now;
        addEventLog("AVVIO: sfalsato (pompa1 ora, pompa2 fra " + msText(cfgp.seqStag2Ms)
                   + ", off dopo " + msText(cfgp.seqStagHoldMs) + ")");
        break;

      case START_NONE:
      default:
        activeStartMode = START_NONE;
        break;
    }
  }

  switch (pumpSeqState) {
    case PSEQ_IDLE:
      break;

    case PSEQ_RELAY4_WAIT:
      if (now - pumpSeqMs >= cfgp.seqRelay4Ms) {
        digitalWrite(relayPins[0], HIGH);
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT;
        pumpSeqMs = now;
        addEventLog("SEQ: master ON (dopo " + msText(cfgp.seqRelay4Ms) + " di relè4)");
      }
      break;

    case PSEQ_MASTER_WAIT:
      if (now - pumpSeqMs >= cfgp.seqMasterMs) {
        digitalWrite(relayPins[1], HIGH);
        addEventLog("SEQ: pompa1 ON");
        if (activeStartMode == START_SINGLE_ONLY) {
          digitalWrite(relayPins[2], LOW);
          pumpSeqState = PSEQ_IDLE;
          activeStartMode = START_NONE;
          addEventLog("SEQ: modo singolo completo");
        } else {
          pumpSeqState = PSEQ_PUMP1_WAIT;
          pumpSeqMs = now;
        }
      }
      break;

    case PSEQ_PUMP1_WAIT:
      if (now - pumpSeqMs >= cfgp.seqPump2Ms) {
        digitalWrite(relayPins[2], HIGH);
        pumpSeqState = PSEQ_IDLE;
        activeStartMode = START_NONE;
        addEventLog("SEQ: pompa2 ON (sequenza completa)");
      }
      break;

    case PSEQ_STAGGERED_PUMP2_WAIT:
      if (now - pumpSeqMs >= cfgp.seqStag2Ms) {
        digitalWrite(relayPins[2], HIGH);
        pumpSeqState = PSEQ_STAGGERED_PUMP2_HOLD;
        pumpSeqMs = now;
        addEventLog("SEQ: pompa2 ON (" + msText(cfgp.seqStag2Ms) + " dopo pompa1)");
      }
      break;

    case PSEQ_STAGGERED_PUMP2_HOLD:
      if (now - pumpSeqMs >= cfgp.seqStagHoldMs) {
        digitalWrite(relayPins[2], LOW);
        pumpSeqState = PSEQ_IDLE;
        activeStartMode = START_NONE;
        addEventLog("SEQ: pompa2 OFF dopo " + msText(cfgp.seqStagHoldMs) + " (pompa1 resta ON)");
      }
      break;
  }
}

// =========================================================================
// [2] PING NON BLOCCANTE
// =========================================================================
//
// Nella 1.3.0 Ping.ping() veniva chiamata direttamente dal loop(): per tutta
// la durata del timeout ICMP il web server non veniva servito e il loop non
// girava. Qui la chiamata sta in un task FreeRTOS dedicato; il loop() si
// limita a farlo partire e a raccogliere il risultato.
//
// Il watchdog della libreria sorveglia solo il task del loop(), quindi un
// task di rete lento non lo fa scattare: è esattamente il caso per cui
// MxWatchdog è fatto così.

void pingTaskFn(void* arg) {
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    bool ok = Ping.ping(pingHostInFlight, 1);
    pingResultOk = ok;
    pingResultReady = true;
    pingBusy = false;
  }
}

void servicePingMonitor() {
  // 1) raccogli un risultato pronto
  if (pingResultReady) {
    pingResultReady = false;
    if (pingResultOk) {
      pingFailCount = 0;
      Serial.println("[PING] ok");
    } else {
      pingFailCount++;
      Serial.printf("[PING] fallito %d/%d\n", pingFailCount, PING_FAIL_REBOOT_COUNT);
    }

    if (pingFailCount >= PING_FAIL_REBOOT_COUNT) {
      unsigned long now = millis();
      if (lastPingRestartMs != 0 && (now - lastPingRestartMs < PING_REBOOT_COOLDOWN_MS)) {
        addEventLog("WATCHDOG: ping fallito, riavvio soppresso dal cooldown");
        pingFailCount = 0;
      } else {
        addEventLog("RIAVVIO: ping fallito " + String(PING_FAIL_REBOOT_COUNT) + " volte (pompe spente)");
        lastPingRestartMs = now;
        delay(100);
        ESP.restart();
      }
    }
  }

  // 2) decidi se lanciarne uno nuovo
  if (!cfgp.pingEnabled)  { pingFailCount = 0; return; }
  if (masterOn)           { pingFailCount = 0; lastPingMs = 0; return; }
  if (!MxNet.connected()) { pingFailCount = 0; return; }
  if (pingBusy)           return;
  if (pingTaskHandle == nullptr) return;

  unsigned long now = millis();
  if (now - bootMs < PING_START_DELAY_MS) return;
  if (lastPingMs != 0 && (now - lastPingMs < PING_PERIOD_MS)) return;
  lastPingMs = now;

  strncpy(pingHostInFlight, cfgp.pingHost, sizeof(pingHostInFlight) - 1);
  pingHostInFlight[sizeof(pingHostInFlight) - 1] = '\0';
  pingBusy = true;
  xTaskNotifyGive(pingTaskHandle);
}

// =========================================================================
// PAGINE WEB
// =========================================================================

String pageHead(const char* title) {
  String h;
  h += "<!DOCTYPE HTML><html><head>";
  h += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  h += "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>";
  h += "<meta http-equiv='Expires' content='0'>";
  h += "<title>";
  h += title;
  h += "</title><style>body{font-family:Arial,sans-serif;font-size:14px}"
       "canvas{border:1px solid #444;max-width:100%}"
       "table.cfg{margin:0 auto;text-align:left}"
       "table.cfg td{padding:4px 8px}"
       "input[type=number],input[type=text]{padding:4px;width:10em}"
       ".hint{color:#666;font-size:12px}</style></head><body><div align=center>";
  return h;
}

String navBar() {
  String q = pwQuery();
  return "<p><a href='/" + q + "'>Home</a> | <a href='/info" + q + "'>Info</a> | "
         "<a href='/logs" + q + "'>Log</a> | <a href='/pressure" + q + "'>Pressione</a> | "
         "<a href='/config" + q + "'>Config</a></p>";
}

void handleRoot() {
  bool authorized = isWebAuthorized();
  bool busy = pumpsBusy();

  String h = pageHead("MxIrrigation - Pompe");
  h += "<h2>MxIrrigation Pump Controller</h2>";
  h += navBar();

  h += "Stato pompe:";
  if (masterOn)   h += " <font color=red>ACCESE</font>";
  else if (busy)  h += " <font color=orange>AVVIO IN CORSO</font>";
  else            h += " <font color=green>SPENTE</font>";

  h += "<br><br>Pressione: <b>" + String(valore, 2) + " bar</b> <span class=hint>(raw " + String(raw) + ")</span>";
  h += "<br><span class=hint>Soglie: max " + String(cfgp.maxCb / 100.0f, 2)
     + " bar dopo " + String(cfgp.maxDelayS) + " s · min " + String(cfgp.minCb / 100.0f, 2)
     + " bar dopo " + String(cfgp.minDelayS) + " s</span>";

  if (!MxIdentity.provisioned()) {
    h += "<p style='color:#b00'><b>Identita' non impostata</b>: comando seriale "
         "<code>id set \"Nome Pista\" \"nome-device\"</code></p>";
  }
  if (MxOta.inProgress()) h += "<p style='color:#b60'><b>Aggiornamento OTA in corso</b></p>";

  if (authorized) {
    String q = pwQuery();
    if (busy) {
      h += "<br><br><font color=gray>Pompe in funzione o avvio in corso: e' possibile solo lo spegnimento.</font>";
      h += "<br><br><a href='/master/off" + q + "'>SPEGNI</a>";
    } else {
      h += "<br><br><a href='/master/on" + q + "'>Accendi entrambe</a>";
      h += "<br><br><a href='/master-solar/on" + q + "'>Accendi master solar</a>";
      h += "<br><br><a href='/single" + q + "'>Accendi singola</a>";
      h += "<br><br><a href='/staggered-auto" + q + "'>Doppia sfalsata 5 s / seconda off dopo 40 s</a>";
      h += "<br><br><a href='/master/off" + q + "'>SPEGNI</a>";
    }
    h += "<br><br><button onclick=\"if(confirm('Riavviare il controller?')){window.location='/reboot" + q + "';}\">Riavvia</button>";
  }

  h += pressureTripLog;
  h += lowPressureTripLog;
  h += "</div></body></html>";
  server.send(200, "text/html", h);
}

void handleInfo() {
  String h = pageHead("MxIrrigation - Info");
  h += "<h2>System info</h2>";
  h += navBar();
  h += "<pre style='text-align:left;display:inline-block'>";
  h += "Firmware        : " + String(FW_VERSION) + "\n";
  h += "MxSolutionCore  : " + String(MXCORE_VERSION) + "\n";
  h += "Pista (site)    : " + htmlEscape(MxIdentity.provisioned() ? MxIdentity.site() : "(non impostata)") + "\n";
  h += "Device          : " + htmlEscape(MxIdentity.provisioned() ? MxIdentity.device() : "(non impostato)") + "\n";
  h += "Device ID       : " + htmlEscape(MxIdentity.provisioned() ? MxIdentity.deviceId() : "-") + "\n";
  h += "IP              : " + htmlEscape(MxNet.ip()) + (MxNet.usingStaticIP() ? " (fisso)\n" : " (DHCP)\n");
  if (MxNet.usingStaticIP()) {
    h += "IP configurato  : " + String(MxNet.staticIpStr()) + " / " + String(MxNet.staticMaskStr())
       + " gw " + String(MxNet.staticGwStr()) + "\n";
  }
  h += "WiFi            : " + String(MxNet.connected() ? "CONNESSO" : "DISCONNESSO")
     + "  RSSI " + String((long)MxNet.rssi()) + " dBm\n";
  h += "Ora             : " + htmlEscape(MxTime.valid() ? nowString() : String("non disponibile")) + "\n";
  h += "NTP             : " + String(MxTime.valid() ? "sincronizzato" : "NON sincronizzato")
     + " (ultimo " + String((unsigned long)MxTime.lastSyncAgeS()) + " s fa)\n";
  h += "Uptime          : " + htmlEscape(formatUptime(millis())) + "\n";
  h += "Watchdog        : " + String((unsigned long)MxWatchdog.timeoutS()) + " s\n";
  h += "ArduinoOTA      : " + String(MxOta.armed() ? "armato" : "disabilitato") + "\n";
  h += "OTA pull        : " + String(otaHttpUrl[0] ? "configurato" : "non configurato")
     + String(otaHttpUrl[0] ? " (fw " + String(OTA_FW_NAME) + ")" : "") + "\n";
  h += "Ping            : " + String(cfgp.pingEnabled ? "attivo verso " : "disattivato (")
     + String(cfgp.pingHost) + String(cfgp.pingEnabled ? "" : ")") + "\n";
  h += "Notifica avvio  : " + String(!cfgp.bootNotify ? "disattivata"
                                   : bootNotifySent   ? "attiva, gia' inviata"
                                                      : "attiva, in attesa di rete/ora") + "\n";
  h += "Topic ntfy      : " + htmlEscape(ntfyTopic())
     + String(ntfyTopicFromSite() ? " (dal nome della pista)"
                                  : " (RIPIEGO: identita' non impostata)") + "\n";
  h += "Causa riavvio   : " + htmlEscape(resetReasonText()) + "\n";
  h += "Heap libero     : " + String((unsigned)ESP.getFreeHeap()) + "\n";
  h += "Campioni press. : " + String(pressHistCount) + " / " + String(PRESSURE_HISTORY_SAMPLES) + "\n";
  h += "</pre>";
  h += String("<p>") + MxOta.linkHtml() + "</p>";
  h += "</div></body></html>";
  server.send(200, "text/html", h);
}

void handleLogs() {
  String h = pageHead("MxIrrigation - Log");
  h += "<h2>Ultimi " + String(EVENT_LOG_CAPACITY) + " eventi</h2>";
  h += navBar();
  h += "<table border=1 cellpadding=6 style='margin:0 auto'>";
  if (eventLogCount == 0) {
    h += "<tr><td>nessun evento</td></tr>";
  } else {
    for (int i = eventLogCount - 1; i >= 0; i--) {
      int idx = (eventLogHead - eventLogCount + i + EVENT_LOG_CAPACITY * 2) % EVENT_LOG_CAPACITY;
      h += "<tr><td><pre style='margin:0'>" + htmlEscape(eventLogs[idx]) + "</pre></td></tr>";
    }
  }
  h += "</table></div></body></html>";
  server.send(200, "text/html", h);
}

void handlePressure() {
  String h = pageHead("MxIrrigation - Pressione");
  h += "<h2>Pressione durante il funzionamento</h2>";
  h += navBar();
  h += "<p class=hint>Lo storico registra <b>solo</b> i periodi con le pompe avviate, "
       "un campione al secondo. Le interruzioni fra un'accensione e l'altra sono "
       "mostrate come stacchi nella linea, non collegate.</p>";
  h += "<div>Ora: " + htmlEscape(MxTime.valid() ? nowString() : String("n/d"))
     + " | attuale: " + String(valore, 2) + " bar | campioni: " + String(pressHistCount) + "</div><br>";

  h += "<canvas id='c' width='1000' height='300'></canvas>";
  h += "<script>const g=" + String((unsigned int)PRESS_GAP) + ";const samplesCb=[";
  if (pressHistCount > 0) {
    int start = pressHistHead - pressHistCount;
    while (start < 0) start += PRESSURE_HISTORY_SAMPLES;
    for (int i = 0; i < pressHistCount; i++) {
      int idx = (start + i) % PRESSURE_HISTORY_SAMPLES;
      h += String((unsigned int)pressureHistoryCb[idx]);
      if (i != pressHistCount - 1) h += ",";
    }
  }
  h += "];\n";

  h += R"JS(
const samples = samplesCb.map(v => v===g ? null : v/100.0);
const canvas=document.getElementById('c');
const ctx=canvas.getContext('2d');
const W=canvas.width, H=canvas.height;

function draw(){
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#444';
  ctx.strokeRect(0.5,0.5,W-1,H-1);

  const real = samples.filter(v => v!==null);
  if(real.length<2){
    ctx.fillStyle='#666';
    ctx.font='13px Arial';
    ctx.fillText('Nessun dato: le pompe non sono ancora state avviate', 12, 24);
    return;
  }

  let min=Math.min(...real), max=Math.max(...real);
  if(min===max){ min-=0.01; max+=0.01; }

  const padL=60, padR=10, padT=10, padB=25;
  const plotW=W-padL-padR, plotH=H-padT-padB;

  ctx.strokeStyle='#999';
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT+plotH);
  ctx.lineTo(padL+plotW, padT+plotH);
  ctx.stroke();

  ctx.fillStyle='#333';
  ctx.font='12px Arial';
  ctx.fillText(max.toFixed(2)+' bar', 5, padT+10);
  ctx.fillText(min.toFixed(2)+' bar', 5, padT+plotH);
  ctx.fillText("piu' vecchio", padL, H-5);
  ctx.fillText('ora', padL+plotW-20, H-5);

  ctx.strokeStyle='#0a6';
  ctx.lineWidth=1;
  ctx.beginPath();
  let pen=false;
  for(let i=0;i<samples.length;i++){
    const v=samples[i];
    if(v===null){ pen=false; continue; }
    const x=padL + (i/(samples.length-1))*plotW;
    const y=padT + (1 - (v-min)/(max-min))*plotH;
    if(!pen){ ctx.moveTo(x,y); pen=true; } else { ctx.lineTo(x,y); }
  }
  ctx.stroke();
}
draw();
)JS";

  h += "\n</script></div></body></html>";
  server.send(200, "text/html", h);
}

// --- [1] pagina di configurazione ----------------------------------------

// Una riga della tabella dei tempi: etichetta, campo in secondi con un
// decimale, valore corrente. Cinque campi identici, tanto vale scriverli una
// volta sola.
String seqRow(const char* label, const char* name, uint32_t ms) {
  String r = "<tr><td>";
  r += label;
  r += "</td><td><input type=number step=0.1 min=0 name='";
  r += name;
  r += "' value='" + String(ms / 1000.0f, 1) + "'> s</td></tr>";
  return r;
}

void handleConfig() {
  String q = pwQuery();
  String h = pageHead("MxIrrigation - Configurazione");
  h += "<h2>Configurazione</h2>";
  h += navBar();

  if (server.hasArg("saved")) {
    h += "<p style='color:green'><b>Salvato.</b> I valori fuori range sono stati "
         "riportati al limite piu' vicino.</p>";
  }

  h += "<form method='POST' action='/config/save" + q + "'>";
  h += "<table class=cfg>";

  h += "<tr><td colspan=2><b>Allarme di pressione</b></td></tr>";
  h += "<tr><td>Pressione massima</td><td><input type=number step='0.01' min='0.1' max='60' name='maxbar' value='"
     + String(cfgp.maxCb / 100.0f, 2) + "'> bar</td></tr>";
  h += "<tr><td>Attesa prima dell'allarme max</td><td><input type=number min='1' max='3600' name='maxdelay' value='"
     + String(cfgp.maxDelayS) + "'> s</td></tr>";
  h += "<tr><td>Pressione minima</td><td><input type=number step='0.01' min='0' max='60' name='minbar' value='"
     + String(cfgp.minCb / 100.0f, 2) + "'> bar</td></tr>";
  h += "<tr><td>Attesa prima dell'allarme min</td><td><input type=number min='1' max='3600' name='mindelay' value='"
     + String(cfgp.minDelayS) + "'> s</td></tr>";
  h += "<tr><td colspan=2 class=hint>La soglia minima vale solo a pompe avviate. "
       "Se viene impostata >= della massima, viene abbassata automaticamente.</td></tr>";

  h += "<tr><td colspan=2><br><b>Taratura del sensore</b></td></tr>";
  h += "<tr><td>Valore sensore</td><td><input type=number min='100' max='4095' name='calraw' value='"
     + String(cfgp.calRaw) + "'></td></tr>";
  h += "<tr><td>corrisponde a</td><td><input type=number step='0.01' min='0.1' max='60' name='calbar' value='"
     + String(cfgp.calCb / 100.0f, 2) + "'> bar</td></tr>";
  h += "<tr><td colspan=2 class=hint><b>Valore sensore letto adesso: " + String(raw)
     + "</b> (pressione calcolata " + String(valore, 2) + " bar)<br>"
       "Per tarare: porta l'impianto a una pressione nota letta dal manometro, "
       "ricarica questa pagina, copia il valore qui sopra nel campo "
       "<i>Valore sensore</i> e scrivi accanto la pressione del manometro.<br>"
       "La curva e' lineare e passa per lo zero.</td></tr>";

  h += "<tr><td colspan=2><br><b>Tempi della sequenza di avvio</b></td></tr>";
  h += seqRow("Rele' 4 -> master",  "seqrelay4",  cfgp.seqRelay4Ms);
  h += seqRow("Master -> pompa 1",  "seqmaster",  cfgp.seqMasterMs);
  h += seqRow("<b>Pompa 1 -> pompa 2</b>", "seqpump2", cfgp.seqPump2Ms);
  h += "<tr><td colspan=2 class=hint>I tre tempi sopra valgono per i modi "
       "<i>entrambe</i>, <i>master solar</i> e <i>singola</i>. Il primo si usa solo "
       "nel modo <i>entrambe</i>, che accende il rele' 4 prima del master.</td></tr>";
  h += seqRow("Sfalsato: pompa 1 -> pompa 2", "seqstag2",    cfgp.seqStag2Ms);
  h += seqRow("Sfalsato: pompa 2 accesa per", "seqstaghold", cfgp.seqStagHoldMs);
  h += "<tr><td colspan=2 class=hint>Tutti in secondi, un decimale. Zero = i due "
       "rele' scattano insieme; il solo tempo che non puo' essere zero e' l'ultimo "
       "(pompa 2 si spegnerebbe subito), minimo " + msText(CFG_SEQ_HOLD_MS_MIN) + ".<br>"
       "<b>Attenzione</b>: questi tempi proteggono l'impianto dallo spunto simultaneo "
       "dei motori. Accorciarli troppo significa far partire due pompe insieme sullo "
       "stesso quadro.<br>"
       "Una sequenza <b>gia' in corso</b> adotta subito i valori nuovi: se si accorcia "
       "un tempo sotto quello gia' trascorso, il passo successivo scatta al primo giro. "
       "Meglio regolarli a pompe ferme.</td></tr>";

  h += "<tr><td colspan=2><br><b>Ping di controllo rete</b></td></tr>";
  h += "<tr><td>Attivo</td><td><input type=checkbox name='pingon' value='1'";
  if (cfgp.pingEnabled) h += " checked";
  h += "></td></tr>";
  h += "<tr><td>Host</td><td><input type=text name='pinghost' value='" + htmlEscape(String(cfgp.pingHost)) + "'></td></tr>";
  h += "<tr><td colspan=2 class=hint>Solo a pompe spente, ogni 30 s, dopo 2 minuti dall'avvio. "
       "Dopo " + String(PING_FAIL_REBOOT_COUNT) + " fallimenti consecutivi il controller si riavvia.<br>"
       "<b>Attenzione</b>: se l'host indicato e' un dispositivo che puo' essere spento, "
       "il controller si riavvia in continuazione. In caso di dubbio usare il "
       "<b>gateway</b> della rete, o disattivare il ping.</td></tr>";

  h += "<tr><td colspan=2><br><b>Notifica ntfy all'avvio</b></td></tr>";
  h += "<tr><td>Attiva</td><td><input type=checkbox name='bootntfy' value='1'";
  if (cfgp.bootNotify) h += " checked";
  h += "></td></tr>";
  h += "<tr><td colspan=2 class=hint>A ogni accensione manda una notifica con data e ora, "
       "<b>causa del riavvio</b> (rete tolta, watchdog, crash, OTA...), stato delle pompe, "
       "pressione, soglie e stato del ping.<br>"
       "Canale: <b>" + htmlEscape(ntfyTopic()) + "</b>";
  h += String(ntfyTopicFromSite()
        ? ", ricavato dal nome della pista."
        : " <b style='color:#b60'>(ripiego: l'identita' non e' impostata, quindi il nome "
          "della pista non c'e'. Impostarla da seriale con <code>id set</code>.)</b>");
  h += "<br>Aspetta rete e ora NTP al massimo " + String(BOOT_NOTIFY_TIMEOUT_MS / 1000) + " s; "
       "oltre quel tempo manda lo stesso, dichiarando l'ora non disponibile.<br>"
       "<b>Attenzione</b>: se il controller entra in ciclo di riavvii (per esempio col ping "
       "attivo verso un host spento) arriva <b>una notifica per ogni riavvio</b>.</td></tr>";

  h += "</table><br><button type=submit>Salva</button></form>";

  h += "<p class=hint>I valori sono salvati in NVS e sopravvivono al riavvio e "
       "all'aggiornamento OTA. Il reset di fabbrica del WiFi non li cancella.</p>";
  h += "</div></body></html>";
  server.send(200, "text/html", h);
}

void handleConfigSave() {
  if (!ensureAuthorized()) return;

  if (server.hasArg("maxbar"))   cfgp.maxCb     = (uint16_t)lroundf(server.arg("maxbar").toFloat() * 100.0f);
  if (server.hasArg("minbar"))   cfgp.minCb     = (uint16_t)lroundf(server.arg("minbar").toFloat() * 100.0f);
  if (server.hasArg("maxdelay")) cfgp.maxDelayS = (uint32_t)server.arg("maxdelay").toInt();
  if (server.hasArg("mindelay")) cfgp.minDelayS = (uint32_t)server.arg("mindelay").toInt();
  if (server.hasArg("calraw"))   cfgp.calRaw    = (uint16_t)server.arg("calraw").toInt();
  if (server.hasArg("calbar"))   cfgp.calCb     = (uint16_t)lroundf(server.arg("calbar").toFloat() * 100.0f);
  cfgp.pingEnabled = server.hasArg("pingon");
  cfgp.bootNotify  = server.hasArg("bootntfy");
  if (server.hasArg("seqrelay4"))   cfgp.seqRelay4Ms   = (uint32_t)lroundf(server.arg("seqrelay4").toFloat()   * 1000.0f);
  if (server.hasArg("seqmaster"))   cfgp.seqMasterMs   = (uint32_t)lroundf(server.arg("seqmaster").toFloat()   * 1000.0f);
  if (server.hasArg("seqpump2"))    cfgp.seqPump2Ms    = (uint32_t)lroundf(server.arg("seqpump2").toFloat()    * 1000.0f);
  if (server.hasArg("seqstag2"))    cfgp.seqStag2Ms    = (uint32_t)lroundf(server.arg("seqstag2").toFloat()    * 1000.0f);
  if (server.hasArg("seqstaghold")) cfgp.seqStagHoldMs = (uint32_t)lroundf(server.arg("seqstaghold").toFloat() * 1000.0f);
  if (server.hasArg("pinghost")) {
    String ph = server.arg("pinghost");
    ph.trim();
    strncpy(cfgp.pingHost, ph.c_str(), sizeof(cfgp.pingHost) - 1);
    cfgp.pingHost[sizeof(cfgp.pingHost) - 1] = '\0';
  }

  cfgSave();
  addEventLog("CONFIG: aggiornata da web (max " + String(cfgp.maxCb / 100.0f, 2)
              + " bar/" + String(cfgp.maxDelayS) + "s, min " + String(cfgp.minCb / 100.0f, 2)
              + " bar/" + String(cfgp.minDelayS) + "s, taratura " + String(cfgp.calRaw)
              + "=" + String(cfgp.calCb / 100.0f, 2) + " bar)");

  // Le temporizzazioni in corso ripartono: una soglia cambiata a metà
  // conteggio renderebbe il trip imprevedibile.
  overStartMs = 0;
  underStartMs = 0;

  String q = pwQuery();
  server.sendHeader("Location", "/config" + String(q.length() ? q + "&" : "?") + "saved=1");
  server.send(303, "text/plain", "See Other");
}

// --- /status: JSON per i consumatori esterni ------------------------------
// Aggiunto in 1.4.1 per il pannello ESP32-2432S028 ([[mxirrigation-display-cyd]]).
// Le chiavi comuni (fw, core, device_id, site, ip, wifi_*, uptime_s, heap,
// time_synced, ota) sono le STESSE della centralina valvole: un consumatore
// legge entrambi i nodi con lo stesso codice, e distingue dal campo "type".
void handleStatusJson() {
  String h;
  h.reserve(768);
  h += "{";
  h += "\"type\":\"pump\",";
  h += "\"firmware\":\"" + String(FW_VERSION) + "\",";
  h += "\"core\":\"" + String(MXCORE_VERSION) + "\",";
  h += "\"device_id\":\"" + String(MxIdentity.provisioned() ? MxIdentity.deviceId() : "") + "\",";
  h += "\"site\":\"" + String(MxIdentity.site()) + "\",";
  h += "\"ip\":\"" + MxNet.ip() + "\",";
  h += "\"wifi_connected\":" + String(MxNet.connected() ? "true" : "false") + ",";
  h += "\"wifi_rssi\":" + String((long)MxNet.rssi()) + ",";
  h += "\"time_synced\":" + String(MxTime.valid() ? "true" : "false") + ",";
  h += "\"datetime\":\"" + (MxTime.valid() ? nowString() : String("")) + "\",";
  h += "\"uptime_s\":" + String((unsigned long)MxCore.uptimeS()) + ",";
  h += "\"heap\":" + String((unsigned)ESP.getFreeHeap()) + ",";
  h += "\"ota\":" + String(MxOta.inProgress() ? "true" : "false") + ",";

  // --- specifico della pompa ---
  h += "\"master_on\":" + String(masterOn ? "true" : "false") + ",";
  h += "\"busy\":" + String(pumpsBusy() ? "true" : "false") + ",";
  h += "\"pressure_bar\":" + String(valore, 2) + ",";
  h += "\"pressure_raw\":" + String(raw) + ",";
  h += "\"max_bar\":" + String(cfgp.maxCb / 100.0f, 2) + ",";
  h += "\"min_bar\":" + String(cfgp.minCb / 100.0f, 2) + ",";
  h += "\"alarm_high\":" + String(pressureTripLog.length() ? "true" : "false") + ",";
  h += "\"alarm_low\":" + String(lowPressureTripLog.length() ? "true" : "false") + ",";
  h += "\"samples\":" + String(pressHistCount) + ",";

  // Stato dei quattro relè, nell'ordine della scheda.
  h += "\"relays\":[";
  for (int i = 0; i < numRelays; i++) {
    h += "{\"id\":" + String(i + 1) + ",\"on\":"
       + String(digitalRead(relayPins[i]) == HIGH ? "true" : "false") + "}";
    if (i < numRelays - 1) h += ",";
  }
  h += "]}";

  server.send(200, "application/json", h);
}

void handleReboot() {
  if (!ensureAuthorized()) return;
  addEventLog("RIAVVIO: richiesto da web");
  server.send(200, "text/html", "<font color=orange size=5>riavvio...</font>");
  delay(250);
  ESP.restart();
}

void registerRoutes() {
  server.on("/",         HTTP_GET, []() { if (ensureAuthorized()) handleRoot(); });
  server.on("/info",     HTTP_GET, []() { if (ensureAuthorized()) handleInfo(); });
  server.on("/logs",     HTTP_GET, []() { if (ensureAuthorized()) handleLogs(); });
  server.on("/pressure", HTTP_GET, []() { if (ensureAuthorized()) handlePressure(); });
  server.on("/config",   HTTP_GET, []() { if (ensureAuthorized()) handleConfig(); });
  server.on("/config/save", HTTP_POST, handleConfigSave);
  // /status resta SENZA auth: è letto dal pannello di stato, che non ha modo
  // di autenticarsi, ed espone solo dati in lettura (nessun comando).
  server.on("/status",   HTTP_GET, handleStatusJson);
  server.on("/reboot",   HTTP_GET, handleReboot);

  server.on("/master/on", HTTP_GET, []() {
    if (!ensureAuthorized() || rejectIfPumpsBusy()) return;
    queueStartMode(START_MASTER_BOTH, "master/on");
    server.sendHeader("Location", "/" + pwQuery());
    server.send(303, "text/plain", "See Other");
  });
  server.on("/master-solar/on", HTTP_GET, []() {
    if (!ensureAuthorized() || rejectIfPumpsBusy()) return;
    queueStartMode(START_MASTER_SOLAR_BOTH, "master-solar/on");
    server.sendHeader("Location", "/" + pwQuery());
    server.send(303, "text/plain", "See Other");
  });
  server.on("/single", HTTP_GET, []() {
    if (!ensureAuthorized() || rejectIfPumpsBusy()) return;
    queueStartMode(START_SINGLE_ONLY, "single");
    server.sendHeader("Location", "/" + pwQuery());
    server.send(303, "text/plain", "See Other");
  });
  server.on("/staggered-auto", HTTP_GET, []() {
    if (!ensureAuthorized() || rejectIfPumpsBusy()) return;
    queueStartMode(START_STAGGERED_AUTO_OFF, "staggered-auto");
    server.sendHeader("Location", "/" + pwQuery());
    server.send(303, "text/plain", "See Other");
  });
  server.on("/master/off", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    pendingStop = true;
    server.sendHeader("Location", "/" + pwQuery());
    server.send(303, "text/plain", "See Other");
  });

  // Rotta /ota registrata dalla libreria, con l'auth di questo progetto.
  static char otaRedirect[48];
  snprintf(otaRedirect, sizeof(otaRedirect), "/%s", pwQuery().c_str());
  MxOta.attachWeb(server, "/ota", otaRedirect, ensureAuthorized);

  server.onNotFound([]() {
    if (!ensureAuthorized()) return;
    server.send(404, "text/plain", "Not found");
  });
}

// =========================================================================
// CONSOLE SERIALE
// =========================================================================

static void handleSerial() {
  static char line[160];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (n < sizeof(line) - 1) line[n++] = c; continue; }
    line[n] = '\0'; n = 0;

    if (strncmp(line, "id set ", 7) == 0) {
      char* args[5] = {0}; int a = 0; char* p = line + 7;
      while (*p && a < 5) {
        while (*p == ' ') p++;
        if (*p == '"') { args[a++] = ++p; while (*p && *p != '"') p++; }
        else           { args[a++] = p;   while (*p && *p != ' ') p++; }
        if (*p) *p++ = '\0';
      }
      if (a >= 2) {
        MxIdentity.provision(args[0], args[1], args[2], args[3], args[4]);
        Serial.println("[ID] scritto. Riavvia per applicare.");
      } else {
        Serial.println("uso: id set \"Nome Pista\" \"nome-device\" [apiBase] [apiToken] [ntfyUrl]");
      }
    } else if (strcmp(line, "id show") == 0) {
      MxIdentity.printTo(Serial);
    } else if (strcmp(line, "wifi reset") == 0) {
      MxNet.factoryReset("comando seriale");
    } else if (strcmp(line, "status") == 0) {
      char buf[420]; MxCore.statusJson(buf, sizeof(buf));
      Serial.println(buf);
      Serial.printf("[PUMP] master=%s raw=%d bar=%.2f campioni=%d\n",
                    masterOn ? "ON" : "OFF", raw, valore, pressHistCount);
    } else if (strcmp(line, "cfg") == 0) {
      Serial.printf("[CFG] max=%.2f bar/%lus  min=%.2f bar/%lus  taratura=%u->%.2f bar  ping=%s %s\n",
                    cfgp.maxCb / 100.0f, (unsigned long)cfgp.maxDelayS,
                    cfgp.minCb / 100.0f, (unsigned long)cfgp.minDelayS,
                    cfgp.calRaw, cfgp.calCb / 100.0f,
                    cfgp.pingEnabled ? "on" : "off", cfgp.pingHost);
      Serial.printf("[CFG] notifica avvio=%s (%s)  topic=%s  causa ultimo riavvio: %s\n",
                    cfgp.bootNotify ? "on" : "off",
                    bootNotifySent ? "gia' inviata" : "in attesa",
                    ntfyTopic().c_str(),
                    resetReasonText().c_str());
      Serial.printf("[CFG] sequenza: rele4->master %s, master->pompa1 %s, "
                    "pompa1->pompa2 %s | sfalsato: %s, tieni %s\n",
                    msText(cfgp.seqRelay4Ms).c_str(), msText(cfgp.seqMasterMs).c_str(),
                    msText(cfgp.seqPump2Ms).c_str(),  msText(cfgp.seqStag2Ms).c_str(),
                    msText(cfgp.seqStagHoldMs).c_str());
    } else if (strcmp(line, "cfg reset") == 0) {
      cfgp = CFG_DEFAULT;
      cfgSave();
      Serial.println("[CFG] riportata ai valori di fabbrica");
    } else if (strcmp(line, "boot ntfy") == 0) {
      // Prova sul campo senza dover riavviare: stesso messaggio dell'avvio.
      sendNtfyNotification("MxIrrigation - pompe: prova notifica",
                           buildBootMessage(), "low", "electric_plug");
      Serial.println("[NTFY] notifica di prova inviata (esito in /logs)");
    } else if (strcmp(line, "ota") == 0) {
      MxOta.pending = true;
      Serial.println("[OTA] check richiesto");
    } else if (strcmp(line, "off") == 0) {
      pendingStop = true;
      Serial.println("[PUMP] stop richiesto");
    } else if (strcmp(line, "reboot") == 0) {
      ESP.restart();
    } else if (line[0]) {
      Serial.println("comandi: id set / id show / wifi reset / status / cfg / cfg reset / boot ntfy / ota / off / reboot");
    }
  }
}

// =========================================================================
// SETUP / LOOP
// =========================================================================

// Compone l'endpoint del pull OTA. Definita PRIMA di setup() apposta: l'IDE
// non genera sempre il prototipo per una funzione usata prima di essere
// definita, ed e' un inciampo gia' visto in questo progetto.
void buildOtaUrl() {
  otaHttpUrl[0] = 0;
  if (!OTA_HTTP_BASE || !OTA_HTTP_BASE[0]) {
    Serial.println("[OTA] pull disabilitato (OTA_HTTP_BASE vuoto)");
    return;
  }

  const char* id = MxIdentity.provisioned() ? MxIdentity.deviceId() : "";
  if (!id[0]) {
    // Senza identita' il server non sa chi ha chiamato, ma il binario dipende
    // da "fw", non da "id": si avvisa e si manda un segnaposto riconoscibile.
    Serial.println("[OTA] identita' non impostata: il server vedra' id=sconosciuto");
    id = "sconosciuto";
  }

  int n = snprintf(otaHttpUrl, sizeof(otaHttpUrl), "%s?fw=%s&id=%s&token=%s",
                   OTA_HTTP_BASE, OTA_FW_NAME, id, OTA_TOKEN ? OTA_TOKEN : "");
  if (n < 0 || n >= (int)sizeof(otaHttpUrl)) {
    otaHttpUrl[0] = 0;
    Serial.println("[OTA] URL troppo lungo (max 223 caratteri): pull disabilitato");
    return;
  }
  Serial.printf("[OTA] pull: fw=%s id=%s\n", OTA_FW_NAME, id);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("MxIrrigation Pump Controller - MxSolutions.it");
  Serial.printf("Firmware %s su MxSolutionCore %s\n", FW_VERSION, MXCORE_VERSION);
  Serial.println("----------------------------------------");

  // Relè a riposo PRIMA della rete: MxCore.begin() può restare fermo minuti
  // sul captive portal, e in quel tempo le pompe non possono essere in uno
  // stato indefinito.
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }
  masterOn = false;
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(analogPin, INPUT);

  cfgLoad();
  pressClear();

  // L'identita' serve PRIMA di comporre l'URL OTA. load() e' una lettura NVS
  // idempotente: MxCore.begin() la rifara' per conto suo.
  MxIdentity.load();
  buildOtaUrl();

  cfg.fwVersion       = FW_VERSION;
  cfg.wdtSeconds      = 15;
  cfg.otaPassword     = OTA_PASSWORD;
  cfg.otaHttpUrl      = otaHttpUrl;
  cfg.factoryResetPin = FACTORY_RESET_PIN;
  cfg.requireIdentity = false;   // le pompe devono funzionare anche senza identita'
  cfg.staticIP        = STATIC_IP;   // default di fabbrica, il portale lo scavalca
  cfg.gateway         = GATEWAY;
  cfg.subnet          = SUBNET;
  cfg.dns1            = DNS1;
  cfg.dns2            = DNS2;

  MxCore.begin(cfg);

  bootMs = millis();
  pressureTimer = millis() + 1000;

  xTaskCreate(pingTaskFn, "mxping", 4096, nullptr, 1, &pingTaskHandle);
  if (pingTaskHandle == nullptr) {
    Serial.println("[PING] task non creato: ping disattivato");
    cfgp.pingEnabled = false;
  }

  registerRoutes();
  server.begin();
  Serial.println("[HTTP] server avviato sulla porta 80");
  addEventLog("BOOT: firmware " + String(FW_VERSION) + " avviato, causa: " + resetReasonText());
  Serial.println("[BOOT] causa del riavvio: " + resetReasonText());
}

void loop() {
  MxCore.loop();            // watchdog + WiFi + OTA + persistenza ora
  server.handleClient();
  runPumpSequencer();
  servicePingMonitor();     // [2] non blocca mai
  serviceBootNotification();// [4] una volta sola, quando rete e ora ci sono
  handleSerial();

  if (millis() >= pressureTimer) {
    pressureTimer = millis() + 1000;

    int r1 = analogRead(analogPin);
    int r2 = analogRead(analogPin);
    int r3 = analogRead(analogPin);
    raw = median3(r1, r2, r3);

    uint16_t cb = rawToCentibar(raw);
    valore = cb / 100.0f;

    // [3] Il sensore si legge sempre (i trip e il valore a video servono
    // anche a pompe ferme), ma nello storico entrano solo i campioni
    // registrati con le pompe in moto.
    if (masterOn) {
      if (!pressWasRecording && pressHistCount > 0) pressPush(PRESS_GAP);
      pressPush(cb);
      pressWasRecording = true;
    } else {
      pressWasRecording = false;
    }

    checkHighPressureTrip();
    checkLowPressureTripWhilePumpsOn();
  }

  // LED: acceso fisso con la rete su, lampeggio se manca.
  if (!MxNet.connected()) {
    static unsigned long ledMs = 0;
    if (millis() - ledMs >= 1000) {
      ledMs = millis();
      digitalWrite(ledPin, !digitalRead(ledPin));
    }
  } else {
    digitalWrite(ledPin, HIGH);
  }
}
