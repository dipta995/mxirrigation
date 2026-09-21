/**
 * ============================================================================
 * Progetto:    MxIrrigation - Pannello di stato
 * File:        mxirrigation-display-cyd.ino
 * Autore:      Nicola Deboni - MxSolutions
 * Versione:    1.2.2
 * Ultima mod.: 2026-09-19
 * Repository:  vault "03 - Elettronica/Progetti/mxirrigation/firmware/
 *              mxirrigation-display-cyd" -> vedi [[status-display]] e
 *              [[mxirrigation]]
 * ----------------------------------------------------------------------------
 * Descrizione:
 *   Pannello da quadro su ESP32-2432S028R ("Cheap Yellow Display"): interroga
 *   via HTTP i nodi di MxIrrigation e mostra a schermo lo stato di pompe e
 *   valvole. Solo lettura: non comanda nulla.
 *
 *   I nodi da interrogare NON sono nel sorgente: si configurano da una pagina
 *   web servita dal pannello stesso (IP, tipo del nodo, nome, intervallo di
 *   aggiornamento) e restano in NVS. Tipi: valvole latch, pompa, stazione 4
 *   rele', valvola a T.
 *
 *   Tre schermate, si cambia toccando lo schermo:
 *     1. Panoramica  - un riquadro per nodo, stato sintetico
 *     2. Valvole     - le 8 zone di ogni centralina latch
 *     3. Pompa       - pressione, soglie, relè, allarmi
 *
 * Hardware:
 *   Scheda:      ESP32-2432S028R (ESP32-WROOM + ILI9341 240x320 + touch
 *                resistivo XPT2046). Variante "R" = resistivo.
 *   Periferiche: display 2,8" SPI, touch su bus SPI separato, retroilluminazione
 *                a PWM, LED RGB di bordo, LDR
 *
 *   Pin (fissi sulla scheda, non modificabili via cablaggio):
 *     GPIO   funzione                   note
 *     -- display ILI9341, bus SPI dedicato --
 *     14     TFT SCLK
 *     13     TFT MOSI
 *     12     TFT MISO
 *     15     TFT CS
 *     2      TFT DC
 *     21     Retroilluminazione         PWM, attiva alta
 *     -- touch XPT2046, secondo bus SPI --
 *     25     TOUCH SCLK
 *     32     TOUCH MOSI
 *     39     TOUCH MISO                 solo ingresso
 *     33     TOUCH CS
 *     36     TOUCH IRQ                  solo ingresso
 *     -- altro, non usato da questo firmware --
 *     4/16/17 LED RGB di bordo          attivi BASSI
 *     34     LDR (sensore di luce)
 *     26/22/27 amplificatore audio e I2C esposti sul connettore
 *     0      Reset di fabbrica (BOOT)   a massa >= 3 s all'avvio
 *
 *     ⚠️ I pin sopra sono quelli documentati per la 2432S028R e NON sono stati
 *     verificati su hardware. Se lo schermo resta bianco o i tocchi cadono
 *     fuori posto, il primo posto da controllare è questa tabella.
 *     ⚠️ GPIO 0 è condiviso col tasto BOOT della scheda: va bene per il reset
 *     di fabbrica, ma tenerlo premuto all'accensione entra anche in modalità
 *     di programmazione se collegato al USB.
 *
 * Arduino IDE / arduino-cli:
 *   Board:            ESP32 Dev Module / esp32:esp32:esp32
 *   Core version:     esp32 3.3.11
 *   Partition Scheme: Minimal SPIFFS (min_spiffs) - obbligatorio
 *   Flash:            QIO 80MHz, 4MB
 *   PSRAM:            disabilitata (la 2432S028R non ne ha)
 *
 *   Librerie (nome - versione esatta testata):
 *     MxSolutionCore  - 0.1.4
 *     LovyanGFX       - 1.2.29   (la 1.1.12 NON compila col core esp32 3.x:
 *                                il ramo legacy v0 si compila sempre e usa API
 *                                rimosse dall'IDF5)
 *     ArduinoJson     - 7.4.2
 *
 *   Scelta LovyanGFX invece di TFT_eSPI: la configurazione dei pin sta dentro
 *   questo file (classe LGFX qui sotto), mentre TFT_eSPI va configurata
 *   modificando User_Setup.h DENTRO la libreria — che è condivisa con gli
 *   sketch TTGO T-Display e T4, e cambiarla li romperebbe.
 *
 * Note di versione:
 *   2026-09-19 1.2.2 - FIX (DS9): un solo poll fallito non marca piu' il nodo
 *                      offline. Servono NODE_FAIL_LIMIT (2) fallimenti
 *                      CONSECUTIVI. Un timeout isolato, su WiFi, e' normale
 *                      amministrazione: prima faceva lampeggiare in rosso
 *                      nodi perfettamente vivi. Il nodo disabilitato resta
 *                      un caso a parte, fuori dal contatore.
 *   2026-09-19 1.2.1 - FIX: i nodi che spediscono /status in chunked (valvole e
 *                      stazione 4 rele') venivano SEMPRE segnalati "risposta non
 *                      valida". deserializeJson leggeva da getStream(), che e' il
 *                      socket grezzo col framing dei chunk ancora dentro. Ora si
 *                      passa da getString(), che il framing lo toglie.
 *                      Inoltre typeMismatch viene azzerato a ogni tentativo.
 *                      Corretto anche il default della centralina valvole:
 *                      era 192.168.5.44 (l'IP del vecchio display TTGO), ma
 *                      il firmware valvole compila 192.168.5.42 dalla 1011.
 *   2026-09-18 1.2.0 - Quarto tipo di nodo: "valvola a T", che ora espone
 *                      /status (tvalve 0.5).
 *   2026-09-18 1.1.0 - Terzo tipo di nodo: "stazione 4 rele'". E il pannello ora
 *                      legge il campo "type" dichiarato dal nodo e segnala il
 *                      TIPO ERRATO invece di mostrare una pagina vuota, che
 *                      sembrerebbe un guasto del nodo.
 *   2026-09-18 1.0.0 - Prima versione. Sostituisce il vecchio
 *                      mxirrigation-display (TTGO T4), che non compila e
 *                      interrogava un solo nodo con IP fisso nel sorgente.
 * ============================================================================
 */

#include <MxCore.h>
#include <WiFi.h>          // WiFiClient: MxCore non lo espone di suo
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

const char* FW_VERSION = "1.2.2";

// =========================================================================
// DISPLAY: configurazione della ESP32-2432S028R
// =========================================================================

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341   _panel;
  lgfx::Bus_SPI         _bus;
  lgfx::Light_PWM       _light;
  lgfx::Touch_XPT2046   _touch;

public:
  LGFX(void) {
    {
      auto c = _bus.config();
      c.spi_host    = SPI2_HOST;
      c.spi_mode    = 0;
      c.freq_write  = 40000000;
      c.freq_read   = 16000000;
      c.spi_3wire   = false;
      c.use_lock    = true;
      c.dma_channel = SPI_DMA_CH_AUTO;
      c.pin_sclk    = 14;
      c.pin_mosi    = 13;
      c.pin_miso    = 12;
      c.pin_dc      = 2;
      _bus.config(c);
      _panel.setBus(&_bus);
    }
    {
      auto c = _panel.config();
      c.pin_cs           = 15;
      c.pin_rst          = -1;
      c.pin_busy         = -1;
      c.panel_width      = 240;
      c.panel_height     = 320;
      c.offset_x         = 0;
      c.offset_y         = 0;
      c.offset_rotation  = 0;
      c.readable         = true;
      c.invert           = false;
      c.rgb_order        = false;
      c.dlen_16bit       = false;
      c.bus_shared       = false;
      _panel.config(c);
    }
    {
      auto c = _light.config();
      c.pin_bl      = 21;
      c.invert      = false;
      c.freq        = 44100;
      c.pwm_channel = 7;
      _light.config(c);
      _panel.setLight(&_light);
    }
    {
      // Il touch sta su un secondo bus SPI: non condivide nulla col display.
      auto c = _touch.config();
      c.x_min      = 300;
      c.x_max      = 3900;
      c.y_min      = 300;
      c.y_max      = 3900;
      c.pin_int    = 36;
      c.bus_shared = false;
      c.offset_rotation = 0;
      c.spi_host   = SPI3_HOST;
      c.freq       = 1000000;
      c.pin_sclk   = 25;
      c.pin_mosi   = 32;
      c.pin_miso   = 39;
      c.pin_cs     = 33;
      _touch.config(c);
      _panel.setTouch(&_touch);
    }
    setPanel(&_panel);
  }
};

LGFX tft;

// Landscape: 320 x 240.
static const int SCR_W = 320;
static const int SCR_H = 240;

// Palette (RGB565)
static const uint16_t COL_BG      = 0x0000;  // nero
static const uint16_t COL_HEAD    = 0x2124;  // grigio scuro
static const uint16_t COL_TEXT    = 0xFFFF;
static const uint16_t COL_DIM     = 0x8410;  // grigio
static const uint16_t COL_OK      = 0x07E0;  // verde
static const uint16_t COL_WARN    = 0xFD20;  // arancio
static const uint16_t COL_ALARM   = 0xF800;  // rosso
static const uint16_t COL_ACCENT  = 0x05BF;  // azzurro
static const uint16_t COL_BOX     = 0x18E3;

// =========================================================================
// TIPI  (prima di qualunque funzione: i prototipi automatici dell'IDE
//        Arduino vengono inseriti davanti alla prima funzione del file)
// =========================================================================

// I quattro tipi corrispondono alle stringhe che i nodi dichiarano nel campo
// "type" del loro /status: "latch", "pump", "relay4", "tvalve".
enum NodeType { NODE_LATCH = 0, NODE_PUMP = 1, NODE_RELAY4 = 2, NODE_TVALVE = 3 };

// Quanti tentativi consecutivi falliti prima di dichiarare offline un nodo.
// A 1 (il comportamento fino alla 1.2.1) un singolo timeout dipingeva di rosso
// un nodo sano: con connectTimeout a 1500 ms su WiFi capita di routine, basta
// che il nodo stia servendo un'altra richiesta. Non alzarlo troppo: il dato a
// schermo resta vecchio fino a NODE_FAIL_LIMIT intervalli di poll.
static const uint8_t NODE_FAIL_LIMIT = 2;

static const int MAX_NODES = 6;
static const int MAX_VALVES = 8;

struct NodeCfg {
  bool     enabled;
  uint8_t  type;          // NodeType
  char     host[40];      // IP o hostname
  char     name[24];      // etichetta a schermo
};

struct NodeState {
  bool     online;
  uint8_t  failCount;          // tentativi falliti CONSECUTIVI (0 = ultimo ok)
  uint32_t lastOkMs;
  uint32_t lastTryMs;
  int      httpCode;
  char     reportedType[12];   // "type" dichiarato dal nodo ("" = non lo dichiara)
  bool     typeMismatch;       // configurato come X ma si dichiara Y

  // comuni
  char     firmware[16];
  char     site[32];
  char     deviceId[40];
  long     rssi;
  bool     timeSynced;
  uint32_t uptimeS;

  // latch
  bool     valves[MAX_VALVES];
  int      valveCount;
  int      queue;
  bool     valveBusy;

  // pompa
  bool     masterOn;
  bool     busy;
  float    pressureBar;
  float    maxBar;
  float    minBar;
  bool     alarmHigh;
  bool     alarmLow;
  bool     relays[4];
  int      relayCount;

  // valvola a T
  bool     valveOpen;
  bool     pulseActive;
};

struct DisplayCfg {
  NodeCfg  nodes[MAX_NODES];
  uint16_t pollSeconds;      // ogni quanti secondi interrogare
  uint16_t rotateSeconds;    // rotazione automatica pagine, 0 = ferma
  uint8_t  brightness;       // 10..255
};

// PRIMA funzione del file: l'IDE Arduino inserisce qui tutti i prototipi
// automatici, quindi da questo punto in giu' ogni tipo dev'essere gia'
// definito. Spostarla piu' in alto rompe la compilazione con
// "'NodeState' does not name a type".
static const char* typeName(uint8_t t) {
  switch (t) {
    case NODE_PUMP:   return "pump";
    case NODE_RELAY4: return "relay4";
    case NODE_TVALVE: return "tvalve";
    default:          return "latch";
  }
}

// =========================================================================
// CONFIGURAZIONE (NVS "mxdisp")
// =========================================================================

DisplayCfg dcfg;
NodeState  nstate[MAX_NODES];

Preferences prefs;
static const char* CFG_NS = "mxdisp";

static const uint16_t POLL_MIN = 2, POLL_MAX = 300;
static const uint16_t ROT_MAX  = 300;

static uint16_t clampU16v(uint16_t v, uint16_t lo, uint16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void cfgDefaults() {
  memset(&dcfg, 0, sizeof(dcfg));
  dcfg.pollSeconds   = 10;
  dcfg.rotateSeconds = 0;
  dcfg.brightness    = 200;

  // Default coerenti con gli indirizzi di fabbrica dell'impianto.
  dcfg.nodes[0].enabled = true;
  dcfg.nodes[0].type    = NODE_LATCH;
  strncpy(dcfg.nodes[0].host, "192.168.5.42", sizeof(dcfg.nodes[0].host) - 1);
  strncpy(dcfg.nodes[0].name, "Valvole", sizeof(dcfg.nodes[0].name) - 1);

  dcfg.nodes[1].enabled = true;
  dcfg.nodes[1].type    = NODE_PUMP;
  strncpy(dcfg.nodes[1].host, "192.168.5.43", sizeof(dcfg.nodes[1].host) - 1);
  strncpy(dcfg.nodes[1].name, "Pompa", sizeof(dcfg.nodes[1].name) - 1);
}

void cfgClampAll() {
  dcfg.pollSeconds   = clampU16v(dcfg.pollSeconds, POLL_MIN, POLL_MAX);
  dcfg.rotateSeconds = clampU16v(dcfg.rotateSeconds, 0, ROT_MAX);
  if (dcfg.brightness < 10) dcfg.brightness = 10;
  for (int i = 0; i < MAX_NODES; i++) {
    if (dcfg.nodes[i].type > NODE_TVALVE) dcfg.nodes[i].type = NODE_LATCH;
    if (dcfg.nodes[i].host[0] == '\0') dcfg.nodes[i].enabled = false;
    dcfg.nodes[i].host[sizeof(dcfg.nodes[i].host) - 1] = '\0';
    dcfg.nodes[i].name[sizeof(dcfg.nodes[i].name) - 1] = '\0';
  }
}

void cfgLoad() {
  cfgDefaults();
  prefs.begin(CFG_NS, true);
  bool has = prefs.isKey("poll");
  if (has) {
    dcfg.pollSeconds   = prefs.getUShort("poll", dcfg.pollSeconds);
    dcfg.rotateSeconds = prefs.getUShort("rot", dcfg.rotateSeconds);
    dcfg.brightness    = prefs.getUChar("bri", dcfg.brightness);
    for (int i = 0; i < MAX_NODES; i++) {
      char k[12];
      snprintf(k, sizeof(k), "en%d", i);  dcfg.nodes[i].enabled = prefs.getBool(k, false);
      snprintf(k, sizeof(k), "ty%d", i);  dcfg.nodes[i].type    = prefs.getUChar(k, NODE_LATCH);
      snprintf(k, sizeof(k), "ho%d", i);  prefs.getString(k, dcfg.nodes[i].host, sizeof(dcfg.nodes[i].host));
      snprintf(k, sizeof(k), "na%d", i);  prefs.getString(k, dcfg.nodes[i].name, sizeof(dcfg.nodes[i].name));
    }
  }
  prefs.end();
  cfgClampAll();
}

void cfgSave() {
  cfgClampAll();
  prefs.begin(CFG_NS, false);
  prefs.putUShort("poll", dcfg.pollSeconds);
  prefs.putUShort("rot", dcfg.rotateSeconds);
  prefs.putUChar("bri", dcfg.brightness);
  for (int i = 0; i < MAX_NODES; i++) {
    char k[12];
    snprintf(k, sizeof(k), "en%d", i);  prefs.putBool(k, dcfg.nodes[i].enabled);
    snprintf(k, sizeof(k), "ty%d", i);  prefs.putUChar(k, dcfg.nodes[i].type);
    snprintf(k, sizeof(k), "ho%d", i);  prefs.putString(k, dcfg.nodes[i].host);
    snprintf(k, sizeof(k), "na%d", i);  prefs.putString(k, dcfg.nodes[i].name);
  }
  prefs.end();
}

// =========================================================================
// STATO
// =========================================================================

WebServer server(80);
MxCoreConfig cfg;

int  page = 0;                 // 0 panoramica, 1 valvole, 2 pompa
int  pollIndex = 0;            // round-robin sui nodi
unsigned long lastPollMs = 0;
unsigned long lastDrawMs = 0;
unsigned long lastTouchMs = 0;
unsigned long lastRotateMs = 0;
bool redrawNeeded = true;

// =========================================================================
// INTERROGAZIONE DEI NODI
// =========================================================================
//
// Un nodo per volta, a rotazione: se si interrogassero tutti nello stesso
// giro, con sei nodi irraggiungibili il loop() resterebbe fermo per la somma
// dei timeout. Così il costo massimo per giro è un timeout solo.

// Un tentativo andato male. Il nodo NON diventa offline subito: lo diventa
// solo quando i fallimenti consecutivi arrivano a NODE_FAIL_LIMIT. Finche' sta
// sotto soglia resta "online" con l'ultimo dato buono, e la griglia lo segnala
// con il bordo arancio (vedi drawOverview).
static void pollFailed(NodeState& ns) {
  if (ns.failCount < 255) ns.failCount++;
  if (ns.failCount >= NODE_FAIL_LIMIT) ns.online = false;
}

void pollNode(int i) {
  NodeCfg& nc = dcfg.nodes[i];
  NodeState& ns = nstate[i];

  ns.lastTryMs = millis();
  // Un esito precedente non deve sopravvivere a un tentativo fallito: senza
  // questo, un nodo spento continuerebbe a essere segnalato "tipo errato"
  // nella pagina /config, che controlla typeMismatch prima di online.
  ns.typeMismatch = false;

  // Nodo disabilitato o senza host: non e' un tentativo fallito, e' un nodo
  // che non si interroga. Fuori dal contatore, altrimenti uno appena
  // riattivato resterebbe grigio per un giro in piu' del necessario.
  if (!nc.enabled || nc.host[0] == '\0') {
    ns.online = false;
    ns.failCount = 0;
    return;
  }

  // WiFi del pannello giu': questo invece e' a tutti gli effetti un dato non
  // scaricato, quindi passa dal contatore come gli altri. Un blip dell'AP non
  // deve spegnere l'intera griglia.
  if (!MxNet.connected()) {
    pollFailed(ns);
    return;
  }

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(1500);
  http.setTimeout(2500);
  http.setReuse(false);

  String url = String("http://") + nc.host + "/status";
  if (!http.begin(client, url)) {
    ns.httpCode = -1;
    pollFailed(ns);
    return;
  }

  int code = http.GET();
  ns.httpCode = code;

  if (code != 200) {
    http.end();
    pollFailed(ns);
    return;
  }

  // ATTENZIONE, non usare deserializeJson(doc, http.getStream()):
  // la centralina valvole e la stazione 4 relè spediscono il JSON con
  // setContentLength(CONTENT_LENGTH_UNKNOWN), cioè in **chunked transfer
  // encoding**. getStream() restituisce il socket GREZZO, con dentro ancora
  // le intestazioni dei chunk ("1a3\r\n{...\r\n0\r\n\r\n"): il parser legge
  // quella cifra esadecimale come inizio del documento e fallisce sempre.
  // HTTPClient toglie il framing solo dentro getString()/writeToStream().
  // I payload sono piccoli (< 2 KB), quindi getString() va benissimo.
  String body = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);

  if (err) {
    ns.httpCode = -2;
    Serial.printf("[POLL] %s JSON: %s\n", nc.host, err.c_str());
    pollFailed(ns);
    return;
  }

  ns.online = true;
  ns.failCount = 0;
  ns.lastOkMs = millis();

  // Il nodo dichiara cosa è: se non combacia col tipo configurato a mano, il
  // pannello mostrerebbe una pagina vuota senza dire perché. Meglio dirlo.
  strncpy(ns.reportedType, doc["type"] | "", sizeof(ns.reportedType) - 1);
  ns.reportedType[sizeof(ns.reportedType) - 1] = '\0';
  ns.typeMismatch = (ns.reportedType[0] != '\0' &&
                     strcmp(ns.reportedType, typeName(nc.type)) != 0);

  strncpy(ns.firmware, doc["firmware"] | "", sizeof(ns.firmware) - 1);
  strncpy(ns.site,     doc["site"]     | "", sizeof(ns.site) - 1);
  strncpy(ns.deviceId, doc["device_id"] | "", sizeof(ns.deviceId) - 1);
  ns.rssi       = doc["wifi_rssi"] | 0L;
  ns.timeSynced = doc["time_synced"] | false;
  ns.uptimeS    = doc["uptime_s"] | 0UL;

  if (nc.type == NODE_LATCH) {
    ns.valveCount = 0;
    JsonArray va = doc["valves"].as<JsonArray>();
    for (JsonObject v : va) {
      if (ns.valveCount >= MAX_VALVES) break;
      ns.valves[ns.valveCount++] = v["on"] | false;
    }
    ns.queue     = doc["valve_queue"] | 0;
    ns.valveBusy = doc["valve_busy"] | false;
  } else if (nc.type == NODE_RELAY4 || nc.type == NODE_TVALVE) {
    int r = 0;
    JsonArray ra = doc["relays"].as<JsonArray>();
    for (JsonObject v : ra) {
      if (r >= 4) break;
      ns.relays[r++] = v["on"] | false;
    }
    ns.relayCount = r;
    ns.valveOpen   = doc["valve_open"] | false;
    ns.pulseActive = doc["pulse_active"] | false;
  } else {
    ns.masterOn    = doc["master_on"] | false;
    ns.busy        = doc["busy"] | false;
    ns.pressureBar = doc["pressure_bar"] | 0.0f;
    ns.maxBar      = doc["max_bar"] | 0.0f;
    ns.minBar      = doc["min_bar"] | 0.0f;
    ns.alarmHigh   = doc["alarm_high"] | false;
    ns.alarmLow    = doc["alarm_low"] | false;
    int r = 0;
    JsonArray ra = doc["relays"].as<JsonArray>();
    for (JsonObject v : ra) {
      if (r >= 4) break;
      ns.relays[r++] = v["on"] | false;
    }
  }

  redrawNeeded = true;
}

void servicePolling() {
  if (dcfg.pollSeconds == 0) return;
  unsigned long now = millis();

  // Il periodo è diviso per il numero di nodi abilitati: ogni nodo viene
  // interrogato una volta per periodo, ma le richieste sono distribuite.
  int enabled = 0;
  for (int i = 0; i < MAX_NODES; i++) if (dcfg.nodes[i].enabled) enabled++;
  if (enabled == 0) return;

  unsigned long slot = (dcfg.pollSeconds * 1000UL) / enabled;
  if (slot < 500) slot = 500;
  if (now - lastPollMs < slot) return;
  lastPollMs = now;

  for (int n = 0; n < MAX_NODES; n++) {
    pollIndex = (pollIndex + 1) % MAX_NODES;
    if (dcfg.nodes[pollIndex].enabled) { pollNode(pollIndex); break; }
  }
}

// =========================================================================
// DISEGNO
// =========================================================================

String ageText(const NodeState& ns) {
  if (ns.lastOkMs == 0) return "mai";
  unsigned long s = (millis() - ns.lastOkMs) / 1000UL;
  if (s < 60) return String(s) + "s fa";
  if (s < 3600) return String(s / 60) + "m fa";
  return String(s / 3600) + "h fa";
}

void drawHeader(const char* title) {
  tft.fillRect(0, 0, SCR_W, 22, COL_HEAD);
  tft.setTextColor(COL_TEXT, COL_HEAD);
  tft.setTextDatum(lgfx::middle_left);
  tft.drawString(title, 6, 11);

  tft.setTextDatum(lgfx::middle_right);
  String right = MxNet.connected() ? MxNet.ip() : String("no rete");
  tft.setTextColor(MxNet.connected() ? COL_OK : COL_ALARM, COL_HEAD);
  tft.drawString(right, SCR_W - 6, 11);
  tft.setTextDatum(lgfx::top_left);
}

void drawFooter(const char* hint) {
  tft.fillRect(0, SCR_H - 16, SCR_W, 16, COL_HEAD);
  tft.setTextColor(COL_DIM, COL_HEAD);
  tft.setTextDatum(lgfx::middle_center);
  tft.drawString(hint, SCR_W / 2, SCR_H - 8);
  tft.setTextDatum(lgfx::top_left);
}

void drawOverview() {
  tft.fillRect(0, 22, SCR_W, SCR_H - 38, COL_BG);
  drawHeader("MxIrrigation");

  int y = 28;
  int shown = 0;

  for (int i = 0; i < MAX_NODES; i++) {
    if (!dcfg.nodes[i].enabled) continue;
    if (shown >= 4) break;

    NodeCfg& nc = dcfg.nodes[i];
    NodeState& ns = nstate[i];

    // Tre stati, non due: online e fresco (bordo normale), online ma con un
    // tentativo gia' fallito — dato vecchio, arancio — e offline (rosso).
    // Senza l'arancio la tolleranza introdotta con NODE_FAIL_LIMIT sarebbe
    // muta: si vedrebbe un nodo verde con numeri fermi e nessun indizio.
    uint16_t border = !ns.online   ? COL_ALARM
                    : ns.failCount ? COL_WARN
                                   : COL_BOX;
    tft.drawRect(4, y, SCR_W - 8, 44, border);

    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextDatum(lgfx::top_left);
    tft.drawString(nc.name, 10, y + 5);

    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString(nc.host, 10, y + 26);

    tft.setTextDatum(lgfx::top_right);
    if (!ns.online) {
      tft.setTextColor(COL_ALARM, COL_BG);
      tft.drawString("OFFLINE", SCR_W - 12, y + 5);
      tft.setTextColor(COL_DIM, COL_BG);
      tft.drawString(String("ultimo ") + ageText(ns), SCR_W - 12, y + 26);
    } else if (ns.typeMismatch) {
      // Risponde, ma non è il tipo che ci si aspetta: senza questo avviso la
      // riga resterebbe vuota e sembrerebbe un guasto del nodo.
      tft.setTextColor(COL_WARN, COL_BG);
      tft.drawString("TIPO ERRATO", SCR_W - 12, y + 5);
      tft.setTextColor(COL_DIM, COL_BG);
      tft.drawString(String("si dichiara '") + ns.reportedType + "'", SCR_W - 12, y + 26);
    } else if (nc.type == NODE_TVALVE) {
      tft.setTextColor(ns.valveOpen ? COL_ACCENT : COL_DIM, COL_BG);
      tft.drawString(ns.valveOpen ? "APERTA" : "chiusa", SCR_W - 12, y + 5);
      tft.setTextColor(ns.pulseActive ? COL_WARN : COL_DIM, COL_BG);
      tft.drawString(ns.pulseActive ? "impulso in corso" : "valvola a T", SCR_W - 12, y + 26);
    } else if (nc.type == NODE_RELAY4) {
      int on = 0;
      for (int r = 0; r < ns.relayCount; r++) if (ns.relays[r]) on++;
      tft.setTextColor(on > 0 ? COL_ACCENT : COL_DIM, COL_BG);
      tft.drawString(String(on) + "/" + String(ns.relayCount) + " attivi", SCR_W - 12, y + 5);
      tft.setTextColor(COL_DIM, COL_BG);
      tft.drawString("4 rele'", SCR_W - 12, y + 26);
    } else if (nc.type == NODE_LATCH) {
      int open = 0;
      for (int v = 0; v < ns.valveCount; v++) if (ns.valves[v]) open++;
      tft.setTextColor(open > 0 ? COL_ACCENT : COL_DIM, COL_BG);
      tft.drawString(String(open) + "/" + String(ns.valveCount) + " aperte",
                     SCR_W - 12, y + 5);
      tft.setTextColor(COL_DIM, COL_BG);
      tft.drawString(ns.queue > 0 ? String("coda ") + ns.queue : String("pronta"),
                     SCR_W - 12, y + 26);
    } else {
      if (ns.alarmHigh || ns.alarmLow) {
        tft.setTextColor(COL_ALARM, COL_BG);
        tft.drawString(ns.alarmHigh ? "ALTA PRESS." : "BASSA PRESS.", SCR_W - 12, y + 5);
      } else {
        tft.setTextColor(ns.masterOn ? COL_OK : COL_DIM, COL_BG);
        tft.drawString(ns.masterOn ? "POMPE ON" : (ns.busy ? "AVVIO" : "ferme"),
                       SCR_W - 12, y + 5);
      }
      tft.setTextColor(COL_TEXT, COL_BG);
      tft.drawString(String(ns.pressureBar, 2) + " bar", SCR_W - 12, y + 26);
    }

    tft.setTextDatum(lgfx::top_left);
    y += 48;
    shown++;
  }

  if (shown == 0) {
    tft.setTextColor(COL_WARN, COL_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.drawString("Nessun nodo configurato", SCR_W / 2, 100);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString(String("http://") + MxNet.ip() + "/config", SCR_W / 2, 124);
    tft.setTextDatum(lgfx::top_left);
  }

  drawFooter("tocca per cambiare pagina");
}

void drawValves() {
  tft.fillRect(0, 22, SCR_W, SCR_H - 38, COL_BG);
  drawHeader("Valvole");

  int y = 28;
  bool any = false;

  for (int i = 0; i < MAX_NODES; i++) {
    if (!dcfg.nodes[i].enabled || dcfg.nodes[i].type != NODE_LATCH) continue;
    any = true;
    NodeState& ns = nstate[i];

    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(dcfg.nodes[i].name, 6, y);
    if (!ns.online) {
      tft.setTextColor(COL_ALARM, COL_BG);
      tft.drawString("offline", 120, y);
    }
    y += 18;

    // Otto riquadri in due file da quattro.
    int bw = 72, bh = 34, gap = 6;
    for (int v = 0; v < MAX_VALVES; v++) {
      int col = v % 4, row = v / 4;
      int x = 6 + col * (bw + gap);
      int by = y + row * (bh + gap);
      bool on = ns.online && v < ns.valveCount && ns.valves[v];

      tft.fillRect(x, by, bw, bh, on ? COL_ACCENT : COL_BOX);
      tft.setTextColor(on ? COL_BG : COL_DIM, on ? COL_ACCENT : COL_BOX);
      tft.setTextDatum(lgfx::middle_center);
      tft.drawString(String("V") + String(v + 1), x + bw / 2, by + bh / 2 - 6);
      tft.drawString(on ? "APERTA" : "chiusa", x + bw / 2, by + bh / 2 + 8);
      tft.setTextDatum(lgfx::top_left);
    }
    y += 2 * (bh + gap) + 6;
    if (y > SCR_H - 60) break;
  }

  if (!any) {
    tft.setTextColor(COL_WARN, COL_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.drawString("Nessuna centralina valvole configurata", SCR_W / 2, 110);
    tft.setTextDatum(lgfx::top_left);
  }

  drawFooter("tocca per cambiare pagina");
}

void drawPump() {
  tft.fillRect(0, 22, SCR_W, SCR_H - 38, COL_BG);
  drawHeader("Pompa");

  int idx = -1;
  for (int i = 0; i < MAX_NODES; i++) {
    if (dcfg.nodes[i].enabled && dcfg.nodes[i].type == NODE_PUMP) { idx = i; break; }
  }

  if (idx < 0) {
    tft.setTextColor(COL_WARN, COL_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.drawString("Nessun controller pompa configurato", SCR_W / 2, 110);
    tft.setTextDatum(lgfx::top_left);
    drawFooter("tocca per cambiare pagina");
    return;
  }

  NodeState& ns = nstate[idx];

  if (!ns.online) {
    tft.setTextColor(COL_ALARM, COL_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.drawString("OFFLINE", SCR_W / 2, 90);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString(String(dcfg.nodes[idx].host) + "  ultimo dato " + ageText(ns),
                   SCR_W / 2, 118);
    tft.setTextDatum(lgfx::top_left);
    drawFooter("tocca per cambiare pagina");
    return;
  }

  // Pressione, grande.
  uint16_t pcol = COL_OK;
  if (ns.alarmHigh || ns.alarmLow) pcol = COL_ALARM;
  else if (ns.pressureBar > ns.maxBar * 0.9f) pcol = COL_WARN;

  tft.setTextDatum(lgfx::middle_center);
  tft.setTextColor(pcol, COL_BG);
  tft.setTextSize(3);
  tft.drawString(String(ns.pressureBar, 2), SCR_W / 2, 62);
  tft.setTextSize(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("bar", SCR_W / 2, 88);

  // Barra soglie min/max.
  int bx = 20, bw = SCR_W - 40, by = 104, bh = 12;
  tft.drawRect(bx, by, bw, bh, COL_BOX);
  float span = ns.maxBar > 0 ? ns.maxBar * 1.2f : 10.0f;
  int fill = (int)((ns.pressureBar / span) * bw);
  if (fill < 0) fill = 0;
  if (fill > bw) fill = bw;
  tft.fillRect(bx + 1, by + 1, fill > 1 ? fill - 1 : 0, bh - 2, pcol);
  int xmin = bx + (int)((ns.minBar / span) * bw);
  int xmax = bx + (int)((ns.maxBar / span) * bw);
  tft.drawFastVLine(xmin, by - 3, bh + 6, COL_WARN);
  tft.drawFastVLine(xmax, by - 3, bh + 6, COL_ALARM);

  tft.setTextDatum(lgfx::top_left);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString(String("min ") + String(ns.minBar, 2), bx, by + bh + 4);
  tft.setTextDatum(lgfx::top_right);
  tft.drawString(String("max ") + String(ns.maxBar, 2), bx + bw, by + bh + 4);
  tft.setTextDatum(lgfx::top_left);

  // Stato e relè.
  tft.setTextColor(ns.masterOn ? COL_OK : COL_DIM, COL_BG);
  tft.drawString(ns.masterOn ? "POMPE IN FUNZIONE"
                             : (ns.busy ? "AVVIO IN CORSO" : "POMPE FERME"), 10, 146);

  const char* rname[4] = {"MASTER", "POMPA1", "POMPA2", "PRE"};
  int rx = 10;
  for (int r = 0; r < 4; r++) {
    uint16_t c = ns.relays[r] ? COL_OK : COL_BOX;
    tft.fillRect(rx, 168, 70, 24, c);
    tft.setTextColor(ns.relays[r] ? COL_BG : COL_DIM, c);
    tft.setTextDatum(lgfx::middle_center);
    tft.drawString(rname[r], rx + 35, 180);
    tft.setTextDatum(lgfx::top_left);
    rx += 76;
  }

  if (ns.alarmHigh || ns.alarmLow) {
    tft.setTextColor(COL_ALARM, COL_BG);
    tft.drawString(ns.alarmHigh ? "ALLARME ALTA PRESSIONE"
                                : "ALLARME BASSA PRESSIONE", 10, 200);
  } else {
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString(String("aggiornato ") + ageText(ns), 10, 200);
  }

  drawFooter("tocca per cambiare pagina");
}

void drawCurrentPage() {
  switch (page) {
    case 1:  drawValves();   break;
    case 2:  drawPump();     break;
    default: drawOverview(); break;
  }
  lastDrawMs = millis();
  redrawNeeded = false;
}

void serviceTouch() {
  int32_t x, y;
  if (!tft.getTouch(&x, &y)) return;
  unsigned long now = millis();
  if (now - lastTouchMs < 400) return;   // antirimbalzo
  lastTouchMs = now;
  page = (page + 1) % 3;
  lastRotateMs = now;
  redrawNeeded = true;
}

// =========================================================================
// PAGINA WEB DI CONFIGURAZIONE
// =========================================================================

String htmlEscape(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;";
    else o += c;
  }
  return o;
}

void handleConfigPage() {
  String h;
  h.reserve(4096);
  h += "<!DOCTYPE HTML><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  h += "<title>MxIrrigation - Pannello</title><style>"
       "body{font-family:Arial,sans-serif;font-size:14px;margin:12px}"
       "table{border-collapse:collapse;width:100%;max-width:760px}"
       "td,th{padding:6px;border-bottom:1px solid #ddd;text-align:left}"
       "input[type=text]{padding:4px;width:100%;box-sizing:border-box}"
       "input[type=number]{padding:4px;width:7em}"
       ".hint{color:#666;font-size:12px}</style></head><body>";
  h += "<h2>Pannello MxIrrigation</h2>";

  if (server.hasArg("saved")) h += "<p style='color:green'><b>Salvato.</b></p>";

  h += "<p class=hint>Firmware " + String(FW_VERSION) + " · core " + String(MXCORE_VERSION)
     + " · IP " + MxNet.ip() + (MxNet.usingStaticIP() ? " (fisso)" : " (DHCP)") + "</p>";

  h += "<form method='POST' action='/config/save'>";
  h += "<h3>Nodi da interrogare</h3>";
  h += "<table><tr><th>Attivo</th><th>Nome</th><th>Indirizzo</th><th>Tipo</th><th>Stato</th></tr>";

  for (int i = 0; i < MAX_NODES; i++) {
    h += "<tr><td><input type=checkbox name='en" + String(i) + "' value=1";
    if (dcfg.nodes[i].enabled) h += " checked";
    h += "></td>";
    h += "<td><input type=text name='na" + String(i) + "' value='" + htmlEscape(dcfg.nodes[i].name) + "'></td>";
    h += "<td><input type=text name='ho" + String(i) + "' value='" + htmlEscape(dcfg.nodes[i].host) + "' placeholder='192.168.5.42'></td>";
    h += "<td><select name='ty" + String(i) + "'>";
    h += "<option value='0'"; if (dcfg.nodes[i].type == NODE_LATCH)  h += " selected"; h += ">Valvole (latch)</option>";
    h += "<option value='1'"; if (dcfg.nodes[i].type == NODE_PUMP)   h += " selected"; h += ">Pompa</option>";
    h += "<option value='2'"; if (dcfg.nodes[i].type == NODE_RELAY4) h += " selected"; h += ">Stazione 4 rele'</option>";
    h += "<option value='3'"; if (dcfg.nodes[i].type == NODE_TVALVE) h += " selected"; h += ">Valvola a T</option>";
    h += "</select></td>";

    h += "<td class=hint>";
    if (!dcfg.nodes[i].enabled)        h += "-";
    else if (nstate[i].typeMismatch)   h += "<b style='color:#b60'>tipo errato</b>: si dichiara '"
                                          + htmlEscape(nstate[i].reportedType) + "'";
    else if (nstate[i].online)         h += "ok, fw " + String(nstate[i].firmware)
                                          + (nstate[i].reportedType[0] ? "" : " <i>(non dichiara il tipo)</i>")
                                          + (nstate[i].failCount ? " <b style='color:#b60'>- ultimo tentativo fallito, dato non fresco</b>" : "");
    else if (nstate[i].httpCode == -2) h += "risposta non valida";
    else if (nstate[i].httpCode == 404) h += "HTTP 404: firmware senza /status";
    else if (nstate[i].httpCode > 0)   h += "HTTP " + String(nstate[i].httpCode);
    else                               h += "irraggiungibile";
    h += "</td></tr>";
  }
  h += "</table>";
  h += "<p class=hint>Il pannello legge <code>/status</code> di ogni nodo. "
       "Richiede firmware <b>1013+</b> sulla centralina valvole e <b>1.4.1+</b> "
       "sul controller pompa: le versioni precedenti non espongono quel dato.</p>";

  h += "<h3>Aggiornamento</h3><table>";
  h += "<tr><td>Ogni quanti secondi chiedere i dati</td><td><input type=number min='2' max='300' name='poll' value='"
     + String(dcfg.pollSeconds) + "'> s</td></tr>";
  h += "<tr><td>Rotazione automatica pagine <span class=hint>(0 = ferma)</span></td>"
       "<td><input type=number min='0' max='300' name='rot' value='" + String(dcfg.rotateSeconds) + "'> s</td></tr>";
  h += "<tr><td>Luminosita' schermo</td><td><input type=number min='10' max='255' name='bri' value='"
     + String(dcfg.brightness) + "'></td></tr>";
  h += "</table>";
  h += "<p class=hint>I nodi vengono interrogati a turno, uno per volta: con "
       "l'intervallo impostato ognuno viene letto una volta per ciclo, ma le "
       "richieste sono distribuite nel tempo.</p>";

  h += "<br><button type=submit>Salva</button></form>";
  h += String("<p class=hint><a href='/'>Stato</a> &middot; ") + MxOta.linkHtml() + "</p>";
  h += "</body></html>";

  server.send(200, "text/html", h);
}

void handleConfigSave() {
  for (int i = 0; i < MAX_NODES; i++) {
    dcfg.nodes[i].enabled = server.hasArg("en" + String(i));
    if (server.hasArg("ty" + String(i)))
      dcfg.nodes[i].type = (uint8_t)server.arg("ty" + String(i)).toInt();
    if (server.hasArg("ho" + String(i))) {
      String v = server.arg("ho" + String(i)); v.trim();
      strncpy(dcfg.nodes[i].host, v.c_str(), sizeof(dcfg.nodes[i].host) - 1);
      dcfg.nodes[i].host[sizeof(dcfg.nodes[i].host) - 1] = '\0';
    }
    if (server.hasArg("na" + String(i))) {
      String v = server.arg("na" + String(i)); v.trim();
      strncpy(dcfg.nodes[i].name, v.c_str(), sizeof(dcfg.nodes[i].name) - 1);
      dcfg.nodes[i].name[sizeof(dcfg.nodes[i].name) - 1] = '\0';
    }
  }
  if (server.hasArg("poll")) dcfg.pollSeconds   = (uint16_t)server.arg("poll").toInt();
  if (server.hasArg("rot"))  dcfg.rotateSeconds = (uint16_t)server.arg("rot").toInt();
  if (server.hasArg("bri"))  dcfg.brightness    = (uint8_t)server.arg("bri").toInt();

  cfgSave();
  tft.setBrightness(dcfg.brightness);

  // I nodi disattivati o cambiati non devono lasciare dati vecchi a schermo.
  for (int i = 0; i < MAX_NODES; i++) {
    if (!dcfg.nodes[i].enabled) memset(&nstate[i], 0, sizeof(NodeState));
  }
  lastPollMs = 0;
  redrawNeeded = true;

  server.sendHeader("Location", "/config?saved=1");
  server.send(303, "text/plain", "See Other");
}

void handleStatusPage() {
  String h;
  h += "<!DOCTYPE HTML><html><head><meta charset='utf-8'>"
       "<meta name='viewport' content='width=device-width, initial-scale=1'>"
       "<meta http-equiv='refresh' content='10'>"
       "<title>Pannello MxIrrigation</title>"
       "<style>body{font-family:Arial;font-size:14px;margin:12px}"
       "table{border-collapse:collapse}td,th{padding:6px;border-bottom:1px solid #ddd}"
       ".off{color:#b00}.on{color:#080}</style></head><body>";
  h += "<h2>Stato nodi</h2><table><tr><th>Nodo</th><th>Indirizzo</th><th>Stato</th></tr>";
  for (int i = 0; i < MAX_NODES; i++) {
    if (!dcfg.nodes[i].enabled) continue;
    h += "<tr><td>" + htmlEscape(dcfg.nodes[i].name) + "</td><td>" + htmlEscape(dcfg.nodes[i].host) + "</td><td>";
    if (!nstate[i].online) h += "<span class=off>offline</span>";
    else if (dcfg.nodes[i].type == NODE_LATCH) {
      int open = 0;
      for (int v = 0; v < nstate[i].valveCount; v++) if (nstate[i].valves[v]) open++;
      h += "<span class=on>" + String(open) + " di " + String(nstate[i].valveCount) + " aperte</span>";
    } else {
      h += "<span class=on>" + String(nstate[i].pressureBar, 2) + " bar, pompe "
         + (nstate[i].masterOn ? "ON" : "ferme") + "</span>";
    }
    h += "</td></tr>";
  }
  h += "</table><p><a href='/config'>Configurazione</a></p></body></html>";
  server.send(200, "text/html", h);
}

void registerRoutes() {
  server.on("/", HTTP_GET, handleStatusPage);
  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  MxOta.attachWeb(server, "/ota", "/config");
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
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
      if (a >= 2) { MxIdentity.provision(args[0], args[1], args[2], args[3], args[4]);
                    Serial.println("[ID] scritto. Riavvia per applicare."); }
      else Serial.println("uso: id set \"Nome Pista\" \"nome-device\"");
    } else if (strcmp(line, "id show") == 0) {
      MxIdentity.printTo(Serial);
    } else if (strcmp(line, "wifi reset") == 0) {
      MxNet.factoryReset("comando seriale");
    } else if (strcmp(line, "nodes") == 0) {
      for (int i = 0; i < MAX_NODES; i++) {
        if (!dcfg.nodes[i].enabled) continue;
        Serial.printf("[%d] %-14s %-16s %s  %s\n", i, dcfg.nodes[i].name,
                      dcfg.nodes[i].host,
                      dcfg.nodes[i].type == NODE_PUMP ? "pompa " : "valvole",
                      nstate[i].online
                        ? (nstate[i].failCount ? "ok (1 fallito)" : "ok")
                        : "OFFLINE");
      }
    } else if (strcmp(line, "poll") == 0) {
      lastPollMs = 0;
      Serial.println("[POLL] forzato");
    } else if (strcmp(line, "ota") == 0) {
      MxOta.pending = true;
    } else if (strcmp(line, "reboot") == 0) {
      ESP.restart();
    } else if (line[0]) {
      Serial.println("comandi: id set / id show / wifi reset / nodes / poll / ota / reboot");
    }
  }
}

// =========================================================================
// SETUP / LOOP
// =========================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("MxIrrigation - pannello di stato");
  Serial.printf("Firmware %s su MxSolutionCore %s\n", FW_VERSION, MXCORE_VERSION);

  cfgLoad();
  memset(nstate, 0, sizeof(nstate));

  tft.init();
  tft.setRotation(1);                 // landscape 320x240
  tft.setBrightness(dcfg.brightness);
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextDatum(lgfx::middle_center);
  tft.drawString("MxIrrigation", SCR_W / 2, SCR_H / 2 - 12);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("connessione...", SCR_W / 2, SCR_H / 2 + 12);
  tft.setTextDatum(lgfx::top_left);

  cfg.fwVersion       = FW_VERSION;
  cfg.wdtSeconds      = 15;
  cfg.otaPassword     = "";
  cfg.otaHttpUrl      = "";
  cfg.factoryResetPin = 0;
  cfg.requireIdentity = false;
  // Nessun IP fisso di fabbrica: il pannello non deve stare a un indirizzo
  // preciso per fare il suo lavoro. Se serve, si imposta dal portale.

  MxCore.begin(cfg);

  registerRoutes();
  server.begin();
  Serial.println("[HTTP] server avviato: / e /config");

  lastRotateMs = millis();
  redrawNeeded = true;
}

void loop() {
  MxCore.loop();
  server.handleClient();
  handleSerial();
  serviceTouch();
  servicePolling();

  // Rotazione automatica delle pagine, se abilitata.
  if (dcfg.rotateSeconds > 0 &&
      millis() - lastRotateMs >= dcfg.rotateSeconds * 1000UL) {
    lastRotateMs = millis();
    page = (page + 1) % 3;
    redrawNeeded = true;
  }

  // Ridisegno: quando cambia qualcosa, e comunque ogni 2 s per aggiornare
  // le età dei dati ("ultimo 45s fa") senza far lampeggiare lo schermo.
  if (redrawNeeded || millis() - lastDrawMs >= 2000) {
    drawCurrentPage();
  }
}
