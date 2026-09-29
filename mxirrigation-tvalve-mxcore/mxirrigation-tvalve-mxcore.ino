/**
 * ============================================================================
 * Progetto:    MxIrrigation - Valvola a T motorizzata (zona ANFITEATRO)
 * File:        mxirrigation-tvalve-mxcore.ino
 * Autore:      Nicola Deboni - MxSolutions
 * Versione:    1.0.0
 * Ultima mod.: 2026-09-27
 * Repository:  vault "03 - Elettronica/Progetti/mxirrigation" -> vedi
 *              [[tvalve-controller]] e [[mxirrigation]]
 * ----------------------------------------------------------------------------
 * Descrizione:
 *   Comanda una valvola a T MOTORIZZATA tramite due relè: uno manda il motore
 *   in apertura, l'altro in chiusura. La valvola non ha finecorsa riportati né
 *   sensori di posizione: il firmware sa dov'è solo perché ha tenuto il motore
 *   acceso per il tempo di corsa completo. È un sistema AD ANELLO APERTO, e
 *   tutta la gestione di §"MOTORE E POSIZIONE" esiste per non mentire su
 *   questo.
 *
 *   Porting della 0.5 su MxSolutionCore: identità in NVS, WiFi con captive
 *   portale, IP fisso configurabile, NTP, watchdog vero, OTA (push + pull) e
 *   autenticazione delle azioni (MxAuth).
 *
 * Hardware:
 *   Scheda:      ESP32 classico + scheda 4 relè (ne usa DUE)
 *   Relè:        {21, 19, 18, 5}, attivi HIGH
 *                  relè 1 (GPIO 21) -> APRE
 *                  relè 2 (GPIO 19) -> CHIUDE
 *                  relè 3 e 4       -> non usati, restano spenti
 *   Reset fabbrica: GPIO 0 (tasto BOOT, libero su questa scheda)
 *
 *   ⚠️ MAI i due relè eccitati insieme: sarebbero i due capi del motore
 *   pilotati contemporaneamente. Ogni cambio di direzione passa da
 *   "tutti spenti" + un TEMPO MORTO (vedi REVERSE_DEADTIME_MS).
 *
 * Arduino IDE / arduino-cli:
 *   Board:            ESP32 Dev Module / esp32:esp32:esp32
 *   Core version:     esp32 3.3.11
 *   Partition Scheme: ⚠️ Minimal SPIFFS (min_spiffs) - OBBLIGATORIO. Con la
 *                     partizione default il binario sta oltre il 90% e non
 *                     resta spazio per l'OTA. La 0.5 non aveva questo vincolo
 *                     perché era uno sketch minuscolo senza libreria.
 *   Flash:            QIO 80MHz, 4MB
 *
 *   Librerie (nome - versione esatta testata):
 *     MxSolutionCore  - 0.1.8   (0.1.6 per MxAuth, 0.1.8 per l'OTA che
 *                                funziona davvero: vedi la nota in fondo)
 *     (ESPping NON serve più: il ping è stato tolto, vedi note di versione)
 *
 * Note di versione:
 *   2026-09-27      - Ricompilato su MxSolutionCore 0.1.8 (nessuna modifica a
 *                      questo sketch). Cosa cambia sotto: l'OTA accetta la
 *                      risposta CHUNKED, che e' quella che manda il server
 *                      mxota reale - prima il pull si fermava su "risposta
 *                      senza Content-Length" e nessun nodo poteva
 *                      aggiornarsi; i timeout di connessione e di attesa
 *                      degli header stanno sotto il timeout del watchdog
 *                      (prima 6+15 = 21 s contro 15: un server lento a
 *                      RISPONDERE riavviava il device); un campo vuoto nel
 *                      captive portale non cancella piu' la password
 *                      salvata; niente piu' "TWDT already initialized" a
 *                      ogni avvio.
 *   2026-09-27 1.0.0 - PORTING SU MxSolutionCore 0.1.7. Cosa cambia:
 *                     + identità in NVS ("mxid"): il nome della zona non è più
 *                       scritto nel sorgente, quindi lo stesso binario vale per
 *                       più valvole a T;
 *                     + WiFi dal captive portale: le credenziali NON stanno più
 *                       in chiaro nel codice;
 *                     + IP fisso 192.168.5.45 come DEFAULT DI FABBRICA,
 *                       cambiabile dal portale (⚠️ vedi la discrepanza .45/.48
 *                       più sotto);
 *                     + NTP, quindi /status e i log hanno data e ora vere;
 *                     + WATCHDOG VERO (esp_task_wdt, 15 s). La 0.5 non ne
 *                       aveva: quello che sembrava un watchdog era un ping;
 *                     + OTA push (ArduinoOTA) e pull (mxota/api.php): da qui
 *                       in avanti non serve più il cavo per aggiornare;
 *                     + MxAuth: /on, /off, /reboot, /ota e /config/save
 *                       vogliono POST + token. Fino alla 0.5 erano GET aperte,
 *                       cioè la valvola si muoveva con un prefetch del browser
 *                       o una <img src> in una pagina qualunque;
 *                     + tempo di corsa CONFIGURABILE da /config (default 20 s),
 *                       in NVS: non serve ricompilare per tararlo;
 *                     + posizione dichiarata "sconosciuta" quando lo è
 *                       davvero, invece di mentire (vedi §MOTORE E POSIZIONE);
 *                     + tempo morto di 300 ms a ogni inversione di marcia;
 *                     + /info, /logs, /config e un log degli ultimi comandi;
 *                     - TOLTO IL PING al gateway e il riavvio dopo 4
 *                       fallimenti. Non serve più: la riconnessione la fa
 *                       MxNet (non bloccante, con backoff) e i blocchi li
 *                       prende il watchdog. Il ping era anche BLOCCANTE (fino
 *                       a ~1 s dentro il loop) e girava SOLO a valvola chiusa,
 *                       cioè non guardava niente proprio mentre l'acqua
 *                       scorreva. Conseguenza buona: un buco di rete non
 *                       chiude più la valvola da solo.
 *   2026-09-18 0.5  - /status JSON per il pannello di stato (invio singolo,
 *                     senza autenticazione perché in sola lettura).
 *                     Versione portata in una costante invece che solo nel
 *                     commento di testa.
 *   2026-xx-xx 0.4  - risparmio energetico: BT spento, CPU a 80 MHz,
 *                     modem-sleep del WiFi.
 * ============================================================================
 */

#include <MxCore.h>
#include <WebServer.h>
#include <WiFi.h>        // solo per WiFi.setSleep(): il resto lo fa MxNet
#include <Preferences.h>
#include <stdarg.h>

// Per btStop() / esp_bt_controller_disable()
#include "esp_bt.h"

const char* FW_VERSION = "1.0.0";

// =========================================================================
// CONFIGURAZIONE
// =========================================================================

// --- rete: IP fisso di DEFAULT -------------------------------------------
// Le credenziali WiFi NON stanno qui: si impostano dal captive portale
// "Mx-Setup-XXXX" al primo avvio. Anche questo indirizzo si può cambiare dal
// portale (o disattivare per andare in DHCP): quello che vince è la scelta
// dell'operatore, questo è solo il valore di fabbrica.
//
// ⚠️ DISCREPANZA STORICA, NON RISOLTA: l'intestazione della 0.4/0.5 dichiarava
// 192.168.5.48, il codice impostava 192.168.5.45. Qui è rimasto il valore che
// il codice usava davvero (.45), perché è quello con cui il nodo ha funzionato
// sul campo. Da verificare sul posto o su ntopng; ora che l'indirizzo si
// cambia dal portale, sbagliarlo qui non costa più un viaggio.
IPAddress STATIC_IP(192, 168, 5, 45);
IPAddress GATEWAY  (192, 168, 5, 1);
IPAddress SUBNET   (255, 255, 255, 0);
IPAddress DNS1     (192, 168, 5, 1);
IPAddress DNS2     (8, 8, 8, 8);

// --- OTA ------------------------------------------------------------------
// Password di ArduinoOTA (push dall'IDE in LAN). Una password NORMALE: NON
// l'hash bcrypt di config.php del server mxota, che è il segreto del pannello
// di amministrazione e non deve finire nel binario di un device.
const char* OTA_PASSWORD  = "mxirr-6fquf15vxeow";   // "" = ArduinoOTA disabilitato

// L'URL del pull non si scrive a mano intero: "fw" sceglie QUALE binario il
// server consegna, e sbagliarlo significa farsi installare il firmware di un
// altro nodo. "id" serve al server solo per sapere chi ha chiamato e viene
// preso dall'identità a runtime, così lo stesso sorgente vale per più valvole.
const char* OTA_HTTP_BASE = "https://www.mxsolutions.it/mxota/api.php";  // "" = pull disabilitato
const char* OTA_FW_NAME   = "mxirrigation-tvalve";   // <-- NON "mxirrigation-latch"
const char* OTA_TOKEN     = "wmp-2026a-123";         // token condiviso col server

// Composto in setup(): base + fw + id + token. MxOta rifiuta un URL oltre i
// 224 caratteri, quindi qui si controlla e si avvisa invece di troncare.
char otaHttpUrl[224] = "";

// --- relè -----------------------------------------------------------------
const int RELAY_PINS[4] = {21, 19, 18, 5};
const int RELAY_OPEN    = 0;   // indice in RELAY_PINS: relè 1 -> apre
const int RELAY_CLOSE   = 1;   // indice in RELAY_PINS: relè 2 -> chiude
const uint8_t RELAY_ON  = HIGH;
const uint8_t RELAY_OFF = LOW;

// --- motore ---------------------------------------------------------------
// Tempo di corsa di DEFAULT. Il valore vero si imposta da /config e vive in
// NVS: questo serve solo al primo avvio e dopo un reset della configurazione.
const uint32_t TRAVEL_MS_DEFAULT = 20000;
const uint32_t TRAVEL_MS_MIN     = 2000;     // sotto i 2 s non è una corsa
const uint32_t TRAVEL_MS_MAX     = 120000;   // oltre i 2 min è un errore di battitura

// Pausa fra "spengo tutto" e "accendo l'altro senso" quando si inverte la
// marcia. Serve al motore, non al software: invertire di colpo mette la
// forza controelettromotrice sui contatti del relè e sul ponte del motore.
// 300 ms non si notano su una corsa da 20 s.
const uint32_t REVERSE_DEADTIME_MS = 300;

// --- risparmio energetico (dalla 0.4) ------------------------------------
// ⚠️ La CPU a 80 MHz è la condizione peggiore per scaricare 1,2 MB di OTA, e
// il modem-sleep ci aggiunge latenza. Per questo si rimettono prestazioni
// piene PRIMA di un aggiornamento (vedi onOtaBefore()).
// Se la console seriale dovesse arrivare sporca, il primo sospetto è il
// downclock: si mette a 0 questo define e si ricompila.
#define ENABLE_CPU_DOWNCLOCK 1
#define ENABLE_WIFI_MODEM_SLEEP 1
#define ENABLE_BT_OFF 1
const uint32_t CPU_MHZ_SAVING = 80;
const uint32_t CPU_MHZ_FULL   = 240;

// --- recovery -------------------------------------------------------------
#define ENABLE_AUTO_RESTART_ON_LOW_HEAP 1
const uint32_t LOW_HEAP_THRESHOLD = 12000;
const uint32_t LOW_HEAP_GRACE_MS  = 60000;

// --- reset di fabbrica ---------------------------------------------------
// GPIO 0 = tasto BOOT, libero su questa scheda (i relè stanno su 21/19/18/5).
// A massa per 3 s all'avvio cancella "mxnet" (rete e IP) e la configurazione
// di questo sketch. "mxid" (identità) NON si tocca.
const int FACTORY_RESET_PIN = 0;

// =========================================================================
// STATO
// =========================================================================

WebServer server(80);
MxCoreConfig cfg;

// --- configurazione persistente (NVS "tvalve") ---------------------------
struct TvConfig {
  uint32_t travelMs;      // tempo di corsa del motore
  bool     closeOnBoot;   // impulso di chiusura all'avvio
};
const TvConfig TV_DEFAULT = { TRAVEL_MS_DEFAULT, true };
TvConfig tv = TV_DEFAULT;

// --- posizione e moto ----------------------------------------------------
// Vedi il blocco §MOTORE E POSIZIONE più sotto: qui ci sono solo i dati.
enum ValveState : uint8_t { VS_UNKNOWN = 0, VS_CLOSED, VS_OPEN, VS_OPENING, VS_CLOSING };
enum MotionPhase : uint8_t { M_IDLE = 0, M_DEADTIME, M_RUNNING };

// Esito di una richiesta di corsa. Sta QUI, fra i tipi, e non accanto a
// requestTravel() che lo usa: l'IDE inserisce i prototipi delle funzioni
// PRIMA della prima definizione di funzione del file, e un tipo dichiarato
// più in basso non esisterebbe ancora per quei prototipi. È un inciampo già
// visto in questo progetto.
enum TravelResult : uint8_t {
  TR_STARTED,    // corsa avviata
  TR_REVERSED,   // stava andando dall'altra parte: invertita
  TR_ALREADY,    // già in corsa nella stessa direzione: niente da fare
  TR_BUSY        // rifiutato: sta chiudendo, e una chiusura non si interrompe
};

ValveState  valveState  = VS_UNKNOWN;   // all'avvio non si sa dov'è: è la verità
MotionPhase motionPhase = M_IDLE;
bool        motionToOpen = false;       // direzione in corso o in attesa
uint32_t    phaseEndMs   = 0;
uint32_t    travelStartedMs = 0;        // per i secondi rimanenti

// --- log comandi ---------------------------------------------------------
#define COMMAND_LOG_SIZE 6
struct CommandLogEntry {
  bool valid;
  char timestamp[20];
  char action[12];
};
CommandLogEntry commandLog[COMMAND_LOG_SIZE];

uint32_t lowHeapSinceMs = 0;
time_t   bootEpoch      = 0;

// =========================================================================
// CONFIGURAZIONE PERSISTENTE
// =========================================================================

static const char* NVS_NS = "tvalve";

void cfgLoad() {
  Preferences p;
  if (!p.begin(NVS_NS, true)) {
    Serial.println("[CFG] NVS 'tvalve' assente: valori di fabbrica");
    return;
  }
  tv.travelMs    = p.getULong("travel", TV_DEFAULT.travelMs);
  tv.closeOnBoot = p.getBool ("bootclose", TV_DEFAULT.closeOnBoot);
  p.end();

  // Clamp e non rifiuto: un valore fuori range in NVS (scrittura vecchia,
  // corruzione, dito scivolato) non deve impedire alla valvola di funzionare.
  if (tv.travelMs < TRAVEL_MS_MIN) tv.travelMs = TRAVEL_MS_MIN;
  if (tv.travelMs > TRAVEL_MS_MAX) tv.travelMs = TRAVEL_MS_MAX;

  Serial.printf("[CFG] corsa %.1f s | chiusura all'avvio: %s\n",
                tv.travelMs / 1000.0f, tv.closeOnBoot ? "sì" : "no");
}

void cfgSave() {
  Preferences p;
  if (!p.begin(NVS_NS, false)) { Serial.println("[CFG] NVS non scrivibile"); return; }
  p.putULong("travel",    tv.travelMs);
  p.putBool ("bootclose", tv.closeOnBoot);
  p.end();
}

// =========================================================================
// LOG COMANDI
// =========================================================================

void addCommandLog(const char* action) {
  for (int i = COMMAND_LOG_SIZE - 1; i > 0; i--) commandLog[i] = commandLog[i - 1];
  commandLog[0].valid = true;
  if (MxTime.valid()) MxTime.local(commandLog[0].timestamp, sizeof(commandLog[0].timestamp), "%d/%m %H:%M:%S");
  else                snprintf(commandLog[0].timestamp, sizeof(commandLog[0].timestamp), "+%lus", (unsigned long)MxCore.uptimeS());
  snprintf(commandLog[0].action, sizeof(commandLog[0].action), "%s", action);
}

// =========================================================================
// MOTORE E POSIZIONE
// =========================================================================
//
// La valvola non dice dov'è. Non ha finecorsa riportati, non ha un
// potenziometro: gli unici due comandi possibili sono "manda il motore in
// apertura" e "manda il motore in chiusura". Il firmware sa dov'è la valvola
// SOLO perché ha tenuto il motore acceso per il tempo di corsa intero.
//
// Da questo derivano tre regole, e ognuna risolve un modo di sbagliare:
//
// [1] UNA CORSA SI FA SEMPRE INTERA.
//     Il motore resta acceso per travelMs anche se la valvola arriva a fondo
//     corsa prima: è il solo modo, senza sensori, di sapere che ci è arrivata.
//     Se travelMs è più CORTO della corsa vera, la valvola si ferma a metà e
//     il firmware crede che sia a fondo corsa - per questo il tempo si misura
//     col cronometro e si imposta da /config, invece di essere compilato.
//
// [2] LA CHIUSURA HA LA PRIORITÀ, L'APERTURA NO.
//     Un comando di chiusura che arriva mentre la valvola sta aprendo viene
//     ESEGUITO SUBITO: inverte la marcia. Un comando di apertura che arriva
//     mentre sta chiudendo viene RIFIUTATO (409) dicendo quanti secondi
//     restano. L'asimmetria non è una dimenticanza: chiudere è la manovra di
//     sicurezza, e non deve mai essere in coda dietro a qualcos'altro.
//     È la stessa logica del "master/off" sempre accettato sulla pompa.
//
// [3] SE LA POSIZIONE NON È NOTA, SI DICE.
//     Dopo un'inversione la corsa nuova è intera, quindi la posizione finale
//     è nota. Ma se il device si RIAVVIA a metà corsa (crash, OTA, tolta
//     l'alimentazione) la valvola resta fisicamente a metà e nessuno lo sa:
//     lo stato diventa "unknown", /status lo pubblica come
//     "position_known": false, e la pagina lo scrive in chiaro. Con la
//     chiusura all'avvio attiva (default) il nodo si riporta da solo a
//     "chiusa"; senza, resta "sconosciuta" finché non arriva un comando.
//     Un coordinatore che legge "unknown" sa che deve chiudere prima di
//     contare su qualsiasi cosa.
//
// Un comando nella stessa direzione di una corsa in atto NON la fa ripartire:
// è idempotente. Due clic su APRI non raddoppiano il tempo di motore acceso.
//
// ⚠️ MAI i due relè insieme: ogni transizione passa da allRelaysOff(), e
// un'inversione aspetta REVERSE_DEADTIME_MS prima di eccitare l'altro senso.

void allRelaysOff() {
  for (int i = 0; i < 4; i++) digitalWrite(RELAY_PINS[i], RELAY_OFF);
}

const char* valveStateName(ValveState s) {
  switch (s) {
    case VS_CLOSED:  return "closed";
    case VS_OPEN:    return "open";
    case VS_OPENING: return "opening";
    case VS_CLOSING: return "closing";
    default:         return "unknown";
  }
}

const char* valveStateLabel(ValveState s) {
  switch (s) {
    case VS_CLOSED:  return "CHIUSA";
    case VS_OPEN:    return "APERTA";
    case VS_OPENING: return "in apertura";
    case VS_CLOSING: return "in chiusura";
    default:         return "POSIZIONE SCONOSCIUTA";
  }
}

bool valveMoving()      { return motionPhase != M_IDLE; }
bool positionKnown()    { return valveState == VS_OPEN || valveState == VS_CLOSED; }

// Secondi che restano alla fine della corsa (0 se ferma).
uint32_t travelRemainingS() {
  if (motionPhase == M_IDLE) return 0;
  int32_t left = (int32_t)(phaseEndMs - millis());
  if (left < 0) left = 0;
  return (uint32_t)((left + 999) / 1000);
}

// Avvia la corsa nella direzione richiesta. La chiamano solo requestTravel()
// e serviceMotion(): non si eccita un relè da nessun'altra parte.
void energize(bool toOpen) {
  allRelaysOff();
  digitalWrite(RELAY_PINS[toOpen ? RELAY_OPEN : RELAY_CLOSE], RELAY_ON);
  motionPhase   = M_RUNNING;
  motionToOpen  = toOpen;
  phaseEndMs    = millis() + tv.travelMs;
  travelStartedMs = millis();
  valveState    = toOpen ? VS_OPENING : VS_CLOSING;
  Serial.printf("[VALVE] %s: motore acceso per %.1f s\n",
                toOpen ? "APERTURA" : "CHIUSURA", tv.travelMs / 1000.0f);
}

TravelResult requestTravel(bool toOpen) {
  // [già in movimento nella stessa direzione] -> idempotente
  if (motionPhase != M_IDLE && motionToOpen == toOpen) return TR_ALREADY;

  // [in movimento nella direzione opposta]
  if (motionPhase != M_IDLE) {
    if (toOpen) {
      // regola [2]: non si interrompe una chiusura per aprire
      return TR_BUSY;
    }
    // chiusura richiesta durante un'apertura: inverte, con tempo morto
    allRelaysOff();
    motionPhase  = M_DEADTIME;
    motionToOpen = false;
    phaseEndMs   = millis() + REVERSE_DEADTIME_MS;
    valveState   = VS_CLOSING;
    Serial.println("[VALVE] inversione: chiusura prioritaria, tempo morto 300 ms");
    return TR_REVERSED;
  }

  energize(toOpen);
  return TR_STARTED;
}

void serviceMotion() {
  if (motionPhase == M_IDLE) return;

  if ((int32_t)(millis() - phaseEndMs) < 0) return;   // non è ancora il momento

  if (motionPhase == M_DEADTIME) {
    energize(motionToOpen);                            // parte la corsa intera
    return;
  }

  // fine corsa: motore spento, posizione NOTA
  allRelaysOff();
  motionPhase = M_IDLE;
  valveState  = motionToOpen ? VS_OPEN : VS_CLOSED;
  Serial.printf("[VALVE] corsa finita: %s\n", valveStateName(valveState));
}

// =========================================================================
// PAGINE WEB - infrastruttura
// =========================================================================

void sendChunk(const char* s) { server.sendContent(s); }

void sendFmt(const char* fmt, ...) {
  char buf[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  server.sendContent(buf);
}

void startHtml(const char* title) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  sendChunk("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  sendChunk("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  sendChunk("<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>");
  sendChunk("<meta http-equiv='Expires' content='0'>");

  // Il token sta nel localStorage del BROWSER e viaggia in un header, mai
  // nell'URL: così non finisce nella cronologia, nei log né nel Referer.
  sendChunk("<script>"
            "function mxTok(f){var t=localStorage.getItem('mxtoken')||'';"
            "if(!t||f){t=prompt('Token del nodo:',t)||'';"
            "if(t)localStorage.setItem('mxtoken',t);}return t;}"
            "function mxCfg(f){var t=localStorage.getItem('mxcfgpw')||'';"
            "if(!t||f){t=prompt('Password della pagina di configurazione:',t)||'';"
            "if(t)localStorage.setItem('mxcfgpw',t);}return t;}"
            "async function mxPost(p,body){"
            "var r=await fetch(p,{method:'POST',headers:{'X-Mx-Token':mxTok(false),"
            "'X-Mx-Cfg':localStorage.getItem('mxcfgpw')||'',"
            "'Content-Type':'application/x-www-form-urlencoded'},body:body||''});"
            "if(r.status===401){var j=await r.json().catch(function(){return {};});"
            "alert('Non autorizzato'+(j.hint?': '+j.hint:'.'));mxTok(true);return null;}"
            "if(r.status===409){var j=await r.json().catch(function(){return {};});"
            "alert('Valvola occupata'+(j.remaining_s?': mancano '+j.remaining_s+' s alla fine della chiusura.':'.'));"
            "return null;}"
            "if(!r.ok){alert('Errore '+r.status);return null;}return r;}"
            "async function mxCmd(p){var r=await mxPost(p);"
            "if(r)setTimeout(function(){location.reload();},600);}"
            "</script>");
  sendChunk("<title>");
  sendChunk(title);
  sendChunk("</title></head><body style='font-family:Arial,sans-serif;font-size:14px'>");
}

void endHtml() { sendChunk("</body></html>"); }

void sendNav() {
  sendChunk("<p><a href='/'>Home</a> | <a href='/info'>Info</a> | "
            "<a href='/config'>Config</a> | <a href='/logs'>Logs</a> | "
            "<a href='/status'>JSON</a></p>");
}

void sendAuthWarnings() {
  if (MxAuth.misconfigured()) {
    sendChunk("<p style='color:#b00'><b>Comandi bloccati.</b> Il token è richiesto "
              "ma non è impostato. Da seriale: <code>auth token &lt;valore&gt;</code>, "
              "oppure togli il token da <a href='/config'>/config</a>.</p>");
  } else if (!MxAuth.requireToken()) {
    sendChunk("<p style='color:#b60'><b>Nessun token richiesto:</b> chiunque sulla "
              "rete locale può muovere la valvola.</p>");
  }
}

// =========================================================================
// PAGINE WEB
// =========================================================================

void sendHomePage() {
  char nowBuf[24];
  MxTime.local(nowBuf, sizeof(nowBuf));

  startHtml("MxIrrigation - Valvola a T");
  sendFmt("<h3>Valvola a T &mdash; %s</h3>",
          MxIdentity.provisioned() ? MxIdentity.device() : "device non provisionato");
  sendNav();
  sendFmt("<p>%s</p>", nowBuf);

  if (!MxIdentity.provisioned()) {
    sendChunk("<p style='color:#b00'><b>Identità non impostata</b>: da seriale "
              "<code>id set \"Nome Pista\" \"nome-device\"</code>.</p>");
  }
  if (MxOta.inProgress()) {
    sendChunk("<p style='color:#b60'><b>Aggiornamento OTA in corso</b></p>");
  }
  sendAuthWarnings();

  // Lo stato, grande e in chiaro. "Sconosciuta" è uno stato legittimo e va
  // detto come tale: non si sceglie un default che sembra una certezza.
  const char* colour = positionKnown() ? "#060" : (valveMoving() ? "#b60" : "#b00");
  sendFmt("<p style='font-size:20px'>Stato: <b style='color:%s'>%s</b></p>",
          colour, valveStateLabel(valveState));

  if (valveMoving()) {
    sendFmt("<p>Motore acceso, mancano <b>%lu s</b> alla fine della corsa.</p>",
            (unsigned long)travelRemainingS());
  } else if (!positionKnown()) {
    sendChunk("<p>La posizione non è nota: il device si è riavviato a metà corsa, "
              "oppure non ha ancora fatto una corsa intera da quando è acceso. "
              "Un comando di chiusura la riporta a uno stato certo.</p>");
  }

  sendChunk("<p><button style='font-size:18px;padding:8px 18px' "
            "onclick=\"mxCmd('/on')\">APRI</button> &nbsp; "
            "<button style='font-size:18px;padding:8px 18px' "
            "onclick=\"mxCmd('/off')\">CHIUDI</button></p>");

  sendFmt("<p style='color:#555;font-size:12px'>Tempo di corsa impostato: "
          "<b>%.1f s</b> (<a href='/config'>modifica</a>). La chiusura ha la "
          "priorità: se la valvola sta aprendo, CHIUDI inverte subito. "
          "L'apertura invece aspetta la fine di una chiusura.</p>",
          tv.travelMs / 1000.0f);
  endHtml();
}

void sendInfoPage() {
  char nowBuf[24], bootBuf[24], ipStr[20];
  MxTime.local(nowBuf, sizeof(nowBuf));
  if (bootEpoch > 0) {
    struct tm tmv;
    localtime_r(&bootEpoch, &tmv);
    strftime(bootBuf, sizeof(bootBuf), "%Y-%m-%d %H:%M:%S", &tmv);
  } else {
    snprintf(bootBuf, sizeof(bootBuf), "n/d");
  }
  snprintf(ipStr, sizeof(ipStr), "%s", MxNet.ip().c_str());

  unsigned long s = MxCore.uptimeS();
  unsigned long days  = s / 86400UL; s %= 86400UL;
  unsigned long hours = s / 3600UL;  s %= 3600UL;
  unsigned long mins  = s / 60UL;    s %= 60UL;

  startHtml("MxIrrigation - Info");
  sendChunk("<h3>System Info</h3>");
  sendNav();
  sendChunk("<pre>");
  sendFmt("Firmware        : %s\n", FW_VERSION);
  sendFmt("MxSolutionCore  : %s\n", MXCORE_VERSION);
  sendFmt("Tipo nodo       : valvola a T (2 relè su scheda a 4)\n");
  sendFmt("Pista (site)    : %s\n", MxIdentity.provisioned() ? MxIdentity.site() : "(non impostata)");
  sendFmt("Device          : %s\n", MxIdentity.provisioned() ? MxIdentity.device() : "(non impostato)");
  sendFmt("Device ID       : %s\n", MxIdentity.provisioned() ? MxIdentity.deviceId() : "-");
  sendFmt("IP              : %s (%s)\n", ipStr, MxNet.usingStaticIP() ? "fisso" : "DHCP");
  sendFmt("WiFi connected  : %s\n", MxNet.connected() ? "YES" : "NO");
  sendFmt("Signal          : %ld dBm\n", (long)MxNet.rssi());
  sendFmt("Data e ora      : %s\n", nowBuf);
  sendFmt("NTP sync        : %s\n", MxTime.valid() ? "YES" : "NO");
  sendFmt("NTP last sync   : %lu s fa\n", (unsigned long)MxTime.lastSyncAgeS());
  sendFmt("Boot time       : %s\n", bootBuf);
  sendFmt("Timezone        : %s\n", cfg.tz);
  sendFmt("CPU             : %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
  sendFmt("ArduinoOTA      : %s\n", MxOta.armed() ? "armato" : "disabilitato");
  sendFmt("OTA pull        : %s\n", otaHttpUrl[0] ? "configurato" : "non configurato");
  if (otaHttpUrl[0]) sendFmt("OTA firmware    : %s\n", OTA_FW_NAME);
  sendFmt("Watchdog        : %lu s\n", (unsigned long)MxWatchdog.timeoutS());
  sendFmt("Valvola         : %s\n", valveStateLabel(valveState));
  sendFmt("Posizione nota  : %s\n", positionKnown() ? "sì" : "NO");
  sendFmt("Tempo di corsa  : %.1f s\n", tv.travelMs / 1000.0f);
  sendFmt("Chiusura avvio  : %s\n", tv.closeOnBoot ? "sì" : "no");
  if (valveMoving()) sendFmt("Corsa in atto   : mancano %lu s\n", (unsigned long)travelRemainingS());
  sendFmt("Relè            : 1=%s 2=%s 3=%s 4=%s\n",
          digitalRead(RELAY_PINS[0]) == RELAY_ON ? "ON" : "off",
          digitalRead(RELAY_PINS[1]) == RELAY_ON ? "ON" : "off",
          digitalRead(RELAY_PINS[2]) == RELAY_ON ? "ON" : "off",
          digitalRead(RELAY_PINS[3]) == RELAY_ON ? "ON" : "off");
  sendFmt("Free heap       : %u\n", (unsigned)ESP.getFreeHeap());
  sendFmt("Min free heap   : %u\n", (unsigned)ESP.getMinFreeHeap());
  sendFmt("Uptime          : %lu g, %lu h, %lu m, %lu s\n", days, hours, mins, s);
  sendChunk("Azioni          : ");
  sendChunk(MxAuth.allowGet() ? "POST e GET (compatibilità)\n" : "solo POST\n");
  sendFmt("Token           : %s\n",
          !MxAuth.requireToken() ? "NON richiesto - chiunque sulla LAN comanda"
                                 : (MxAuth.tokenSet()
                                      ? "richiesto, impostato"
                                      : "RICHIESTO MA NON IMPOSTATO - azioni bloccate"));
  sendFmt("Pagina config   : %s\n", MxAuth.configPwSet() ? "protetta da password" : "aperta");
  sendChunk("</pre>");

  sendChunk("<p><button onclick=\"if(confirm('Cercare un aggiornamento adesso? "
            "Se ce n\\'e uno, il device si riavvia.'))mxCmd('/ota')\">"
            "Cerca aggiornamenti</button></p>");
  sendChunk("<p><button onclick=\"if(confirm('Riavviare il nodo? Se la valvola è a "
            "metà corsa la posizione diventa sconosciuta.'))mxCmd('/reboot')\">"
            "REBOOT DEVICE</button></p>");
  endHtml();
}

void sendLogsPage() {
  startHtml("MxIrrigation - Logs");
  sendChunk("<h3>Ultimi comandi</h3>");
  sendNav();
  sendChunk("<ul>");
  bool any = false;
  for (int i = 0; i < COMMAND_LOG_SIZE; i++) {
    if (!commandLog[i].valid) continue;
    any = true;
    sendFmt("<li>%s &mdash; %s</li>", commandLog[i].timestamp, commandLog[i].action);
  }
  if (!any) sendChunk("<li><i>nessun comando dall'avvio</i></li>");
  sendChunk("</ul>");
  sendChunk("<p style='color:#555;font-size:12px'>Il log sta in RAM: si azzera "
            "a ogni riavvio. Lo storico vero lo tiene il collettore dati.</p>");
  endHtml();
}

// Definita prima di sendConfigPage() perché la usa: l'IDE non genera sempre
// il prototipo per una funzione usata prima di essere definita, ed è un
// inciampo già visto in questo progetto.
void sendConfigPage() {
  startHtml("MxIrrigation - Configurazione");
  sendChunk("<h3>Configurazione</h3>");
  sendNav();

  sendChunk("<form id='cfgform' onsubmit='return mxSaveCfg(event)'>");
  sendChunk("<table border=0 cellpadding=4 style='margin:0 auto;text-align:left'>");

  sendFmt("<tr><td>Tempo di corsa della valvola</td>"
          "<td><input type=number step='0.5' min='%.1f' max='%.1f' name='travel' "
          "value='%.1f'> s</td></tr>",
          TRAVEL_MS_MIN / 1000.0f, TRAVEL_MS_MAX / 1000.0f, tv.travelMs / 1000.0f);
  sendFmt("<tr><td>Chiusura all'avvio</td>"
          "<td><input type=checkbox name='bootclose' value='1'%s></td></tr>",
          tv.closeOnBoot ? " checked" : "");

  sendChunk("<tr><td colspan=2><br><b>Sicurezza</b></td></tr>");
  sendFmt("<tr><td>Metodo delle azioni</td><td>"
          "<label><input type=radio name='authget' value='0'%s> solo POST</label><br>"
          "<label><input type=radio name='authget' value='1'%s> POST e GET</label>"
          "</td></tr>",
          MxAuth.allowGet() ? "" : " checked",
          MxAuth.allowGet() ? " checked" : "");
  sendFmt("<tr><td>Token richiesto</td>"
          "<td><input type=checkbox name='authreq' value='1'%s></td></tr>",
          MxAuth.requireToken() ? " checked" : "");
  sendFmt("<tr><td>Token</td>"
          "<td><input type=text name='authtok' placeholder='%s' size=30></td></tr>",
          MxAuth.tokenSet() ? "impostato - scrivi per cambiarlo" : "NON impostato");
  sendFmt("<tr><td>Password di questa pagina</td>"
          "<td><input type=text name='authcfgpw' placeholder='%s' size=30></td></tr>",
          MxAuth.configPwSet() ? "impostata - scrivi per cambiarla" : "non impostata");
  sendChunk("</table>");

  sendChunk("<p><button type=submit>Salva</button></p></form>");
  sendChunk("<p id='cfgnote' style='color:#555'></p>");

  sendChunk("<p style='max-width:34em;margin:1em auto;text-align:left;color:#555'>"
            "<b>Il tempo di corsa va misurato col cronometro</b>, non indovinato. "
            "Questa valvola non ha finecorsa riportati: il nodo sa dov'è solo "
            "perché ha tenuto il motore acceso per tutto quel tempo. Se il valore "
            "è più <i>corto</i> della corsa vera, la valvola si ferma a metà e il "
            "nodo crede che sia a fondo corsa. Se è più <i>lungo</i>, il motore "
            "spinge contro il fermo per la differenza: meglio qualche secondo in "
            "più che in meno, ma non dieci.<br><br>"
            "<b>Chiusura all'avvio</b>: a ogni accensione il nodo fa una corsa di "
            "chiusura, così parte da uno stato certo. Togliendola, dopo un "
            "riavvio la posizione resta dichiarata <i>sconosciuta</i> finché non "
            "arriva un comando &mdash; più onesto, ma nessuno chiude l'acqua al "
            "posto suo.<br><br>"
            "<b>Solo POST</b> è il modo sicuro: un comando non si può più mettere "
            "in un segnalibro né innescare con un prefetch del browser. "
            "<b>POST e GET</b> serve solo durante la transizione; in quella "
            "modalità il token viaggia in query string e finisce nei log e nella "
            "cronologia. I campi del token e della password si lasciano vuoti per "
            "non cambiarli; per cancellarli si usa la seriale: "
            "<code>auth token off</code>, <code>auth cfgpw off</code>.</p>");

  sendChunk("<script>"
            "async function mxSaveCfg(e){e.preventDefault();"
            "var f=document.getElementById('cfgform');"
            "var r=await mxPost('/config/save',"
            "new URLSearchParams(new FormData(f)).toString());"
            "if(r){alert('Salvato.');location.reload();}return false;}"
            "(async function(){"
            "var h={'X-Mx-Cfg':localStorage.getItem('mxcfgpw')||''};"
            "var r=await fetch('/config/data',{headers:h});"
            "if(r.status===401){mxCfg(true);"
            "r=await fetch('/config/data',{headers:{'X-Mx-Cfg':"
            "localStorage.getItem('mxcfgpw')||''}});}"
            "if(!r.ok){document.getElementById('cfgnote').textContent="
            "'Valori non disponibili: password della pagina mancante o errata.';return;}"
            "var d=await r.json();"
            "document.getElementById('cfgnote').textContent="
            "'Corsa: '+(d.travel_s)+' s - token: '+(d.auth.token_set?'impostato':'NON impostato')+"
            "' - azioni: '+(d.auth.allow_get?'POST e GET':'solo POST')+"
            "(d.auth.misconfigured?' - ATTENZIONE: comandi bloccati':'');"
            "})();"
            "</script>");

  sendChunk("<p style='color:#555;font-size:12px'>I valori stanno in NVS e "
            "sopravvivono al riavvio e all'aggiornamento OTA. Il reset di fabbrica "
            "del WiFi li riporta ai valori iniziali.</p>");
  endHtml();
}

// Valori correnti, separati dal guscio HTML: un browser che NAVIGA non può
// mettere header, quindi la pagina si apre sempre e i valori arrivano solo a
// chi ha la password.
void handleConfigData() {
  String h = "{";
  h += "\"travel_s\":"  + String(tv.travelMs / 1000.0f, 1) + ",";
  h += "\"travel_ms\":" + String(tv.travelMs) + ",";
  h += "\"close_on_boot\":" + String(tv.closeOnBoot ? "true" : "false") + ",";
  h += MxAuth.statusJson();
  h += "}";
  server.send(200, "application/json", h);
}

void handleConfigSave() {
  if (server.hasArg("travel")) {
    uint32_t ms = (uint32_t)lroundf(server.arg("travel").toFloat() * 1000.0f);
    if (ms < TRAVEL_MS_MIN) ms = TRAVEL_MS_MIN;    // clamp, non rifiuto
    if (ms > TRAVEL_MS_MAX) ms = TRAVEL_MS_MAX;
    tv.travelMs = ms;
  }
  tv.closeOnBoot = server.hasArg("bootclose");

  // Sicurezza. Qui si arriva solo dopo checkAction + checkConfig: senza il
  // secondo, chiunque sulla LAN potrebbe rimettere "GET senza token" dalla
  // pagina stessa - il lucchetto sulla porta con la chiave appesa fuori.
  if (server.hasArg("authget")) MxAuth.setAllowGet(server.arg("authget") == "1");
  MxAuth.setRequireToken(server.hasArg("authreq"));
  if (server.hasArg("authtok") && server.arg("authtok").length())
    MxAuth.setToken(server.arg("authtok").c_str());
  if (server.hasArg("authcfgpw") && server.arg("authcfgpw").length())
    MxAuth.setConfigPassword(server.arg("authcfgpw").c_str());

  cfgSave();
  addCommandLog("CONFIG");
  Serial.printf("[CFG] salvata da web: corsa %.1f s, chiusura avvio %s\n",
                tv.travelMs / 1000.0f, tv.closeOnBoot ? "sì" : "no");

  // Una corsa in atto NON si tocca: cambiare il tempo a metà corsa renderebbe
  // imprevedibile dove si ferma. Il valore nuovo vale dalla prossima.
  server.send(200, "application/json", "{\"ok\":true,\"reboot\":false}");
}

// =========================================================================
// /status
// =========================================================================
//
// Un solo server.send() con il suo Content-Length: o il client riceve tutta
// la risposta, o vede un errore onesto. La versione a sendContent() multiple
// non controllate è quella che faceva sembrare offline il nodo .40.
//
// SENZA autenticazione, di proposito: la leggono il pannello di stato e il
// collettore dati, che non hanno un posto sicuro dove tenere un segreto. Da
// qui non passa nessun comando, e non esce nessun segreto: il blocco "auth"
// dice COM'È configurata l'autenticazione, non i valori.
void sendStatusJson() {
  String b;
  b.reserve(1100);

  char nowBuf[24], bootBuf[24];
  MxTime.local(nowBuf, sizeof(nowBuf));
  if (bootEpoch > 0) {
    struct tm tmv;
    localtime_r(&bootEpoch, &tmv);
    strftime(bootBuf, sizeof(bootBuf), "%Y-%m-%d %H:%M:%S", &tmv);
  } else {
    snprintf(bootBuf, sizeof(bootBuf), "n/d");
  }

  b += "{";
  b += "\"type\":\"tvalve\",";
  b += "\"firmware\":\"" + String(FW_VERSION) + "\",";
  b += "\"core\":\"" + String(MXCORE_VERSION) + "\",";
  b += "\"device_id\":\"" + String(MxIdentity.provisioned() ? MxIdentity.deviceId() : "") + "\",";
  b += "\"site\":\"" + String(MxIdentity.site()) + "\",";
  b += "\"ip\":\"" + MxNet.ip() + "\",";
  b += "\"wifi_connected\":" + String(MxNet.connected() ? "true" : "false") + ",";
  b += "\"wifi_rssi\":" + String((long)MxNet.rssi()) + ",";
  b += "\"ping_enabled\":false,";          // il ping è stato tolto nella 1.0
  b += "\"monitor_online\":" + String(MxNet.connected() ? "true" : "false") + ",";
  b += "\"time_synced\":" + String(MxTime.valid() ? "true" : "false") + ",";
  b += "\"datetime\":\"" + String(nowBuf) + "\",";
  b += "\"boot_datetime\":\"" + String(bootBuf) + "\",";
  b += "\"timezone\":\"" + String(cfg.tz) + "\",";
  b += "\"uptime_s\":" + String((unsigned long)MxCore.uptimeS()) + ",";
  b += "\"heap\":" + String((unsigned)ESP.getFreeHeap()) + ",";
  b += "\"ntp_age_s\":" + String((unsigned long)MxTime.lastSyncAgeS()) + ",";
  b += "\"ota\":" + String(MxOta.inProgress() ? "true" : "false") + ",";
  b += MxAuth.statusJson() + ",";

  // --- specifico della valvola a T ---
  b += "\"valve\":\"" + String(valveStateName(valveState)) + "\",";
  b += "\"position_known\":" + String(positionKnown() ? "true" : "false") + ",";
  b += "\"moving\":" + String(valveMoving() ? "true" : "false") + ",";
  b += "\"remaining_s\":" + String((unsigned long)travelRemainingS()) + ",";
  b += "\"travel_ms\":" + String(tv.travelMs) + ",";
  b += "\"close_on_boot\":" + String(tv.closeOnBoot ? "true" : "false") + ",";

  // Chiavi della 0.5, tenute per non rompere il pannello di stato che è già
  // in funzione. "valve_open" è vero solo a valvola CERTAMENTE aperta: a
  // posizione sconosciuta è false, e chi vuole la differenza legge "valve".
  b += "\"valve_open\":" + String(valveState == VS_OPEN ? "true" : "false") + ",";
  b += "\"pulse_active\":" + String(valveMoving() ? "true" : "false") + ",";
  b += "\"pulse_ms\":" + String(tv.travelMs) + ",";

  b += "\"relays\":[";
  for (int i = 0; i < 4; i++) {
    b += "{\"id\":" + String(i + 1) + ",\"on\":"
       + String(digitalRead(RELAY_PINS[i]) == RELAY_ON ? "true" : "false") + "}";
    if (i < 3) b += ",";
  }
  b += "],";

  b += "\"last_commands\":[";
  bool first = true;
  for (int i = 0; i < COMMAND_LOG_SIZE; i++) {
    if (!commandLog[i].valid) continue;
    if (!first) b += ",";
    first = false;
    b += "{\"timestamp\":\"" + String(commandLog[i].timestamp)
       + "\",\"action\":\"" + String(commandLog[i].action) + "\"}";
  }
  b += "]}";

  server.send(200, "application/json", b);
}

// =========================================================================
// ROTTE D'AZIONE
// =========================================================================

// Risposta di un comando accettato. Dice "accodato/avviato", non "fatto": la
// corsa dura travelMs, e chi comanda deve saperlo per non contare su una
// valvola che sta ancora viaggiando.
void sendTravelReply(TravelResult r, bool toOpen) {
  if (r == TR_BUSY) {
    String b = "{\"ok\":false,\"error\":\"closing\",\"hint\":\"la chiusura in corso non si interrompe\",";
    b += "\"remaining_s\":" + String((unsigned long)travelRemainingS()) + "}";
    server.send(409, "application/json", b);
    return;
  }

  const char* what = (r == TR_ALREADY) ? "already" : (r == TR_REVERSED) ? "reversed" : "started";
  String b = "{\"ok\":true,\"action\":\"";
  b += toOpen ? "open" : "close";
  b += "\",\"result\":\"" + String(what) + "\"";
  b += ",\"valve\":\"" + String(valveStateName(valveState)) + "\"";
  b += ",\"remaining_s\":" + String((unsigned long)travelRemainingS());
  b += ",\"travel_ms\":" + String(tv.travelMs);
  b += "}";
  server.send(200, "application/json", b);
}

void handleOpen() {
  TravelResult r = requestTravel(true);
  addCommandLog(r == TR_BUSY ? "APRI rifiut." : r == TR_ALREADY ? "APRI (già)" : "APRI");
  sendTravelReply(r, true);
}

void handleClose() {
  TravelResult r = requestTravel(false);
  addCommandLog(r == TR_REVERSED ? "CHIUDI inv." : r == TR_ALREADY ? "CHIUDI (già)" : "CHIUDI");
  sendTravelReply(r, false);
}

// L'OTA è un'azione: cambia il firmware. Passa dallo stesso controllo dei
// comandi valvola. La libreria vuole un puntatore a funzione senza argomenti.
static bool otaWebAuth() { return MxAuth.checkAction(server); }

void registerRoutes() {
  // ⚠️ Senza questa il WebServer SCARTA gli header che non gli sono stati
  // dichiarati: server.header("X-Mx-Token") tornerebbe sempre vuoto e ogni
  // comando sarebbe rifiutato con 401 senza che si capisca perché.
  MxAuth.collectHeaders(server);

  server.on("/",     HTTP_GET, sendHomePage);
  server.on("/info", HTTP_GET, sendInfoPage);
  server.on("/logs", HTTP_GET, sendLogsPage);
  server.on("/status", HTTP_GET, sendStatusJson);   // senza auth, vedi sopra

  server.on("/config",      HTTP_GET, sendConfigPage);
  server.on("/config/data", HTTP_GET, []() { if (MxAuth.checkConfig(server)) handleConfigData(); });
  server.on("/config/save", HTTP_ANY, []() {
    if (!MxAuth.checkAction(server)) return;
    if (!MxAuth.checkConfig(server)) return;
    handleConfigSave();
  });

  // Valvola. HTTP_ANY perché il metodo lo decide MxAuth: così una GET rimasta
  // in un segnalibro riceve 405 con l'header Allow, che dice cosa è cambiato,
  // e non un 404 che sembra un guasto. I nomi /on e /off sono quelli della
  // 0.5: cambiarli avrebbe rotto tutto quello che già li usa.
  server.on("/on",  HTTP_ANY, []() { if (MxAuth.checkAction(server)) handleOpen(); });
  server.on("/off", HTTP_ANY, []() { if (MxAuth.checkAction(server)) handleClose(); });

  server.on("/ota", HTTP_POST, []() {
    if (!MxAuth.checkAction(server)) return;
    MxOta.pending = true;   // il download parte dal loop(), l'handler non resta appeso
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"ota\"}");
  });
  MxOta.attachWeb(server, "/ota", "/", otaWebAuth);

  server.on("/reboot", HTTP_ANY, []() {
    if (!MxAuth.checkAction(server)) return;
    addCommandLog("REBOOT");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reboot\"}");
    delay(500);
    ESP.restart();
  });

  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// =========================================================================
// RECOVERY
// =========================================================================

void checkLowHeapRecovery() {
#if ENABLE_AUTO_RESTART_ON_LOW_HEAP
  uint32_t heap = ESP.getFreeHeap();
  if (heap < LOW_HEAP_THRESHOLD) {
    if (lowHeapSinceMs == 0) {
      lowHeapSinceMs = millis();
      Serial.printf("[HEAP] basso: %u byte\n", (unsigned)heap);
    } else if (millis() - lowHeapSinceMs >= LOW_HEAP_GRACE_MS) {
      // ⚠️ Un riavvio a metà corsa lascia la valvola a metà: lo si dice, e al
      // boot successivo la posizione sarà "sconosciuta".
      if (valveMoving()) Serial.println("[HEAP] riavvio con la valvola in movimento!");
      Serial.println("[HEAP] sotto soglia da troppo tempo: riavvio");
      allRelaysOff();
      delay(100);
      ESP.restart();
    }
  } else {
    lowHeapSinceMs = 0;
  }
#endif
}

void captureBootEpochIfNeeded() {
  if (bootEpoch == 0 && MxTime.valid()) {
    bootEpoch = MxTime.epoch() - (time_t)MxCore.uptimeS();
  }
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
      // id set "Nome Pista" "nome-device" [apiBase] [apiToken] [ntfyUrl]
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
      Serial.printf("[VALVE] %s | posizione nota: %s | in moto: %s (mancano %lu s)\n",
                    valveStateName(valveState), positionKnown() ? "sì" : "NO",
                    valveMoving() ? "sì" : "no", (unsigned long)travelRemainingS());
    } else if (strcmp(line, "open") == 0) {
      TravelResult r = requestTravel(true);
      addCommandLog("APRI ser.");
      Serial.println(r == TR_BUSY    ? "[VALVE] rifiutato: sta chiudendo"
                   : r == TR_ALREADY ? "[VALVE] già in apertura"
                                     : "[VALVE] apertura avviata");
    } else if (strcmp(line, "close") == 0) {
      TravelResult r = requestTravel(false);
      addCommandLog("CHIUDI ser.");
      Serial.println(r == TR_REVERSED ? "[VALVE] inversione: chiusura"
                   : r == TR_ALREADY  ? "[VALVE] già in chiusura"
                                      : "[VALVE] chiusura avviata");
    } else if (strncmp(line, "travel ", 7) == 0) {
      float s = atof(line + 7);
      uint32_t ms = (uint32_t)lroundf(s * 1000.0f);
      if (ms < TRAVEL_MS_MIN) ms = TRAVEL_MS_MIN;
      if (ms > TRAVEL_MS_MAX) ms = TRAVEL_MS_MAX;
      tv.travelMs = ms;
      cfgSave();
      Serial.printf("[CFG] tempo di corsa: %.1f s\n", tv.travelMs / 1000.0f);
    } else if (strcmp(line, "cfg show") == 0) {
      Serial.printf("[CFG] corsa %.1f s (min %.1f, max %.1f)\n",
                    tv.travelMs / 1000.0f, TRAVEL_MS_MIN / 1000.0f, TRAVEL_MS_MAX / 1000.0f);
      Serial.printf("[CFG] chiusura all'avvio: %s\n", tv.closeOnBoot ? "sì" : "no");
      MxAuth.printTo(Serial);
    } else if (strcmp(line, "cfg reset") == 0) {
      tv = TV_DEFAULT;
      cfgSave();
      Serial.println("[CFG] riportata ai valori di fabbrica");
    } else if (strncmp(line, "auth ", 5) == 0) {
      char* a = line + 5;
      if (strncmp(a, "token ", 6) == 0) {
        MxAuth.setToken(a + 6);
      } else if (strcmp(a, "token off") == 0) {
        MxAuth.setToken("");
      } else if (strncmp(a, "cfgpw ", 6) == 0) {
        MxAuth.setConfigPassword(a + 6);
      } else if (strcmp(a, "cfgpw off") == 0) {
        // Via d'uscita se la password della pagina si dimentica.
        MxAuth.setConfigPassword("");
      } else if (strcmp(a, "get on") == 0) {
        MxAuth.setAllowGet(true);
        Serial.println("[AUTH] accetto anche GET (compatibilità). Il token, se richiesto,");
        Serial.println("[AUTH] viaggia in query string: finisce nei log e nella cronologia.");
      } else if (strcmp(a, "get off") == 0) {
        MxAuth.setAllowGet(false);
        Serial.println("[AUTH] solo POST");
      } else if (strcmp(a, "require on") == 0) {
        MxAuth.setRequireToken(true);
      } else if (strcmp(a, "require off") == 0) {
        MxAuth.setRequireToken(false);
        Serial.println("[AUTH] *** token NON più richiesto: chiunque sulla LAN comanda.");
      } else if (strcmp(a, "show") == 0) {
        MxAuth.printTo(Serial);   // non stampa mai i segreti
      } else {
        Serial.println("uso: auth token <v> | token off | cfgpw <v> | cfgpw off |");
        Serial.println("     get on|off | require on|off | show");
      }
    } else if (strcmp(line, "ota") == 0) {
      MxOta.pending = true;
      Serial.println("[OTA] check richiesto");
    } else if (strcmp(line, "reboot") == 0) {
      allRelaysOff();
      ESP.restart();
    } else if (line[0]) {
      Serial.println("comandi: id set / id show / wifi reset / status / open / close /");
      Serial.println("         travel <s> / cfg show / cfg reset / auth ... / ota / reboot");
    }
  }
}

// =========================================================================
// SETUP / LOOP
// =========================================================================

// Compone l'endpoint del pull OTA. Definita qui sopra apposta: l'IDE non
// genera sempre il prototipo per una funzione usata prima di essere definita.
void buildOtaUrl() {
  otaHttpUrl[0] = 0;
  if (!OTA_HTTP_BASE || !OTA_HTTP_BASE[0]) {
    Serial.println("[OTA] pull disabilitato (OTA_HTTP_BASE vuoto)");
    return;
  }

  const char* id = MxIdentity.provisioned() ? MxIdentity.deviceId() : "";
  if (!id[0]) {
    // Senza identità il server non sa chi ha chiamato, ma il binario dipende
    // da "fw", non da "id": non è un motivo per rinunciare all'aggiornamento.
    Serial.println("[OTA] identità non impostata: il server vedrà id=sconosciuto");
    id = "sconosciuto";
  }

  int n = snprintf(otaHttpUrl, sizeof(otaHttpUrl), "%s?fw=%s&id=%s&token=%s",
                   OTA_HTTP_BASE, OTA_FW_NAME, id, OTA_TOKEN ? OTA_TOKEN : "");
  if (n < 0 || n >= (int)sizeof(otaHttpUrl)) {
    otaHttpUrl[0] = 0;
    Serial.printf("[OTA] URL troppo lungo (%d caratteri, max %u): pull disabilitato\n",
                  n, (unsigned)sizeof(otaHttpUrl) - 1);
    return;
  }
  Serial.printf("[OTA] pull: fw=%s id=%s\n", OTA_FW_NAME, id);
}

// Prima di un aggiornamento: prestazioni piene. A 80 MHz con il modem che
// dorme, scaricare 1,2 MB è lento e ci si mette di mezzo la latenza della
// radio. Al termine il device si riavvia comunque, quindi il risparmio
// energetico torna da sé senza doverlo ripristinare.
void onOtaBefore() {
#if ENABLE_CPU_DOWNCLOCK
  setCpuFrequencyMhz(CPU_MHZ_FULL);
#endif
#if ENABLE_WIFI_MODEM_SLEEP
  WiFi.setSleep(false);
#endif
  Serial.println("[OTA] prestazioni piene per l'aggiornamento");
}

void applyPowerSavings() {
#if ENABLE_BT_OFF
  // btStop() ferma già il controller: esp_bt_controller_disable() in più era
  // ridondante nella 0.4 e loggava un errore. Tolta.
  btStop();
#endif
#if ENABLE_WIFI_MODEM_SLEEP
  WiFi.setSleep(true);
#endif
#if ENABLE_CPU_DOWNCLOCK
  setCpuFrequencyMhz(CPU_MHZ_SAVING);
  Serial.printf("[PWR] CPU a %lu MHz, modem-sleep attivo\n", (unsigned long)CPU_MHZ_SAVING);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("MxIrrigation T-Valve by MxSolutions.it - www.mxsolutions.it");
  Serial.printf("Firmware %s su MxSolutionCore %s\n", FW_VERSION, MXCORE_VERSION);
  Serial.println("----------------------------------------");

  // PRIMA di tutto: relè in uno stato noto. La posizione della valvola invece
  // NON è nota e resta dichiarata tale finché non si fa una corsa intera.
  for (int i = 0; i < 4; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], RELAY_OFF);
  }
  valveState = VS_UNKNOWN;

  cfgLoad();

  // L'identità serve PRIMA di comporre l'URL OTA. load() è una lettura NVS
  // idempotente: MxCore.begin() la rifarà per conto suo.
  MxIdentity.load();
  buildOtaUrl();

  cfg.fwVersion       = FW_VERSION;
  cfg.wdtSeconds      = 15;
  cfg.otaPassword     = OTA_PASSWORD;
  cfg.otaHttpUrl      = otaHttpUrl;
  cfg.factoryResetPin = FACTORY_RESET_PIN;
  cfg.requireIdentity = false;   // senza identità la valvola deve comunque funzionare
  cfg.onIdle          = handleSerial;   // seriale viva anche dentro il captive portale
  cfg.staticIP        = STATIC_IP;
  cfg.gateway         = GATEWAY;
  cfg.subnet          = SUBNET;
  cfg.dns1            = DNS1;
  cfg.dns2            = DNS2;

  MxOta.onBefore(onOtaBefore);

  MxCore.begin(cfg);   // identità -> watchdog -> WiFi/portale -> NTP -> OTA

  // Il risparmio energetico si applica DOPO: il captive portale e il primo
  // sync NTP girano a piena velocità.
  applyPowerSavings();

  registerRoutes();
  server.begin();
  Serial.println("[HTTP] server avviato sulla porta 80");

  // Chiusura all'avvio: una corsa intera, così il nodo parte da uno stato
  // certo invece che da "sconosciuto". È configurabile perché è una scelta
  // d'impianto: chi non la vuole accetta di ripartire senza sapere dov'è la
  // valvola. NON è bloccante: la corsa la porta avanti serviceMotion().
  if (tv.closeOnBoot) {
    Serial.println("[VALVE] chiusura all'avvio");
    requestTravel(false);
    addCommandLog("CHIUDI boot");
  } else {
    Serial.println("[VALVE] chiusura all'avvio disattivata: posizione SCONOSCIUTA");
  }

  if (MxAuth.misconfigured()) {
    Serial.println("[AUTH] *** COMANDI VALVOLA BLOCCATI: manca il token.");
    Serial.println("[AUTH]     impostalo adesso con:  auth token <valore>");
  } else if (!MxAuth.requireToken()) {
    Serial.println("[AUTH] *** nessun token: chiunque sulla LAN muove la valvola.");
  }
}

void loop() {
  MxCore.loop();          // watchdog + WiFi + OTA + persistenza ora
  server.handleClient();
  serviceMotion();        // motore: tempo morto, corsa, fine corsa
  handleSerial();
  checkLowHeapRecovery();
  captureBootEpochIfNeeded();
}
