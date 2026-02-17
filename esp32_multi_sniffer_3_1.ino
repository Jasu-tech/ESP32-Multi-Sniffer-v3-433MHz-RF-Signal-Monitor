/*
 * ESP32 Multi-Sniffer v3 - 433MHz RF Signal Monitor
 * 
 * Multi-radio 433MHz RF signal monitoring system using ESP32 and CC1101
 * radio modules. Supports up to 3 simultaneous CC1101 radios with
 * independent modulation settings (ASK/OOK, 2-FSK, GFSK, MSK, Wideband ASK).
 * 
 * Features:
 *   - Real-time mobile-optimized web UI with dark/light theme
 *   - Automatic device identification (PT2262, EV1527, Nexa, Came, Nice, etc.)
 *   - RAW/rolling code signal detection via configurable RSSI threshold
 *   - Duplicate filtering with 30-second cooldown
 *   - Push notifications and remote control via ntfy.sh
 *   - AP+STA hybrid WiFi (AP for web UI, STA for internet/ntfy)
 *   - NTP time synchronization for accurate timestamps
 * 
 * Hardware: ESP32 DevKit V1 (dual-core) or ESP32-S3 Zero
 *           CC1101 radio module(s) on SPI bus at 433.92 MHz
 * 
 * (C) Jasu-tech
 * Licensed under the MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <time.h>

// --- SELECT BOARD (Uncomment the one you are using) ---
#define BOARD_ESP32_STANDARD  // Standard ESP32 (DevKit V1, dual-core)
// #define BOARD_ESP32_S3_ZERO   // ESP32-S3 Zero (dual-core)

// --- AP SETTINGS (Web UI) --- <- Change what you like
const char* ap_ssid = "ESP32-Multi-Sniffer";
const char* ap_password = "ESP3admin";

// --- STA SETTINGS (Internet/ntfy) ---
const char* sta_ssid = "HOME_WIFI";       // <- Change to your WiFi name
const char* sta_password = "HOME_PASSWORD"; // <- Change to your WiFi password

// --- NTFY SETTINGS ---
// IMPORTANT: Change the topic name below to something unique (e.g. "MyHome-Sniffer-42")
// so you don't share the channel with other users!
const char* ntfyTopic = "MultiSniffer";    // <- Change this to YOUR OWN unique topic name!
const char* ntfyCmdTopic = "MultiSniffer";  // <- Must be the SAME as ntfyTopic above!
bool ntfyEnabled = false;
bool staConnected = false;
bool ntpSynced = false;

// --- PIN DEFINITIONS ---

#if defined(BOARD_ESP32_STANDARD)
  // Standard ESP32
  const int SCK_PIN = 18;
  const int MISO_PIN = 19;
  const int MOSI_PIN = 23;
  // Radios
  const int CSN1 = 5;  const int GDO1 = 17;
  const int CSN2 = 4;  const int GDO2 = 15; // Warning: GPIO 15 is a strapping pin!
  const int CSN3 = 14; const int GDO3 = 27;

#elif defined(BOARD_ESP32_S3_ZERO)
  // ESP32-S3 Zero (e.g. Waveshare)
  // S3 allows SPI pin configuration on almost any GPIO
  const int SCK_PIN = 12;
  const int MISO_PIN = 13;
  const int MOSI_PIN = 11;
  // Radios
  const int CSN1 = 10; const int GDO1 = 9;
  const int CSN2 = 14; const int GDO2 = 15; // Warning: GPIO 0 is BOOT pin
  const int CSN3 = 16; const int GDO3 = 17;

#else
  #error "Select a board at the top of the code!"
#endif


// --- DATA STRUCTURES ---
struct SignalLog {
  unsigned long timestamp;
  int radioId;
  long value;
  int bitLength;
  int protocol;
  int rssi;
  bool rawDetection;
  char deviceType[32];
};

// Radio states (defaults)
int radioModulation[4] = {0, 2, 0, 2}; // Index 1-3. 2=ASK, 0=FSK
bool radioPresent[4] = {false, false, false, false}; // Is radio connected?
String radioModeName[4] = {"", "ASK/OOK", "2-FSK", "Wideband ASK"};

const int HISTORY_SIZE = 100;
SignalLog history[HISTORY_SIZE];
int historyIndex = 0;
int totalSignals = 0;
int rssiThreshold = -60; // Default threshold, adjustable from web UI
bool rssiLocked = true; // RSSI slider lock (locked by default)

RCSwitch mySwitch = RCSwitch();

// Returns epoch seconds (NTP) or millis/1000 if not synced
unsigned long getTimestamp() {
  if (ntpSynced) {
    time_t now;
    time(&now);
    return (unsigned long)now;
  }
  return millis() / 1000;
}

// --- NTFY DUPLICATE FILTERING ---
struct NtfyRecent {
  int radioId;
  long value;
  unsigned long timestamp;
};
const int NTFY_RECENT_SIZE = 10;
NtfyRecent ntfyRecent[NTFY_RECENT_SIZE];
int ntfyRecentIndex = 0;
const unsigned long DEDUP_MS = 30000; // 30s same radio+value = skip

// --- LOG DUPLICATE FILTERING ---
struct LogRecent {
  int radioId;
  long value;
  unsigned long timestamp;
};
const int LOG_RECENT_SIZE = 10;
LogRecent logRecent[LOG_RECENT_SIZE];
int logRecentIndex = 0;

bool isLogDuplicate(int radioId, long value) {
  unsigned long now = millis();
  for (int i = 0; i < LOG_RECENT_SIZE; i++) {
    if (logRecent[i].radioId == radioId && logRecent[i].value == value) {
      if (now - logRecent[i].timestamp < DEDUP_MS) {
        return true;
      }
    }
  }
  logRecent[logRecentIndex].radioId = radioId;
  logRecent[logRecentIndex].value = value;
  logRecent[logRecentIndex].timestamp = now;
  logRecentIndex = (logRecentIndex + 1) % LOG_RECENT_SIZE;
  return false;
}

bool isNtfyDuplicate(int radioId, long value) {
  unsigned long now = millis();
  for (int i = 0; i < NTFY_RECENT_SIZE; i++) {
    if (ntfyRecent[i].radioId == radioId && ntfyRecent[i].value == value) {
      if (now - ntfyRecent[i].timestamp < DEDUP_MS) {
        return true; // Duplicate
      }
    }
  }
  // Store new entry
  ntfyRecent[ntfyRecentIndex].radioId = radioId;
  ntfyRecent[ntfyRecentIndex].value = value;
  ntfyRecent[ntfyRecentIndex].timestamp = now;
  ntfyRecentIndex = (ntfyRecentIndex + 1) % NTFY_RECENT_SIZE;
  return false;
}

// --- NTFY QUEUE ---
struct NtfyMsg {
  char title[60];
  char body[120];
};
const int NTFY_QUEUE_SIZE = 10;
NtfyMsg ntfyQueue[NTFY_QUEUE_SIZE];
int ntfyQueueHead = 0;
int ntfyQueueTail = 0;

void queueNtfy(int radioId, long value, const char* title, const char* body, bool force = false) {
  if (!ntfyEnabled && !force) return;
  if (!force && isNtfyDuplicate(radioId, value)) return; // Same radio+value within 30s
  int next = (ntfyQueueHead + 1) % NTFY_QUEUE_SIZE;
  if (next == ntfyQueueTail) return;
  strncpy(ntfyQueue[ntfyQueueHead].title, title, 59);
  ntfyQueue[ntfyQueueHead].title[59] = '\0';
  strncpy(ntfyQueue[ntfyQueueHead].body, body, 119);
  ntfyQueue[ntfyQueueHead].body[119] = '\0';
  ntfyQueueHead = next;
}

void processNtfyQueue() {
  if (!staConnected) return;
  if (ntfyQueueTail == ntfyQueueHead) return;
  
  WiFiClient client;
  client.setTimeout(3); // 3 second timeout
  
  if (client.connect("ntfy.sh", 80)) {
    String body = String(ntfyQueue[ntfyQueueTail].body);
    String request = "POST /" + String(ntfyTopic) + " HTTP/1.1\r\n";
    request += "Host: ntfy.sh\r\n";
    request += "Title: " + String(ntfyQueue[ntfyQueueTail].title) + "\r\n";
    request += "Tags: signal\r\n";
    request += "Content-Length: " + String(body.length()) + "\r\n";
    request += "Connection: close\r\n\r\n";
    request += body;
    
    client.print(request);
    
    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 3000) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        if (line.startsWith("HTTP/")) {
          Serial.printf("ntfy: %s [%s]\n", ntfyQueue[ntfyQueueTail].title, line.c_str());
        }
        break;
      }
      delay(10);
    }
    client.stop();
  } else {
    Serial.println("ntfy: connect failed!");
  }
  ntfyQueueTail = (ntfyQueueTail + 1) % NTFY_QUEUE_SIZE;
}

// --- NTFY COMMAND HANDLER ---
void handleNtfyCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  Serial.printf("ntfy CMD: '%s'\n", cmd.c_str());
  
  if (cmd == "status") {
    char msg[200];
    snprintf(msg, sizeof(msg), "Radio1: %s (%s) | RSSI: %d dBm | Signals: %d | ntfy: %s | STA: %s",
      radioPresent[1] ? "OK" : "OFF", radioModeName[1].c_str(),
      rssiThreshold, historyIndex,
      ntfyEnabled ? "ON" : "OFF",
      staConnected ? "connected" : "disconnected");
    queueNtfy(0, -999, "Status", msg, true);
  }
  else if (cmd == "clear") {
    for (int i = 0; i < HISTORY_SIZE; i++) history[i].timestamp = 0;
    historyIndex = 0;
    queueNtfy(0, -998, "Log Cleared", "Signal log cleared via ntfy", true);
    Serial.println("Log cleared via ntfy");
  }
  else if (cmd == "ntfy off") {
    ntfyEnabled = false;
    queueNtfy(0, -997, "ntfy Disabled", "Notifications turned OFF", true);
    Serial.println("ntfy disabled via command");
  }
  else if (cmd == "ntfy on") {
    ntfyEnabled = true;
    queueNtfy(0, -997, "ntfy Enabled", "Notifications turned ON", true);
  }
  else if (cmd.startsWith("rssi ")) {
    int val = cmd.substring(5).toInt();
    if (val >= -100 && val <= -30 && !rssiLocked) {
      rssiThreshold = val;
      char msg[60];
      snprintf(msg, sizeof(msg), "RSSI threshold set to %d dBm", val);
      queueNtfy(0, -996, "RSSI Updated", msg, true);
      Serial.printf("RSSI set to %d via ntfy\n", val);
    }
  }
  else if (cmd.startsWith("mod ")) {
    // "mod 1 2" = Radio 1 -> ASK/OOK (2)
    int radio = 0, mod = 0;
    if (sscanf(cmd.c_str(), "mod %d %d", &radio, &mod) == 2) {
      if (radio >= 1 && radio <= 3 && radioPresent[radio]) {
        int csn = (radio == 1) ? CSN1 : (radio == 2) ? CSN2 : CSN3;
        setupRadio(csn, radio, mod);
        mySwitch.enableReceive(digitalPinToInterrupt(getGDO(radio)));
        char msg[80];
        snprintf(msg, sizeof(msg), "Radio %d -> %s", radio, radioModeName[radio].c_str());
        queueNtfy(0, -995, "Mod Changed", msg, true);
      }
    }
  }
  else if (cmd == "lock") {
    rssiLocked = true;
    queueNtfy(0, -994, "RSSI Locked", "RSSI threshold locked", true);
  }
  else if (cmd == "unlock") {
    rssiLocked = false;
    queueNtfy(0, -993, "RSSI Unlocked", "RSSI threshold unlocked", true);
  }
  else if (cmd == "reboot") {
    queueNtfy(0, -991, "Rebooting", "ESP32 restarting now...", true);
    delay(500);
    ESP.restart();
  }
  else if (cmd == "scan") {
    char msg[200];
    snprintf(msg, sizeof(msg), "R1: %s (%s) | R2: %s (%s) | R3: %s (%s)",
      radioPresent[1] ? "OK" : "OFF", radioModeName[1].c_str(),
      radioPresent[2] ? "OK" : "OFF", radioModeName[2].c_str(),
      radioPresent[3] ? "OK" : "OFF", radioModeName[3].c_str());
    queueNtfy(0, -990, "Radio Scan", msg, true);
  }
  else if (cmd == "threshold") {
    char msg[60];
    snprintf(msg, sizeof(msg), "RSSI threshold: %d dBm (%s)", rssiThreshold, rssiLocked ? "locked" : "unlocked");
    queueNtfy(0, -989, "Threshold", msg, true);
  }
  else if (cmd == "signals") {
    int count = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) if (history[i].timestamp != 0) count++;
    char msg[120];
    if (count > 0) {
      int last = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
      snprintf(msg, sizeof(msg), "Total: %d | Last: %s val=%ld RSSI=%d dBm R%d",
        count, history[last].deviceType, history[last].value, history[last].rssi, history[last].radioId);
    } else {
      snprintf(msg, sizeof(msg), "No signals recorded");
    }
    queueNtfy(0, -988, "Signals", msg, true);
  }
  else if (cmd == "info") {
    unsigned long upSec = millis() / 1000;
    int hrs = upSec / 3600;
    int mins = (upSec % 3600) / 60;
    char msg[200];
    snprintf(msg, sizeof(msg), "Uptime: %dh%dm | Free RAM: %d bytes | AP IP: %s | STA IP: %s | NTP: %s",
      hrs, mins, ESP.getFreeHeap(),
      WiFi.softAPIP().toString().c_str(),
      staConnected ? WiFi.localIP().toString().c_str() : "N/A",
      ntpSynced ? "synced" : "not synced");
    queueNtfy(0, -987, "Device Info", msg, true);
  }
  else if (cmd == "help") {
    queueNtfy(0, -992, "Commands", "status|clear|ntfy on/off|rssi -70|mod 1 2|lock|unlock|scan|threshold|signals|info|reboot|help", true);
  }
}

unsigned long lastPollTime = 0; // Unix timestamp of last successful poll

void pollNtfyCommands() {
  if (!staConnected) return;
  Serial.println("ntfy: polling...");
  
  WiFiClient client;
  client.setTimeout(10);
  
  if (client.connect("ntfy.sh", 80)) {
    String sinceParam = "30s";
    if (lastPollTime > 0) {
      sinceParam = String(lastPollTime);
    }
    String request = "GET /" + String(ntfyCmdTopic) + "/raw?poll=1&since=" + sinceParam + " HTTP/1.0\r\n";
    request += "Host: ntfy.sh\r\n\r\n";
    client.print(request);
    
    unsigned long timeout = millis();
    while (!client.available() && millis() - timeout < 5000) {
      delay(10);
    }
    
    bool headersDone = false;
    int cmdCount = 0;
    
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      
      if (!headersDone) {
        if (line.length() == 0) headersDone = true;
        continue;
      }
      
      if (line.length() == 0) continue;
      
      Serial.printf("ntfy raw [%d]: '%s'\n", cmdCount + 1, line.c_str());
      
      if (line.indexOf("|") >= 0 || line.startsWith("Radio") || line.startsWith("RSSI") || 
          line.startsWith("Value") || line.startsWith("Uptime") || line.startsWith("Total") ||
          line.startsWith("No signals") || line.startsWith("ESP32") || line.startsWith("R1 ") || 
          line.startsWith("R2 ") || line.startsWith("R3 ") || line.startsWith("Notifications") ||
          line.startsWith("Signal") || line.startsWith("Log") || line.startsWith("Commands")) {
        Serial.printf("ntfy: skip own: '%s'\n", line.c_str());
      } else {
        cmdCount++;
        Serial.printf("ntfy CMD: '%s'\n", line.c_str());
        handleNtfyCommand(line);
      }
    }
    client.stop();
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec > 1700000000) {
      lastPollTime = tv.tv_sec;
    }
    Serial.printf("ntfy: done, %d cmds\n", cmdCount);
  } else {
    Serial.println("ntfy: connect FAILED!");
  }
}

// Device identification based on decoded signal
void identifyDevice(int protocol, int bitLength, long value, int modulation, char* result) {
  // ASK/OOK (mod 2 or 5)
  if (modulation == 2 || modulation == 5) {
    
    // --- PROTOCOL 1 (350µs, sync 1:31) - Most common ---
    if (protocol == 1) {
      if (bitLength == 12) {
        strcpy(result, "PT2260");
        return;
      }
      if (bitLength == 24) {
        long address = (value >> 4) & 0xFFFFF;
        int cmd = value & 0xF;
        if (address > 0 && address < 0xFFFFF) {
          if (cmd <= 0x5) {
            strcpy(result, "EV1527 Remote");
          } else {
            strcpy(result, "PT2262 Switch");
          }
        } else {
          strcpy(result, "PT2262/EV1527");
        }
        return;
      }
      if (bitLength == 32) {
        long unitCode = value & 0xF;
        bool onOff = (value >> 4) & 1;
        if (unitCode <= 15) {
          strcpy(result, "Nexa/HomeEasy");
        } else {
          strcpy(result, "ASK 32bit");
        }
        return;
      }
      if (bitLength == 20) {
        strcpy(result, "Dooya Shutter");
        return;
      }
      if (bitLength == 36) {
        strcpy(result, "Watchman Sonic");
        return;
      }
      if (bitLength == 28) {
        strcpy(result, "Lacrosse Sensor");
        return;
      }
      if (bitLength == 40) {
        strcpy(result, "Temp/Hum Sensor");
        return;
      }
      if (bitLength == 48) {
        strcpy(result, "Oregon Sensor");
        return;
      }
      if (bitLength == 64) {
        strcpy(result, "Rolling Code 64b");
        return;
      }
      snprintf(result, 30, "ASK P1/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 2 (650µs, sync 1:10) ---
    if (protocol == 2) {
      if (bitLength == 12) {
        strcpy(result, "Came TOP");
        return;
      }
      if (bitLength == 24) {
        strcpy(result, "Came Wireless");
        return;
      }
      snprintf(result, 30, "Came P2/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 3 (100µs, sync 30:71) ---
    if (protocol == 3) {
      if (bitLength == 12) {
        strcpy(result, "Nice FLO");
        return;
      }
      if (bitLength == 24) {
        strcpy(result, "Nice Flor-S");
        return;
      }
      snprintf(result, 30, "Nice P3/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 4 (380µs, sync 1:6) ---
    if (protocol == 4) {
      if (bitLength == 12 || bitLength == 24) {
        strcpy(result, "V2 Phoenix");
        return;
      }
      snprintf(result, 30, "V2 P4/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 5 (500µs, sync 6:14, inverted) - HT6P20B ---
    if (protocol == 5) {
      if (bitLength == 24) {
        strcpy(result, "HT6P20B Remote");
        return;
      }
      if (bitLength == 28 || bitLength == 32) {
        strcpy(result, "Intertechno");
        return;
      }
      snprintf(result, 30, "HT6P20B P5/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 6 (450µs, sync 23:1) - HS2303-PT / Aukey ---
    if (protocol == 6) {
      if (bitLength == 24) {
        strcpy(result, "HS2303/Aukey");
        return;
      }
      strcpy(result, "FHT80");
      return;
    }
    
    // --- PROTOCOL 7 (150µs, sync 2:62) - Conrad RS-200 RX ---
    if (protocol == 7) {
      strcpy(result, "Conrad RS-200");
      return;
    }
    
    // --- PROTOCOL 8 (200µs, sync 130:7, inverted) - Conrad TX ---
    if (protocol == 8) {
      strcpy(result, "Conrad RS-200 TX");
      return;
    }
    
    // --- PROTOCOL 9 (365µs, sync 18:1, inverted) - 1ByOne ---
    if (protocol == 9) {
      strcpy(result, "1ByOne Doorbell");
      return;
    }
    
    // --- PROTOCOL 10 (270µs, sync 36:1, inverted) - HT12E ---
    if (protocol == 10) {
      if (bitLength == 12) {
        strcpy(result, "HT12E Encoder");
        return;
      }
      snprintf(result, 30, "HT12E P10/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 11 (320µs, sync 36:1, inverted) - SM5212 ---
    if (protocol == 11) {
      if (bitLength == 24) {
        strcpy(result, "SM5212 Switch");
        return;
      }
      snprintf(result, 30, "SM5212 P11/%db", bitLength);
      return;
    }
    
    // --- PROTOCOL 12 (300µs, sync 1:32) ---
    if (protocol == 12) {
      strcpy(result, "Proove/Anslut");
      return;
    }
    
    // --- Bit length based fallback ---
    if (bitLength == 8) {
      strcpy(result, "Simple Remote 8b");
      return;
    }
    if (bitLength == 16) {
      strcpy(result, "Simple Remote 16b");
      return;
    }
    if (bitLength == 36) {
      strcpy(result, "Sensor 36bit");
      return;
    }
    if (bitLength == 28) {
      strcpy(result, "Sensor 28bit");
      return;
    }
    
    // Generic ASK fallback
    snprintf(result, 30, "ASK P%d/%db", protocol, bitLength);
    return;
  }
  
  // --- FSK (mod 0) ---
  if (modulation == 0) {
    if (bitLength >= 64) {
      strcpy(result, "FSK Rolling Code");
      return;
    }
    if (bitLength == 52) {
      strcpy(result, "FSK Car Remote");
      return;
    }
    if (bitLength == 48) {
      strcpy(result, "FSK Sensor");
      return;
    }
    if (bitLength == 32) {
      strcpy(result, "FSK Keyfob");
      return;
    }
    snprintf(result, 30, "FSK %db", bitLength);
    return;
  }
  
  // --- GFSK (mod 1) ---
  if (modulation == 1) {
    if (bitLength >= 64) {
      strcpy(result, "GFSK Smart Device");
      return;
    }
    if (bitLength >= 32) {
      strcpy(result, "GFSK Sensor");
      return;
    }
    snprintf(result, 30, "GFSK %db", bitLength);
    return;
  }
  
  // --- MSK (mod 4) ---
  if (modulation == 4) {
    if (bitLength >= 64) {
      strcpy(result, "MSK Digital");
      return;
    }
    snprintf(result, 30, "MSK %db", bitLength);
    return;
  }
  
  strcpy(result, "Unknown");
}
WebServer server(80);

// HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Multi-Sniffer</title>
  <style>
    :root {
      --bg: #f4f4f9;
      --card-bg: #fff;
      --text: #333;
      --text-secondary: #555;
      --border: #eee;
      --input-bg: #fff;
      --input-border: #ddd;
      --table-hover: #f1f1f1;
      --shadow: rgba(0,0,0,0.1);
    }
    body.dark {
      --bg: #1a1a2e;
      --card-bg: #16213e;
      --text: #e0e0e0;
      --text-secondary: #a0a0b0;
      --border: #2a2a4a;
      --input-bg: #0f3460;
      --input-border: #2a2a4a;
      --table-hover: #1a1a3e;
      --shadow: rgba(0,0,0,0.3);
    }
    
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; text-align: center; margin:0; padding:10px; background: var(--bg); color: var(--text); transition: background 0.3s, color 0.3s; }
    h1 { color: var(--text); font-size: 1.5rem; margin-bottom: 15px; }
    h3 { color: var(--text); }
    
    .theme-toggle {
      position: fixed;
      top: 10px;
      right: 10px;
      background: var(--card-bg);
      border: 1px solid var(--border);
      border-radius: 50%;
      width: 40px;
      height: 40px;
      font-size: 1.2rem;
      cursor: pointer;
      z-index: 100;
      color: var(--text);
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 0;
    }

    .settings { 
      background: var(--card-bg); 
      padding: 15px; 
      border-radius: 12px; 
      box-shadow: 0 2px 5px var(--shadow); 
      margin-bottom: 20px; 
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    
    .control-group {
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid var(--border);
      padding-bottom: 8px;
    }
    .control-group:last-child { border-bottom: none; }
    
    label { font-weight: bold; color: var(--text-secondary); }
    select { padding: 10px; border-radius: 8px; border: 1px solid var(--input-border); background: var(--input-bg); color: var(--text); font-size: 1rem; width: 60%; }
    input[type="range"] { accent-color: #007bff; }
    
    button { 
      padding: 12px 20px; 
      background: #007bff; 
      color: white; 
      border: none; 
      border-radius: 8px; 
      cursor: pointer; 
      font-size: 1rem; 
      font-weight: bold;
      width: 100%;
      margin-top: 10px;
    }
    button:active { background: #0056b3; transform: scale(0.98); }

    .table-container {
      overflow-x: auto;
      background: var(--card-bg);
      border-radius: 12px;
      box-shadow: 0 2px 5px var(--shadow);
    }

    table { width: 100%; border-collapse: collapse; min-width: 600px; }
    th, td { padding: 12px 8px; border-bottom: 1px solid var(--border); text-align: left; font-size: 0.9rem; color: var(--text); }
    th { background-color: #007bff; color: white; white-space: nowrap; }
    tr:hover { background-color: var(--table-hover); }
    
    @media (max-width: 600px) {
      .settings { padding: 10px; }
      select { width: 55%; }
      th, td { padding: 10px 5px; font-size: 0.85rem; }
    }

    .r1 { border-left: 5px solid #ff4d4d; }
    .r2 { border-left: 5px solid #4dff4d; }
    .r3 { border-left: 5px solid #4d4dff; }
  </style>
  <script>
    setInterval(function() {
      fetch('/data').then(response => response.json()).then(data => {
        let table = document.getElementById("logTable");
        table.innerHTML = "";
        data.forEach(row => {
          let tr = document.createElement("tr");
          tr.className = "r" + row.radio;
          let valText = row.raw ? '<i>RAW</i>' : row.value;
          let bitsText = row.raw ? '-' : row.bits;
          let protoText = row.raw ? '-' : row.proto;
          let devText = row.device || '-';
          if (row.raw) tr.style.opacity = '0.7';
          let timeStr;
          if (row.time > 1700000000) {
            let d = new Date(row.time * 1000);
            timeStr = String(d.getHours()).padStart(2,'0') + ':' + String(d.getMinutes()).padStart(2,'0') + ':' + String(d.getSeconds()).padStart(2,'0');
          } else {
            let secs = row.time;
            let mins = Math.floor(secs / 60);
            let hrs = Math.floor(mins / 60);
            timeStr = String(hrs).padStart(2,'0') + ':' + String(mins % 60).padStart(2,'0') + ':' + String(secs % 60).padStart(2,'0');
          }
          tr.innerHTML = `<td>${timeStr}</td><td>R${row.radio}</td><td>${valText}</td><td>${bitsText}</td><td>${protoText}</td><td>${row.rssi}</td><td>${devText}</td>`;
          table.appendChild(tr);
        });
      });
    }, 1000); 

    function updateSettings() {
      let r1 = document.getElementById("r1").value;
      let r2 = document.getElementById("r2").value;
      let r3 = document.getElementById("r3").value;
      fetch(`/update?r1=${r1}&r2=${r2}&r3=${r3}`).then(response => {
         alert("Settings Updated!");
         // Refresh UI to match server state
         fetch('/settings').then(response => response.json()).then(data => {
            document.getElementById("r1").value = data.r1;
            document.getElementById("r2").value = data.r2;
            document.getElementById("r3").value = data.r3;
         });
      });
    }
    
    function clearLog() {
      fetch('/clear').then(response => {
        document.getElementById("logTable").innerHTML = "";
      });
    }
    
    function toggleNtfy() {
      fetch('/ntfy_toggle').then(r => r.json()).then(data => {
        var btn = document.getElementById('ntfyBtn');
        var staText = data.sta ? '' : ' (no WiFi)';
        btn.style.background = data.enabled ? '#28a745' : '#6c757d';
        btn.textContent = data.enabled ? 'ntfy ON' + staText : 'ntfy OFF';
      });
    }

    var rssiLocked = true;
    
    function updateRssiLabel(val) {
      document.getElementById("rssiLabel").textContent = val + " dBm";
    }
    
    function toggleLock() {
      rssiLocked = !rssiLocked;
      document.getElementById("rssiSlider").disabled = rssiLocked;
      document.getElementById("lockBtn").innerHTML = rssiLocked ? '&#128274;' : '&#128275;';
      document.getElementById("lockBtn").style.background = rssiLocked ? '#dc3545' : '#007bff';
      fetch('/rssilock?lock=' + (rssiLocked ? '1' : '0'));
    }
    
    function setRssi() {
      if (rssiLocked) { alert("RSSI is locked!"); return; }
      let val = document.getElementById("rssiSlider").value;
      fetch('/rssi?val=' + val).then(response => {
        alert("RSSI threshold: " + val + " dBm");
      });
    }
    
    function toggleTheme() {
      document.body.classList.toggle('dark');
      let btn = document.getElementById('themeBtn');
      let isDark = document.body.classList.contains('dark');
      btn.textContent = isDark ? '\u2600' : '\u263D';
      localStorage.setItem('theme', isDark ? 'dark' : 'light');
    }

    // Load current settings on page load
    window.onload = function() {
        // Theme from local storage
        if (localStorage.getItem('theme') === 'dark') {
            document.body.classList.add('dark');
            document.getElementById('themeBtn').textContent = '\u2600';
        }
        
        fetch('/settings').then(response => response.json()).then(data => {
            document.getElementById("r1").value = data.r1;
            document.getElementById("r2").value = data.r2;
            document.getElementById("r3").value = data.r3;
            document.getElementById("rssiSlider").value = data.rssi;
            updateRssiLabel(data.rssi);
            // Color radio labels: green = connected, gray = not found
            ['r1','r2','r3'].forEach(function(id, i) {
                var lbl = document.getElementById('lbl_' + id);
                var present = data['p' + (i+1)];
                if (lbl) {
                    lbl.style.color = present ? '#28a745' : '#999';
                }
            });
            // ntfy + STA status from server
            var nbtn = document.getElementById('ntfyBtn');
            var staText = data.sta ? '' : ' (no WiFi)';
            nbtn.style.background = data.ntfy ? '#28a745' : '#6c757d';
            nbtn.textContent = data.ntfy ? 'ntfy ON' + staText : 'ntfy OFF';
        });
    }
  </script>
</head>
<body>
  <button id="themeBtn" class="theme-toggle" onclick="toggleTheme()">&#9789;</button>
  <h1>ESP32 Multi-Sniffer</h1>
  
  <div class="settings">
    <h3>Radio Configuration</h3>
    
    <div class="control-group">
      <label id="lbl_r1">Radio 1:</label>
      <select id="r1">
        <option value="2">ASK/OOK</option>
        <option value="0">2-FSK</option>
        <option value="1">GFSK</option>
        <option value="4">MSK</option>
        <option value="5">Wideband ASK</option>
      </select>
    </div>
    
    <div class="control-group">
      <label id="lbl_r2">Radio 2:</label>
      <select id="r2">
        <option value="2">ASK/OOK</option>
        <option value="0">2-FSK</option>
        <option value="1">GFSK</option>
        <option value="4">MSK</option>
        <option value="5">Wideband ASK</option>
      </select>
    </div>
    
    <div class="control-group">
      <label id="lbl_r3">Radio 3:</label>
      <select id="r3">
        <option value="2">ASK/OOK</option>
        <option value="0">2-FSK</option>
        <option value="1">GFSK</option>
        <option value="4">MSK</option>
        <option value="5">Wideband ASK</option>
      </select>
    </div>
    
    <div style="display:flex; gap:5px; margin-top:5px;">
      <button onclick="updateSettings()" style="flex:1;">Update</button>
      <button onclick="clearLog()" style="flex:1; background:#dc3545;">Clear</button>
      <button id="ntfyBtn" onclick="toggleNtfy()" style="flex:1; background:#6c757d;">ntfy OFF</button>
    </div>
  </div>

  <div class="settings" style="padding:8px; gap:5px;">
    <div style="font-size:0.8rem; font-weight:bold; color:var(--text-secondary);">RAW RSSI Threshold</div>
    <div style="display:flex; align-items:center; gap:6px;">
      <label id="rssiLabel" style="font-size:0.8rem; min-width:55px;">-60 dBm</label>
      <input type="range" id="rssiSlider" min="-100" max="-30" value="-60" disabled style="flex:1; height:20px;" oninput="updateRssiLabel(this.value)">
      <button id="lockBtn" onclick="toggleLock()" style="width:32px; min-width:32px; height:32px; margin:0; padding:4px; font-size:0.9rem; border-radius:6px; background:#dc3545;">&#128274;</button>
      <button onclick="setRssi()" style="width:auto; min-width:50px; margin:0; padding:6px 10px; font-size:0.75rem; border-radius:6px;">Set</button>
    </div>
  </div>

  <div class="table-container">
    <table>
      <thead><tr><th>Time</th><th>Src</th><th>Value</th><th>Bits</th><th>Proto</th><th>RSSI</th><th>Device</th></tr></thead>
      <tbody id="logTable"></tbody>
    </table>
  </div>
</body>
</html>
)rawliteral";


// --- RADIO MANAGEMENT ---

void selectRadio(int csnPin) {
  digitalWrite(CSN1, HIGH);
  digitalWrite(CSN2, HIGH);
  digitalWrite(CSN3, HIGH);
  
  ELECHOUSE_cc1101.setSpiPin(SCK_PIN, MISO_PIN, MOSI_PIN, csnPin);
  
  digitalWrite(csnPin, LOW); 
  delayMicroseconds(10); 
}

int getGDO(int radioId) {
  if (radioId == 1) return GDO1;
  if (radioId == 2) return GDO2;
  return GDO3;
}

void setupRadio(int csnPin, int radioId, int modulation) {
  selectRadio(csnPin);
  
  if (ELECHOUSE_cc1101.getCC1101()) {
    Serial.printf("Radio %d (CSN %d) OK -> Mod: %d\n", radioId, csnPin, modulation);
    radioPresent[radioId] = true;
  } else {
    Serial.printf("Radio %d (CSN %d) ERROR - Not Found\n", radioId, csnPin);
    radioPresent[radioId] = false;
    return;
  }
  
  ELECHOUSE_cc1101.Init();
  
  ELECHOUSE_cc1101.setModulation(modulation); 
  ELECHOUSE_cc1101.setMHZ(433.92);
  
  if (modulation == 0 || modulation == 1) { // FSK / GFSK
      ELECHOUSE_cc1101.setDeviation(47.60);   
      ELECHOUSE_cc1101.setRxBW(812.0);        
      radioModeName[radioId] = (modulation == 0) ? "2-FSK" : "GFSK";
  } 
  else if (modulation == 4) { // MSK
      ELECHOUSE_cc1101.setRxBW(812.0);
      radioModeName[radioId] = "MSK";
  }
  else if (modulation == 5) { // Wideband ASK (for all radios)
      ELECHOUSE_cc1101.setModulation(2); // CC1101 uses 2 = ASK/OOK
      ELECHOUSE_cc1101.setRxBW(812.0);   // Wide bandwidth
      radioModeName[radioId] = "Wideband ASK";
  }
  else { // ASK/OOK (2)
      radioModeName[radioId] = "ASK/OOK";
  }
  
  radioModulation[radioId] = modulation;
  
  ELECHOUSE_cc1101.SetRx();
  digitalWrite(csnPin, HIGH); 
}

void setup() {
  Serial.begin(115200);
  
  // Configure CSN pins as outputs
  pinMode(CSN1, OUTPUT); digitalWrite(CSN1, HIGH);
  pinMode(CSN2, OUTPUT); digitalWrite(CSN2, HIGH);
  pinMode(CSN3, OUTPUT); digitalWrite(CSN3, HIGH);

  Serial.println("Initializing Radios...");
  
  setupRadio(CSN1, 1, 2); // ASK
  setupRadio(CSN2, 2, 0); // FSK
  setupRadio(CSN3, 3, 2); // ASK (Wideband logic handled in setupRadio)

  pinMode(GDO1, INPUT);
  pinMode(GDO2, INPUT);
  pinMode(GDO3, INPUT);
  
  mySwitch.enableReceive(digitalPinToInterrupt(GDO1)); 

  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(100);
  
  // AP+STA hybrid mode: AP for web UI, STA for ntfy notifications
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);
  Serial.println("AP Created: " + String(ap_ssid));
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  
  // Connect to home network (non-blocking)
  WiFi.begin(sta_ssid, sta_password);
  Serial.printf("Connecting to %s...\n", sta_ssid);

  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });
  
  server.on("/update", []() {
    if (server.hasArg("r1")) {
       int m = server.arg("r1").toInt();
       if (radioPresent[1]) setupRadio(CSN1, 1, m);
    }
    if (server.hasArg("r2")) {
       int m = server.arg("r2").toInt();
       if (radioPresent[2]) setupRadio(CSN2, 2, m);
    }
    if (server.hasArg("r3")) {
       int m = server.arg("r3").toInt();
       if (radioPresent[3]) setupRadio(CSN3, 3, m);
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/rssilock", []() {
      if (server.hasArg("lock")) {
          rssiLocked = server.arg("lock").toInt() == 1;
          Serial.printf("RSSI lock: %s\n", rssiLocked ? "ON" : "OFF");
      }
      server.send(200, "text/plain", "OK");
  });
  
  server.on("/rssi", []() {
      if (rssiLocked) {
          server.send(403, "text/plain", "LOCKED");
          return;
      }
      if (server.hasArg("val")) {
          rssiThreshold = server.arg("val").toInt();
          Serial.printf("RSSI threshold: %d dBm\n", rssiThreshold);
      }
      server.send(200, "text/plain", "OK");
  });
  
  server.on("/clear", []() {
      memset(history, 0, sizeof(history));
      historyIndex = 0;
      totalSignals = 0;
      server.send(200, "text/plain", "OK");
      Serial.println("Log cleared");
  });
  
  // Returns current settings as JSON
  server.on("/settings", []() {
      String json = "{";
      json += "\"r1\":" + String(radioModulation[1]) + ",";
      json += "\"r2\":" + String(radioModulation[2]) + ",";
      json += "\"r3\":" + String(radioModulation[3]) + ",";
      json += "\"rssi\":" + String(rssiThreshold) + ",";
      json += "\"p1\":" + String(radioPresent[1] ? "true" : "false") + ",";
      json += "\"p2\":" + String(radioPresent[2] ? "true" : "false") + ",";
      json += "\"p3\":" + String(radioPresent[3] ? "true" : "false") + ",";
      json += "\"ntfy\":" + String(ntfyEnabled ? "true" : "false") + ",";
      json += "\"sta\":" + String(staConnected ? "true" : "false");
      json += "}";
      server.send(200, "application/json", json);
  });
  
  server.on("/ntfy_toggle", []() {
      ntfyEnabled = !ntfyEnabled;
      String json = "{\"enabled\":" + String(ntfyEnabled ? "true" : "false") + ",\"sta\":" + String(staConnected ? "true" : "false") + "}";
      server.send(200, "application/json", json);
      Serial.printf("ntfy: %s (STA: %s)\n", ntfyEnabled ? "ON" : "OFF", staConnected ? "connected" : "disconnected");
  });
  
  server.on("/data", []() {
    String json = "[";
    int count = 0;
    int idx = historyIndex - 1;
    if (idx < 0) idx = HISTORY_SIZE - 1;
    
    for (int i = 0; i < HISTORY_SIZE; i++) {
      if (history[idx].timestamp == 0) break; 
      
      if (count > 0) json += ",";
      json += "{";
      json += "\"time\":" + String(history[idx].timestamp);
      json += ",\"radio\":" + String(history[idx].radioId);
      json += ",\"value\":" + String(history[idx].value);
      json += ",\"bits\":" + String(history[idx].bitLength);
      json += ",\"proto\":" + String(history[idx].protocol);
      json += ",\"rssi\":" + String(history[idx].rssi);
      json += ",\"raw\":" + String(history[idx].rawDetection ? "true" : "false");
      json += ",\"device\":\"" + String(history[idx].deviceType) + "\"";
      json += "}";
      
      idx--;
      if (idx < 0) idx = HISTORY_SIZE - 1;
      count++;
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();
  
  static int currentRadio = 1;
  static unsigned long lastSwitch = 0;
  
  // Find next available radio
  bool anyRadioFound = false;
  for(int i=1; i<=3; i++) {
      if(radioPresent[i]) {
          anyRadioFound = true;
          break;
      }
  }
  
  if (!anyRadioFound) {
      // If no radios found, do nothing
      delay(100);
      return;
  }

  // Count how many radios are connected
  int radioCount = 0;
  for (int i = 1; i <= 3; i++) if (radioPresent[i]) radioCount++;

  // Switch radio ONLY if more than 1 is connected
  if (radioCount > 1 && millis() - lastSwitch > 500) {
      int nextRadio = currentRadio;
      do {
          nextRadio++;
          if (nextRadio > 3) nextRadio = 1;
      } while (!radioPresent[nextRadio] && nextRadio != currentRadio);
      
      if (nextRadio != currentRadio) {
          currentRadio = nextRadio;
          if (currentRadio == 1) {
              mySwitch.enableReceive(digitalPinToInterrupt(GDO1));
              selectRadio(CSN1);
          } else if (currentRadio == 2) {
              mySwitch.enableReceive(digitalPinToInterrupt(GDO2));
              selectRadio(CSN2);
          } else if (currentRadio == 3) {
              mySwitch.enableReceive(digitalPinToInterrupt(GDO3));
              selectRadio(CSN3);
          }
      }
      lastSwitch = millis();
  }

  // Check RCSwitch EVERY loop iteration (not just every 50ms)
  static int rawCooldown = 0;
  static int rawConfirmCount = 0;
  
  if (mySwitch.available()) {
      long value = mySwitch.getReceivedValue();
      if (value != 0) {
          int rssi = ELECHOUSE_cc1101.getRssi();
          int activeRadio = currentRadio;
          int bits = mySwitch.getReceivedBitlength();
          int proto = mySwitch.getReceivedProtocol();
          
          if (!isLogDuplicate(activeRadio, value)) {
            history[historyIndex].timestamp = getTimestamp();
            history[historyIndex].radioId = activeRadio;
            history[historyIndex].value = value;
            history[historyIndex].bitLength = bits;
            history[historyIndex].protocol = proto;
            history[historyIndex].rssi = rssi;
            history[historyIndex].rawDetection = false;
            identifyDevice(proto, bits, value, radioModulation[activeRadio], history[historyIndex].deviceType);
            historyIndex = (historyIndex + 1) % HISTORY_SIZE;
            Serial.printf("Radio %d DECODED: %ld (%s)\n", activeRadio, value, history[(historyIndex-1+HISTORY_SIZE)%HISTORY_SIZE].deviceType);
          }
          
          char ntTitle[60], ntBody[120];
          snprintf(ntTitle, sizeof(ntTitle), "R%d DECODED", activeRadio);
          snprintf(ntBody, sizeof(ntBody), "Value: %ld | Bits: %d | Proto: %d | RSSI: %d dBm", value, bits, proto, rssi);
          queueNtfy(activeRadio, value, ntTitle, ntBody);
      }
      mySwitch.resetAvailable();
      rawCooldown = 10;
      rawConfirmCount = 0;
  }

  // RAW signal: requires 3 consecutive strong readings (filters noise)
  static unsigned long lastRawCheck = 0;
  if (millis() - lastRawCheck > 50) {
      int rssi = ELECHOUSE_cc1101.getRssi();
      lastRawCheck = millis();
      
      if (rssi > rssiThreshold && rawCooldown == 0) {
          rawConfirmCount++;
          if (rawConfirmCount >= 3) {
              int activeRadio = currentRadio;
              
              long rawKey = (long)(rssi / 5) * 5; // Round to 5dBm for device differentiation
              if (!isLogDuplicate(activeRadio, rawKey)) {
                history[historyIndex].timestamp = getTimestamp();
                history[historyIndex].radioId = activeRadio;
                history[historyIndex].value = 0;
                history[historyIndex].bitLength = 0;
                history[historyIndex].protocol = 0;
                history[historyIndex].rssi = rssi;
                history[historyIndex].rawDetection = true;
                strcpy(history[historyIndex].deviceType, "RAW/Rolling");
                historyIndex = (historyIndex + 1) % HISTORY_SIZE;
                Serial.printf("Radio %d RAW: RSSI %d dBm\n", activeRadio, rssi);
              }
              
              char ntTitle[60], ntBody[120];
              snprintf(ntTitle, sizeof(ntTitle), "R%d RAW Signal", activeRadio);
              snprintf(ntBody, sizeof(ntBody), "RSSI: %d dBm | RAW/Rolling code", rssi);
              queueNtfy(activeRadio, rawKey, ntTitle, ntBody);
              rawCooldown = 20;
              rawConfirmCount = 0;
          }
      } else {
          rawConfirmCount = 0;
      }
      if (rawCooldown > 0) rawCooldown--;
  }
  
  // STA connection status monitoring
  static unsigned long lastStaCheck = 0;
  if (millis() - lastStaCheck > 5000) {
    lastStaCheck = millis();
    bool wasConnected = staConnected;
    staConnected = (WiFi.status() == WL_CONNECTED);
    if (staConnected && !wasConnected) {
      Serial.printf("STA connected! IP: %s\n", WiFi.localIP().toString().c_str());
      // NTP sync (Finland time UTC+2 / daylight saving UTC+3)
      configTime(2 * 3600, 3600, "pool.ntp.org", "time.google.com");
      Serial.println("NTP sync started...");
    }
    // Check NTP sync status
    if (staConnected && !ntpSynced) {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 0)) {
        ntpSynced = true;
        Serial.printf("NTP synced: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      }
    }
    if (!staConnected && wasConnected) {
      Serial.println("STA disconnected, reconnecting...");
      WiFi.begin(sta_ssid, sta_password);
    }
  }
  
  // Process ntfy queue (1 message per loop iteration, non-blocking)
  static unsigned long lastNtfy = 0;
  if (millis() - lastNtfy > 500) {
    lastNtfy = millis();
    processNtfyQueue();
  }
  
  // Poll ntfy commands (every 10 seconds)
  static unsigned long lastCmdPoll = 0;
  if (millis() - lastCmdPoll > 10000) {
    lastCmdPoll = millis();
    pollNtfyCommands();
  }
  
  delay(1);
}
