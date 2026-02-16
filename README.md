# ESP32 Multi-Sniffer - 433MHz RF Signal Monitor

(C) Jasu-tech | MIT License

A multi-radio 433MHz RF signal monitoring system using ESP32 and CC1101 radio modules. Features a mobile-optimized web interface for real-time signal viewing, device identification, and remote control via ntfy push notifications.

## Features

- **Multi-radio support**: Up to 3 CC1101 radio modules with independent modulation settings
- **Multiple modulation types**: ASK/OOK, 2-FSK, GFSK, MSK, Wideband ASK
- **Real-time web UI**: Mobile-optimized dark/light theme interface with live signal log
- **Device identification**: Automatic recognition of PT2262, EV1527, Nexa/HomeEasy, Came TOP, Nice FLO, and more
- **RAW signal detection**: Captures rolling code and undecodable signals via RSSI threshold
- **Duplicate filtering**: 30-second cooldown prevents log spam from repeated transmissions
- **ntfy push notifications**: Get alerts on your phone when signals are detected
- **Remote control via ntfy**: Send commands to ESP32 from anywhere using ntfy app
- **NTP time sync**: Accurate timestamps when connected to home WiFi
- **AP+STA hybrid WiFi**: Access Point for web UI + Station for internet connectivity

## Hardware Requirements

- **ESP32 DevKit V1** (dual-core) or **ESP32-S3 Zero**
- **CC1101** 433MHz radio module(s) (1-3 modules supported)
- Jumper wires

### Pin Configuration

#### ESP32 Standard (DevKit V1)

| Function | GPIO |
|----------|------|
| SPI SCK  | 18   |
| SPI MISO | 19   |
| SPI MOSI | 23   |
| Radio 1 CSN | 5 |
| Radio 1 GDO | 17 |
| Radio 2 CSN | 4 |
| Radio 2 GDO | 15 |
| Radio 3 CSN | 14 |
| Radio 3 GDO | 27 |

#### ESP32-S3 Zero

| Function | GPIO |
|----------|------|
| SPI SCK  | 12   |
| SPI MISO | 13   |
| SPI MOSI | 11   |
| Radio 1 CSN | 10 |
| Radio 1 GDO | 9  |
| Radio 2 CSN | 14 |
| Radio 2 GDO | 15 |
| Radio 3 CSN | 16 |
| Radio 3 GDO | 17 |

## Setup

### 1. Install Arduino Libraries

- [ELECHOUSE CC1101](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib)
- [RCSwitch](https://github.com/sui77/rc-switch)

### 2. Configure WiFi

Edit these lines at the top of `esp32_multi_sniffer_3.ino`:

```cpp
const char* sta_ssid = "YOUR_WIFI_NAME";       // <- Your home WiFi name
const char* sta_password = "YOUR_WIFI_PASSWORD"; // <- Your home WiFi password
```

### 3. Select Board

Uncomment the correct board definition:

```cpp
#define BOARD_ESP32_STANDARD  // Standard ESP32 (DevKit V1, dual-core)
// #define BOARD_ESP32_S3_ZERO   // ESP32-S3 Zero (dual-core)
```

### 4. Upload

Upload to your ESP32 using Arduino IDE with the appropriate board selected.

## Usage

### Web Interface

1. Connect to the **"ESP32-Multi-Sniffer"** WiFi network (password: `ESP3admin`). You can change the AP name and password at the top of the sketch (`ap_ssid` and `ap_password`).
2. Open `http://192.168.4.1` in your browser
3. Or access via your home network IP (shown in Serial Monitor)

### Web UI Features

- **Radio Configuration**: Change modulation type for each radio
- **Signal Log**: Real-time table showing decoded and RAW signals
- **RSSI Threshold**: Adjustable sensitivity for RAW signal detection (locked by default at -60 dBm)
- **Dark/Light Theme**: Toggle with the moon/sun button
- **ntfy Toggle**: Enable/disable push notifications
- **Clear Log**: Remove all logged signals

### ntfy Remote Control

Send commands to topic `ESP32-MultiSniffer-CMD` using the [ntfy app](https://ntfy.sh):

| Command | Description |
|---------|-------------|
| `status` | Returns radio status, RSSI threshold, signal count |
| `clear` | Clears the signal log |
| `ntfy on` / `ntfy off` | Enable/disable signal notifications |
| `rssi -60` | Set RSSI threshold (e.g. -60 dBm) |
| `mod 1 2` | Change radio modulation (radio number, mod type) |
| `lock` / `unlock` | Lock/unlock RSSI threshold adjustment |
| `scan` | Show all radios and their current modulation modes |
| `threshold` | Show current RSSI threshold and lock status |
| `signals` | Show signal count and last captured signal |
| `info` | Device info: uptime, free RAM, IP addresses, NTP status |
| `reboot` | Restart the ESP32 remotely |
| `help` | List available commands |

**Modulation types**: 0=2-FSK, 1=GFSK, 2=ASK/OOK, 4=MSK, 5=Wideband ASK

**ntfy Topics**:
- `ESP32-MultiSniffer` - All communication (commands and responses) on the same topic

## Supported Devices

The system can identify signals from:

- PT2262 / EV1527 (wireless outlets, doorbells) - 24-bit ASK
- Nexa / HomeEasy - 32-bit ASK
- PT2260 - 12-bit ASK
- Came TOP - Protocol 2, 12-bit
- Nice FLO - Protocol 3, 12-bit
- V2 Phoenix - Protocol 4
- Intertechno - Protocol 5
- FHT80 - Protocol 6
- Various sensors (28-bit, 36-bit)
- RAW/Rolling code signals (detected via RSSI)

## Architecture

- **WiFi Mode**: AP+STA hybrid - Access Point serves the web UI while Station connects to home WiFi for internet access (ntfy, NTP)
- **Signal Processing**: RCSwitch handles decoded signals; RSSI threshold detects RAW/rolling code signals
- **Duplicate Filtering**: Both log entries and ntfy notifications use a 30-second cooldown per unique signal (radio + value combination)
- **RAW Differentiation**: RAW signals are differentiated by RSSI level (rounded to 5 dBm) since they lack unique identifiers
- **NTP Sync**: Automatic time synchronization via pool.ntp.org when connected to home WiFi (Finland timezone UTC+2/+3)
- **Command Polling**: ESP32 checks for ntfy commands every 10 seconds via HTTP JSON polling

## Important Notes

- **GPIO 2 must NOT be used for GDO** - it's a strapping pin that causes interrupt issues
- **ESP32-C3 is not supported** - single-core architecture conflicts with WiFi + radio interrupts
- **Only dual-core ESP32 variants are supported** (standard ESP32, ESP32-S3)
- RSSI threshold is locked by default at -60 dBm to prevent accidental changes

## License

MIT License - (C) Jasu-tech

See the license header in `esp32_multi_sniffer_3.ino` for the full license text.
