Below is a complete manual you can drop directly into a repository or hand to firmware developers.

---

# **K‑MD7 Radar Sensor – Binary Protocol Implementation Manual**

## **1. Introduction**

The K‑MD7 radar sensor communicates over a UART‑based binary protocol.  
This manual describes how to implement the protocol on a host system (MCU, SBC, or PC), including:

- Packet structure  
- Connection procedure  
- Commands  
- Messages  
- Parameter structure  
- Data formats (RADC, RFFT, PDAT, TDAT)  
- Bootloader protocol  

This document is intended for firmware and embedded developers integrating the K‑MD7 into their systems.

---

# **2. Physical Layer**

## **2.1 UART Settings**

| Parameter | Value |
|----------|--------|
| Baud rate | 115200 (default) |
| Data bits | 8 |
| Parity | Even |
| Stop bits | 1 |
| Flow control | None |

The baud rate may be changed after connection using the `INIT` command.

---

# **3. Packet Format**

Every command (host → radar) and message (radar → host) uses the same binary structure:

```
+------------------+------------------+------------------+
| Header (4 ASCII) | Payload Length   | Payload (binary) |
|                  | UINT32 LE        | (0..N bytes)     |
+------------------+------------------+------------------+
```

### **3.1 Header**
- 4 ASCII characters (e.g., `INIT`, `GNFD`, `TDAT`)

### **3.2 Payload Length**
- 32‑bit unsigned integer  
- Little‑endian  
- Always present, even if zero

### **3.3 Payload**
- Binary data  
- Multi‑byte values are little‑endian  

---

# **4. Connection Procedure**

The host must establish a session before any other commands.

## **4.1 Connection Sequence**

1. **Sensor starts at 115200 baud**
2. Host sends:

```
INIT <baudrate_code>
```

3. Sensor replies:
   - `RESP` (acknowledge)
   - `VERS` (firmware string)
4. Sensor switches to the new baud rate

### **4.2 Baud Rate Codes**

| Code | Baud Rate |
|------|-----------|
| 0 | 115200 |
| 1 | 460800 |
| 2 | 921600 |
| 3 | 2000000 |
| 4 | 3000000 |

### **4.3 Disconnecting**

Send:

```
GBYE
```

Sensor returns to default baud rate.

---

# **5. Command Set (Application Mode)**

Below is a concise implementation‑ready list.

## **5.1 Overview Table**

| Command | Payload | Description |
|---------|---------|-------------|
| `INIT` | 1 byte | Start connection, set baud rate |
| `GNFD` | 1 byte | Request next frame (handshake mode) |
| `RDOT` | 1 byte | Enable/disable streaming mode |
| `GRPS` | 0 | Read parameter structure |
| `SRPS` | 31 bytes | Write parameter structure |
| `RFSE` | 0 | Restore factory settings |
| `GBYE` | 0 | Disconnect |
| `RBFR` | 1 byte | Set frequency channel |
| `RSPI` | 1 byte | Set speed range |
| `RRAI` | 1 byte | Set distance range |
| `THOF` | 1 byte | Set threshold offset |
| `TRFT` | 1 byte | Set tracking filter |
| `MIRA` | 1 byte | Min detection zone distance |
| `MARA` | 1 byte | Max detection zone distance |
| `MIAN` | 1 byte | Min detection zone angle |
| `MAAN` | 1 byte | Max detection zone angle |
| `MISP` | 1 byte | Min detection speed filter |
| `MASP` | 1 byte | Max detection speed filter |
| `DEDI` | 1 byte | Direction filter |
| `JBTL` | 0 | Jump to bootloader |

---

# **6. Message Set (Application Mode)**

| Message | Payload | Description |
|---------|----------|-------------|
| `RESP` | 1 byte | Acknowledgement + error code |
| `VERS` | 19 bytes | Firmware string |
| `RADC` | 6144 bytes | Raw ADC data |
| `RFFT` | 2048 bytes | FFT + threshold |
| `PDAT` | 0–192 bytes | Raw targets (max 24) |
| `TDAT` | 0–72 bytes | Tracked targets (max 8) |
| `DONE` | 4 bytes | Frame number |
| `RPST` | 31 bytes | Parameter structure |

---

# **7. Error Codes**

| Code | Meaning |
|------|---------|
| 0 | OK |
| 1 | Unknown command |
| 2 | Invalid parameter |
| 3 | Invalid RPST version |
| 4 | UART error |
| 5 | Missing calibration |
| 6 | Timeout |
| 7 | Application corrupt / not programmed |

---

# **8. Parameter Structure**

The structure is always 31 bytes.

| Field | Type | Description |
|-------|------|-------------|
| Firmware string | STRING(19) | Zero‑terminated |
| Frequency channel | UINT8 | 0=Low,1=Mid,2=High |
| Speed setting | UINT8 | 0=50,1=100,2=200 km/h |
| Range setting | UINT8 | 0=100,1=200,2=300 m |
| Threshold offset | UINT8 | 0–60 dB |
| Tracking filter | UINT8 | 0=Std,1=Fast,2=Long |
| Min zone distance | UINT8 | % of range |
| Max zone distance | UINT8 | % of range |
| Min zone angle | INT8 | -30..+30° |
| Max zone angle | INT8 | -30..+30° |
| Min speed filter | UINT8 | % of speed |
| Max speed filter | UINT8 | % of speed |
| Direction filter | UINT8 | 0=Rec,1=App,2=Both |

---

# **9. Data Formats**

## **9.1 RADC (Raw ADC)**

Three blocks of 2048 bytes each:

| Block | Content |
|--------|----------|
| IF1 A | 512 I + 512 Q |
| IF2 A | 512 I + 512 Q |
| IF1 B | 512 I + 512 Q |

All values: `UINT16 LE`

---

## **9.2 RFFT (Raw FFT)**

| Section | Type | Count |
|---------|-------|--------|
| Spectrum | UINT16 | 512 |
| Threshold | UINT16 | 512 |

Values are scaled: **dB × 100**

---

## **9.3 PDAT (Raw Targets)**

Up to 24 targets, each 8 bytes:

| Field | Type | Unit |
|--------|--------|--------|
| Distance | UINT16 | cm |
| Speed | INT16 | km/h × 100 |
| Angle | INT16 | deg × 100 |
| Magnitude | UINT16 | dB × 100 |

---

## **9.4 TDAT (Tracked Targets)**

Up to 8 targets, each 9 bytes:

| Field | Type | Unit |
|--------|--------|--------|
| Distance | UINT16 | cm |
| Speed | INT16 | km/h × 100 |
| Angle | INT16 | deg × 100 |
| Magnitude | UINT16 | dB × 100 |
| Track ID | UINT8 | 0–255 |

---

# **10. Data Acquisition Modes**

## **10.1 Handshake Mode**

Host repeatedly sends:

```
GNFD <bitmask>
```

Bitmask enables which messages to fetch.

## **10.2 Streaming Mode**

Host sends:

```
RDOT <bitmask>
```

Sensor streams enabled messages every frame.

---

# **11. Bootloader Protocol**

Used for firmware updates.

## **11.1 Entering Bootloader**

Send:

```
JBTL
```

Sensor resets into bootloader.

## **11.2 Bootloader Commands**

| Command | Description |
|---------|-------------|
| `INIT` | Set baud rate |
| `WMEM` | Write flash page |
| `GBYE` | Exit bootloader |

### **WMEM Packet Format**

```
+-------------------------------+
| Address (4 bytes LE)         |
| Data length (4 bytes LE)     |
| Data (1–2048 bytes)          |
+-------------------------------+
```

Address must be:
- Multiple of `0x800`
- Max `0x06E000`

---

# **12. Implementation Tips**

- Always check `RESP` after every command  
- Use highest baud rate for RADC/RFFT  
- Validate frame numbers via `DONE`  
- Use streaming mode only if host can keep up  
- Use parameter structure (`GRPS`/`SRPS`) for bulk configuration  

---

If you want, I can also generate:

- A C/C++ header file for the protocol  
- A Python library  
- A state machine diagram  
- A full example implementation for STM32 or ESP32  

Just tell me what direction you want to take next.