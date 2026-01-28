# K-MD7 COM Port Test & GUI

This repository contains tools for interfacing with the K-MD7 radar sensor, including a command-line tester and a real-time web dashboard.

## 🚀 Quick Start (GUI Dashboard)

Copy and paste this into PowerShell to start the real-time visualization:

```powershell
.\venv\Scripts\Activate.ps1; python app.py
```

Then open your browser to: [http://127.0.0.1:5000](http://127.0.0.1:5000)

## 📡 CLI Scanner

If you just want to see the closest target in your terminal:

```powershell
.\venv\Scripts\Activate.ps1; python closest_target.py --port COM4
```

## 🛠 Setup

If you haven't installed dependencies yet (PowerShell):

```powershell
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Examples:

Send a raw hex frame:

```powershell
python serial_test.py --port COM3 --baud 115200 --hex "AA0102FF"
```

Build a frame from parts (start + addr + cmd + payload, XOR checksum):

```powershell
python serial_test.py --port COM3 --baud 115200 --build --start "AA" --addr "01" --cmd "02" --payload "FF"
```

Notes:
- The script is intentionally generic: fill `--start`, `--addr`, `--cmd`, `--payload` and choose checksum type.
- Replace `--checksum xor` with `--checksum none` if your protocol uses no trailing checksum.
- For exact K-MD7 command bytes, CRC algorithm, and framing details, refer to the device datasheet and update the `build_frame` and checksum logic accordingly.
