import os
import threading
import time
import struct
import serial
import math
import csv
from datetime import datetime
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from kalman_filter import Tracker

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global state
ser = None
PORT = "COM4"
BAUD = 115200

# CSV Logging Setup
app_start_time = time.time()
logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
os.makedirs(logs_dir, exist_ok=True)
log_filename = os.path.join(logs_dir, f"radar_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
csv_header = ['timestamp_sec', 'epoch', 'distance_cm', 'angle_deg', 'velocity_m_s', 'magnitude_db']
epoch_counter = 0

def log_targets_to_csv(targets, epoch):
    try:
        with open(log_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            timestamp = time.time() - app_start_time
            for t in targets:
                writer.writerow([
                    f"{timestamp:.4f}",
                    epoch,
                    t['dist'],
                    t['angle'],
                    t['speed'],
                    t['mag']
                ])
    except Exception as e:
        print(f"CSV Logging Error: {e}")

# Initialize CSV file with header
with open(log_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(csv_header)

# Kalman Tracker
tracker = Tracker(max_age=10, min_hits=3, distance_threshold=200) # threshold in cm (2m)
last_track_time = time.time()
last_pdat_time = time.time()

def emit_kdat(tracked):
    kdat = []
    for trk in tracked:
        # Convert back to polar for visualization compatibility
        tdist = math.sqrt(trk['x']**2 + trk['y']**2)
        tangle = math.degrees(math.atan2(trk['x'], trk['y']))
        kdat.append({
            'dist': tdist,
            'angle': tangle,
            'id': trk['id'],
            'speed': math.sqrt(trk['vx']**2 + trk['vy']**2),
            'mag': trk.get('mag', 70.0) # Use the real filtered magnitude
        })
    socketio.emit('targets', {'type': 'KDAT', 'data': kdat})

def kalman_animator():
    """Background thread to keep Kalman tracks moving between radar updates."""
    global last_track_time, last_pdat_time
    while True:
        time.sleep(0.05)
        now = time.time()
        # If no PDAT for > 60ms, step the filter forward anyway
        if now - last_pdat_time > 0.06:
            dt = now - last_track_time
            if dt > 0.01:
                last_track_time = now
                tracked = tracker.update([], dt)
                emit_kdat(tracked)

# Start animator thread
threading.Thread(target=kalman_animator, daemon=True).start()

# Cache the last parameter structure (31 bytes)
current_rpst = bytearray([0]*31)

def build_kmd7_frame(cmd: str, payload: bytes = b'') -> bytes:
    header = cmd.encode('ascii')[:4].ljust(4, b' ')
    length = struct.pack('<I', len(payload))
    return header + length + payload

def parse_frames(buf: bytearray):
    i = 0
    while i + 8 <= len(buf):
        hdr_bytes = buf[i:i+4]
        if all(32 <= b <= 126 for b in hdr_bytes):
            payload_len = struct.unpack('<I', buf[i+4:i+8])[0]
            if payload_len > 200000:
                i += 1
                continue
            if i + 8 + payload_len <= len(buf):
                header = hdr_bytes.decode('ascii', errors='replace')
                payload = bytes(buf[i+8 : i+8+payload_len])
                del buf[:i + 8 + payload_len]
                return header, payload
            else:
                break
        i += 1
    return None, None

def serial_reader():
    global ser
    print(f"Connecting to K-MD7 on {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, parity=serial.PARITY_EVEN, timeout=0.1)
        # Init
        ser.write(build_kmd7_frame("INIT", b"\x00"))
        time.sleep(0.1)
        # Request initial config state
        ser.write(build_kmd7_frame("GRPS"))
        time.sleep(0.1)
        # Enable targets by default
        ser.write(build_kmd7_frame("RDOT", b"\x0C"))
    except Exception as e:
        print(f"Serial Error: {e}")
        return

    buf = bytearray()
    while True:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buf.extend(data)
            
            while True:
                header, payload = parse_frames(buf)
                if not header:
                    break
                
                if header in ("PDAT", "TDAT"):
                    entry_size = 8 if header == "PDAT" else 9
                    count = len(payload) // entry_size
                    targets = []
                    for j in range(count):
                        off = j * entry_size
                        # dist(u16), speed(i16), angle(i16), mag(u16)
                        d, s, a, m = struct.unpack_from('<HhhH', payload, off)
                        
                        target_id = j
                        if header == "TDAT" and len(payload) >= off + 9:
                            target_id = struct.unpack_from('B', payload, off + 8)[0]

                        targets.append({
                            'dist': d,
                            'speed': s / 100.0,
                            'angle': a / 100.0,
                            'mag': m / 100.0,
                            'id': target_id
                        })
                    
                    if header == "PDAT":
                        global epoch_counter
                        epoch_counter += 1
                        log_targets_to_csv(targets, epoch_counter)

                    socketio.emit('targets', {'type': header, 'data': targets})

                    # Kalman tracking for PDAT
                    if header == "PDAT":
                        global last_track_time, last_pdat_time
                        now = time.time()
                        dt = now - last_track_time
                        last_track_time = now
                        last_pdat_time = now

                        measurements = []
                        for t in targets:
                            dist = t['dist']
                            if dist > 3000: # Filter targets > 30 meters
                                continue
                                
                            # Measurements are now passed as (range, angle, magnitude, speed)
                            # for polar EKF processing
                            measurements.append((float(dist), float(t['angle']), float(t['mag']), float(t['speed'] * 100.0)))
                        
                        tracked = tracker.update(measurements, dt)
                        emit_kdat(tracked)
                
                elif header == "RFFT":
                    # Spectrum(512*u16) + Threshold(512*u16)
                    spectrum = []
                    for k in range(512):
                        val = struct.unpack_from('<H', payload, k*2)[0]
                        spectrum.append(val / 100.0)
                    socketio.emit('fft', {'data': spectrum})
                
                elif header == "RPST":
                    if len(payload) >= 31:
                        global current_rpst
                        current_rpst = bytearray(payload[:31])
                        # Parse out for the frontend
                        socketio.emit('rpst', {
                            'fw': payload[:19].split(b'\x00', 1)[0].decode('ascii', errors='replace'),
                            'freq': payload[19],
                            'speed': payload[20],
                            'range': payload[21],
                            'thresh': payload[22],
                            'tracking': payload[23],
                            'min_dist': payload[24],
                            'max_dist': payload[25],
                            'min_ang': struct.unpack('b', payload[26:27])[0],
                            'max_ang': struct.unpack('b', payload[27:28])[0],
                            'min_speed': payload[28],
                            'max_speed': payload[29],
                            'dir': payload[30]
                        })
                
                elif header == "RESP":
                    socketio.emit('info', {'msg': f"Sensor Response: {payload[0] if payload else '?'}"})

        except Exception as e:
            print(f"Reader Loop Error: {e}")
            break
        time.sleep(0.01)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('update_setting')
def handle_update_setting(json):
    global ser, current_rpst
    field = json.get('field')
    value = int(json.get('value'))
    
    # Map field to RPST byte offset
    offsets = {
        'freq': 19, 'speed': 20, 'range': 21, 'thresh': 22,
        'tracking': 23, 'min_dist': 24, 'max_dist': 25,
        'min_ang': 26, 'max_ang': 27, 'min_speed': 28,
        'max_speed': 29, 'dir': 30
    }
    
    if field in offsets:
        idx = offsets[field]
        # Handle signed bytes for angles if needed, but struct pack 'b' or simple byte assignment works for -30..30
        if field in ('min_ang', 'max_ang'):
            current_rpst[idx] = struct.pack('b', value)[0]
        else:
            current_rpst[idx] = value & 0xFF
        
        if ser and ser.is_open:
            # Send updated structure
            ser.write(build_kmd7_frame("SRPS", bytes(current_rpst)))
            print(f"Updated setting {field} to {value}, sent SRPS")
            # Also request fresh data to confirm
            time.sleep(0.05)
            ser.write(build_kmd7_frame("GRPS"))


@socketio.on('set_streams')
def handle_set_streams(json):
    """Receive a numeric 1-byte bitmask (or dict) from the GUI and send RDOT accordingly.
    Expected numeric mapping (bit positions):
      bit0 = RADC, bit1 = RFFT, bit2 = PDAT, bit3 = TDAT, bit5 = DONE
    """
    global ser
    mask = None
    if isinstance(json, dict):
        mask = json.get('mask')
    else:
        mask = json

    try:
        mask_int = int(mask) & 0xFF
    except Exception:
        print(f"Invalid stream mask received: {json}")
        return

    if ser and ser.is_open:
        try:
            # STOP first to ensure clean state
            ser.write(build_kmd7_frame("RDOT", b"\x00"))
            time.sleep(0.05)
            ser.reset_input_buffer()
            
            # Send new mask
            ser.write(build_kmd7_frame("RDOT", bytes([mask_int])))
            print(f"Sent RDOT with mask 0x{mask_int:02X}")
        except Exception as e:
            print(f"Failed sending RDOT: {e}")
    else:
        print(f"RDOT requested (0x{mask_int:02X}) but serial not open")

@socketio.on('command')
def handle_command(json):
    global ser
    cmd = json.get('cmd')
    payload_hex = json.get('payload', '')
    if ser and ser.is_open:
        try:
            # For mode switching (RDOT), let's ensure we stop first
            if cmd == "RDOT" and payload_hex != "00":
                ser.write(build_kmd7_frame("RDOT", b"\x00"))
                time.sleep(0.05)
                # clear buffers
                ser.reset_input_buffer()
            
            payload = bytes.fromhex(payload_hex)
            ser.write(build_kmd7_frame(cmd, payload))
            print(f"Sent command: {cmd} with payload {payload_hex}")
        except Exception as e:
            print(f"Send Error: {e}")


# --- Playback API and Page ---
@app.route('/playback')
def playback_page():
    return render_template('playback.html')


@app.route('/api/logs')
def api_list_logs():
    try:
        files = [f for f in os.listdir(logs_dir) if f.lower().endswith('.csv')]
        files.sort(reverse=True)
        return jsonify({'files': files})
    except Exception as e:
        return jsonify({'error': str(e)}), 500





@app.route('/api/log/<path:name>')
def api_get_log(name):
    # Prevent path traversal
    filename = secure_filename(name)
    file_path = os.path.join(logs_dir, filename)
    print(f"[DEBUG] api_get_log requested name={name!r}, filename={filename!r}")
    print(f"[DEBUG] resolved file_path={file_path!r}, exists={os.path.exists(file_path)}")
    if os.path.commonpath([os.path.abspath(file_path), os.path.abspath(logs_dir)]) != os.path.abspath(logs_dir):
        return jsonify({'error': 'invalid filename'}), 400

    if not os.path.exists(file_path):
        return jsonify({'error': 'not found'}), 404

    try:
        epochs = {}
        rows = []
        with open(file_path, newline='') as f:
            reader = csv.DictReader(f)
            for r in reader:
                # Expected columns: timestamp_sec, epoch, distance_cm, angle_deg, velocity_m_s, magnitude_db
                try:
                    ts = float(r.get('timestamp_sec', 0.0))
                except Exception:
                    ts = 0.0
                try:
                    epoch = int(r.get('epoch', 0))
                except Exception:
                    epoch = 0
                try:
                    dist = float(r.get('distance_cm', 0.0))
                except Exception:
                    dist = 0.0
                try:
                    angle = float(r.get('angle_deg', 0.0))
                except Exception:
                    angle = 0.0
                try:
                    vel = float(r.get('velocity_m_s', 0.0))
                except Exception:
                    vel = 0.0
                try:
                    mag = float(r.get('magnitude_db', 0.0))
                except Exception:
                    mag = 0.0

                entry = {
                    'timestamp_sec': ts,
                    'epoch': epoch,
                    'dist': dist,
                    'angle': angle,
                    'speed': vel,
                    'mag': mag
                }
                rows.append(entry)
                epochs.setdefault(epoch, {'timestamp_sec': ts, 'targets': []})['targets'].append(entry)

        # Build ordered epoch list
        epoch_list = []
        for e in sorted(epochs.keys()):
            epoch_list.append({'epoch': e, 'timestamp_sec': epochs[e].get('timestamp_sec', 0.0), 'targets': epochs[e]['targets']})

        return jsonify({'rows': rows, 'epochs': epoch_list})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    # Start serial thread
    thread = threading.Thread(target=serial_reader, daemon=True)
    thread.start()
    socketio.run(app, debug=True, use_reloader=False, port=5000)