import os
import threading
import time
import struct
import serial
from serial.tools import list_ports
import math
import csv
import traceback
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
# Set KMD7_PORT to restrict discovery to a specific serial port.
PORT = os.environ.get("KMD7_PORT")
BAUD = 115200
RETRY_INTERVAL = 5
HANDSHAKE_TIMEOUT = 2
IDLE_TIMEOUT = 10
PROBE_TIMEOUT = 3
stream_mask = 0x0C
connection_status = {'state': 'disconnected', 'port': None, 'message': 'Waiting for radar discovery'}
serial_lock = threading.RLock()


def set_connection_status(state, message, port=None):
    global connection_status
    connection_status = {'state': state, 'port': port, 'message': message}
    print(f"[Radar] {message}", flush=True)
    socketio.emit('radar_status', connection_status)


@socketio.on('connect')
def handle_connect():
    emit('radar_status', connection_status)


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
current_rpst = None

def build_kmd7_frame(cmd: str, payload: bytes = b'') -> bytes:
    header = cmd.encode('ascii')[:4].ljust(4, b' ')
    length = struct.pack('<I', len(payload))
    return header + length + payload

# Only accept known sensor messages, with protocol-defined payload bounds. This
# also allows recovery when a serial stream begins in the middle of a frame.
FRAME_LENGTHS = {
    'RESP': (1, 1), 'VERS': (1, 19), 'RPST': (31, 31),
    'PDAT': (0, 192), 'TDAT': (0, 72), 'RADC': (6144, 6144),
    'RFFT': (2048, 2048), 'DONE': (4, 4),
}
RESPONSE_CODES = {0: 'OK', 1: 'unknown command', 2: 'invalid parameter',
                  3: 'invalid RPST version', 4: 'UART error',
                  5: 'missing calibration', 6: 'timeout', 7: 'application corrupt'}


def parse_frames(buf: bytearray):
    i = 0
    while i + 8 <= len(buf):
        try:
            header = bytes(buf[i:i + 4]).decode('ascii')
        except UnicodeDecodeError:
            i += 1
            continue
        bounds = FRAME_LENGTHS.get(header)
        payload_len = struct.unpack_from('<I', buf, i + 4)[0]
        if not bounds or not bounds[0] <= payload_len <= bounds[1]:
            i += 1
            continue
        if i + 8 + payload_len > len(buf):
            if i:
                del buf[:i]
            return None, None
        payload = bytes(buf[i + 8:i + 8 + payload_len])
        del buf[:i + 8 + payload_len]
        return header, payload
    if i:
        del buf[:i]
    return None, None


def discover_radar():
    """Open and initialize each candidate until a radar acknowledges INIT."""
    if PORT:
        candidates = [PORT]
    else:
        try:
            candidates = [p.device for p in sorted(list_ports.comports(), key=lambda p: p.device)]
        except (serial.SerialException, OSError) as exc:
            set_connection_status('disconnected', f"Could not list serial ports: {exc}; retrying in 5 seconds.")
            return None, bytearray()
    if not candidates:
        set_connection_status('disconnected', 'No serial ports found; retrying in 5 seconds.')
        return None, bytearray()
    set_connection_status('searching', f"Searching serial ports: {', '.join(candidates)}")
    for port in candidates:
        candidate = None
        try:
            print(f"[Radar] Trying {port} (115200 8E1): sending INIT", flush=True)
            candidate = serial.Serial(port, BAUD, parity=serial.PARITY_EVEN,
                                      timeout=0.1, write_timeout=1)
            candidate.write(build_kmd7_frame('INIT', b'\x00'))
            deadline = time.monotonic() + HANDSHAKE_TIMEOUT
            buf = bytearray()
            while time.monotonic() < deadline:
                data = candidate.read(candidate.in_waiting or 1)
                if data:
                    buf.extend(data)
                while True:
                    header, payload = parse_frames(buf)
                    if header is None:
                        break
                    if header == 'VERS':
                        print(f"[Radar] {port} firmware: {payload.split(bytes([0]), 1)[0].decode('ascii', errors='replace')}", flush=True)
                    if header == 'RESP':
                        code = payload[0]
                        if code != 0:
                            raise RuntimeError(f"INIT rejected: RESP {code} ({RESPONSE_CODES.get(code, 'unknown error')})")
                        print(f"[Radar] {port} acknowledged INIT (RESP 0).", flush=True)
                        return candidate, buf
            print(f"[Radar] {port}: no INIT acknowledgement within {HANDSHAKE_TIMEOUT}s.", flush=True)
        except (serial.SerialException, OSError, RuntimeError) as exc:
            print(f"[Radar] {port}: connection failed: {exc}", flush=True)
        if candidate is not None:
            candidate.close()
    set_connection_status('disconnected', 'No radar acknowledged INIT; retrying in 5 seconds.')
    return None, bytearray()


def serial_reader():
    global ser, current_rpst
    while True:
        candidate, buf = discover_radar()
        if candidate is None:
            time.sleep(RETRY_INTERVAL)
            continue
        try:
            # Publish the port only after the INIT acknowledgement has been verified.
            with serial_lock:
                current_rpst = None
                ser = candidate
                candidate.write(build_kmd7_frame("GRPS"))
                candidate.write(build_kmd7_frame("RDOT", bytes([stream_mask])))
            set_connection_status('connected', f"Connected to radar on {candidate.port}; requested settings and target stream.", candidate.port)
            read_radar(candidate, buf)
        except (serial.SerialException, OSError) as exc:
            set_connection_status('disconnected', f"Connection to {candidate.port} lost: {exc}; retrying in 5 seconds.")
        except Exception as exc:
            traceback.print_exc()
            set_connection_status('disconnected', f"Radar reader failed: {exc}; retrying in 5 seconds.")
        finally:
            with serial_lock:
                if ser is candidate:
                    ser = None
                candidate.close()
        time.sleep(RETRY_INTERVAL)


def read_radar(connection, buf):
    last_frame = time.monotonic()
    probe_sent = None
    while True:
        try:
            if connection.in_waiting:
                data = connection.read(connection.in_waiting)
                buf.extend(data)
            
            while True:
                header, payload = parse_frames(buf)
                if not header:
                    break
                last_frame = time.monotonic()
                probe_sent = None

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
                    code = payload[0]
                    message = f"Sensor response: {code} ({RESPONSE_CODES.get(code, 'unknown error')})"
                    print(f"[Radar] {message}", flush=True)
                    socketio.emit('info', {'msg': message})
                elif header == "VERS":
                    firmware = payload.split(b'\x00', 1)[0].decode('ascii', errors='replace')
                    print(f"[Radar] Firmware: {firmware}", flush=True)

            now = time.monotonic()
            if probe_sent is not None and now - probe_sent > PROBE_TIMEOUT:
                raise serial.SerialException('sensor stopped responding to GRPS')
            if probe_sent is None and now - last_frame > IDLE_TIMEOUT:
                print(f"[Radar] No frames for {IDLE_TIMEOUT}s on {connection.port}; checking connection with GRPS.", flush=True)
                with serial_lock:
                    connection.write(build_kmd7_frame('GRPS'))
                probe_sent = now

        except (serial.SerialException, OSError):
            raise
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
    
    if field not in offsets:
        return
    with serial_lock:
        if not ser or not ser.is_open:
            print(f"[Radar] Cannot update {field}: radar is not connected.", flush=True)
            return
        if current_rpst is None:
            print(f"[Radar] Cannot update {field}: waiting for sensor settings (RPST).", flush=True)
            return
        idx = offsets[field]
        updated = bytearray(current_rpst)
        updated[idx] = struct.pack('b', value)[0] if field in ('min_ang', 'max_ang') else value & 0xFF
        try:
            ser.write(build_kmd7_frame("SRPS", bytes(updated)))
            ser.write(build_kmd7_frame("GRPS"))
            print(f"[Radar] Sent setting {field}={value}; awaiting sensor response.", flush=True)
        except (serial.SerialException, OSError) as exc:
            print(f"[Radar] Failed to update {field}: {exc}", flush=True)


@socketio.on('set_streams')
def handle_set_streams(json):
    """Receive a numeric 1-byte bitmask (or dict) from the GUI and send RDOT accordingly.
    Expected numeric mapping (bit positions):
      bit0 = RADC, bit1 = RFFT, bit2 = PDAT, bit3 = TDAT, bit5 = DONE
    """
    global stream_mask
    mask = json.get('mask') if isinstance(json, dict) else json
    try:
        mask_int = int(mask) & 0xFF
    except (TypeError, ValueError):
        print(f"[Radar] Invalid stream mask: {json}", flush=True)
        return
    with serial_lock:
        stream_mask = mask_int
        if not ser or not ser.is_open:
            print(f"[Radar] Stream mask 0x{mask_int:02X} saved for the next connection.", flush=True)
            return
        try:
            ser.write(build_kmd7_frame("RDOT", bytes([mask_int])))
            print(f"[Radar] Sent RDOT 0x{mask_int:02X}; awaiting sensor response.", flush=True)
        except (serial.SerialException, OSError) as exc:
            print(f"[Radar] Failed to send RDOT: {exc}", flush=True)


@socketio.on('command')
def handle_command(json):
    cmd = json.get('cmd')
    payload_hex = json.get('payload', '')
    with serial_lock:
        if not ser or not ser.is_open:
            print(f"[Radar] Cannot send {cmd}: radar is not connected.", flush=True)
            return
        try:
            payload = bytes.fromhex(payload_hex)
            ser.write(build_kmd7_frame(cmd, payload))
            print(f"[Radar] Sent {cmd} {payload_hex}; awaiting sensor response.", flush=True)
        except (serial.SerialException, OSError, ValueError, TypeError, AttributeError) as exc:
            print(f"[Radar] Failed to send {cmd}: {exc}", flush=True)


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