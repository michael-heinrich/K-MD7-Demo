import serial
import time
import struct
import sys
import argparse

def build_kmd7_frame(cmd: str, payload: bytes = b'') -> bytes:
    header = cmd.encode('ascii')[:4].ljust(4, b' ')
    length = struct.pack('<I', len(payload))
    return header + length + payload

def read_frames(ser, buf: bytearray):
    """Scan bytearray for K-MD7 frames and yield them."""
    i = 0
    while i + 8 <= len(buf):
        hdr_bytes = buf[i:i+4]
        # check if 4 printable ASCII
        if all(32 <= b <= 126 for b in hdr_bytes):
            payload_len = struct.unpack('<I', buf[i+4:i+8])[0]
            if payload_len > 100000: # sanity check
                i += 1
                continue
            if i + 8 + payload_len <= len(buf):
                frame = buf[i : i + 8 + payload_len]
                # consume from buffer
                header = hdr_bytes.decode('ascii', errors='replace')
                payload = bytes(buf[i+8 : i+8+payload_len])
                del buf[:i + 8 + payload_len]
                yield header, payload
                i = 0
                continue
            else:
                break # need more data
        i += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM4", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Connecting to K-MD7 on {args.port} (115200 8E1)...")
    try:
        ser = serial.Serial(args.port, args.baud, parity=serial.PARITY_EVEN, timeout=0.1)
    except Exception as e:
        print(f"Error: {e}")
        return

    # 1. Init
    ser.write(build_kmd7_frame("INIT", b"\x00"))
    time.sleep(0.1)
    
    # 2. Start Streaming (PDAT + TDAT = 0x0C)
    print("Enabling streaming mode (RDOT 0x0C: PDAT + TDAT)...")
    ser.write(build_kmd7_frame("RDOT", b"\x0C"))
    
    buf = bytearray()
    last_print_time = 0
    try:
        print("Reading targets. Press Ctrl+C to stop.")
        while True:
            # check for any bytes
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buf.extend(data)
            else:
                # read 1 byte with timeout to avoid CPU spin
                data = ser.read(1)
                if data:
                    buf.extend(data)

            for header, payload in read_frames(ser, buf):
                if header in ("PDAT", "TDAT"):
                    entry_size = 8 if header == "PDAT" else 9
                    count = len(payload) // entry_size
                    
                    min_dist = float('inf')
                    for j in range(count):
                        off = j * entry_size
                        # Dist is uint16 LE @ offset 0, cm
                        dist_cm = struct.unpack_from('<H', payload, off)[0]
                        # Magnitude is uint16 LE @ offset 6 for PDAT, 6 for TDAT too?
                        # PDAT: dist(2), speed(2), angle(2), mag(2) = 8
                        # TDAT: dist(2), speed(2), angle(2), mag(2), tid(1) = 9
                        # We only check distance for now
                        if dist_cm > 0 and dist_cm < min_dist:
                            min_dist = dist_cm
                    
                    if min_dist != float('inf'):
                        sys.stdout.write(f"\rClosest target: {min_dist:4d} cm ({min_dist/100.0:5.2f} m)    ")
                    else:
                        sys.stdout.write("\rNo targets in range...                     ")
                    sys.stdout.flush()
                elif header == "RESP":
                    code = payload[0] if payload else -1
                    if code != 0:
                        print(f"\n[INFO] Sensor Error: RESP code {code}")
                elif header == "VERS":
                    fw = payload.split(b'\x00', 1)[0].decode('ascii', errors='replace')
                    print(f"\n[INFO] Sensor Firmware: {fw}")
                elif header == "DONE":
                    pass # Frame complete
                else:
                    # Ignore other headers silently or log if unknown
                    if header not in ("RADC", "RFFT", "RPST"):
                        print(f"\n[DEBUG] Unknown header: {header} (len {len(payload)})")

            # If no data for 2 seconds, warn
            if time.time() - last_print_time > 2.0:
                # sys.stdout.write("\r(waiting for data...)                          ")
                # sys.stdout.flush()
                pass
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nStopping stream...")
        ser.write(build_kmd7_frame("RDOT", b"\x00"))
        time.sleep(0.1)
    finally:
        ser.close()
        print("Done.")

if __name__ == "__main__":
    main()
