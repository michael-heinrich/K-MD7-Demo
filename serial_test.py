import argparse
import serial
import time
import sys


def hex_to_bytes(s: str) -> bytes:
    s = s.replace(" ", "")
    s = s.replace("0x", "")
    if len(s) % 2:
        s = "0" + s
    return bytes.fromhex(s)


def xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def build_frame(start_hex: str, addr_hex: str, cmd_hex: str, payload_hex: str, checksum: str) -> bytes:
    start = hex_to_bytes(start_hex) if start_hex else b""
    addr = hex_to_bytes(addr_hex) if addr_hex else b""
    cmd = hex_to_bytes(cmd_hex) if cmd_hex else b""
    payload = hex_to_bytes(payload_hex) if payload_hex else b""
    body = addr + cmd + payload
    if checksum == "xor":
        c = bytes([xor_checksum(body)])
    elif checksum == "none":
        c = b""
    else:
        c = b""
    return start + body + c


def build_kmd7_frame(cmd: str, payload_hex: str) -> bytes:
    # K-MD7 frame: 4 ASCII header + uint32 LE payload length + payload bytes
    if len(cmd) != 4:
        raise ValueError('K-MD7 command must be 4 ASCII characters')
    header = cmd.encode('ascii')
    payload = hex_to_bytes(payload_hex) if payload_hex else b''
    length = len(payload).to_bytes(4, 'little')
    return header + length + payload


def read_kmd7_frames(ser: serial.Serial, timeout: float, max_payload: int = 200000) -> list:
    """Read all available K-MD7 frames until timeout."""
    deadline = time.time() + timeout
    buf = bytearray()
    frames = []

    def is_printable(b):
        return 32 <= b <= 126

    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))

        # scan for frames in buffer
        i = 0
        while i + 8 <= len(buf):
            hdr = buf[i:i+4]
            if all(is_printable(b) for b in hdr):
                # read length
                payload_len = int.from_bytes(buf[i+4:i+8], 'little')
                if payload_len < 0 or payload_len > max_payload:
                    i += 1
                    continue
                # check if full payload available
                if i + 8 + payload_len <= len(buf):
                    frames.append(bytes(buf[i:i+8+payload_len]))
                    del buf[:i+8+payload_len]
                    i = 0 # reset scan
                    continue
                else:
                    # found header but waiting for payload
                    break
            else:
                i += 1

        # avoid busy wait
        time.sleep(0.01)

    return frames


def parse_kmd7_response(resp: bytes) -> None:
    # Parse K-MD7: header(4 ASCII) + uint32 LE length + payload
    if len(resp) < 8:
        print('(response too short for K-MD7 parsing)')
        decode_response(resp)
        return

    header = resp[:4].decode('ascii', errors='replace')
    payload_len = int.from_bytes(resp[4:8], 'little')
    print(f"Header: {header}")
    print(f"Declared payload length: {payload_len}")
    payload = resp[8:8+payload_len]
    if len(payload) < payload_len:
        print(f"(incomplete payload: got {len(payload)} of {payload_len})")
    print(f"Actual payload bytes: {len(payload)}")

    # Message-specific decoding
    import struct
    if header == 'RESP':
        if len(payload) >= 1:
            code = payload[0]
            meanings = {0: 'OK', 1: 'Unknown command', 2: 'Invalid parameter', 3: 'Invalid RPST version', 4: 'UART error', 5: 'Missing calibration', 6: 'Timeout', 7: 'App corrupt'}
            print(f"RESP code: {code} -> {meanings.get(code, 'UNKNOWN')}")
        else:
            print('RESP payload missing')
    elif header == 'VERS':
        try:
            s = payload.split(b'\x00', 1)[0].decode('ascii', errors='replace')
            print(f"VERS: {s}")
        except Exception:
            print('Failed to decode VERS')
    elif header == 'PDAT':
        entry_size = 8
        count = len(payload) // entry_size
        print(f"PDAT entries: {count}")
        for i in range(count):
            off = i * entry_size
            dist, speed, angle, mag = struct.unpack_from('<Hh h H', payload, off)
            # angle is signed int16; magnitude unsigned
            print(f" T[{i}]: distance={dist} cm, speed={speed/100.0} km/h, angle={angle/100.0}°, mag={mag/100.0} dB")
    elif header == 'TDAT':
        entry_size = 9
        count = len(payload) // entry_size
        print(f"TDAT entries: {count}")
        for i in range(count):
            off = i * entry_size
            dist, speed, angle, mag, tid = struct.unpack_from('<Hh h H B', payload, off)
            print(f" TT[{i}]: id={tid}, distance={dist} cm, speed={speed/100.0} km/h, angle={angle/100.0}°, mag={mag/100.0} dB")
    elif header == 'RPST':
        if len(payload) >= 31:
            fw = payload[:19].split(b'\x00', 1)[0].decode('ascii', errors='replace')
            freq = payload[19]
            speed = payload[20]
            range_val = payload[21]
            thresh = payload[22]
            filter = payload[23]
            min_dist = payload[24]
            max_dist = payload[25]
            # angle is int8
            min_ang = struct.unpack('b', payload[26:27])[0]
            max_ang = struct.unpack('b', payload[27:28])[0]
            min_speed = payload[28]
            max_speed = payload[29]
            dir_filter = payload[30]
            
            print(f"RPST Firmware: {fw}")
            print(f"  Freq Channel: {freq}, Speed Set: {speed}, Range Set: {range_val}")
            print(f"  Threshold: {thresh} dB, Filter: {filter}")
            print(f"  Dist Zone: {min_dist}-{max_dist}%, Angle Zone: {min_ang} to {max_ang}°")
            print(f"  Speed Filter: {min_speed}-{max_speed}%, Dir Filter: {dir_filter}")
        else:
            print(f"RPST payload too short: {len(payload)}")
    elif header in ('RADC', 'RFFT'):
        print(f"{header} payload: {len(payload)} bytes (large binary) -- not fully decoded here")
    else:
        print('(no specific decoder for this header)')
        decode_response(payload)


def read_response(ser: serial.Serial, timeout: float) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.02)
    return bytes(buf)


def decode_response(resp: bytes) -> None:
    if not resp:
        print("(no data)")
        return

    def is_printable(b):
        return 32 <= b <= 126

    print("\n--- Decoding response ---")
    print(f"Raw hex: {resp.hex().upper()}")
    print(f"Length: {len(resp)} bytes")

    # ASCII header (first 3-4 printable bytes)
    header = b''
    for c in resp[:8]:
        if is_printable(c):
            header += bytes([c])
        else:
            break
    if header:
        try:
            print(f"ASCII header: {header.decode('ascii')}")
        except Exception:
            pass

    # bytes list
    print("Bytes:", ' '.join(f"{b:02X}" for b in resp))

    # simple XOR checksum check (last byte equals XOR of all previous?)
    if len(resp) >= 2:
        xor_calc = 0
        for b in resp[:-1]:
            xor_calc ^= b
        xor_ok = (xor_calc == resp[-1])
        print(f"XOR of all bytes except last: {xor_calc:02X}  last: {resp[-1]:02X}  OK={xor_ok}")

    # Try numeric decodings for offsets
    import struct

    def try_unpack(fmt, data, desc):
        try:
            v = struct.unpack(fmt, data)[0]
            print(f"{desc}: {v} (fmt={fmt})")
        except Exception:
            pass

    # iterate offsets and attempt 16/32-bit interpretations
    for off in range(0, max(0, len(resp) - 1)):
        rem = resp[off:]
        if len(rem) >= 2:
            try_unpack('<H', rem[:2], f"uint16 LE @+{off}")
            try_unpack('>H', rem[:2], f"uint16 BE @+{off}")
            try_unpack('<h', rem[:2], f"int16 LE @+{off}")
            try_unpack('>h', rem[:2], f"int16 BE @+{off}")
        if len(rem) >= 4:
            try_unpack('<I', rem[:4], f"uint32 LE @+{off}")
            try_unpack('>I', rem[:4], f"uint32 BE @+{off}")
            try_unpack('<i', rem[:4], f"int32 LE @+{off}")
            try_unpack('>i', rem[:4], f"int32 BE @+{off}")
            try_unpack('<f', rem[:4], f"float32 LE @+{off}")
            try_unpack('>f', rem[:4], f"float32 BE @+{off}")

    print("--- End decode ---\n")


def main():
    p = argparse.ArgumentParser(description="Basic COM-port tester for K-MD7 (send raw or build frame)")
    p.add_argument("--port", required=True, help="COM port (e.g. COM3)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--timeout", type=float, default=1.0, help="Read timeout seconds")
    p.add_argument("--parity", choices=["N", "E", "O"], default="E", help="Parity: N=None, E=Even, O=Odd (default: E for K-MD7)")

    grp = p.add_mutually_exclusive_group(required=True)
    grp.add_argument("--hex", help="Raw hex frame to send, e.g. 'AA0102FF' or '0xAA 01 02 FF'")
    grp.add_argument("--build", action="store_true", help="Build frame from parts (use --start --addr --cmd --payload --checksum)")
    grp.add_argument("--kmd7", action="store_true", help="Use K-MD7 protocol framing (4 ASCII header + u32 LE length + payload)")

    p.add_argument("--start", default="", help="Start/preamble bytes in hex (optional)")
    p.add_argument("--addr", default="", help="Address byte in hex (optional)")
    p.add_argument("--cmd", default="", help="Command byte in hex (optional)")
    p.add_argument("--payload", default="", help="Payload bytes in hex (optional)")
    p.add_argument("--checksum", choices=["xor","none"], default="xor", help="Checksum type when building frames")

    args = p.parse_args()

    parity_map = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}

    try:
        ser = serial.Serial(args.port, args.baud, parity=parity_map[args.parity], timeout=0)
    except Exception as e:
        print(f"Failed to open {args.port}: {e}")
        sys.exit(2)

    if args.hex:
        frame = hex_to_bytes(args.hex)
    else:
        if args.kmd7:
            # when building K-MD7 frame, expect --cmd as 4-char string (not hex)
            if args.cmd:
                cmd_str = args.cmd
            else:
                print('When using --kmd7 you must supply --cmd (4 ascii chars)')
                sys.exit(2)
            frame = build_kmd7_frame(cmd_str, args.payload)
        else:
            frame = build_frame(args.start, args.addr, args.cmd, args.payload, args.checksum)

    print("Sending:", frame.hex().upper())
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    if args.kmd7:
        frames = read_kmd7_frames(ser, args.timeout)
        if frames:
            for f in frames:
                print("\nReceived frame hex:", f.hex().upper())
                parse_kmd7_response(f)
        else:
            print("No valid K-MD7 response (timeout)")
    else:
        resp = read_response(ser, args.timeout)
        if resp:
            print("Received:", resp.hex().upper())
            decode_response(resp)
        else:
            print("No response (timeout)")

    ser.close()


if __name__ == '__main__':
    main()
