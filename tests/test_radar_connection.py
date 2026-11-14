import unittest
from types import SimpleNamespace
from unittest.mock import patch

import serial
import app


class FakeSerial:
    def __init__(self, port, response=b''):
        self.port = port
        self.response = bytearray(response)
        self.writes = []
        self.closed = False
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.response)

    def read(self, count):
        result = self.response[:count]
        del self.response[:count]
        return bytes(result)

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True
        self.is_open = False


class RadarConnectionTests(unittest.TestCase):
    def test_parser_recovers_from_noise_and_fragmented_frames(self):
        frame = app.build_kmd7_frame('RESP', b'\x00')
        buf = bytearray(b'garbage' + frame[:6])
        self.assertEqual(app.parse_frames(buf), (None, None))
        buf.extend(frame[6:])
        self.assertEqual(app.parse_frames(buf), ('RESP', b'\x00'))
        self.assertEqual(buf, bytearray())

    def test_discovery_skips_busy_and_rejected_ports_then_accepts_ack(self):
        rejected = FakeSerial('COM2', app.build_kmd7_frame('RESP', b'\x02'))
        accepted = FakeSerial('COM3', app.build_kmd7_frame('RESP', b'\x00') + app.build_kmd7_frame('VERS', b'v1'))

        def open_port(port, *args, **kwargs):
            if port == 'COM1':
                raise serial.SerialException('access denied')
            return {'COM2': rejected, 'COM3': accepted}[port]

        ports = [SimpleNamespace(device=f'COM{i}') for i in (1, 2, 3)]
        with patch.object(app, 'PORT', None), patch.object(app.list_ports, 'comports', return_value=ports), patch.object(app.serial, 'Serial', side_effect=open_port):
            found, buf = app.discover_radar()
        self.assertIs(found, accepted)
        self.assertTrue(rejected.closed)
        self.assertFalse(accepted.closed)
        self.assertEqual(app.parse_frames(buf), ('VERS', b'v1'))
        self.assertEqual(accepted.writes, [app.build_kmd7_frame('INIT', b'\x00')])

    def test_unresponsive_port_is_closed_and_override_is_used(self):
        device = FakeSerial('COM5')
        with patch.object(app, 'PORT', 'COM5'), patch.object(app, 'HANDSHAKE_TIMEOUT', 0), patch.object(app.serial, 'Serial', return_value=device) as factory, patch.object(app.list_ports, 'comports') as enumerate_ports:
            found, _ = app.discover_radar()
        self.assertIsNone(found)
        self.assertTrue(device.closed)
        self.assertEqual(factory.call_args.args[0], 'COM5')
        enumerate_ports.assert_not_called()
        self.assertEqual(app.connection_status['state'], 'disconnected')

    def test_reader_closes_port_and_reports_disconnect(self):
        device = FakeSerial('COM7')
        with patch.object(app, 'discover_radar', return_value=(device, bytearray())), patch.object(app, 'read_radar', side_effect=serial.SerialException('unplugged')), patch.object(app.time, 'sleep', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                app.serial_reader()
        self.assertTrue(device.closed)
        self.assertIsNone(app.ser)
        self.assertEqual(app.connection_status['state'], 'disconnected')
        self.assertIn('unplugged', app.connection_status['message'])

    def test_new_web_client_receives_current_radar_state(self):
        app.set_connection_status('disconnected', 'No radar found')
        client = app.socketio.test_client(app.app)
        messages = client.get_received()
        client.disconnect()
        status = next(msg for msg in messages if msg['name'] == 'radar_status')
        self.assertEqual(status['args'][0]['message'], 'No radar found')


if __name__ == '__main__':
    unittest.main()
