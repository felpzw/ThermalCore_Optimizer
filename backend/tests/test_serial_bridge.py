"""Testes da ponte serial (Módulo 1).

O loop de serviço é exercitado com um transporte fake (sem hardware). O
adaptador :class:`SerialLineTransport` é testado com um duble de ``Serial`` e,
quando ``pyserial`` está disponível, contra uma porta loopback ``loop://`` real.
"""

import json
import unittest
from dataclasses import replace

from thermalcore.optimizer import optimize, response_payload
from thermalcore.serial_bridge import (
    SerialLineTransport,
    parse_request,
    process_line,
    serve,
)
from thermalcore.thermal_model import Conditions


class FakeTransport:
    """Transporte orientado a linhas em memória. Encerra com ``None`` ao esgotar."""

    def __init__(self, lines):
        self._inbox = list(lines)
        self.outbox = []

    def read_line(self):
        if self._inbox:
            return self._inbox.pop(0)
        return None  # canal encerrado

    def write_line(self, line):
        self.outbox.append(line)


class FakeSerial:
    """Duble mínimo de pyserial.Serial (readline/write/flush)."""

    def __init__(self, chunks):
        self._chunks = list(chunks)   # lista de bytes já quebrada em linhas
        self.written = b""

    def readline(self):
        return self._chunks.pop(0) if self._chunks else b""

    def write(self, data):
        self.written += data

    def flush(self):
        pass


class TestParseRequest(unittest.TestCase):
    def test_parses_qc_and_tmax(self):
        cond = parse_request('{"q_c":20.0,"T_max":85.0}')
        self.assertEqual(cond.q_c, 20.0)
        self.assertEqual(cond.t_max, 85.0)

    def test_accepts_lowercase_tmax_key(self):
        cond = parse_request('{"q_c":30.0,"t_max":70.0}')
        self.assertEqual((cond.q_c, cond.t_max), (30.0, 70.0))

    def test_base_conditions_are_preserved(self):
        base = Conditions(q_c=0.0, t_max=0.0, h=99.0, fin_height=0.03)
        cond = parse_request('{"q_c":40.0,"T_max":75.0}', base)
        self.assertEqual(cond.q_c, 40.0)
        self.assertEqual(cond.t_max, 75.0)
        self.assertEqual(cond.h, 99.0)          # não sobrescrito pela HMI
        self.assertEqual(cond.fin_height, 0.03)

    def test_invalid_json_raises(self):
        with self.assertRaises(ValueError):
            parse_request("{not json}")

    def test_missing_field_raises(self):
        with self.assertRaises(ValueError):
            parse_request('{"q_c":20.0}')


class TestProcessLine(unittest.TestCase):
    def test_response_matches_optimizer(self):
        line = '{"q_c":20.0,"T_max":85.0}'
        expected = json.dumps(
            response_payload(optimize(Conditions(q_c=20.0, t_max=85.0),
                                      criterion="mass")),
            separators=(",", ":"))
        self.assertEqual(process_line(line), expected)

    def test_response_is_compact_json_with_n_and_t(self):
        payload = json.loads(process_line('{"q_c":20.0,"T_max":85.0}'))
        self.assertIn("N", payload)
        self.assertIn("t", payload)
        self.assertIsInstance(payload["N"], int)

    def test_bad_request_does_not_raise(self):
        self.assertEqual(json.loads(process_line("garbage")), {"error": "bad_request"})

    def test_infeasible_request(self):
        line = '{"q_c":80.0,"T_max":75.0}'
        cond = replace(Conditions(q_c=80.0, t_max=75.0), h=25.0)
        # com h=25 é inviável; usa base para reproduzir o cenário
        payload = json.loads(process_line(line, base=cond))
        self.assertEqual(payload, {"error": "infeasible"})

    def test_criterion_is_respected(self):
        base = replace(Conditions(q_c=45.0, t_max=70.0), h=30.0)
        line = '{"q_c":45.0,"T_max":70.0}'
        by_mass = json.loads(process_line(line, base=base, criterion="mass"))
        by_spacing = json.loads(process_line(line, base=base, criterion="spacing"))
        self.assertNotEqual(by_mass, by_spacing)  # critérios divergem neste cenário


class TestServe(unittest.TestCase):
    def test_round_trip_multiple_requests(self):
        t = FakeTransport([
            '{"q_c":20.0,"T_max":85.0}',
            '   ',                              # linha vazia -> ignorada
            '{"q_c":45.0,"T_max":70.0}',
        ])
        served = serve(t)
        self.assertEqual(served, 2)
        self.assertEqual(len(t.outbox), 2)
        for out in t.outbox:
            self.assertIn("N", json.loads(out))

    def test_survives_bad_request_and_continues(self):
        t = FakeTransport(['bad', '{"q_c":20.0,"T_max":85.0}'])
        served = serve(t)
        self.assertEqual(served, 2)
        self.assertEqual(json.loads(t.outbox[0]), {"error": "bad_request"})
        self.assertIn("N", json.loads(t.outbox[1]))


class TestSerialLineTransport(unittest.TestCase):
    def test_read_decodes_line_and_write_appends_newline(self):
        fake = FakeSerial([b'{"q_c":20.0,"T_max":85.0}\n'])
        transport = SerialLineTransport(fake)
        self.assertEqual(transport.read_line(), '{"q_c":20.0,"T_max":85.0}\n')
        transport.write_line('{"N":5,"t":0.5}')
        self.assertEqual(fake.written, b'{"N":5,"t":0.5}\n')

    def test_empty_read_is_timeout_not_eof(self):
        transport = SerialLineTransport(FakeSerial([]))
        self.assertEqual(transport.read_line(), "")  # "" = timeout, não None

    def test_serve_over_serial_adapter(self):
        # Loop completo pelo adaptador serial com um duble de porta. Como
        # read_line devolve "" (timeout) ao esgotar, um wrapper OneShot encerra
        # o loop (retorna None) para o teste terminar.
        class OneShot(SerialLineTransport):
            def read_line(self):
                return super().read_line() or None

        fake = FakeSerial([b'{"q_c":20.0,"T_max":85.0}\n'])
        served = serve(OneShot(fake))
        self.assertEqual(served, 1)
        self.assertIn(b'"N"', fake.written)


@unittest.skipUnless(
    __import__("importlib").util.find_spec("serial") is not None,
    "pyserial não instalado")
class TestRealPySerialLoopback(unittest.TestCase):
    def test_roundtrip_over_loop_url(self):
        import serial  # pyserial

        port = serial.serial_for_url("loop://", timeout=1)
        transport = SerialLineTransport(port)
        transport.write_line('{"q_c":20.0,"T_max":85.0}')
        echoed = transport.read_line().strip()
        self.assertEqual(echoed, '{"q_c":20.0,"T_max":85.0}')
        # e o request ecoado é processável pelo pipeline
        self.assertIn("N", json.loads(process_line(echoed)))
        port.close()


if __name__ == "__main__":
    unittest.main()
