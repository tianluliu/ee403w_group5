"""Platform-independent tests for the Pi<->Arduino line protocol framing."""

import random
import unittest

import _bootstrap  # noqa: F401  (sets sys.path)
from lidar_robot import protocol as p


class EncodeTests(unittest.TestCase):
    def test_known_vector_hand_verified(self):
        # 'P'^'I'^'N'^'G'^','^'1' = 0x0D, computed by hand independently of the code.
        self.assertEqual(p.encode("PING", 1), b"$PING,1*0D\n")

    def test_checksum_function_matches_known_values(self):
        self.assertEqual(p.checksum(b""), 0x00)
        self.assertEqual(p.checksum(b"PING,1"), 0x0D)
        self.assertEqual(p.checksum(b"DRV,7,-500,500,250"), 0x6D)

    def test_roundtrip(self):
        cases = [("PING", (1,)), ("DRV", (7, -500, 500, 250)), ("ARM", (65535,)),
                 ("NAK", (3, "NOT_ARMED")), ("STOP", ())]
        for cmd, fields in cases:
            frame = p.decode(p.encode(cmd, *fields))
            self.assertEqual(frame, p.Frame(cmd, tuple(str(f) for f in fields)))

    def test_max_size_boundary(self):
        # frame length == len(body) + 5  ('$', '*', 2 hex digits, '\n')
        ok = p.encode("A", "x" * (p.MAX_FRAME_BYTES - 5 - 2))
        self.assertEqual(len(ok), p.MAX_FRAME_BYTES)
        with self.assertRaises(p.ProtocolError):
            p.encode("A", "x" * (p.MAX_FRAME_BYTES - 5 - 2 + 1))

    def test_invalid_tokens_rejected(self):
        bad = ["", "a,b", "a*b", "a$b", "a\nb", "a\rb", "a b", "a\tb", "a\x00b",
               "caf\u00e9", "\x7f"]
        for token in bad:
            with self.subTest(token=token):
                with self.assertRaises(p.ProtocolError):
                    p.encode("DRV", token)
                with self.assertRaises(p.ProtocolError):
                    p.encode(token)


class DecodeTests(unittest.TestCase):
    def test_crlf_tolerated(self):
        self.assertEqual(p.decode(b"$PING,1*0D\r\n").cmd, "PING")
        self.assertEqual(p.decode(b"$PING,1*0D").cmd, "PING")

    def test_bad_checksum_value(self):
        for line in (b"$PING,1*0C\n", b"$PING,1*00\n", b"$PING,1*FF\n"):
            with self.subTest(line=line):
                with self.assertRaises(p.ProtocolError):
                    p.decode(line)

    def test_every_single_byte_corruption_detected(self):
        good = p.encode("DRV", 7, -500, 500, 250)
        body_end = good.rfind(b"*")
        for i in range(1, body_end):
            corrupted = bytearray(good)
            corrupted[i] = corrupted[i] ^ 0x01
            with self.subTest(position=i):
                with self.assertRaises(p.ProtocolError):
                    p.decode(bytes(corrupted))

    def test_checksum_text_must_be_two_uppercase_hex_digits(self):
        # int(x, 16) alone would accept lowercase, '+D', ' D'; the spec does not.
        for cs in (b"0d", b"+D", b" D", b"-0", b"0x", b"0", b"", b"00D", b"0D "):
            with self.subTest(cs=cs):
                with self.assertRaises(p.ProtocolError):
                    p.decode(b"$PING,1*" + cs + b"\n")

    def test_structural_errors(self):
        for line in (b"PING,1*0D\n", b"$PING,1\n", b"", b"$", b"$*00\n", b"*0D\n",
                     b"$PING,1,*" + b"%02X" % p.checksum(b"PING,1,") + b"\n",   # trailing comma
                     b"$PING,,1*" + b"%02X" % p.checksum(b"PING,,1") + b"\n"):  # empty field
            with self.subTest(line=line):
                with self.assertRaises(p.ProtocolError):
                    p.decode(line)

    def test_direct_decode_enforces_max_frame_size(self):
        # Spec: whole frame incl. '$' and '\n' is <= MAX_FRAME_BYTES. decode() must not be
        # more lenient than the stream assembler or than the Arduino's fixed buffer.
        def build(total_len):
            body = b"A," + b"x" * (total_len - 5 - 2)
            return b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\n"

        at_limit = build(p.MAX_FRAME_BYTES)
        self.assertEqual(len(at_limit), p.MAX_FRAME_BYTES)
        self.assertEqual(p.decode(at_limit).cmd, "A")
        for extra in (1, 2, 100, 10_000):
            over = build(p.MAX_FRAME_BYTES + extra)
            with self.subTest(extra=extra):
                self.assertEqual(len(over), p.MAX_FRAME_BYTES + extra)
                with self.assertRaises(p.ProtocolError):
                    p.decode(over)
                with self.assertRaises(p.ProtocolError):
                    p.decode(over.rstrip(b"\n") + b"\r\n")

    def test_decode_limit_counts_cr_and_missing_terminator(self):
        # Rule: every wire byte counts (CR included); a missing LF is measured as if present.
        def line_of(total_with_lf):
            body = b"A," + b"x" * (total_with_lf - 5 - 2)
            return b"$" + body + b"*" + b"%02X" % p.checksum(body)  # no terminator

        at_limit = line_of(p.MAX_FRAME_BYTES)           # 63 bytes + LF == 64
        self.assertEqual(p.decode(at_limit).cmd, "A")                    # LF absent
        self.assertEqual(p.decode(at_limit + b"\n").cmd, "A")           # LF present
        with self.assertRaises(p.ProtocolError):                         # CRLF => 65 on the wire
            p.decode(at_limit + b"\r\n")
        one_short = line_of(p.MAX_FRAME_BYTES - 1)      # 62 bytes
        self.assertEqual(p.decode(one_short + b"\r\n").cmd, "A")        # CRLF => 64 on the wire
        over = line_of(p.MAX_FRAME_BYTES + 1)
        with self.assertRaises(p.ProtocolError):
            p.decode(over)                                               # absent LF still counted

    def test_encode_decode_assembler_agree_on_the_length_limit(self):
        asm_results = {}
        for total in range(p.MAX_FRAME_BYTES - 6, p.MAX_FRAME_BYTES + 7):
            body = b"A," + b"x" * (total - 5 - 2)
            lf = b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\n"
            crlf = lf[:-1] + b"\r\n"
            for name, wire in (("LF", lf), ("CRLF", crlf)):
                with self.subTest(total=total, terminator=name):
                    allowed = len(wire) <= p.MAX_FRAME_BYTES
                    # direct decode
                    if allowed:
                        self.assertEqual(p.decode(wire).cmd, "A")
                    else:
                        with self.assertRaises(p.ProtocolError):
                            p.decode(wire)
                    # stream assembler, whole chunk and 1-byte fragments
                    for chunk in (len(wire), 1, 7):
                        asm = p.FrameAssembler()
                        out = []
                        for i in range(0, len(wire), chunk):
                            out += asm.feed(wire[i:i + chunk])
                        if allowed:
                            self.assertEqual([o.cmd for o in out], ["A"], f"chunk={chunk}")
                        else:
                            self.assertEqual(len(out), 1)
                            self.assertIsInstance(out[0], p.ProtocolError, f"chunk={chunk}")
                        asm_results[(total, name, chunk)] = bool(allowed)
            # encode(): LF-terminated only
            fields = ("x" * (total - 5 - 2),)
            with self.subTest(total=total, terminator="encode"):
                if total <= p.MAX_FRAME_BYTES:
                    self.assertEqual(len(p.encode("A", *fields)), total)
                else:
                    with self.assertRaises(p.ProtocolError):
                        p.encode("A", *fields)
        self.assertTrue(asm_results)

    def test_unterminated_overlong_input_is_bounded_and_reported_at_the_limit(self):
        # Buffer bound matters for a fixed-size MCU buffer: a frame that has used
        # MAX_FRAME_BYTES-1 non-LF bytes can no longer fit once its LF arrives.
        asm = p.FrameAssembler()
        self.assertEqual(asm.feed(b"$" + b"A" * (p.MAX_FRAME_BYTES - 2)), [])  # 63 bytes buffered
        out = asm.feed(b"A")                                                     # 64th byte
        self.assertEqual(len(out), 1)
        self.assertIsInstance(out[0], p.ProtocolError)
        # Remaining junk (and its LF) is ignored until a new '$', then a good frame parses.
        self.assertEqual(asm.feed(b"AAAA\n"), [])
        self.assertEqual(asm.feed(p.encode("PING", 1)), [p.Frame("PING", ("1",))])

    def test_assembler_recovers_after_oversized_crlf_frame(self):
        body = b"A," + b"x" * (p.MAX_FRAME_BYTES - 5 - 2)
        too_long = b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\r\n"  # 65 bytes
        good = p.encode("PING", 1)
        out = p.FrameAssembler().feed(too_long + good)
        self.assertIsInstance(out[0], p.ProtocolError)
        self.assertEqual(out[-1], p.Frame("PING", ("1",)))
        self.assertEqual(len(out), 2)

    def test_non_ascii_body_rejected(self):
        body = "PING,\u00e9".encode("utf-8")
        line = b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\n"
        with self.assertRaises(p.ProtocolError):
            p.decode(line)

    def test_control_character_in_body_rejected(self):
        body = b"PI\x01NG,1"
        line = b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\n"
        with self.assertRaises(p.ProtocolError):
            p.decode(line)


class AssemblerTests(unittest.TestCase):
    def setUp(self):
        self.asm = p.FrameAssembler()
        self.good = p.encode("PING", 1)
        self.good_frame = p.Frame("PING", ("1",))

    def test_single_frame(self):
        self.assertEqual(self.asm.feed(self.good), [self.good_frame])

    def test_byte_by_byte_fragmentation(self):
        out = []
        for i in range(len(self.good)):
            out += self.asm.feed(self.good[i:i + 1])
        self.assertEqual(out, [self.good_frame])

    def test_every_split_point(self):
        for cut in range(1, len(self.good)):
            asm = p.FrameAssembler()
            out = asm.feed(self.good[:cut]) + asm.feed(self.good[cut:])
            with self.subTest(cut=cut):
                self.assertEqual(out, [self.good_frame])

    def test_two_frames_in_one_chunk(self):
        out = self.asm.feed(self.good + p.encode("STOP", 2))
        self.assertEqual(out, [self.good_frame, p.Frame("STOP", ("2",))])

    def test_garbage_between_and_before_frames_ignored(self):
        out = self.asm.feed(b"\x00\xff junk \n" + self.good + b"\r\n###" + self.good)
        self.assertEqual(out, [self.good_frame, self.good_frame])

    def test_recovery_after_checksum_error(self):
        bad = bytearray(self.good)
        bad[-2] = ord("F") if bad[-2] != ord("F") else ord("E")
        out = self.asm.feed(bytes(bad) + self.good)
        self.assertIsInstance(out[0], p.ProtocolError)
        self.assertEqual(out[1:], [self.good_frame])

    def test_restart_in_the_middle_of_a_frame(self):
        out = self.asm.feed(b"$DRV,1,2" + self.good)
        self.assertIsInstance(out[0], p.ProtocolError)
        self.assertEqual(out[1:], [self.good_frame])

    def test_oversized_frame_rejected_then_recovers(self):
        out = self.asm.feed(b"$" + b"A" * 200 + b"\n" + self.good)
        errors = [o for o in out if isinstance(o, p.ProtocolError)]
        self.assertEqual(len(errors), 1)
        self.assertEqual(out[-1], self.good_frame)

    def test_exactly_max_length_frame_accepted(self):
        frame = p.encode("A", "x" * (p.MAX_FRAME_BYTES - 7))
        self.assertEqual(len(frame), p.MAX_FRAME_BYTES)
        self.assertEqual(self.asm.feed(frame), [p.decode(frame)])

    def test_one_byte_over_max_length_rejected(self):
        body = b"A," + b"x" * (p.MAX_FRAME_BYTES - 6)
        line = b"$" + body + b"*" + b"%02X" % p.checksum(body) + b"\n"
        self.assertEqual(len(line), p.MAX_FRAME_BYTES + 1)
        out = self.asm.feed(line + self.good)
        self.assertIsInstance(out[0], p.ProtocolError)
        self.assertEqual(out[-1], self.good_frame)

    def test_non_ascii_inside_frame_is_error_not_exception(self):
        out = self.asm.feed(b"$PI\xffNG,1*00\n" + self.good)
        self.assertIsInstance(out[0], p.ProtocolError)
        self.assertEqual(out[-1], self.good_frame)

    def test_newline_while_idle_ignored(self):
        self.assertEqual(self.asm.feed(b"\n\n\n"), [])

    def test_fuzz_never_raises_and_always_resyncs(self):
        rng = random.Random(1234)
        for _ in range(300):
            noise = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 200)))
            asm = p.FrameAssembler()
            out = []
            data = noise + self.good
            i = 0
            while i < len(data):
                step = rng.randrange(1, 9)
                out += asm.feed(data[i:i + step])
                i += step
            self.assertEqual(out[-1], self.good_frame)


if __name__ == "__main__":
    unittest.main()
