"""Protocol checks for the actual mixed text / binary FPGA UART stream."""
import unittest
from pathlib import Path

from capture_cnn_uart import FrameParser
from replay_cnn_uart_frame import parse_rtl, compare


def fixture(frame="0001", overflow=False):
    # Payload deliberately contains protocol-like text and all byte values.
    mask = bytes([0, 255]) * 28800
    gray = (bytes(range(256)) * 15)[:3600]
    gray = b"\r\nF,DEAD,00,00,0,12345678\nimage:0,12,4,3,24\n" + gray
    gray = gray[:3600]
    stream = f"F,{frame},{'17' if overflow else '01'},01,{int(overflow)},EEC8E020\n".encode()
    stream += f"D,{frame},00,010,020,030,040,2,123456AB\n".encode()
    stream += b"image:0,57600,320,180,24\n" + mask
    stream += b"image:1,3600,80,45,24\n" + gray
    stream += f"E,{frame}\n".encode()
    return stream, mask, gray


def decode(data, block):
    parser = FrameParser()
    events = []
    for start in range(0, len(data), block):
        events.extend(parser.feed(data[start:start+block]))
    return parser, events


class ProtocolTests(unittest.TestCase):
    def test_arbitrary_chunks_and_binary_content(self):
        raw, mask, gray = fixture()
        for block in (1, 2, 7, 23, 997, 65536):
            with self.subTest(block=block):
                parser, events = decode(raw, block)
                self.assertEqual(len(parser.groups), 1)
                group = parser.groups[0]
                self.assertTrue(group["complete"], group["errors"])
                self.assertEqual(group["digits"][0]["patch_crc32"], "123456AB")
                self.assertEqual(group["digits"][0]["class"], 2)
                images = [e for e in events if e["kind"] == "image"]
                self.assertEqual(images[0]["picture"].tobytes(), mask)
                self.assertEqual(images[1]["picture"].tobytes(), gray)
                self.assertEqual([e["image"]["frame_id"] for e in images], ["0001", "0001"])
                self.assertEqual(parser.protocol_errors, [])

    def test_midframe_then_complete_group(self):
        old, _, _ = fixture("FFFF")
        new, _, _ = fixture("0000")
        parser, events = decode(old[1400:] + new, 23)
        self.assertEqual(parser.groups[-1]["frame_id"], "0000")
        self.assertTrue(parser.groups[-1]["complete"])
        # A thumbnail seen before F is never assigned to the next frame.
        self.assertTrue(any(e["kind"] == "image" and e["image"]["frame_id"] is None for e in events))

    def test_legacy_images(self):
        parser, events = decode(b"noise\r\nimage:0,4,2,2,24\r\n\x00\xff\x0a\x0d", 1)
        self.assertEqual(parser.groups, [])
        self.assertEqual(len(events), 1)
        self.assertIsNone(events[0]["image"]["frame_id"])
        self.assertEqual(events[0]["picture"].tobytes(), b"\x00\xff\x0a\x0d")

    def test_truncated_and_bad_counts(self):
        raw, _, _ = fixture()
        parser, _ = decode(raw[:-20], 997)
        self.assertFalse(parser.groups[0]["complete"])
        self.assertIsNotNone(parser.pending)
        parser, _ = decode(raw.replace(b",01,01,0,", b",02,02,0,"), 997)
        self.assertFalse(parser.groups[0]["complete"])
        self.assertIn("Digit record count mismatch", parser.groups[0]["errors"])

    def test_wrong_frame_and_overflow(self):
        raw, _, _ = fixture()
        parser, _ = decode(raw.replace(b"D,0001,", b"D,0002,"), 997)
        self.assertFalse(parser.groups[0]["complete"])
        self.assertTrue(parser.protocol_errors)
        parser, _ = decode(fixture(overflow=True)[0], 997)
        self.assertTrue(parser.groups[0]["complete"])
        self.assertTrue(parser.groups[0]["overflow"])

    def test_real_rtl_uart_fixture_when_present(self):
        source = Path(__file__).resolve().parents[1] / "r51_uart_sim.bin"
        if not source.exists():
            self.skipTest("Run tb_cnn_frame_report_uart first")
        parser, events = decode(source.read_bytes(), 997)
        self.assertEqual([g["stored_count"] for g in parser.groups], [3,0,16,16])
        self.assertEqual([g["frame_id"] for g in parser.groups], ["0001","0002","0003","0004"])
        self.assertTrue(all(g["complete"] for g in parser.groups), parser.groups)
        self.assertEqual(parser.protocol_errors, [])
        self.assertEqual(len([e for e in events if e["kind"] == "image"]), 8)

    def test_replay_comparison_finds_first_stage_differences(self):
        log = "RTL_DIGIT x=10 y=20 w=30 h=40 class=2 patch_crc=123456ab\nRTL_FRAME_DONE count=1 reported=1 weight_crc=eec8e020\n"
        rtl = parse_rtl(log)
        group = {"detected_count":1,"weight_crc32":"EEC8E020","digits":[{
            "index":0,"x":10,"y":20,"w":30,"h":40,"class":2,"patch_crc32":"123456AB"}]}
        self.assertEqual(compare(group,rtl),[])
        group["digits"][0].update(x=11,patch_crc32="BAD0C0DE",**{"class":3})
        self.assertEqual([e["field"] for e in compare(group,rtl)],["x","patch_crc32","class"])
        group["weight_crc32"]="BAD0C0DE"
        self.assertEqual(compare(group,rtl)[0]["field"],"weight_crc32")
        with self.assertRaises(ValueError):
            parse_rtl("RTL_DIGIT x=10 y=20 w=30 h=40 class=2 patch_crc=123456ab\n")
        with self.assertRaises(ValueError):
            parse_rtl("FATAL: missing weights\n"+log)


if __name__ == "__main__":
    unittest.main(verbosity=2)
