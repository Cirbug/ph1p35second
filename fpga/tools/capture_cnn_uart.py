"""Save B-board image UART packets as PNG; VOFA is not required.

Use a Python environment with pyserial and Pillow. Only receives data; does
not send commands, reconfigure the sensor, or program FPGA/Flash.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import re
import time
import zlib

from PIL import Image, ImageDraw

HEADER = re.compile(rb"image:(\d+),(\d+),(\d+),(\d+),(\d+)\r?\n")
TOKEN = re.compile(rb"image:|[FDE],")
FRAME = re.compile(rb"F,([0-9A-Fa-f]{4}),(\d{2}),(\d{2}),([01]),([0-9A-Fa-f]{8})\r?\n")
DIGIT = re.compile(rb"D,([0-9A-Fa-f]{4}),(\d{2}),(\d{3}),(\d{3}),(\d{3}),(\d{3}),([0-9]),([0-9A-Fa-f]{8})\r?\n")
END = re.compile(rb"E,([0-9A-Fa-f]{4})\r?\n")


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.pending = None
        self.current = None
        self.groups = []
        self.protocol_errors = []

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while True:
            if self.pending is None:
                token = TOKEN.search(self.buffer)
                if token is None:
                    if len(self.buffer) > 6:
                        del self.buffer[:-6]
                    return
                begin = token.start()
                if begin:
                    del self.buffer[:begin]
                end = self.buffer.find(b"\n")
                if end < 0:
                    if len(self.buffer) > 128:
                        del self.buffer[:6]
                        continue
                    return
                header = bytes(self.buffer[:end + 1])
                match = FRAME.fullmatch(header)
                if match:
                    fid, detected, stored, overflow, weight = match.groups()
                    if self.current is not None:
                        self.current["errors"].append("New F before matching E")
                    self.current = {
                        "frame_id": fid.decode().upper(), "detected_count": int(detected),
                        "stored_count": int(stored), "overflow": bool(int(overflow)),
                        "weight_crc32": weight.decode().upper(), "digits": [], "images": [],
                        "ended": False, "complete": False, "errors": [],
                    }
                    if int(stored) > 16 or int(detected) > 31 or int(stored) > int(detected):
                        self.current["errors"].append("Invalid record counts")
                    self.groups.append(self.current)
                    del self.buffer[:end + 1]
                    yield {"kind": "frame", "group": self.current}
                    continue
                match = DIGIT.fullmatch(header)
                if match:
                    fid, index, x, y, width, height, digit, crc = match.groups()
                    record = dict(zip(("index", "x", "y", "w", "h", "class"),
                                      map(int, (index, x, y, width, height, digit))))
                    record["patch_crc32"] = crc.decode().upper()
                    if self.current is None or fid.decode().upper() != self.current["frame_id"]:
                        self.protocol_errors.append("D without matching F: " + header.decode().strip())
                    else:
                        if record["index"] != len(self.current["digits"]):
                            self.current["errors"].append("Digit index out of order")
                        if not (0 < record["w"] <= 320-record["x"] and 0 < record["h"] <= 180-record["y"]):
                            self.current["errors"].append("Digit box outside 320x180")
                        self.current["digits"].append(record)
                        yield {"kind": "digit", "frame_id": self.current["frame_id"], "digit": record}
                    del self.buffer[:end + 1]
                    continue
                match = END.fullmatch(header)
                if match:
                    fid = match[1].decode().upper()
                    if self.current is None or fid != self.current["frame_id"]:
                        self.protocol_errors.append("E without matching F: " + fid)
                    else:
                        group = self.current
                        group["ended"] = True
                        if len(group["digits"]) != group["stored_count"]:
                            group["errors"].append("Digit record count mismatch")
                        if not group["overflow"] and group["detected_count"] != group["stored_count"]:
                            group["errors"].append("Unreported detections without overflow flag")
                        if [(im["image_id"], im["width"], im["height"]) for im in group["images"]] != [(0,320,180),(1,80,45)]:
                            group["errors"].append("Expected mask then thumbnail in this group")
                        group["complete"] = not group["errors"]
                        self.current = None
                        yield {"kind": "end", "group": group}
                    del self.buffer[:end + 1]
                    continue
                match = HEADER.fullmatch(header)
                if not match:
                    del self.buffer[:1]
                    continue
                ident, size, width, height, fmt = map(int, match.groups())
                expected = width * height if fmt == 24 else ((width + 31) // 32) * 4 * height
                if fmt not in (1, 24) or not (0 < width <= 2048 and 0 < height <= 2048) or size != expected:
                    del self.buffer[:end + 1]
                    continue
                self.pending = (ident, size, width, height, fmt)
                del self.buffer[:end + 1]
            ident, size, width, height, fmt = self.pending
            if len(self.buffer) < size:
                return
            payload = bytes(self.buffer[:size])
            del self.buffer[:size]
            self.pending = None
            if fmt == 24:
                picture = Image.frombytes("L", (width, height), payload)
            else:
                stride = ((width + 31) // 32) * 4
                picture = Image.frombytes("1", (width, height), payload, "raw", "1", stride).convert("L")
            metadata = describe(ident, picture)
            metadata["payload_crc32"] = f"{zlib.crc32(payload):08X}"
            metadata["frame_id"] = self.current["frame_id"] if self.current else None
            if self.current is not None:
                self.current["images"].append(metadata)
            yield {"kind": "image", "image": metadata, "picture": picture}


def describe(ident, picture):
    histogram = picture.histogram()
    count = picture.width * picture.height
    total = 0
    p75 = 255
    for value, n in enumerate(histogram):
        total += n
        if total * 4 >= count * 3:
            p75 = value
            break
    return {
        "image_id": ident, "width": picture.width, "height": picture.height,
        "min": picture.getextrema()[0], "max": picture.getextrema()[1],
        "mean": round(sum(i * n for i, n in enumerate(histogram)) / count, 3),
        "p75": p75,
        "black_fraction": round(histogram[0] / count, 6),
        "below_128_fraction": round(sum(histogram[:128]) / count, 6),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM16")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=22)
    ap.add_argument("--input", type=Path, help="Decode an existing raw UART capture instead of opening a serial port")
    ap.add_argument("--output-dir", type=Path)
    args = ap.parse_args()
    if args.seconds <= 0:
        ap.error("--seconds must be positive")
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    out = args.output_dir or Path(__file__).resolve().parents[1] / "cnn_captures" / stamp
    out.mkdir(parents=True, exist_ok=True)
    parser = FrameParser()
    records = []
    counts = {}

    def receive(chunk):
        for event in parser.feed(chunk):
            if event["kind"] == "frame":
                print("FRAME " + json.dumps({k:v for k,v in event["group"].items() if k not in ("digits", "images")}), flush=True)
                continue
            if event["kind"] == "digit":
                print("HARDWARE_DIGIT " + json.dumps(event), flush=True)
                continue
            if event["kind"] == "end":
                group = event["group"]
                if group["complete"]:
                    mask = next(im for im in group["images"] if im["image_id"] == 0)
                    with Image.open(out / mask["file"]) as original:
                        overlay = original.convert("RGB").resize((960,540), Image.Resampling.NEAREST)
                    draw = ImageDraw.Draw(overlay)
                    for d in group["digits"]:
                        x,y,w,h = (d[k]*3 for k in ("x","y","w","h"))
                        draw.rectangle((x,y,x+w-1,y+h-1), outline="red", width=2)
                        draw.text((x,max(0,y-12)), str(d["class"]), fill="red")
                    annotated = Image.new("RGB",(960,565),"white")
                    annotated.paste(overlay,(0,25))
                    ImageDraw.Draw(annotated).text((5,6),f"HARDWARE frame={group['frame_id']} weight={group['weight_crc32']} overflow={int(group['overflow'])}",fill="black")
                    group["overlay_file"] = f"hardware_frame_{group['frame_id']}_{len(parser.groups):04d}.png"
                    annotated.save(out / group["overlay_file"])
                print("FRAME_END " + json.dumps(group), flush=True)
                continue
            record, picture = event["image"], event["picture"]
            ident = record["image_id"]
            counts[ident] = counts.get(ident, 0) + 1
            label = {0: "mask_320x180", 1: "raw8_thumbnail"}.get(ident, f"image_{ident}")
            frame_label = f"frame_{record['frame_id']}_" if record["frame_id"] is not None else ""
            name = f"{frame_label}{label}_{counts[ident]:04d}.png"
            picture.save(out / name)
            record["file"] = name
            records.append(record)
            print(json.dumps(record), flush=True)

    if args.input:
        raw = args.input.read_bytes()
        # Exercise the same incremental parser used for the serial port.
        for offset in range(0, len(raw), 997):
            receive(raw[offset:offset + 997])
    else:
        import serial
        connection = serial.Serial(port=None, baudrate=args.baud, timeout=0.2)
        connection.port = args.port
        connection.dtr = False
        connection.rts = False
        raw = bytearray()
        try:
            connection.open()
            print(f"Receiving {args.port} {args.baud} 8N1 for {args.seconds:g}s -> {out}", flush=True)
            started = time.monotonic()
            while time.monotonic() - started < args.seconds:
                chunk = connection.read(max(1, connection.in_waiting))
                raw.extend(chunk)
                receive(chunk)
        except serial.SerialException as exc:
            print(f"Serial error: {exc}")
            return 2
        finally:
            connection.close()
        (out / "uart_capture.bin").write_bytes(raw)
    summary = {"bytes": len(raw), "images": records,
               "frames": parser.groups, "protocol_errors": parser.protocol_errors,
               "incomplete_payload": parser.pending,
               "pending_bytes": len(parser.buffer)}
    (out / "capture_report.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Saved {len(records)} complete images; {len(raw)} received bytes; report: {out / 'capture_report.json'}")
    if not records:
        print("No complete image frame. Close other serial applications and check the loaded bit/port/baud.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
