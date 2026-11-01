"""Replay one complete hardware UART frame in current RTL and compare evidence.

Agreement with RTL is an implementation check, not a recognition accuracy score.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import zlib

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ["uisrc/02_sim/tb_cnn_captured_mask.v",
           "uisrc/01_rtl/cnn/cnn_frame_digit_detector.v",
           "uisrc/01_rtl/cnn/cnn_digit_infer.v",
           "uisrc/01_rtl/cnn/cnn_digit_params.v"]
DIGIT = re.compile(r"RTL_DIGIT x=(\d+) y=(\d+) w=(\d+) h=(\d+) class=(\d+) patch_crc=([0-9a-fA-F]{8})")
END = re.compile(r"RTL_FRAME_DONE count=(\d+) reported=(\d+) weight_crc=([0-9a-fA-F]{8})")


def parse_rtl(log):
    digits = []
    for match in DIGIT.finditer(log):
        record = dict(zip(("x","y","w","h","class"), map(int,match.groups()[:5])))
        record["patch_crc32"] = match[6].upper()
        digits.append(record)
    endings = list(END.finditer(log))
    if len(endings) != 1 or "FATAL" in log or "ERROR" in log:
        raise ValueError("RTL log did not complete exactly one successful frame")
    final = endings[0]
    if int(final[2]) != len(digits):
        raise ValueError("RTL reported digit count does not match log")
    return {"detected_count":int(final[1]), "weight_crc32":final[3].upper(), "digits":digits}


def compare(group, rtl):
    differences = []
    for key in ("detected_count", "weight_crc32"):
        if group[key] != rtl[key]:
            differences.append({"field":key,"hardware":group[key],"rtl":rtl[key]})
    if len(group["digits"]) != len(rtl["digits"]):
        differences.append({"field":"record_count","hardware":len(group["digits"]),"rtl":len(rtl["digits"])})
    for index, (hw, sim) in enumerate(zip(group["digits"],rtl["digits"])):
        for key in ("x","y","w","h","patch_crc32","class"):
            if hw[key] != sim[key]:
                differences.append({"index":index,"field":key,"hardware":hw[key],"rtl":sim[key]})
    return differences


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("report",type=Path,help="capture_report.json produced by capture_cnn_uart.py")
    ap.add_argument("--frame-id",help="Hexadecimal report frame ID; default first complete non-overflow group")
    ap.add_argument("--timeout",type=float,default=600,help="RTL subprocess timeout in seconds")
    args = ap.parse_args()
    report_path = args.report.resolve()
    capture = json.loads(report_path.read_text(encoding="utf-8"))
    groups = [g for g in capture.get("frames",[]) if g.get("complete") and not g.get("overflow")]
    if args.frame_id is not None:
        wanted = f"{int(args.frame_id,16):04X}"
        groups = [g for g in groups if g["frame_id"] == wanted]
    if not groups:
        ap.error("No complete, non-overflow hardware report group selected; image-only R50 captures cannot prove hardware classification")
    if args.frame_id is not None and len(groups) > 1:
        ap.error("Frame ID appears more than once; split the capture before replaying a wrapped ID")
    group = groups[0]
    mask = next(im for im in group["images"] if im["image_id"] == 0)
    mask_path = (report_path.parent / mask["file"]).resolve()
    out = report_path.parent / f"rtl_compare_{group['frame_id']}"
    out.mkdir(exist_ok=True)
    with Image.open(mask_path) as picture:
        if picture.size != (320,180):
            ap.error("Mask must be 320x180")
        pixels = picture.convert("L").tobytes()
    if set(pixels)-{0,255}:
        ap.error("Expected the actual binary mask containing only 0 and 255")
    if mask.get("payload_crc32") != f"{zlib.crc32(pixels):08X}":
        ap.error("Mask PNG content differs from the captured UART payload CRC")
    image_hex = out / "mask.hex"
    image_hex.write_text("\n".join(f"{v:02x}" for v in pixels)+"\n",encoding="ascii")
    iverilog = shutil.which("iverilog")
    vvp = shutil.which("vvp")
    if not iverilog or not vvp:
        ap.error("iverilog and vvp must be installed and on PATH")
    executable = out / "replay.out"
    compile_command = [iverilog,"-g2012","-DCNN_SIM","-s","tb_cnn_captured_mask","-o",str(executable),*SOURCES]
    compiled = subprocess.run(compile_command,cwd=ROOT,capture_output=True,text=True,timeout=60)
    (out/"compile.log").write_text(compiled.stdout+compiled.stderr,encoding="utf-8")
    if compiled.returncode:
        raise RuntimeError(f"RTL compilation failed; see {out/'compile.log'}")
    print(f"Replaying hardware frame={group['frame_id']} (RTL simulation can take several minutes)",flush=True)
    started = time.monotonic()
    # The weight HEX uses a path relative to ROOT. Forward slash keeps +IMAGE portable on Windows.
    simulated = subprocess.run([vvp,str(executable),f"+IMAGE={image_hex.as_posix()}"],cwd=ROOT,
                               capture_output=True,text=True,timeout=args.timeout)
    log = simulated.stdout+simulated.stderr
    (out/"rtl.log").write_text(log,encoding="utf-8")
    if simulated.returncode:
        raise RuntimeError(f"RTL simulation failed; see {out/'rtl.log'}")
    rtl = parse_rtl(log)
    differences = compare(group,rtl)
    fingerprints = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest().upper() for name in SOURCES}
    result = {
        "origin":"Hardware F/D records compared with current RTL replay of their same-frame captured mask",
        "frame_id":group["frame_id"],"mask_file":str(mask_path),"report":str(report_path),
        "hardware":group,"rtl":rtl,"match":not differences,"differences":differences,
        "elapsed_seconds":round(time.monotonic()-started,2),"current_source_sha256":fingerprints,
        "accuracy_note":"Implementation agreement does not establish the handwritten ground-truth class. Current RTL sources must correspond to the downloaded bit.",
    }
    (out/"comparison.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps({"frame_id":group["frame_id"],"match":not differences,"differences":differences,"output":str(out)},indent=2))
    return 0 if not differences else 1


if __name__ == "__main__":
    raise SystemExit(main())
