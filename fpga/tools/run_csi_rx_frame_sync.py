"""Run the B-board RX/frame-sync and affected video regressions, no hardware access."""
from pathlib import Path
import argparse
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--tops', nargs='+')
a = p.parse_args()
root = Path(__file__).resolve().parent.parent
rtl = root / 'uisrc/01_rtl'
tests = root / 'uisrc/02_sim'
out = a.out.resolve()
out.mkdir(parents=True, exist_ok=True)
rx = [rtl / 'csi2/csi2_rx_parse.v', rtl / 'csi2/csi2_ecc.v']
tx = [rtl / 'csi2/csi2_tx_packet.v', rtl / 'csi2/csi2_ecc.v']
crop = rtl / 'csi2/csi2_raw10_scale_crop.v'
cache = [rtl / 'ddr/ddr_triple_frame_cache.v', rtl / 'csi2/csi2_ptr_fifo.v']
cases = {
    'tb_csi2_rx_shared_hs_probe': rx,
    'tb_csi2_rx_recovery': rx,
    'tb_csi2_rx_frame_sync': rx + [crop],
    'tb_csi2_rx_parse': rx,
    'tb_csi2_pipeline': rx + tx,
    'tb_csi2_raw10_scale_crop': [crop],
    'tb_csi2_scale_tx': rx + tx + [crop],
    'tb_ddr_triple_frame_cache': cache,
    'tb_csi2_crop_ddr_fanout': cache + [crop, rtl / 'cnn/cnn_raw10_resize_bin.v'],
    'tb_dbg_uart_frame_stats': [rtl / 'dbg/dbg_uart.v'],
}
for top in a.tops or cases:
    sources = list(dict.fromkeys(cases[top] + [tests / (top + '.v')]))
    executable = out / (top + '.vvp')
    compile_result = subprocess.run(
        ['iverilog', '-g2012', '-s', top, '-o', str(executable)] + list(map(str, sources)),
        capture_output=True, text=True, errors='replace')
    (out / (top + '_compile.log')).write_text(compile_result.stdout + compile_result.stderr)
    if compile_result.returncode:
        raise RuntimeError(f'{top} compile failed: {compile_result.stderr}')
    result = subprocess.run(['vvp', str(executable)], capture_output=True, text=True, errors='replace', timeout=120)
    log = result.stdout + result.stderr
    (out / (top + '.log')).write_text(log)
    # Several legacy benches print FAIL but still return zero; check both.
    if result.returncode or 'PASS' not in log or 'FAIL' in log or 'FATAL' in log:
        raise RuntimeError(f'{top} failed\n{log}')
    print(f'{top}: PASS', flush=True)
