#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
selfcheck_crc.py
  在 Python 里直接验证 gen_crc_par2.py 生成的掩码是否与串行 CRC 等价。
  这样不用反复跑 ModelSim，几十毫秒就能定位问题。
"""
import random
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("gen", os.path.join(HERE, "gen_crc_par2.py"))
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def serial_crc(state, b0, b1):
    """严格照抄 crc16_csi.v 的位串行行为：先 b0，再 b1"""
    c = state
    for byte in (b0, b1):
        for i in range(7, -1, -1):
            b = ((c >> 15) & 1) ^ ((byte >> i) & 1)
            c = (c << 1) & 0xFFFF
            if b:
                c ^= 0x1021
    return c


def par_crc_from_masks(masks, state, data):
    V = (data << 16) | state          # V[15:0]=crc, V[31:16]=data
    out = 0
    for i, m in enumerate(masks):
        if bin(V & m).count("1") & 1:
            out |= (1 << i)
    return out


def main():
    masks = gen.build()
    random.seed(1)
    bad = 0
    for _ in range(20000):
        st = random.getrandbits(16)
        w = random.getrandbits(16)
        b0 = (w >> 8) & 0xFF
        b1 = w & 0xFF
        ref = serial_crc(st, b0, b1)
        got = par_crc_from_masks(masks, st, w)
        if ref != got:
            if bad < 5:
                print("MISMATCH state=%04x data=%04x ref=%04x got=%04x" % (st, w, ref, got))
            bad += 1
    if bad == 0:
        print("PASS: parallel masks match bit-serial CRC on 20000 random vectors")
    else:
        print("FAIL: %d / 20000 mismatches" % bad)

    # 再跑一遍标准校验向量：123456789 补一个 0x00 凑成整字
    st = 0xFFFF
    ref = serial_crc(st, ord("1"), ord("2"))
    got = par_crc_from_masks(masks, st, (ord("1") << 8) | ord("2"))
    print("vector check:", "OK" if ref == got else "BAD")


if __name__ == "__main__":
    main()
