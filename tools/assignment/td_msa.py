#!/usr/bin/env python3
"""Method of successive averages on time-dependent link costs.

Averages the experienced link travel times of iterations 1..k (TdCostTable
files written by nomad_cli via output.td_costs_out) cell by cell:
C_k = C_{k-1} + (M_k - C_{k-1}) / k = mean(M_1..M_k). A cell with no data in
an iteration is skipped for that iteration; cells with no data at all stay
NaN (nomad then uses free flow). Writes the result in the same format, to be
read via routing.td_costs_in.

usage: td_msa.py OUT M_1 [M_2 ...]
"""
import struct
import sys

import numpy as np

HDR = struct.Struct("<4sIIIfd")


def load(path):
    b = open(path, "rb").read()
    magic, ver, E, B, bin_s, t0 = HDR.unpack_from(b, 0)
    if magic != b"NTDC" or ver != 1:
        raise ValueError(f"{path}: not a TdCostTable v1")
    o = HDR.size
    tt = np.frombuffer(b, "<f4", E * B, o)
    n = np.frombuffer(b, "<u4", E * B, o + 4 * E * B)
    return (E, B, bin_s, t0), tt, n


def save(path, meta, tt, n):
    E, B, bin_s, t0 = meta
    with open(path, "wb") as f:
        f.write(HDR.pack(b"NTDC", 1, E, B, bin_s, t0))
        f.write(tt.astype("<f4").tobytes())
        f.write(n.astype("<u4").tobytes())


def main(argv):
    if len(argv) < 3:
        sys.exit(__doc__)
    out, inputs = argv[1], argv[2:]
    meta = None
    acc = cnt = nsum = None
    for p in inputs:
        m, tt, n = load(p)
        if meta is None:
            meta = m
            acc = np.zeros(tt.size)
            cnt = np.zeros(tt.size, np.uint32)
            nsum = np.zeros(tt.size, np.uint64)
        elif m[:3] != meta[:3]:
            sys.exit(f"{p}: shape/bin mismatch {m} vs {meta}")
        ok = n > 0
        acc[ok] += tt[ok]
        cnt[ok] += 1
        nsum += n
    tt = np.full(acc.size, np.nan, np.float32)
    ok = cnt > 0
    tt[ok] = acc[ok] / cnt[ok]
    save(out, meta, tt, np.minimum(nsum, 2**32 - 1))
    print(f"td_msa: {len(inputs)} iteration(s), {ok.sum()} cells with data -> {out}")


if __name__ == "__main__":
    main(sys.argv)
