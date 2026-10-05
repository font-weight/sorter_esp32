"""Compare independent Python and firmware codecs using a compiled native bridge."""
import argparse
from pathlib import Path
import random
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.protocol import (Packet, Scene, Detection, ProtocolError, crc16,
                            encode_packet, decode_packet, encode_scene)


def frame(body):
    return b"@" + body + f"*{crc16(body):04X}\n".encode("ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bridge", type=Path)
    args = parser.parse_args()
    rng = random.Random(20260923)
    cases = []
    for _ in range(200):
        session, seq, cal = (rng.randint(1, 2**32-1) for _ in range(3))
        cases.append(encode_packet(Packet("Q", session, seq, str(cal))))
        cases.append(encode_packet(Packet("E", session, seq, rng.choice(("BUSY", "STALE", "CALIBRATION_MISMATCH")))))
        scene = Scene(rng.randint(1, 2**32-1), cal, tuple(
            Detection(rng.randint(1, 3), rng.randint(-2**31, 2**31-1),
                      rng.randint(-2**31, 2**31-1), rng.randint(1, 2**32-1))
            for _ in range(rng.randint(0, 8))))
        cases.append(encode_packet(Packet("D", session, seq, encode_scene(scene))))
    # Corruption must agree in both implementations. Avoid newline as the bridge is line based.
    for good in tuple(cases):
        corrupt = bytearray(good)
        index = rng.randrange(1, len(corrupt)-1)
        corrupt[index] = 33 + ((corrupt[index]-33+1) % 94)
        cases.append(bytes(corrupt))
    bodies = [b"1|Q|00001|00002|00003", b"1|Q|1|1|4294967296", b"1|Q|1|1|-1",
              b"1|D|1|1|1,2,1;1,-0,0001,0005", b"1|D|1|1|1,2,1;4,0,0,1",
              b"1|D|1|1|1,2,0;", b"1|D|1|1|1,2,1;1,-2147483649,0,1",
              b"1|E|1|1|"+b"A"*32, b"1|E|1|1|"+b"A"*33,
              b"1|Q|1|1|"+b"0"*399+b"1", b"1|Q|1|1|"+b"0"*400+b"1",
              b"1|Q|1|1|2\x00", b"1|Q|1|1|2\x80", b"1|Q|1|1|+2"]
    cases.extend(frame(body) for body in bodies)
    cases.extend(case[:-1]+b"\r\n" for case in cases[:25])
    expected = []
    for case in cases:
        try:
            expected.append(encode_packet(decode_packet(case)).rstrip(b"\n"))
        except ProtocolError:
            expected.append(b"INVALID")
    result = subprocess.run([str(args.bridge.resolve())], input=b"".join(cases),
                            capture_output=True, check=True, timeout=30)
    actual = result.stdout.splitlines()
    if len(actual) != len(expected):
        raise AssertionError(f"Bridge returned {len(actual)} lines, expected {len(expected)}: {result.stderr!r}")
    for index, (got, want) in enumerate(zip(actual, expected)):
        if got != want:
            raise AssertionError(f"Codec disagreement at case {index}: {cases[index]!r}: {got!r} != {want!r}")
    print(f"Protocol interoperability: {len(cases)} cases passed (seed 20260923)")


if __name__ == "__main__":
    main()
