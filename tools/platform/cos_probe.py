#!/usr/bin/env python3
"""Print cos(TAU/3) bits as this host's libm computes them (the int7 one-ulp case)."""
import math
import platform
import struct

x = math.cos(math.tau / 3)
bits = struct.unpack("<Q", struct.pack("<d", x))[0]
print(f"{platform.system()}-{platform.machine()} cos(TAU/3) = {x.hex()} bits {bits:#018x}")
