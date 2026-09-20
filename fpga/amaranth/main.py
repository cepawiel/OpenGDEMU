#!/usr/bin/env python3
"""Generate Verilog for the OpenGDEMU FPGA core.

Run from the fpga/amaranth directory:

    amaranth_env/bin/python3 main.py

Output goes to ./out/. The Quartus project picks up the generated file
via fpga/Quartus/OpenGDEMU.qsf.
"""

import os
import sys

# Generate with the yosys amaranth ships rather than whatever the host has
# installed. Both emit Verilog that Quartus fits to the same bitstream, but
# they format it differently, so a system yosys makes this file churn on every
# distro upgrade and disagree with the container. Set before importing the
# backend, which resolves yosys at import time.
os.environ.setdefault("AMARANTH_USE_YOSYS", "builtin")

from amaranth.back import verilog  # noqa: E402

from hw.OpenGDEMU import OpenGDEMUCore


OUT_DIR = "out"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)

    core = OpenGDEMUCore()
    rtl = verilog.convert(core, name="OpenGDEMUCore", ports=core.ports(), emit_src=False)
    out_path = os.path.join(OUT_DIR, "OpenGDEMUCore.v")
    with open(out_path, "w") as f:
        f.write(rtl)
    print(f"wrote {out_path} ({len(rtl)} bytes)", file=sys.stderr)


if __name__ == "__main__":
    main()
