#!/usr/bin/env bash
# Load the chipset into the DE1-SoC FPGA over JTAG (volatile: redo after every power cycle).
#   flash_de1.sh            load chipset/build/video.sof
#   flash_de1.sh --build    compile first (a few minutes)
#   flash_de1.sh foo.sof    load another bitstream
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
source "$here/bench.sh"

sof="$root/chipset/build/video.sof"
do_build=0
for arg in "$@"; do
    case "$arg" in
        --build) do_build=1 ;;
        *.sof) sof="$arg" ;;
        *) die "usage: $0 [--build] [file.sof]" ;;
    esac
done

if (( do_build )); then
    (cd "$root/chipset" && quartus_sh --flow compile video)
fi
[[ -f $sof ]] || die "$sof missing (run with --build)"
# The FPGA is device 2 on the chain, after the HPS.
quartus_pgm -c "DE-SoC" -m JTAG -o "p;$sof@2"
