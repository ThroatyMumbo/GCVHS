#!/usr/bin/env bash
# Simulate one chipset block in Questa against its tools/ stimulus, and score it. ARGS go to the tool's stim, e.g.
#   run.sh data --width 5 --drop 0.01 --burst 3 --noise 4 --skew 6 --jitter 0.5   (data path, tb_data.v)
#   run.sh cpu                                                         (game CPU link, tb_cpu.v)
#   run.sh ltc --noise 800 --audio-delay 3 --dropout 1500:300 --splice 60:500
#   run.sh ptag --frag 0.8 --piece-min 150 --garbage 0.3 --noise 4
#   run.sh vitc --bw-khz 2000 --noise 4 --gain 0.2 --shift 1
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$(dirname "$here")")"
[[ -z ${QUESTA_BIN:-} ]] || PATH="$QUESTA_BIN:$PATH"   # else Questa on PATH; it needs LM_LICENSE_FILE

cd "$here"
name=${1:-}
(( $# )) && shift
# The stimulus tool, whether it also scores the output (data's testbench checks itself), and the sources.
do_score=1
case $name in
    data) tool=datatape.py do_score=0
          src=(tb_data.v ../src/data_slicer.v ../src/data_loader.v ../src/data_verify.v ../src/sdram_ctrl.v
               ../src/dc_ram.v ../src/req_cdc.v) ;;
    cpu)  tool=cpulink.py src=(tb_cpu.v ../src/cpu/*.v) ;;
    ltc)  tool=ltc.py src=(tb_ltc.v ../src/ltc_rx.v ../src/tc_frames.v ../src/peak_meter.v) ;;
    ptag) tool=ptag.py src=(tb_ptag.v ../src/ptag_slicer.v ../src/tag_slicer.v) ;;
    vitc) tool=vitc.py src=(tb_vitc.v ../src/vitc_slicer.v ../src/tag_slicer.v) ;;
    *) echo "usage: $0 data|cpu|ltc|ptag|vitc [stim args]" >&2; exit 1 ;;
esac

"$root/.venv/bin/python" "$root/tools/$tool" stim --out-dir out "$@"
lib=work_$name
rm -rf "$lib"
vlib "$lib" >/dev/null
vlog -quiet -work "$lib" +incdir+. +incdir+../src +incdir+../src/cpu "${src[@]}"
vsim -c -quiet -lib "$lib" "tb_$name" -do "run -all; quit -f" | grep -vE "^# (Loading|//)|^Reading" || true
if (( do_score )); then
    "$root/.venv/bin/python" "$root/tools/$tool" score --out-dir out
fi
