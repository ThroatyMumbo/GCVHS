#!/usr/bin/env bash
# Build (unless --no-build) and flash a vcr_sim target (vhs_pattern, or vhs_streams) to the GCV2 over SWD.
# `clips` also bakes videos/ and flashes it at 0x10040000 for vhs_streams (the default target then);
# `data` bakes the mock game (tools/datatape.py) and flashes it at 0x10F00000 the same way; GAME.cart bakes that
# game instead, onto the tape's data leader, where the console's BIOS loads it from; leader=N sets that
# leader's length in fields (default 600, which Night Trap's positions assume); programme=clips|sd pins what
# follows the leader, whatever card is in. fault=header|crc bakes the cart broken, to test the BIOS error
# paths (datatape.py --fault). sched=FILE flashes the schedule an asset-store card plays (tools/tapestore.py).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
source "$here/bench.sh"

proj_dir="$root/vcr_sim"
build_dir="$proj_dir/build"
target_name=
clips_bin="$build_dir/clips.bin"
clips_addr=0x10040000
data_bin="$build_dir/datatape.bin"
data_addr=0x10F00000

cart_file=
leader_fields=0
programme=
fault=
sched_file=
sched_addr=0x10030000

do_build=1
want_clips=0
want_data=0
for arg in "$@"; do
    case "$arg" in
        --no-build) do_build=0 ;;
        clips) want_clips=1 ;;
        data) want_data=1 ;;
        *.cart) cart_file=$arg want_data=1 ;;
        leader=*) leader_fields=${arg#leader=} ;;
        programme=*) programme=${arg#programme=} ;;
        fault=*) fault=${arg#fault=} ;;
        sched=*) sched_file=${arg#sched=} ;;
        vhs_*) target_name=$arg ;;
        *) die "unknown option: $arg" ;;
    esac
done
[[ -n $target_name ]] || { (( want_clips || want_data )) && target_name=vhs_streams || target_name=vhs_pattern; }
elf="$build_dir/$target_name.elf"

if (( do_build )); then
    [[ -f $build_dir/CMakeCache.txt ]] || cmake -S "$proj_dir" -B "$build_dir" -G Ninja
    cmake --build "$build_dir" --target "$target_name"
    (( want_clips )) && cmake --build "$build_dir" --target clips_bin
    (( want_data )) && [[ -z $cart_file ]] && cmake --build "$build_dir" --target datatape_bin
fi
if [[ -n $cart_file ]]; then
    [[ -f $cart_file ]] || die "$cart_file missing"
    data_bin="$build_dir/cart_tape.bin"
    "$root/.venv/bin/python" "$root/tools/datatape.py" bake "$cart_file" -o "$data_bin" --leader "$leader_fields" \
        ${programme:+--programme "$programme"} ${fault:+--fault "$fault"}
fi
program() {
    oocd "$PROBE_RP2350" rp2350.cfg -c "program $*" && return
    echo "flash_rp2350: target would not halt; retrying through the bootrom rescue reset" >&2
    OOCD_PRE=(-c "set RESCUE 1")
    oocd "$PROBE_RP2350" rp2350.cfg -c init -c exit
    OOCD_PRE=()
    oocd "$PROBE_RP2350" rp2350.cfg -c "program $*"
}

[[ -f $elf ]] || die "$elf missing"
if (( want_clips )); then
    [[ -f $clips_bin ]] || die "$clips_bin missing"
    program "$clips_bin $clips_addr verify exit"
fi
if (( want_data )); then
    [[ -f $data_bin ]] || die "$data_bin missing"
    program "$data_bin $data_addr verify exit"
fi
if [[ -n $sched_file ]]; then
    [[ -f $sched_file ]] || die "$sched_file missing"
    program "$sched_file $sched_addr verify exit"
fi
program "$elf verify reset exit"
