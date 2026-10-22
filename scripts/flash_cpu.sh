#!/usr/bin/env bash
# Build (unless --no-build) and flash the main-CPU BIOS over SWD. Games load off the tape: put one on the
# GCV2's data leader with scripts/flash_rp2350.sh GAME.cart.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
source "$here/bench.sh"

proj_dir="$root/cpu"
build_dir="$proj_dir/build"
elf="$build_dir/bios.elf"

do_build=1
for arg in "$@"; do
    case "$arg" in
        --no-build) do_build=0 ;;
        *) die "unknown option: $arg" ;;
    esac
done

if (( do_build )); then
    [[ -f $build_dir/CMakeCache.txt ]] || cmake -S "$proj_dir" -B "$build_dir" -G Ninja
    cmake --build "$build_dir" --target bios
fi
[[ -f $elf ]] || die "$elf missing"
if ! oocd "$PROBE_CPU" rp2350.cfg -c "program $elf verify reset exit"; then
    echo "flash_cpu: target would not halt; retrying through the bootrom rescue reset" >&2
    OOCD_PRE=(-c "set RESCUE 1")
    oocd "$PROBE_CPU" rp2350.cfg -c init -c exit
    OOCD_PRE=()
    oocd "$PROBE_CPU" rp2350.cfg -c "program $elf verify reset exit"
fi
