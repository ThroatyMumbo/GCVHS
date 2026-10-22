# Sourced by the other scripts: probe serials are stable, ttyACM numbers are not.
# Machine setup comes from the environment: OPENOCD_DIR, the PROBE_* serials, QUARTUS_BIN (else on PATH).
OPENOCD_DIR="${OPENOCD_DIR:-}"
PROBE_RP2350="${PROBE_RP2350:-}"
PROBE_CPU="${PROBE_CPU:-}"
SWD_KHZ="${SWD_KHZ:-5000}"
[[ -z ${QUARTUS_BIN:-} ]] || PATH="$QUARTUS_BIN:$PATH"

die() { echo "error: $*" >&2; exit 1; }

oocd() {
    local serial=$1 target=$2
    shift 2
    [[ -x $OPENOCD_DIR/src/openocd ]] || die "set OPENOCD_DIR to an openocd tree with src/openocd and tcl/"
    [[ -n $serial ]] || die "no debugprobe serial: set PROBE_CPU or PROBE_RP2350"
    [[ -e /dev/serial/by-id/usb-Raspberry_Pi_Debugprobe_on_Pico__CMSIS-DAP__${serial}-if01 ]] \
        || die "debugprobe $serial not found in /dev/serial/by-id"
    # OOCD_PRE: commands that must come before the target config, e.g. (-c "set RESCUE 1").
    "$OPENOCD_DIR/src/openocd" -s "$OPENOCD_DIR/tcl" \
        -f interface/cmsis-dap.cfg -c "adapter serial $serial" ${OOCD_PRE[@]+"${OOCD_PRE[@]}"} -f "target/$target" \
        -c "adapter speed $SWD_KHZ" "$@"
}
