#!/usr/bin/env bash
# Read the chipset's JTAG stats, or choose the stream shown.
#   de1_ctl.sh                      print stats
#   de1_ctl.sh stream N [mosaic]    show stream N (mosaic: raw slot N, no crop)
#   de1_ctl.sh raw                  every field as it arrives, as a plain TV would show it
#   de1_ctl.sh flip N               stream N, toggling its LSB every frame (switching stress test)
#   de1_ctl.sh keys                 hand control back to the keys
#   de1_ctl.sh audio [N|follow] [tape|notape] [line|noline] [strip|nostrip]
#                                   in-line audio stream N (0-7) or the video's; mute the tape audio
#                                   or the codec line-in; show the audio cells on the VGA
# Source bits, MSB first: audio (tape off, line-in off, strip, own stream, stream[2:0]), then raw,
# reserved, use_jtag, flip, mosaic, stream[3:0] (see video_top; the decoder combs are on KEY[3], reported as comb_off).
# Each command leaves the other group as is.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
source "$here/bench.sh"

bits4() { local n=$1; echo "$(( n >> 3 & 1 ))$(( n >> 2 & 1 ))$(( n >> 1 & 1 ))$(( n & 1 ))"; }
stream_ok() { [[ ${1:-} =~ ^([0-9]|1[0-5])$ ]] || die "stream 0-15"; }
has() { local w; for w in "${@:2}"; do [[ $w == "$1" ]] && return 0; done; return 1; }
src=""
case "${1:-}" in
    "") ;;
    stream) stream_ok "${2:-}"
            mo=0
            has mosaic "$@" && mo=1
            src="0010${mo}$(bits4 "$2")" ;;
    raw)    src="101000000" ;;
    flip)   stream_ok "${2:-}"; src="00110$(bits4 "$2")" ;;
    keys)   src="000000000" ;;
    audio)  a="xxxxxxx"
            for w in "${@:2}"; do
                case $w in
                    [0-7])  a="${a:0:3}1$(bits4 "$w" | cut -c2-)" ;;
                    follow) a="${a:0:3}0000" ;;
                    tape)   a="0${a:1}" ;;
                    notape) a="1${a:1}" ;;
                    line)   a="${a:0:1}0${a:2}" ;;
                    noline) a="${a:0:1}1${a:2}" ;;
                    strip)  a="${a:0:2}1${a:3}" ;;
                    nostrip) a="${a:0:2}0${a:3}" ;;
                    *) die "audio: N (0-7), follow, tape, notape, line, noline, strip, nostrip" ;;
                esac
            done ;;
    *) die "usage: $0 [stream N [mosaic] | raw | flip N | keys | audio ...]" ;;
esac

# Control passes on a change of the source value, so first write it with the flip bit inverted: a
# change whose effect lasts a few milliseconds at most.
cmds=()
if [[ -n $src ]]; then
    cmds=("src xxxxxxx${src:0:3}$(( 1 - ${src:3:1} ))${src:4}" "src xxxxxxx$src")
elif [[ -n ${a:-} ]]; then
    cmds=("src ${a}xxxxxxxxx")
fi
out=$(quartus_stp -t "$root/chipset/probe.tcl" DE-SoC VSTA "${cmds[@]}" read 2>&1) \
    || { echo "$out" >&2; exit 1; }
bits=$(awk '/^BITS/ { print $2 }' <<<"$out")
[[ ${#bits} == 337 ]] || { echo "$out" >&2; die "expected 337 probe bits, got ${#bits}"; }

fields=(codec_err:1 aud_fill:9 aud_underruns:16 aud_overruns:16 aud_peak:16
        bist_ok:1 sd_lat:3 bist_counts:50 status1:8 cfg_done:1 i2c_err:1
        aud_stream:3 aud_rate:16 aud_no_pilot:16 aud_cell0:11 aud_pilot:12
        raw:1 comb_off:1 slot_shown:2 stream:4 tag_bad:16 tag_seen:1 tag_scheme:4 tag_count:8
        fields_per_sec:8 lines_f0:10 lines_f1:10 active_f0:10 active_f1:10 active_bytes:12
        line_min:12 line_max:12 parity_errs:16 bad_codes:16 codes_seen:1 parity_ok:1 frozen:1 blue:1)
layouts=(full halves quads lines)
i=0
for f in "${fields[@]}"; do
    name=${f%%:*} w=${f##*:}
    val=$((2#${bits:i:w}))
    i=$((i + w))
    case $name in
        bist_counts) continue ;;
        status1) printf '%-16s 0x%02x\n' "$name" "$val" ;;
        tag_scheme) printf '%-16s T=%d %s\n' "$name" $(( 1 << (val >> 2) )) "${layouts[val & 3]}" ;;
        tag_count) printf '%-16s %d%s\n' "$name" $(( val & 127 )) "$( (( val & 128 )) && echo " (240p)")" ;;
        *) printf '%-16s %d\n' "$name" "$val" ;;
    esac
done
