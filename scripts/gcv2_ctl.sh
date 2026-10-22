#!/usr/bin/env bash
# Read or set vhs_streams' ctl block over SWD while it runs.
#   gcv2_ctl.sh                         print ctl, with fields/s and underruns/s over one second
#   gcv2_ctl.sh NAME VALUE [NAME VALUE]  set fields, e.g. t_log2 2 layout 2 (4 fields x quads: 16
#                                       streams; layouts full, halves, quads, lines), vhs 1, c_bw_khz 400;
#                                       see ctl_t in streams.c
#   gcv2_ctl.sh seek H:MM:SS[:FF]|FRAME  move the simulated tape (LTC, tag count, clips) there; seek 0 rewinds
#   gcv2_ctl.sh vcr POWER|PLAY|PAUSE|STOP|FF|REW  press a deck key, as if sent by IR; vcr 0 / vcr 1 bypass / emulate;
#                                       vcr_fast 1: FF/REW reach the end of the tape at once (debug)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
source "$here/bench.sh"

elf="$root/vcr_sim/build/vhs_streams.elf"
[[ -f $elf ]] || die "$elf missing"
base=$(arm-none-eabi-nm "$elf" | awk '$3 == "ctl" { print "0x" $1 }')
[[ -n $base ]] || die "no ctl symbol in $elf"

fields=(magic t_log2 layout interlaced fields underruns active_t_log2 active_layout active_interlaced
        plain guard_px guard_rows
        vhs y_bw_khz c_bw_khz c_poles c_delay_ns c_comb y_noise c_noise hs_lines hs_shift_px cycles_max cycles_avg pad_connected pad_buttons pad_edges
        aud aud_level aud_lines aud_mid16 aud_lin aud_lead aud_cycles src clips_ok clip_fields hs_blank
        data_ok data_width data_rows data_black data_white data_drop data_burst data_skew data_packets data_sent
        data_parity ltc ltc_amp ltc_frame ltc_user left_hz left_amp ltc_seek vitc ptag
        vcr vcr_state vcr_pos vcr_len vcr_cmd ir_frames ir_bad ir_last vbi_from vbi_keep tag_keep vbi_min vbi_max leader
        vcr_cass vcr_saves sd_ok sd_fields sd_div sd_init sd_reads sd_crc sd_timeouts sd_cmd_err sd_miss sd_stage sd_resp sd_hdr_magic sd_fail sd_test sd_test_bad vcr_fast
        st_assets st_entries st_dead st_reads st_fails)
# vcr.h: the Kaseikyo command bytes (plus the front panel's EJECT and INSERT), VS_* states and cassettes
declare -A vcr_keys=([STOP]=0 [REW]=2 [FF]=3 [PAUSE]=6 [PLAY]=10 [POWER]=61 [EJECT]=200 [INSERT]=201)
vcr_states=(OFF POWERING STOPPED THREADING PLAY PAUSE CUE_FWD CUE_REV RESUME STOPPING BLIP SPINUP_FWD SPINUP_REV
            WIND_FWD WIND_REV WIND_STOP POWERING_OFF LOADING EJECTING)
vcr_cassettes=(none game movie)
offset_of() {
    local i
    for i in "${!fields[@]}"; do [[ ${fields[i]} == "$1" ]] && { printf '0x%x' $((base + 4 * i)); return; }; done
    die "unknown field: $1"
}

if [[ ${1:-} == seek ]]; then
    (( $# == 2 )) || die "usage: $0 seek H:MM:SS[:FF]|FRAME"
    IFS=: read -r -a t <<<"$2"
    for v in "${t[@]}"; do [[ $v =~ ^[0-9]+$ ]] || die "seek: bad position $2"; done
    case ${#t[@]} in
        1) frame=$((10#${t[0]})) ;;
        3) frame=$(( ((10#${t[0]} * 60 + 10#${t[1]}) * 60 + 10#${t[2]}) * 30 )) ;;
        4) frame=$(( ((10#${t[0]} * 60 + 10#${t[1]}) * 60 + 10#${t[2]}) * 30 + 10#${t[3]} )) ;;
        *) die "seek: H:MM:SS[:FF] or a frame number" ;;
    esac
    set -- ltc_seek $((frame + 1))
fi
if [[ ${1:-} == vcr && ${2:-} =~ ^[A-Za-z]+$ ]]; then
    key=${2^^}
    [[ -n ${vcr_keys[$key]:-} ]] || die "vcr: one of ${!vcr_keys[*]}"
    set -- vcr_cmd $((vcr_keys[$key] + 1))
fi

if (( $# > 0 )); then
    (( $# % 2 == 0 )) || die "usage: $0 [NAME VALUE ...]"
    cmds=()
    while (( $# )); do
        [[ $2 =~ ^[0-9]+$ ]] || die "$1: value must be a non-negative integer"
        cmds+=(-c "mww $(offset_of "$1") $2")
        shift 2
    done
    out=$(oocd "$PROBE_RP2350" rp2350.cfg -c init "${cmds[@]}" -c exit 2>&1) || { echo "$out" >&2; exit 1; }
    exit 0
fi

out=$(oocd "$PROBE_RP2350" rp2350.cfg -c init \
    -c "mdw $base ${#fields[@]}" -c "sleep 1000" -c "mdw $base ${#fields[@]}" -c exit 2>&1) \
    || { echo "$out" >&2; exit 1; }
blocks=$(grep -cE "^0x[0-9a-f]+:" <<<"$out")
per=$((blocks / 2))
mapfile -t a < <(grep -E "^0x[0-9a-f]+:" <<<"$out" | head -n "$per" | cut -d: -f2 | tr -s ' ' '\n' | sed '/^$/d')
mapfile -t b < <(grep -E "^0x[0-9a-f]+:" <<<"$out" | tail -n "$per" | cut -d: -f2 | tr -s ' ' '\n' | sed '/^$/d')
(( ${#b[@]} == ${#fields[@]} )) || { echo "$out" >&2; die "unexpected openocd output"; }
for i in "${!fields[@]}"; do
    printf '%-18s %d\n' "${fields[i]}" "$((16#${b[i]}))"
done
index_of() { local i; for i in "${!fields[@]}"; do [[ ${fields[i]} == "$1" ]] && { echo "$i"; return; }; done; }
f=$(index_of fields) u=$(index_of underruns)
printf '%-18s %d\n' "fields/s" "$(( 16#${b[f]} - 16#${a[f]} ))"
printf '%-18s %d\n' "underruns/s" "$(( 16#${b[u]} - 16#${a[u]} ))"
s=$(index_of vcr_state)
printf '%-18s %s\n' "deck" "${vcr_states[16#${b[s]}]:-?}"
c=$(index_of vcr_cass)
printf '%-18s %s\n' "cassette" "${vcr_cassettes[16#${b[c]}]:-?}"
