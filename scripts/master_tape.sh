#!/usr/bin/env bash
# Record the GCV2's game tape onto a real cassette: blue pre-roll, then position 0 to the end (or DURATION) in
# one continuous play over composite, then blue again. Flash the tape first (scripts/flash_rp2350.sh GAME.cart).
#   scripts/master_tape.sh [H:MM:SS | FIELDS]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/bench.sh"

preroll_s=3
tail_s=3
ctl() { "$here/gcv2_ctl.sh" "$@"; }
field() { ctl | awk -v k="$1" '$1 == k { print $2 }'; }
hms() { printf '%d:%02d:%02d' $(($1 / 3600)) $(($1 / 60 % 60)) $(($1 % 60)); }

status=$(ctl)
get() { awk -v k="$1" '$1 == k { print $2 }' <<<"$status"; }
(( $(get src) == 3 )) || die "GCV2 is not playing the SD card (ctl.src $(get src))"
(( $(get sd_ok) == 1 && $(get st_fails) == 0 )) || die "SD card not healthy (sd_ok $(get sd_ok), st_fails $(get st_fails))"
(( $(get data_ok) == 1 )) || die "no game image on the data leader (data_ok 0)"

tape_fields=$(get vcr_len)
if [[ ${1:-} =~ ^[0-9]+$ ]]; then
    end_fields=$1
    (( end_fields <= tape_fields )) || die "$1 fields is longer than the tape"
elif [[ -n ${1:-} ]]; then
    IFS=: read -r h m s <<<"$1"
    [[ $h =~ ^[0-9]+$ && $m =~ ^[0-9]+$ && $s =~ ^[0-9]+$ ]] || die "usage: $0 [H:MM:SS]"
    end_fields=$(( (10#$h * 3600 + 10#$m * 60 + 10#$s) * 60000 / 1001 ))
    (( end_fields <= tape_fields )) || die "$1 is longer than the tape"
else
    end_fields=$(( tape_fields - 600 ))
fi
end_s=$(( (end_fields * 1001 + 30000) / 60000 ))
echo "Recording $(hms "$end_s") of programme, plus ${preroll_s} s blue before and ${tail_s} s after: $(hms $((end_s + preroll_s + tail_s))) of an SP cassette."

ctl vcr 1
if [[ $(field deck) == OFF ]]; then ctl vcr POWER; fi
for _ in $(seq 20); do [[ $(field deck) == STOPPED ]] && break; sleep 0.5; done
[[ $(field deck) == STOPPED ]] || die "deck emulation did not reach STOPPED (blue screen)"

finish() {
    trap - INT TERM
    ctl vcr 1 vcr_cmd 1 || true
    echo "Blue tail for ${tail_s} s..."
    sleep "$tail_s"
    echo "Done: stop the recording."
}

read -rp "GCV2 is on blue. Start recording on the deck, then press Enter. " _
echo "Blue pre-roll for ${preroll_s} s..."
sleep "$preroll_s"
trap 'echo; echo "Interrupted."; finish; exit 130' INT TERM
ctl vcr 0 ltc_seek 1
echo "Playing from position 0."

while :; do
    pos=$(field vcr_pos)
    (( pos >= end_fields )) && break
    printf '\r%s / %s  (field %d of %d)' "$(hms $((pos * 1001 / 60000)))" "$(hms "$end_s")" "$pos" "$end_fields"
    left=$(( (end_fields - pos) * 1001 / 60000 ))
    sleep $(( left < 10 ? (left > 0 ? left : 1) : 10 ))
done
echo
finish
