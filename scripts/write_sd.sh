#!/usr/bin/env bash
# Write a raw SD tape image (tools/clips.py --sd) to the whole card in the USB reader, then read it back.
#   sudo SD_READER=NAME scripts/write_sd.sh IMAGE
# The card is found by the reader's by-id name (SD_READER), and only a removable USB disk is written.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/bench.sh"

SD_READER="${SD_READER:-}"
image=${1:-}
[[ -n $image && -f $image ]] || die "usage: sudo SD_READER=NAME $0 IMAGE"
[[ -n $SD_READER ]] || die "set SD_READER to the reader's name in /dev/disk/by-id (e.g. usb-Generic_SD_Reader)"
(( EUID == 0 )) || die "needs root: sudo $0 $image"

mapfile -t links < <(ls /dev/disk/by-id/"$SD_READER"* 2>/dev/null | grep -v -- '-part[0-9]*$' || true)
(( ${#links[@]} == 1 )) || die "expected one disk matching /dev/disk/by-id/$SD_READER*, found ${#links[@]}"
disk=$(readlink -f "${links[0]}")
name=${disk#/dev/}
[[ $(cat "/sys/block/$name/removable") == 1 ]] || die "$disk is not removable"
[[ $(lsblk -dno TRAN "$disk") == usb ]] || die "$disk is not on USB"
size=$(blockdev --getsize64 "$disk")
(( size < 256 * 2**30 )) || die "$disk is $((size >> 30)) GiB: not an SD card"
(( $(stat -c %s "$image") <= size )) || die "$image is larger than the card"

echo "writing $image to $disk ($(lsblk -dno MODEL "$disk" | xargs), $((size >> 30)) GiB)"
for part in $(lsblk -lno PATH,MOUNTPOINT "$disk" | awk 'NF == 2 { print $1 }'); do umount "$part"; done
dd if="$image" of="$disk" bs=4M conv=fsync status=progress
sync
echo "verifying"
cmp -n "$(stat -c %s "$image")" "$image" "$disk" && echo "ok: $disk holds $image"
