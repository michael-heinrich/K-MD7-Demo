#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run this script yourself with sudo: sudo $0 [username]" >&2
    exit 1
fi

target_user=${1:-${SUDO_USER:-}}
if [[ ! $target_user =~ ^[a-z_][a-z0-9_-]*\$?$ ]] || ! id "$target_user" &>/dev/null; then
    echo "Specify an existing Linux username: sudo $0 username" >&2
    exit 1
fi

rule_file=$(mktemp)
trap 'rm -f "$rule_file"' EXIT
cat > "$rule_file" <<EOF
# Give $target_user access to all matching Teensy serial, HID and bootloader devices.
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", OWNER:="$target_user", MODE:="0600"
KERNEL=="hidraw*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", OWNER:="$target_user", MODE:="0600"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", OWNER:="$target_user", MODE:="0600"
KERNEL=="hidraw*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", OWNER:="$target_user", MODE:="0600"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", OWNER:="$target_user", MODE:="0600"
EOF

install -m 0644 "$rule_file" /etc/udev/rules.d/00-kmd7-teensy.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=16c0 --attr-match='idProduct=04*'
udevadm settle
echo "Installed persistent Teensy device access for $target_user."
