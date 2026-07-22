#!/bin/sh
# Prefer an available Wi-Fi uplink while keeping the 4G interface ready as fallback.

set -u

WIFI_IFACE="${EGGY_WIFI_IFACE:-wlan0}"
MODEM_IFACE="${EGGY_MODEM_IFACE:-enx020c29a39b6d}"
WIFI_METRIC="${EGGY_WIFI_METRIC:-40}"
MODEM_ACTIVE_METRIC="${EGGY_MODEM_ACTIVE_METRIC:-500}"
MODEM_FALLBACK_METRIC="${EGGY_MODEM_FALLBACK_METRIC:-100}"

device_gateway() {
    nmcli -g IP4.GATEWAY device show "$1" 2>/dev/null \
        | sed -n '/./{s/[[:space:]]//g;p;q;}'
}

device_connected() {
    [ "$(nmcli -g GENERAL.STATE device show "$1" 2>/dev/null \
        | sed -n '1{s/[^0-9].*//;p;}')" = "100" ]
}

replace_device_defaults() {
    iface="$1"
    gateway="$2"
    metric="$3"

    [ -n "$gateway" ] || return 0
    while ip -4 route del default via "$gateway" dev "$iface" 2>/dev/null; do
        :
    done
    ip -4 route add default via "$gateway" dev "$iface" metric "$metric"
}

wifi_gateway="$(device_gateway "$WIFI_IFACE")"
modem_gateway="$(device_gateway "$MODEM_IFACE")"

if device_connected "$WIFI_IFACE" && [ -n "$wifi_gateway" ]; then
    replace_device_defaults "$WIFI_IFACE" "$wifi_gateway" "$WIFI_METRIC"
    replace_device_defaults "$MODEM_IFACE" "$modem_gateway" "$MODEM_ACTIVE_METRIC"
    logger -t eggy-network "Wi-Fi preferred on $WIFI_IFACE; 4G kept as fallback"
else
    replace_device_defaults "$MODEM_IFACE" "$modem_gateway" "$MODEM_FALLBACK_METRIC"
    logger -t eggy-network "Wi-Fi unavailable; 4G fallback active on $MODEM_IFACE"
fi
