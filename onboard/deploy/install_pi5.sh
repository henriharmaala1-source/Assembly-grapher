#!/usr/bin/env bash
# install_pi5.sh -- build kestrel on a Raspberry Pi 5 and start it at boot.
#
#   cd onboard && sudo deploy/install_pi5.sh [--enable-uart] [--user NAME]
#                                            [--no-deps] [--no-build] [--no-service]
#
# Raspberry Pi OS Bookworm (64-bit). What it does, in order:
#   1. apt: build tools and OpenCV                      (skip: --no-deps)
#   2. cmake Release build, natively tuned for the A76  (skip: --no-build)
#   3. installs the binary to /usr/local/bin/kestrel
#   4. /etc/kestrel: kestrel.conf (the ArduPilot + EdgeTX sample), kestrel.env
#      (the command line, DRY-RUN), missions/ -- EXISTING FILES ARE KEPT
#   5. --enable-uart: dtparam=uart0=on in config.txt, for /dev/ttyAMA0 on
#      GPIO 14/15 (needs a reboot). Never done without being asked.
#   6. the systemd unit, enabled and (re)started        (skip: --no-service)
#
# Re-running is safe: it rebuilds, reinstalls the binary and the unit, and
# leaves your configuration alone.
#
# PREFIX / ETCDIR / UNITDIR / BOOTCFG can be overridden from the environment,
# which is how it is exercised off the Pi (test: everything into a temp dir).

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "$HERE/.." && pwd)"                      # onboard/
PREFIX="${PREFIX:-/usr/local}"
ETCDIR="${ETCDIR:-/etc/kestrel}"
UNITDIR="${UNITDIR:-/etc/systemd/system}"
BOOTCFG="${BOOTCFG:-/boot/firmware/config.txt}"
BUILD="${BUILD:-$SRC/build-pi}"

deps=1 build=1 service=1 uart=0
user="${SUDO_USER:-$(id -un)}"
while [ $# -gt 0 ]; do
    case "$1" in
        --no-deps)     deps=0 ;;
        --no-build)    build=0 ;;
        --no-service)  service=0 ;;
        --enable-uart) uart=1 ;;
        --user)        user="$2"; shift ;;
        -h|--help)     sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

say()  { printf '\n== %s\n' "$*"; }
warn() { printf '!! %s\n' "$*" >&2; }

model="$(tr -d '\0' 2>/dev/null < /proc/device-tree/model || true)"
case "$model" in
    *"Raspberry Pi 5"*) ;;
    *) warn "this is not a Pi 5 (${model:-unknown}); carrying on, but the paths below are the Pi 5's" ;;
esac

if [ "$deps" = 1 ]; then
    say "1/6 packages"
    apt-get update
    apt-get install -y build-essential cmake git libopencv-dev
fi

if [ "$build" = 1 ]; then
    say "2/6 build ($BUILD)"
    # As the invoking user, so the build tree is not root's.
    runas=()
    if [ "$(id -u)" = 0 ] && [ "$user" != root ]; then runas=(sudo -u "$user"); fi
    "${runas[@]}" cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
    "${runas[@]}" cmake --build "$BUILD" -j"$(nproc)" --target kestrel
fi

say "3/6 binary -> $PREFIX/bin/kestrel"
[ -x "$BUILD/kestrel" ] || { warn "no $BUILD/kestrel: build first (drop --no-build)"; exit 1; }
install -D -m 0755 "$BUILD/kestrel" "$PREFIX/bin/kestrel"

say "4/6 configuration in $ETCDIR"
install -d -m 0755 "$ETCDIR" "$ETCDIR/missions"
keep() {   # keep() SRC DEST: install only if DEST does not exist
    if [ -e "$2" ]; then echo "   kept     $2"
    else install -m 0644 "$1" "$2"; echo "   created  $2"; fi
}
keep "$HERE/kestrel-ardupilot.conf" "$ETCDIR/kestrel.conf"
keep "$HERE/kestrel.env"            "$ETCDIR/kestrel.env"
for m in "$SRC"/missions/*.kmb; do
    if [ -e "$m" ]; then keep "$m" "$ETCDIR/missions/$(basename "$m")"; fi
done
# The FC half of the OSD line, ready to copy onto the flight controller's SD card.
install -m 0644 "$SRC/ardupilot/kestrel_osd.lua" "$ETCDIR/kestrel_osd.lua"
echo "   updated  $ETCDIR/kestrel_osd.lua  (copy to the FC's APM/scripts/)"
if [ "$(id -u)" = 0 ] && id "$user" >/dev/null 2>&1; then
    usermod -aG dialout,video "$user" || true
fi

if [ "$uart" = 1 ]; then
    say "5/6 UART (asked for)"
    if [ ! -f "$BOOTCFG" ]; then
        warn "$BOOTCFG not found: add 'dtparam=uart0=on' to the boot config by hand"
    elif grep -qE '^\s*dtparam=uart0=on' "$BOOTCFG"; then
        echo "   already on in $BOOTCFG"
    else
        cp "$BOOTCFG" "$BOOTCFG.kestrel-backup"
        printf '\n# kestrel: GPIO 14/15 UART to the flight controller (/dev/ttyAMA0)\ndtparam=uart0=on\n' >> "$BOOTCFG"
        echo "   added dtparam=uart0=on ($BOOTCFG.kestrel-backup kept) -- REBOOT to apply"
    fi
fi
# A login console on the same UART would fight the FC for it.
cmdline="$(dirname "$BOOTCFG")/cmdline.txt"
if [ -f "$cmdline" ] && grep -qE 'console=(serial0|ttyAMA0)' "$cmdline"; then
    warn "$cmdline puts a login console on the serial port: remove 'console=serial0,...'"
    warn "(raspi-config > Interface > Serial: login shell NO, hardware port YES)"
fi

if [ "$service" = 1 ]; then
    say "6/6 systemd unit"
    install -d -m 0755 "$UNITDIR"
    sed -e "s|@USER@|$user|g" -e "s|@BINDIR@|$PREFIX/bin|g" -e "s|@ETCDIR@|$ETCDIR|g" \
        "$HERE/kestrel.service" > "$UNITDIR/kestrel.service"
    chmod 0644 "$UNITDIR/kestrel.service"
    echo "   $UNITDIR/kestrel.service (runs as $user)"
    # Only a real install touches the running system: root, the system unit
    # directory, systemd up. A scratch-prefix run (CI, testing) writes the
    # unit and stops there -- as a normal user on a systemd host, calling
    # systemctl would fail and abort the script.
    if [ "$(id -u)" = 0 ] && [ "$UNITDIR" = /etc/systemd/system ] &&
       command -v systemctl >/dev/null && [ -d /run/systemd/system ]; then
        systemctl daemon-reload
        systemctl enable kestrel
        systemctl restart kestrel
        echo "   enabled and started: journalctl -u kestrel -f"
    else
        warn "not a system install (needs root, UNITDIR=/etc/systemd/system, systemd): unit written, not enabled"
    fi
fi

cat <<EOF

Done. kestrel starts at boot, DRY-RUN (nothing commanded).
  watch it            journalctl -u kestrel -f
  first check         sudo systemctl stop kestrel
                      $PREFIX/bin/kestrel --fc=mavlink --fc-port=/dev/ttyAMA0 --bench-test
                      -> no line starting "!!" in the parameter report
  the FC's OSD line   copy $ETCDIR/kestrel_osd.lua to the FC SD card APM/scripts/
  go live             add --allow-control to $ETCDIR/kestrel.env, restart
                      (only after the checklist in onboard/docs/pi5-ardupilot-setup.md)
EOF
