# Pi 5 + ArduPilot + EdgeTX radio: setup

From a bare Pi 5 to kestrel's state on the analog OSD, mode chosen from the
radio. The order matters: every stage proves something the next one relies on.

**What is and is not proven.** Everything here is built and tested off the
aircraft:

- the MAVLink messages against pymavlink;
- the stick mapping against a model of ArduCopter's own stick handling;
- the OSD script against a mock of ArduPilot's scripting interface, using its
  real message layouts;
- the installer into a scratch directory.

None of it has yet run on a real Pi 5 against a real flight controller. This
document is that test; tick the boxes as you go.

## 0. What you get

```
 radio ──CRSF──▶ FC (ArduPilot) ◀──UART MAVLink──▶ Pi 5 (kestrel)
   CH5  ArduPilot flight mode (yours: ALT_HOLD / LOITER / GUIDED ...)
   CH8  kestrel mode wheel:  FLY | SHADOW | SCRIPT | AUTONOMY
   CH9  GO switch
                     OSD:  ALTHOLD         <- ArduPilot's own mode item
                           K SCRIPT LIVE RC GO DOOR   <- kestrel, via kestrel_osd.lua
```

The `K` line, left to right:

1. **kestrel's mode** (or `RTH` / `LAND`).
2. **`LIVE` or `DRY`:** whether commands are sent or only computed.
3. **How commands reach the FC in the mode it is in now:**

   | Tag | Meaning |
   |---|---|
   | `VEL` | velocity (GUIDED) |
   | `ATT` | attitude (GUIDED_NOGPS) |
   | `RC` | sticks (ALT_HOLD / LOITER / POSHOLD) |
   | `AST` | assist |
   | `OFF` | **this FC mode gets nothing** |
   | `PAR` | still reading parameters |

4. **`GO`**, if the GO switch is on.
5. **A script's current state.**

`K LOST` means the Pi has stopped talking, so the last line shown is stale.

## 1. The Pi

- [ ] Raspberry Pi OS Bookworm 64-bit Lite. A 27 W supply and active cooling
      (`hardware-bringup-checklist.md` stage 1).
- [ ] Wiring, 3.3 V logic on both sides, no level shifter:

      | Pi 5 pin | goes to | note |
      |---|---|---|
      | GPIO14 (TX, pin 8) | FC RXn | |
      | GPIO15 (RX, pin 10) | FC TXn | |
      | GND | FC GND | |
      | | | do not join the 5 V rails unless you mean to |

- [ ] Install:
  ```bash
  git clone <this repo> && cd Assembly-grapher/onboard
  sudo deploy/install_pi5.sh --enable-uart
  sudo reboot
  ```
  The script:
  - builds kestrel and puts it in `/usr/local/bin`;
  - writes `/etc/kestrel/kestrel.conf` and `kestrel.env`, keeping any existing
    ones;
  - installs the systemd service (`kestrel`, running as you);
  - with `--enable-uart`, turns on the GPIO UART (`/dev/ttyAMA0`);
  - warns if a login console is on that port. If so, fix it in
    `raspi-config`: Interface → Serial, login shell **no**, hardware **yes**.
- [ ] After the reboot, `ls -l /dev/ttyAMA0` shows the port.
- [ ] `journalctl -u kestrel -f` shows it running. Until a camera is
      plugged in it exits with `Cannot open camera` and systemd retries; that
      is expected.

## 2. ArduPilot parameters

Set these in Mission Planner or QGC. The FC port number `n` is whichever
UART the Pi is wired to.

| Parameter | Value | Why |
|---|---|---|
| `SERIALn_PROTOCOL` | 2 | MAVLink2 on the Pi's port |
| `SERIALn_BAUD` | 115 | matches `--fc-baud=115200` |
| `SYSID_MYGCS` (newer: `MAV_GCS_SYSID`) | 255 | otherwise ArduPilot **ignores** every command and mode change from the Pi |
| `FLTMODE_CH` | 5 | ArduPilot's mode switch stays yours |
| `FLTMODE1..6` | e.g. ALT_HOLD, LOITER, GUIDED_NOGPS, GUIDED | the modes kestrel can command in; anything else gets `OFF` |
| `RC8_OPTION`, `RC9_OPTION` | 0 | CH8/CH9 belong to kestrel; ArduPilot must not act on them |
| `SCR_ENABLE` | 1 | Lua scripting (reboot after) |
| `OSD_TYPE` | 1 | MAX7456 analog OSD |
| `OSD1_ENABLE` | 1 | |
| `OSD1_FLTMODE_EN` | 1 | ArduPilot's own mode |
| `OSD1_MESSAGE_EN` | 1 | **the line the `K` status appears in.** Place it with `OSD1_MESSAGE_X/Y` |
| `OSD_MSG_TIME` | 10 (default) | the script refreshes every 8 s, so the line stays up |

**Scripting needs a 2 MB-flash FC** (F7/H7). Most F405 builds ship without it.
On such a board everything else works, but the `K` line cannot be shown.
`SCR_ENABLE` is then absent from the parameter list, which tells you so.

- [ ] Copy `/etc/kestrel/kestrel_osd.lua` (also in `onboard/ardupilot/`) to the
      FC's SD card under `APM/scripts/`, and reboot the FC.
- [ ] With the Pi off, the OSD shows nothing from kestrel. Within a second of
      the Pi starting, it shows `K FLY DRY ...`. Stop the service
      (`sudo systemctl stop kestrel`) and `K LOST` appears within 3 s.

## 3. The radio (EdgeTX, RadioMaster Pocket)

In the model's **Mixes**:

| Channel | Source | Weight | Notes |
|---|---|---|---|
| CH5 | a switch | | ArduPilot flight mode, as you already fly it |
| CH8 | the wheel / pot you want as kestrel's mode selector | 100 | full travel must reach about 1000 and 2000 µs; check in the radio's Outputs (channel monitor) screen |
| CH9 | a 2-position switch | | GO: low = hold, high = go |

With ELRS/CRSF all 16 channels reach the FC. Confirm with
`kestrel --bench-test` (below): its `rc in` line prints CH1-12 as the FC sees
them every half second -- roll the wheel and watch CH8 sweep.

The wheel is cut into four equal bands, low to high:

| Range | Mode | Notes |
|---|---|---|
| 1000–1250 µs | FLY | kestrel sends no flight commands: rolling the wheel to the bottom is always a way out. One exception: see the note below the table |
| 1250–1500 µs | SHADOW | computes, flies nothing |
| 1500–1750 µs | SCRIPT | |
| 1750–2000 µs | AUTONOMY | |

**The exception.** In LIVE, in any mode including FLY, kestrel's
low-battery failsafe switches ArduPilot to RTL below `safety.rtl_batt_pct`
(15 %). If you would rather leave that to ArduPilot's own battery failsafe,
set `safety.rtl_batt_pct = 0`. In DRY nothing is sent, mode changes included.

To change mode, the wheel must be 30 µs past a band edge and stay there for
0.3 s (`rc.mode_hyst_us`, `rc.mode_dwell_s`). Resting it on an edge cannot
make the mode flicker. Change the bands or their order in
`/etc/kestrel/kestrel.conf` (`rc.mode_map`).

## 4. On the bench (props off)

```bash
sudo systemctl stop kestrel
kestrel --fc=mavlink --fc-port=/dev/ttyAMA0 --bench-test
```

- [ ] `link UP`, attitude moves when you tilt the FC.
- [ ] The **parameter report** has no line starting `!!`. The first lines
      should read `GCS sysid 255 matches ours`.
- [ ] `mavlink : FC mode N -> control path: ...` follows your mode switch:
      ALT_HOLD gives `sticks`, STABILIZE gives `none`.
- [ ] `rc in` shows CH8 sweeping about 1000-2000 with the wheel and CH9
      flipping with GO.
- [ ] `sudo systemctl start kestrel`. With the camera connected, the OSD
      shows `K FLY DRY OFF` (in STABILIZE). Then:
  - roll the wheel up: `K SHADOW DRY ...`, then `K SCRIPT ...`;
  - flip GO: `... GO` appears;
  - in ALT_HOLD the tag reads `RC`;
  - in STABILIZE it reads `OFF`.

## 5. Going live

Only after the stages above, and the SHADOW flights in
`hardware-bringup-checklist.md` stage 5:

```bash
sudoedit /etc/kestrel/kestrel.env      # add --allow-control to KESTREL_ARGS
sudo systemctl restart kestrel
```

The OSD now says `LIVE`. Commands are sent only when **all** of these hold:

- kestrel's mode is not FLY;
- GO is on (for the modes that use it);
- ArduPilot is in a mode kestrel can command (tag not `OFF`).

Taking it back, fastest first:
- flick ArduPilot's mode switch to STABILIZE / ACRO: kestrel releases the
  sticks immediately (`OFF`);
- roll the wheel to FLY;
- GO off.

`LIVE` cannot be switched on from the radio. It is set at service start, on
purpose: going live should be a decision made on the ground.
