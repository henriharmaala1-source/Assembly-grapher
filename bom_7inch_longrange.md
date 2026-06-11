# BOM: 7" Long Range Drone — Analog / ELRS 2.4GHz (Kehitys)

**Tarkoitus:** Long-range kehitysdrone — Betaflight / iNav / PX4  
**Budjetti:** ~350–420 € (ilman lähetintä ja latureita)  
**Päivätty:** 2026-06-11

---

## Komponenttilista

| # | Komponentti | Malli / Vaihtoehto | Hinta (€) | Huomiot |
|---|-------------|-------------------|-----------|---------|
| 1 | **Frame** | GEPRC GEP-MK5 7" | ~40 | Kevyt, hyvä kamerapaikka, 30×30 + 20×20 mount |
| 2 | **Flight Controller** | Matek H743-WING v2 | ~40 | ✅ BF + iNav + **PX4**, 7× UART, Barometer, OSD |
| 3 | **ESC 4-in-1** | SpeedyBee BLS 55A 4-in-1 | ~35 | 6S-yhteensopiva, BLHeli_32, DSHOT600 |
| 4 | **Moottorit × 4** | BrotherHobby Returner R5 2207 1750KV | ~60 | Sopii 4S/6S, vakaa LR-käyttöön |
| 5 | **Potkurit** | HQProp 7×4×3 V1S (pari pakkauksia) | ~10 | Tehokas + pitkä lento-aika |
| 6 | **Analog VTX** | Rush Tank Solo V2 (25–800 mW) | ~25 | SmartAudio, hyvä kantama |
| 7 | **FPV-kamera** | Caddx Ratel 2 Micro | ~20 | Hyvä low-light, 1/1.8" sensori |
| 8 | **Vastaanotin (RX)** | RadioMaster RP2 ELRS 2.4GHz | ~9 | UART-yhteys, diversity-antennit |
| 9 | **GPS-moduuli** | Matek M10-L-4.11 (M10 GPS + kompassi) | ~25 | iNav/PX4 tuettu, nopea fix |
| 10 | **Akku × 2** | CNHL 6S 3000mAh 60C | ~90 (2 kpl) | 6S → enemmän teho + kantama |
| 11 | **VTX-antenni** | Foxeer Lollipop 4 RHCP 5.8GHz | ~8 | SMA, kestävä |
| 12 | **Kondensaattori** | 1500µF 35V low-ESR | ~3 | Jännitepiikit poikki |
| 13 | **Sekalaista** | XT60-liitin, johdot, standoffit | ~10 | Kytkentätarvikkeet |

---

## Yhteishinta (arvio)

| Kategoria | Hinta |
|-----------|-------|
| Rakenne (frame, moottorit, potkurit) | ~110 € |
| Elektroniikka (FC, ESC) | ~75 € |
| Video (VTX, kamera, antenni) | ~53 € |
| Radio (ELRS RX) | ~9 € |
| GPS | ~25 € |
| Akut (2 kpl) | ~90 € |
| Muut | ~13 € |
| **Yhteensä** | **~375 €** |

---

## FC: Matek H743-WING v2 — UARTs ja pinout

Kehityskäyttöön kriittinen valinta. Ominaisuudet:

- **MCU:** STM32H743 (480 MHz)
- **UART-portit:** 7 kpl (UART1–7)
- **Firmware-tuki:** Betaflight ✅ · iNav ✅ · **PX4** ✅
- **Barometri:** onboard (BMP280)
- **OSD:** AT7456E
- **I2C, SPI, CAN:** kyllä
- **Virransyöttö:** 2–6S suoraan

### Suositeltu UART-jako

| UART | Käyttö |
|------|--------|
| UART1 | ELRS 2.4GHz vastaanotin (RadioMaster RP2) |
| UART2 | GPS (Matek M10) |
| UART3 | VTX SmartAudio (Rush Tank Solo) |
| UART4 | Telemetria (MAVLink / vapaa kehitykseen) |
| UART5 | Vapaa (LiDAR, rangefinder, companion computer) |
| UART6 | Vapaa (toinen GPS, optical flow) |
| UART7 | Vapaa / MSP-debug |

---

## Firmware-suositukset

| Tarkoitus | Firmware | Huomiot |
|-----------|----------|---------|
| Akrobatia / testaus | **Betaflight 4.5** | Nopea setup, DSHOT, RPM-filter |
| Autonominen lento / waypoint | **iNav 8.x** | GPS-navigointi, RTH, loiter |
| Kehitys / ROS2 / MAVSDK | **PX4 1.14+** | MAVLink, full offboard-tuki |

---

## Lisäosat kehityskäyttöön (valinnainen)

| Komponentti | Malli | Hinta | Käyttö |
|-------------|-------|-------|--------|
| Companion computer liitäntä | USB-UART adapteri | ~5 € | Raspberry Pi / Jetson Nano |
| Optical flow | Matek 3901-L0X | ~25 € | Sisätilapositiointi |
| Lidar / rangefinder | TFmini-S | ~25 € | Korkeusmittaus |
| Telemetria-radio | SiK 433MHz 100mW | ~15 € | GCS-yhteys maassa |

---

## Rakennushuomiot

1. **Kapasaattori** suoraan ESC:n tuloon — tärkeä 6S:llä jännitepiikeille
2. **GPS-maston** korkeus vähintään 30 mm FC:stä — kompassin häiriöt
3. **VTX-virta** suoraan akusta LC-suotimella (ei ESC:n 5V)
4. **ELRS RP2** kiinnitetään kauemmaksi FC:stä — 2.4 GHz häiriöherkkyys
5. **PX4-setup:** tarvitset MAVLink-telemetrian debuggaukseen → UART4 GCS:lle

---

## Vaihtoehtoinen FC (jos PX4 ei kriittinen)

| FC | Firmware | UARTs | Hinta |
|----|----------|-------|-------|
| SpeedyBee F7 V3 | BF + iNav | 7 | ~35 € |
| Matek F405-CTR | BF + iNav | 6 | ~30 € |
| Holybro Kakute H7 v2 | BF + iNav + **PX4** | 6 | ~45 € |

Matek H743-WING v2 pysyy suosituksena — hinta/ominaisuus-suhde paras kehityskäyttöön.
