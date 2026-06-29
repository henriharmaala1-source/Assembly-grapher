# BOM: 7" Long Range / Freestyle Drone — Analog + ELRS 2.4GHz + AI Companion

**Tarkoitus:** Kehitysalusta — Betaflight (OSD-fork) + AI companion computer (RPi 5)
**Akku:** 6S · **Video:** Analog FPV · **Budjetti:** ~480 €
**Päivitetty:** 2026-06-29

---

## Arkkitehtuuri lyhyesti

Analoginen FPV-ketju pysyy koskemattomana (nollaviive laseihin). RPi 5 kaappaa saman
videosignaalin splitterin kautta, ajaa CV:n, ja piirtää tulokset FC:n omaan
AT7456E-OSD:hen forkatun Betaflightin MSP-laajennuksen kautta. Ei erillistä
OSD-moduulia, ei erillistä CV-kameraa, ei HDMI→composite-muunnosta.

```
[Ratel Pro] ──→ [FC: BF-fork + AT7456E OSD] ──→ [TX500] ──→ Lasit
                         ↑
                  UART / MSP (custom OSD-komento)
                         │
        [RPi 5: CV] ←── [USB CVBS capture] ←── [splitteri]
```

---

## Komponenttilista — drone

| # | Komponentti | Malli | Hinta (€) | Huomiot |
|---|-------------|-------|-----------|---------|
| 1 | **Frame** | GEPRC GEP-MK5 7" | ~40 | 30×30 + 20×20 mount |
| 2 | **Flight Controller** | Matek H743-SLIM v4 | ~40 | BF · iNav · ArduPilot · 7× UART · AT7456E OSD · dual ICM42688P |
| 3 | **ESC 4-in-1** | SpeedyBee BLS 60A | ~32 | 6S · BLHeli_S → **flashaa Bluejay** (bidir DSHOT + RPM-filter) |
| 4 | **Moottorit × 4** | iFlight XING2 2207 1900KV | ~55 | Velox V2207.5 loppu varastosta; 1900KV = lisää nopeutta, 6S-yhteensopiva |
| 5 | **Potkurit** | HQProp 7040 (7×4×3) 3-blade | ~12 | 5mm akseli · 2 settiä |
| 6 | **VTX** | SpeedyBee TX500 | ~20 | Max 400 mW · SmartAudio |
| 7 | **FPV-kamera** | Caddx Ratel Pro | ~30 | 1/1.8" Starvis · terävämpi + vähemmän vääristymää → parempi CV-syöte |
| 8 | **Vastaanotin** | RadioMaster RP2 ELRS 2.4 | ~9 | Diversity · UART |
| 9 | **GPS** | HGLRC M100-5883 (M10) | ~20 | u-blox M10 + QMC5883L kompassi |
| 10 | **Akku × 2** | CNHL 6S 3000mAh 60C | ~90 | |
| 11 | **VTX-antenni** | Foxeer Lollipop 4 RHCP | ~8 | SMA |
| 12 | **Kondensaattori** | 50V 1000µF low-ESR | ~3 | 50V pakollinen 6S:lle (25.2V) |
| 13 | **BEC** | Matek UBEC DUO (4A/5V + 4A/adj.) | ~15 | RPi 5 + VTX virransyöttö |
| 14 | **Sekalaista** | XT60 · johdot · standoffit | ~10 | |

**Drone yhteensä: ~384 €**

## Komponenttilista — AI companion

| # | Komponentti | Malli | Hinta (€) | Huomiot |
|---|-------------|-------|-----------|---------|
| 15 | **Companion computer** | Raspberry Pi 5 (4GB) | ~80 | CV-laskenta · MSP/UART → FC |
| 16 | **Video-splitteri** | Passiivinen CVBS-splitteri | ~5 | Yksi haara FC:lle/VTX:lle, toinen RPi:lle |
| 17 | **Capture** | USB CVBS -dongle (UVC) | ~8 | Analoginen kuva → RPi V4L2 |
| 18 | **USB-C power** | USB-C breakout + 5.1kΩ CC | ~3 | UBEC → RPi 5 (CC-vastukset pakollisia) |

**Companion yhteensä: ~96 €**

---

## Kokonaishinta

| Kategoria | Hinta |
|-----------|-------|
| Drone | ~384 € |
| AI companion | ~96 € |
| **Yhteensä** | **~480 €** |

*Ei sisällä: lähetintä, latureita, laseja.*

---

## FC: Matek H743-SLIM v4 — UART-jako

| UART | Käyttö |
|------|--------|
| UART1 | ELRS 2.4GHz (RadioMaster RP2) |
| UART2 | GPS (HGLRC M100) |
| UART3 | VTX SmartAudio (TX500) |
| UART4 | RPi 5 → MSP (custom OSD-komento) |
| UART5 | Telemetria / GCS |
| UART6 | Vapaa (optical flow, LiDAR) |
| UART7 | Vapaa / debug |

---

## Virransyöttö

```
6S Akku (max 25.2V)
    │
    ├──→ ESC (suoraan) → Moottorit
    │
    ├──→ [50V 1000µF kondensaattori]
    │
    └──→ [Matek UBEC DUO]
              ├──→ 5V / 4A → USB-C breakout → RPi 5
              └──→ adj. / 4A → VTX TX500
```

---

## Ostohuomiot

1. **FC vain viralliselta myyjältä** — [MATEKSYS Official Store](https://mateksys.aliexpress.com/store/1102413620). Tarkista että target raportoi `MATEKH743` Configuratorissa.
2. **ESC on BLHeli_S** (BLS) — flashaa Bluejay saadaksesi RPM-filterin Betaflightiin.
3. **UBEC DUO max tulo 26V** — 6S täydellä (25.2V) tiukka marginaali, tarkkaile lämpöä.
4. **USB-C CC-vastukset** (2× 5.1kΩ) pakollisia, muuten RPi 5 rajoittaa virran.
5. **Kapasitanssi 50V**, ei 35V.

---

## Nopeus / suorituskyky (arvio)

| | Arvio |
|--|-------|
| Max nopeus (syöksy) | ~140–160 km/h |
| Lentoaika (cruise) | ~14–17 min |
| AUW (arvio) | ~850–950 g |

*7" runko rajoittaa huippunopeuden ~160 km/h:iin riippumatta moottoreista — 200 km/h vaatisi 5" speed buildin.*
