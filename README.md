# Assembly Grapher — AI-lisäys analogiseen FPV-droneen

Hei, tervetuloa seuraamaan projektiani tekoälyn lisäämisestä analogiseen FPV droneen. Tarkoituksena on luoda helposti kytkettävä kaveritietokone, joka toimii betaflightia käyttävien analog dronejen kanssa, lisäten eri ominaisuuksia.

---

## Projektin tavoite

Rakentaa **7" long range FPV-drone** johon on integroitu **Raspberry Pi 5** -kaveritietokone, joka:
- Käsittelee kamerakuvaa reaaliajassa (C++ / Python / OpenCV)
- Lähettää tulokset FC:lle OSD-näyttöä varten → pilotin laseihin
- Kommunikoi FC:n kanssa MSP-protokollalla (Betaflight)
- On helposti kytkettävissä olemassaoleviin analog FPV -drooneihin

---

## BOM — Bill of Materials

**Budjetti:** ~490 € (ilman lähetintä ja latureita)  
**Päivätty:** 2026-06-11

### Drone-runko

| # | Komponentti | Malli | Hinta (€) | Huomiot |
|---|-------------|-------|-----------|---------|
| 1 | **Frame** | GEPRC GEP-MK5 7" | ~40 | Kevyt, 30×30 + 20×20 mount |
| 2 | **Flight Controller** | Matek H743-SLIM v4 | ~40 | BF ✅ iNav ✅ ArduPilot ✅, 7× UART, dual ICM42688P, CAN |
| 3 | **ESC 4-in-1** | SpeedyBee BLS 55A 4-in-1 | ~35 | 6S, BLHeli_32, DSHOT600 |
| 4 | **Moottorit × 4** | BrotherHobby Returner R5 2207 1750KV | ~60 | 4S/6S, vakaa LR-käyttöön |
| 5 | **Potkurit** | HQProp 7×4×3 V1S | ~10 | |
| 6 | **VTX** | SpeedyBee TX500 (max 400 mW) | ~20 | SmartAudio, kevyt |
| 7 | **FPV-kamera** | Caddx Ratel 2 Micro | ~20 | Low-light, 1/1.8" sensori |
| 8 | **Vastaanotin** | RadioMaster RP2 ELRS 2.4GHz | ~9 | Diversity-antennit |
| 9 | **GPS** | Matek M10-L-4.11 | ~25 | iNav/ArduPilot tuettu |
| 10 | **Akku × 2** | CNHL 6S 3000mAh 60C | ~90 | |
| 11 | **VTX-antenni** | Foxeer Lollipop 4 RHCP 5.8GHz | ~8 | |
| 12 | **Kondensaattori** | 50V 1000µF low-ESR | ~3 | 6S:lle tarvitaan 50V, ei 35V |
| 13 | **BEC** | Matek UBEC DUO (4A/5V + 4A/adj.) | ~15 | RPi 5 + VTX virransyöttö |
| 14 | **Sekalaista** | XT60, johdot, standoffit | ~10 | |

### Kaveritietokone (companion computer)

| # | Komponentti | Malli | Hinta (€) | Huomiot |
|---|-------------|-------|-----------|---------|
| 15 | **Companion computer** | Raspberry Pi 5 (4GB) | ~80 | CV-laskenta, MSP/UART → FC |
| 16 | **CV-kamera** | RPi Camera Module 3 | ~25 | Digitaalinen, suoraan CSI-porttiin |
| 17 | **Video-adapteri** | HDMI → Composite muunnin | ~8 | RPi 5 HDMI → VTX analogiseksi |
| 18 | **Dev-capture** | USB CVBS -dongle (kehitykseen) | ~8 | Analog cam → RPi dev-vaiheessa |
| 19 | **USB-C power** | USB-C breakout + 5.1kΩ CC-vastukset | ~3 | UBEC → RPi 5 USB-C |

### Yhteishinta

| Kategoria | Hinta |
|-----------|-------|
| Drone-runko (frame + moottorit + potkurit) | ~110 € |
| Elektroniikka (FC + ESC) | ~75 € |
| Video (VTX + kamera + antenni) | ~48 € |
| Radio (ELRS RX) | ~9 € |
| GPS | ~25 € |
| Akut (2 kpl) | ~90 € |
| BEC + kondensaattori | ~18 € |
| **Companion computer (RPi 5 + tarvikkeet)** | **~124 €** |
| Muut | ~10 € |
| **Yhteensä** | **~509 €** |

---

## Video-pipeline

```
[Caddx Ratel 2]  ──────────────────────────→  [FC OSD]  →  [TX500]  →  Lasit
                                                  ↑
[RPi Camera M3]  →  [RPi 5: CV + overlay]  →  [UART/MSP]
                         ↓
                  [HDMI → Composite]  ──────────────────→  [TX500]  →  Lasit
```

**Kehitysvaiheessa:** USB CVBS -dongle siirtää analogikuvan RPi:lle testikäyttöön.  
**Lopullisessa buildissa:** RPi Camera Module 3 (digitaalinen) → CV → HDMI → composite → VTX.  
Yksi kamera molempiin tarkoituksiin — ei parallaksiongelmaa bounding boxeissa.

---

## FC: Matek H743-SLIM v4 — UART-jako

| UART | Käyttö |
|------|--------|
| UART1 | ELRS 2.4GHz (RadioMaster RP2) |
| UART2 | GPS (Matek M10) |
| UART3 | VTX SmartAudio (TX500) |
| UART4 | RPi 5 → MSP / MAVLink |
| UART5 | Telemetria / GCS |
| UART6 | Vapaa (optical flow, LiDAR) |
| UART7 | Vapaa / debug |

---

## Virransyöttö

```
6S Akku (25.2V max)
    │
    ├──→ ESC (suoraan)
    │       └──→ Moottorit
    │
    ├──→ [50V 1000µF kondensaattori]  (jännitepiikit)
    │
    └──→ [Matek UBEC DUO]
              ├──→ 5V 4A → [USB-C breakout] → RPi 5
              └──→ adj. 4A → VTX TX500
```

---

## Rakennushuomiot

1. **Kondensaattori 50V** (ei 35V) — 6S täyteen = 25.2V, tarvitaan marginaali
2. **UBEC DUO max 26V** — 6S:llä todella tiukka marginaali, tarkkaile
3. **USB-C CC-pinnit** — 5.1kΩ resistorit CC1+CC2 → RPi 5 hyväksyy täyden virran
4. **GPS-masto** vähintään 30mm FC:stä — kompassihäiriöt
5. **ELRS RP2** kauas FC:stä — 2.4 GHz häiriöherkkyys
6. **VTX-virta** UBEC:ltä suoraan, ei FC:n 5V:ltä
