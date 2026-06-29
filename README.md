# Assembly Grapher — AI-lisäys analogiseen FPV-droneen

Hei, tervetuloa seuraamaan projektiani tekoälyn lisäämisestä analogiseen FPV droneen. Tarkoituksena on luoda helposti kytkettävä kaveritietokone, joka toimii betaflightia käyttävien analog dronejen kanssa, lisäten eri ominaisuuksia.

---

## Projektin tavoite

Rakentaa **7" FPV-drone** johon on integroitu **Raspberry Pi 5** -kaveritietokone, joka:
- Käsittelee kamerakuvaa reaaliajassa (C++ / Python / OpenCV)
- Piirtää tulokset FC:n omaan OSD-piiriin (AT7456E) forkatun Betaflightin MSP-laajennuksen kautta
- Pitää analogisen videoketjun koskemattomana → ei viivettä laseihin
- On helposti kytkettävissä olemassa oleviin analog FPV -drooneihin

---

## Dokumentit

- **[bom_7inch_longrange.md](bom_7inch_longrange.md)** — täysi Bill of Materials, UART-jako, virransyöttö, ostohuomiot
- **[docs/](docs/)** — blogityylinen projektisivu (GitHub Pages)

## BOM lyhyesti

| | |
|--|--|
| **Budjetti** | ~480 € (ilman lähetintä, latureita, laseja) |
| **FC** | Matek H743-SLIM v4 — BF · iNav · ArduPilot · 7× UART · AT7456E OSD |
| **ESC** | SpeedyBee BLS 60A (Bluejay) |
| **Moottorit** | iFlight XING2 2207 1900KV × 4 (6S) |
| **VTX / kamera** | SpeedyBee TX500 (≤400 mW) · Caddx Ratel Pro |
| **RX / GPS** | RadioMaster RP2 ELRS 2.4 · HGLRC M100-5883 (M10) |
| **Companion** | Raspberry Pi 5 + CVBS-splitteri + USB capture |

Täysi erittely ja perustelut: [bom_7inch_longrange.md](bom_7inch_longrange.md).
