"""Aircraft description and analysis settings.

Anything the log cannot tell us (mass, battery capacity, prop size...) lives here.
Analyses that need a missing value are skipped and say so in the report.
"""

from dataclasses import dataclass, asdict

# Betaflight / iNav Quad X motor order (both inherited from Cleanflight):
#   motor[0] = M1 rear-right, motor[1] = M2 front-right,
#   motor[2] = M3 rear-left,  motor[3] = M4 front-left
QUADX_POSITIONS = {0: "rear-right", 1: "front-right", 2: "rear-left", 3: "front-left"}


@dataclass
class Aircraft:
    mass_kg: float | None = None          # all-up weight including battery
    capacity_mah: float | None = None     # pack capacity, for C-rate
    prop_diameter_in: float | None = None  # for induced-power model / disc loading
    rotors: int = 4
    blades: int = 2                       # blade-pass harmonic in vibration plots
    cells: int | None = None              # auto-detected from voltage if None
    motor_poles: int = 14                 # only used when the log carries eRPM
    eta_drag: float = 0.55                # electrical -> drag work efficiency, for absolute CdA
    air_temp_c: float = 15.0
    pressure_pa: float = 101325.0
    coast_throttle: float = 0.20          # throttle at or below this counts as "coasting"
    motor_layout: str = "quadx"

    @property
    def rho(self) -> float:
        return self.pressure_pa / (287.05 * (self.air_temp_c + 273.15))

    @property
    def weight_n(self) -> float | None:
        return None if self.mass_kg is None else self.mass_kg * 9.81

    @property
    def disc_area_m2(self) -> float | None:
        if self.prop_diameter_in is None:
            return None
        r = self.prop_diameter_in * 0.0254 / 2
        return self.rotors * 3.141592653589793 * r * r

    def to_dict(self) -> dict:
        d = asdict(self)
        d["rho"] = round(self.rho, 4)
        return d
