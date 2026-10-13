"""
Assembly Resource Calculator — determines the complete tool kit needed to
assemble (or service) a set of fasteners.

For every FastenerSpec the calculator looks up:
  - the specific tool required (hex key, socket, Torx bit, screwdriver bit)
  - its exact size (mm for Allen/socket, T-number for Torx, PH-number for Phillips)
  - the recommended torque range (Grade 8.8 dry assembly, per ISO 898-1)

Results are grouped by tool (so you see "3 mm Hex Key × 3 fasteners") and,
optionally, by subassembly zone, so a production planner can kit each
workstation with only the tools that zone actually needs.

Lookup tables
─────────────
  ISO 4762  — socket-cap internal hex key sizes
  ISO 4014  — hex bolt AF (across-flats) ≡ socket size  (read from head_diameter_mm)
  ISO 10664 — Torx drive sizes for metric machine screws
  DIN 7985  — Phillips drive sizes for metric machine screws
  ISO 898-1 — Grade 8.8 torque recommendations (dry)

Usage
─────
    from dfma.rules.resource_calculator import calculate_resources
    resources = calculate_resources(fasteners)
    print(resources.report())
"""

from __future__ import annotations
from collections import defaultdict
from dataclasses import dataclass, field

from ..models.fastener import FastenerSpec, DriveType


# ── ISO / DIN standard lookup tables ─────────────────────────────────────────

# ISO 4762: socket-cap screw internal hex-key size (mm) by nominal diameter
_HEX_KEY_MM: dict[float, float] = {
    1.6: 1.5,  2.0: 1.5,  2.5: 2.0,  3.0: 2.5,  3.5: 2.5,
    4.0: 3.0,  5.0: 4.0,  6.0: 5.0,  8.0: 6.0,  10.0: 8.0,
    12.0: 10.0, 14.0: 12.0, 16.0: 14.0, 20.0: 17.0, 24.0: 19.0,
}

# ISO 10664: Torx drive designation by nominal diameter
_TORX_SIZE: dict[float, str] = {
    1.6: "T4",  2.0: "T6",  2.5: "T8",  3.0: "T10", 3.5: "T15",
    4.0: "T20", 5.0: "T25", 6.0: "T30", 8.0: "T40", 10.0: "T45",
    12.0: "T50", 14.0: "T55", 16.0: "T60",
}

# DIN 7985: Phillips drive number by nominal diameter
_PHILLIPS_NO: dict[float, int] = {
    1.0: 0, 1.6: 0, 2.0: 0, 2.5: 0, 3.0: 1, 3.5: 1,
    4.0: 2, 5.0: 2, 6.0: 3, 8.0: 3, 10.0: 4, 12.0: 4,
}

# Pozi (Pozidriv) — same number scheme as Phillips
_POZI_NO: dict[float, int] = _PHILLIPS_NO

# ISO 898-1 Grade 8.8, dry assembly torque (Nm): (min, nominal, max)
_TORQUE_88: dict[float, tuple[float, float, float]] = {
    1.6: (0.18, 0.23, 0.28), 2.0:  (0.38, 0.48, 0.57), 2.5: (0.70, 0.88, 1.05),
    3.0: (1.10, 1.30, 1.60), 3.5:  (1.90, 2.30, 2.80), 4.0: (2.70, 3.10, 3.80),
    5.0: (5.50, 6.10, 7.50), 6.0:  (9.50, 10.4, 12.7), 8.0: (23.0, 25.0, 31.0),
    10.0: (45.0, 50.0, 61.0), 12.0: (79.0, 86.0, 105.0), 14.0: (127.0, 137.0, 168.0),
    16.0: (196.0, 211.0, 258.0), 20.0: (392.0, 420.0, 513.0), 24.0: (672.0, 720.0, 880.0),
}


def _nearest_key(table: dict, value: float):
    """Return the table value whose key is nearest to *value*."""
    best = min(table.keys(), key=lambda k: abs(k - value))
    return table[best]


def _torque_for(nominal_mm: float) -> tuple[float, float]:
    """Return (min_Nm, max_Nm) for the nearest standard size."""
    t = _nearest_key(_TORQUE_88, nominal_mm)
    return t[0], t[2]


# ── data classes ──────────────────────────────────────────────────────────────

@dataclass
class ToolRequirement:
    """One unique tool needed during assembly."""

    tool_type:    str        # "hex_key" | "socket" | "torx_bit" | "phillips_bit"
                             # | "slotted_bit" | "pozi_bit"
    label:        str        # human-readable full description, e.g. "3 mm Hex Key"
    size_label:   str        # compact size descriptor, e.g. "3 mm" / "T20" / "PH2"
    drive_type:   DriveType
    size_mm:      float      # numeric size (mm for allen/socket, Torx nr, PH nr)
    torque_min_nm: float     # ISO 898-1 Grade 8.8 dry (minimum)
    torque_max_nm: float     # ISO 898-1 Grade 8.8 dry (maximum)
    fastener_ids:  list[str] = field(default_factory=list)
    fastener_names: list[str] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.fastener_ids)

    def torque_str(self) -> str:
        if self.torque_min_nm == self.torque_max_nm == 0.0:
            return "—"
        return f"{self.torque_min_nm:.1f}–{self.torque_max_nm:.1f} Nm"


@dataclass
class ZoneResources:
    """Tool requirements for a single subassembly zone."""
    zone_id:      str
    tools:        list[ToolRequirement]
    fastener_ids: list[str]
    tool_changes: int   # n_distinct_tools – 1 (minimum tool changes for this zone)


@dataclass
class AssemblyResources:
    """Complete tool and torque resource summary for the assembly."""
    tools:           list[ToolRequirement]   # all tools, deduplicated, sorted
    zones:           list[ZoneResources]     # per-zone breakdown
    total_fasteners: int
    ungrouped_fastener_ids: list[str]        # fasteners without a subassembly_id

    @property
    def unique_tool_count(self) -> int:
        return len(self.tools)

    @property
    def total_tool_changes(self) -> int:
        """Minimum tool changes if assembling all zones sequentially."""
        return max(0, self.unique_tool_count - 1)

    def tool_kit(self) -> list[str]:
        """Compact list of tool labels needed."""
        return [t.label for t in self.tools]

    def report(self, show_zones: bool = True) -> str:
        """Return a formatted plain-text report."""
        lines: list[str] = []

        # ── summary ──────────────────────────────────────────────────────────
        lines += [
            "═" * 68,
            "  ASSEMBLY RESOURCE CALCULATOR",
            "═" * 68,
            f"  Fasteners analysed : {self.total_fasteners}",
            f"  Unique tools needed: {self.unique_tool_count}",
            f"  Min. tool changes  : {self.total_tool_changes}",
        ]
        if self.ungrouped_fastener_ids:
            lines.append(
                f"  Ungrouped fasteners: {', '.join(self.ungrouped_fastener_ids)}"
                " (no subassembly_id — not in zone breakdown)"
            )
        lines.append("")

        # ── tool table ────────────────────────────────────────────────────────
        lines.append("  TOOL INVENTORY")
        lines.append("  " + "─" * 66)
        hdr = f"  {'Tool':<26} {'Size':<9} {'Qty':>4}  {'Torque':>16}  Fasteners"
        lines.append(hdr)
        lines.append("  " + "─" * 66)
        for t in self.tools:
            fids = ", ".join(t.fastener_ids)
            lines.append(
                f"  {t.label:<26} {t.size_label:<9} {t.count:>3}×  "
                f"{t.torque_str():>16}  {fids}"
            )
        lines.append("  " + "─" * 66)
        lines.append("")

        # ── zone breakdown ────────────────────────────────────────────────────
        if show_zones and self.zones:
            lines.append("  TOOL KIT PER ZONE")
            lines.append("  " + "─" * 66)
            for zone in self.zones:
                tool_labels = " + ".join(t.size_label for t in zone.tools)
                changes_str = f"{zone.tool_changes} change(s)" if zone.tool_changes else "no changes"
                lines.append(
                    f"  {zone.zone_id:<22} {tool_labels}"
                )
                lines.append(
                    f"  {'':22} fasteners: {', '.join(zone.fastener_ids)}"
                    f"  ({changes_str})"
                )
            lines.append("  " + "─" * 66)
            lines.append("")

        lines.append("═" * 68)
        return "\n".join(lines)


# ── tool derivation ───────────────────────────────────────────────────────────

_TOOL_ORDER = [
    "hex_key", "socket", "torx_bit",
    "phillips_bit", "pozi_bit", "slotted_bit",
]


def _tool_for(fs: FastenerSpec) -> ToolRequirement:
    """Derive the ToolRequirement for a single FastenerSpec."""
    d  = fs.nominal_diameter_mm
    dt = fs.drive_type
    t_min, t_max = _torque_for(d)

    if dt in (DriveType.HEX_BOLT, DriveType.FLANGE_HEX):
        # Socket wrench; AF size read directly from head_diameter_mm
        af = fs.head_diameter_mm
        size_label = f"{af:g} mm AF"
        label      = f"{af:g} mm Socket (AF)"
        return ToolRequirement(
            tool_type="socket", label=label, size_label=size_label,
            drive_type=dt, size_mm=af,
            torque_min_nm=t_min, torque_max_nm=t_max,
        )

    elif dt == DriveType.SOCKET_CAP:
        key_mm = _nearest_key(_HEX_KEY_MM, d)
        size_label = f"{key_mm:g} mm"
        label      = f"{key_mm:g} mm Hex Key"
        return ToolRequirement(
            tool_type="hex_key", label=label, size_label=size_label,
            drive_type=dt, size_mm=key_mm,
            torque_min_nm=t_min, torque_max_nm=t_max,
        )

    elif dt == DriveType.TORX:
        t_str      = _nearest_key(_TORX_SIZE, d)
        size_label = t_str
        label      = f"{t_str} Torx Bit"
        return ToolRequirement(
            tool_type="torx_bit", label=label, size_label=size_label,
            drive_type=dt, size_mm=float(t_str[1:]),  # numeric T-value
            torque_min_nm=t_min, torque_max_nm=t_max,
        )

    elif dt == DriveType.PHILLIPS:
        ph_no      = _nearest_key(_PHILLIPS_NO, d)
        size_label = f"PH{ph_no}"
        label      = f"PH{ph_no} Phillips Bit"
        return ToolRequirement(
            tool_type="phillips_bit", label=label, size_label=size_label,
            drive_type=dt, size_mm=float(ph_no),
            torque_min_nm=t_min, torque_max_nm=t_max,
        )

    elif dt == DriveType.POZI:
        pz_no      = _nearest_key(_POZI_NO, d)
        size_label = f"PZ{pz_no}"
        label      = f"PZ{pz_no} Pozidriv Bit"
        return ToolRequirement(
            tool_type="pozi_bit", label=label, size_label=size_label,
            drive_type=dt, size_mm=float(pz_no),
            torque_min_nm=t_min, torque_max_nm=t_max,
        )

    else:  # SLOTTED or unknown
        size_label = f"M{d:g}"
        label      = f"M{d:g} Slotted Bit"
        return ToolRequirement(
            tool_type="slotted_bit", label=label, size_label=size_label,
            drive_type=dt, size_mm=d,
            torque_min_nm=t_min, torque_max_nm=t_max,
        )


# ── public API ────────────────────────────────────────────────────────────────

def calculate_resources(fasteners: list[FastenerSpec]) -> AssemblyResources:
    """
    Calculate the complete tool kit and torque requirements for a fastener list.

    Parameters
    ----------
    fasteners :
        All FastenerSpec objects in the assembly.

    Returns
    -------
    AssemblyResources
        Deduplicated tool inventory, per-zone breakdowns, and torque ranges.
    """
    # ── build a deduplicated tool map keyed by (tool_type, size_mm) ──────────
    tool_map: dict[tuple[str, float], ToolRequirement] = {}

    for fs in fasteners:
        req = _tool_for(fs)
        key = (req.tool_type, req.size_mm)
        if key not in tool_map:
            tool_map[key] = req
        # accumulate fastener references
        tool_map[key].fastener_ids.append(fs.id)
        tool_map[key].fastener_names.append(fs.name)

    # sort: hex_key → socket → torx → phillips → pozi → slotted; then by size
    tools = sorted(
        tool_map.values(),
        key=lambda t: (_TOOL_ORDER.index(t.tool_type)
                       if t.tool_type in _TOOL_ORDER else 99,
                       t.size_mm),
    )

    # ── per-zone breakdown ────────────────────────────────────────────────────
    zone_fasteners: dict[str, list[FastenerSpec]] = defaultdict(list)
    ungrouped: list[str] = []

    for fs in fasteners:
        gid = (fs.subassembly_id or "").strip()
        if gid:
            zone_fasteners[gid].append(fs)
        else:
            ungrouped.append(fs.id)

    zones: list[ZoneResources] = []
    for zone_id in sorted(zone_fasteners):
        zone_fs = zone_fasteners[zone_id]
        zone_tool_map: dict[tuple[str, float], ToolRequirement] = {}
        for fs in zone_fs:
            req = _tool_for(fs)
            key = (req.tool_type, req.size_mm)
            if key not in zone_tool_map:
                zone_tool_map[key] = ToolRequirement(
                    tool_type=req.tool_type, label=req.label,
                    size_label=req.size_label, drive_type=req.drive_type,
                    size_mm=req.size_mm,
                    torque_min_nm=req.torque_min_nm,
                    torque_max_nm=req.torque_max_nm,
                )
            zone_tool_map[key].fastener_ids.append(fs.id)
            zone_tool_map[key].fastener_names.append(fs.name)

        zone_tools = sorted(
            zone_tool_map.values(),
            key=lambda t: (_TOOL_ORDER.index(t.tool_type)
                           if t.tool_type in _TOOL_ORDER else 99, t.size_mm),
        )
        zones.append(ZoneResources(
            zone_id=zone_id,
            tools=zone_tools,
            fastener_ids=[fs.id for fs in zone_fs],
            tool_changes=max(0, len(zone_tools) - 1),
        ))

    return AssemblyResources(
        tools=tools,
        zones=zones,
        total_fasteners=len(fasteners),
        ungrouped_fastener_ids=ungrouped,
    )
