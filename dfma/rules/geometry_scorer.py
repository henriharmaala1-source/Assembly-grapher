"""
Geometry Scorer — estimates Boothroyd-Dewhurst handling and insertion times.

Reference:
  Boothroyd G., Dewhurst P., Knight W.A. (2010)
  "Product Design for Manufacture and Assembly", CRC Press (3rd ed.)

Simplified time tables used here are derived from the published lookup tables.
All times are in seconds.

Handling time model
───────────────────
  t_handle = base_time × symmetry_factor × size_factor × difficulty_factor

  base_time      = 1.13 s  (ideal: easy-to-grasp, small, symmetric part)
  symmetry_factor: driven by α (insertion-axis symmetry) and β (perpendicular)
  size_factor:     penalty for very small (<3mm), very large (>150mm), or heavy
  difficulty_factor: cumulative multiplier for flexible, tangling, fragile,
                     two-hand handling

Insertion time model
────────────────────
  t_insert = base_insert × access_factor × alignment_factor

  base_insert    = 1.50 s  (straight, unobstructed insertion, no alignment)
  access_factor  : penalty for restricted/obstructed access
  alignment_factor: penalty for precise angular alignment requirement
"""

from ..models.part import Geometry

# ── base times (seconds) ────────────────────────────────────────────────────
BASE_HANDLING_TIME  = 1.13   # ideal part: small, symmetric, one-handed
BASE_INSERTION_TIME = 1.50   # ideal insertion: clear access, no alignment needed
IDEAL_ASSEMBLY_TIME = 2.93   # Boothroyd-Dewhurst standard for DFA Index denominator


def symmetry_factor(alpha: float, beta: float) -> float:
    """
    Penalty multiplier based on rotational symmetry angles.

    α = 360, β = 360  → fully symmetric  → factor 1.0  (fastest)
    α = 180, β = 360  → one axis         → factor 1.5
    α =   0, β =   0  → no symmetry      → factor 3.0  (slowest)

    Intermediate values are linearly interpolated.
    """
    # Normalise to [0, 1] where 1 = full symmetry
    alpha_norm = min(alpha, 360.0) / 360.0
    beta_norm  = min(beta,  360.0) / 360.0
    symmetry_score = (alpha_norm + beta_norm) / 2.0   # 0 → 1

    # Map symmetry_score 1→1.0, 0→3.0 (linear)
    return 3.0 - symmetry_score * 2.0


def size_factor(length: float, width: float, height: float, mass_g: float) -> float:
    """
    Penalty multiplier for parts that are difficult to handle due to size/mass.

    Very small parts (<3 mm on any dimension) need tweezers or fixtures.
    Very large parts (>150 mm) or heavy (>1000 g) may need two people or a crane.
    """
    min_dim = min(d for d in [length, width, height] if d > 0) if any(
        d > 0 for d in [length, width, height]
    ) else 0.0
    max_dim = max(length, width, height)

    factor = 1.0
    if min_dim > 0 and min_dim < 3.0:      # very small
        factor *= 2.0
    elif max_dim > 150.0:                   # large
        factor *= 1.5
    if mass_g > 1000.0:                     # heavy (>1 kg)
        factor *= 1.8
    elif mass_g > 250.0:                    # moderately heavy
        factor *= 1.3
    return factor


def handling_difficulty_factor(geo: Geometry) -> float:
    """
    Cumulative multiplier for handling difficulty flags.
    Each flag adds a time penalty based on B&D classification data.
    """
    factor = 1.0
    if geo.is_flexible:    factor *= 1.8   # e.g. gaskets, seals, wires
    if geo.can_tangle:     factor *= 1.6   # e.g. springs, clips
    if geo.is_fragile:     factor *= 1.4   # careful handling required
    if geo.needs_2_hands:  factor *= 1.5   # significant handling overhead
    return factor


def access_factor(obstructed: bool) -> float:
    """Penalty for restricted access to the insertion point."""
    return 1.8 if obstructed else 1.0


def alignment_factor(requires_alignment: bool) -> float:
    """Penalty for needing precise angular pre-alignment before insertion."""
    return 1.5 if requires_alignment else 1.0


def estimate_handling_time(geo: Geometry) -> float:
    """Return estimated handling time (seconds) for a single part."""
    t = BASE_HANDLING_TIME
    t *= symmetry_factor(geo.alpha, geo.beta)
    t *= size_factor(geo.length, geo.width, geo.height, geo.mass_grams)
    t *= handling_difficulty_factor(geo)
    return round(t, 3)


def estimate_insertion_time(geo: Geometry) -> float:
    """Return estimated insertion time (seconds) for a single part."""
    t = BASE_INSERTION_TIME
    t *= access_factor(geo.obstructed_access)
    t *= alignment_factor(geo.requires_alignment)
    return round(t, 3)


def estimate_total_time(geo: Geometry) -> float:
    """Return total assembly operation time (handling + insertion) in seconds."""
    return estimate_handling_time(geo) + estimate_insertion_time(geo)
