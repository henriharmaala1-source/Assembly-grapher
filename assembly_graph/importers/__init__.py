"""
assembly_graph.importers — read CAD assembly files into Assembly + LiaisonMatrix.

Public API
──────────
    from assembly_graph.importers import load_assembly, check_collisions

    result  = load_assembly("path/to/file.step")   # or .json, .plmxml, .sldasm, .prt
    report  = check_collisions(result)
    print(report.summary())

Supported formats (auto-detected from extension)
────────────────────────────────────────────────
  .json / .bom  → JSON BOM (no dependencies beyond stdlib)
  .step / .stp  → STEP AP214/AP242 (requires: pip install pythonocc-core)
  .plmxml / .xml→ PLMXML product structure (requires: pip install lxml)
  .sldasm       → SolidWorks (requires: pywin32 + SolidWorks on Windows)
  .prt          → Siemens NX  (requires: NXOpen + NX installation)
"""

from .base      import ImportResult, ImportError, PartPose, CollisionPair, detect_format
from .collision import check_collisions, CollisionReport

__all__ = [
    "load_assembly",
    "check_collisions",
    "ImportResult",
    "ImportError",
    "PartPose",
    "CollisionPair",
    "CollisionReport",
    "detect_format",
]


def load_assembly(path: str, **kwargs) -> ImportResult:
    """
    Auto-detect format from file extension and delegate to the appropriate importer.

    Extra keyword arguments are forwarded to the format-specific importer.
    For example:
        load_assembly("my.step", density_map={"alumin": 2.70e-3})
        load_assembly("my.step", infer_contacts=False)
    """
    fmt = detect_format(path)

    if fmt == "json":
        from .json_importer import import_json
        return import_json(path, **kwargs)

    elif fmt == "step":
        from .step_importer import import_step
        return import_step(path, **kwargs)

    elif fmt == "plmxml":
        from .plmxml_importer import import_plmxml   # type: ignore[import]
        return import_plmxml(path, **kwargs)

    elif fmt == "solidworks":
        from .solidworks_importer import import_solidworks  # type: ignore[import]
        return import_solidworks(path, **kwargs)

    elif fmt == "nx":
        from .nx_importer import import_nx           # type: ignore[import]
        return import_nx(path, **kwargs)

    else:
        raise ImportError(
            f"Unsupported or unrecognised file format: '{path}'\n"
            "Supported extensions: .json, .step, .stp, .plmxml, .xml, .sldasm, .prt"
        )
