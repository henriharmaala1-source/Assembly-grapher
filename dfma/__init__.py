"""DFMA Warning System — Design for Manufacture and Assembly analysis."""

from .analyzer import analyze
from .models.part    import Part, Assembly, Geometry, ManufacturingProcess, Material
from .models.warning import Warning, AnalysisResult, Severity

__all__ = [
    "analyze",
    "Part", "Assembly", "Geometry", "ManufacturingProcess", "Material",
    "Warning", "AnalysisResult", "Severity",
]
