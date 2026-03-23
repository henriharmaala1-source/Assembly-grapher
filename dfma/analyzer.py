"""
DFMAAnalyzer — unified entry point that runs DFA, DFM, and DFS checks.
"""

from .models.part     import Assembly
from .models.fastener import FastenerSpec
from .models.warning  import AnalysisResult
from .rules.dfa_rules  import check_assembly      as _dfa_check
from .rules.dfm_rules  import check_assembly_dfm  as _dfm_check
from .rules.screw_rules import check_fasteners    as _dfs_check


def analyze(
    assembly:  Assembly,
    fasteners: list[FastenerSpec] | None = None,
) -> AnalysisResult:
    """
    Run the full DFMA analysis (DFA + DFM + DFS) on an assembly.

    Parameters
    ----------
    assembly:
        The assembly to analyse.
    fasteners:
        Optional list of FastenerSpec objects representing bolts/screws in the
        assembly.  When provided, obstruction checks (DFS rules) are included.

    Returns an AnalysisResult containing:
      - all DFA, DFM, and DFS warnings sorted by severity
      - DFA Index (assembly efficiency score)
      - estimated total assembly time
      - theoretical minimum part count
    """
    # DFA analysis (also computes timing metrics)
    result = _dfa_check(assembly)

    # DFM analysis
    result.warnings.extend(_dfm_check(assembly))

    # DFS fastener / screw obstruction analysis
    if fasteners:
        result.warnings.extend(_dfs_check(fasteners))

    # Sort: ERRORs first, then WARNINGs, then INFOs
    severity_order = {"ERROR": 0, "WARNING": 1, "INFO": 2}
    result.warnings.sort(key=lambda w: severity_order[w.severity.value])

    return result
