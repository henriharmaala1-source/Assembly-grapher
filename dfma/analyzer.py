"""
DFMAAnalyzer — unified entry point that runs both DFA and DFM checks.
"""

from .models.part    import Assembly
from .models.warning import AnalysisResult
from .rules.dfa_rules import check_assembly   as _dfa_check
from .rules.dfm_rules import check_assembly_dfm as _dfm_check


def analyze(assembly: Assembly) -> AnalysisResult:
    """
    Run the full DFMA analysis (DFA + DFM) on an assembly.

    Returns an AnalysisResult containing:
      - all DFA and DFM warnings
      - DFA Index (assembly efficiency score)
      - estimated total assembly time
      - theoretical minimum part count
    """
    # DFA analysis (also computes timing metrics)
    result = _dfa_check(assembly)

    # DFM analysis — append warnings to the same result
    dfm_warnings = _dfm_check(assembly)
    result.warnings.extend(dfm_warnings)

    # Sort: ERRORs first, then WARNINGs, then INFOs
    severity_order = {"ERROR": 0, "WARNING": 1, "INFO": 2}
    result.warnings.sort(key=lambda w: severity_order[w.severity.value])

    return result
