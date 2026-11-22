"""bbanalyzer – blackbox log analysis for multirotors (iNav / Betaflight)."""

from .analysis import run, to_json
from .config import Aircraft
from .loader import load

__all__ = ["Aircraft", "load", "run", "to_json"]
__version__ = "0.1.0"
