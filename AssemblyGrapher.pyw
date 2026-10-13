"""
Assembly Grapher — double-click launcher for Windows.

.pyw files are run by pythonw.exe which suppresses the console window.
"""
import sys
from pathlib import Path

# Make sure imports work regardless of working directory
sys.path.insert(0, str(Path(__file__).parent))

from gui_main import main
main()
