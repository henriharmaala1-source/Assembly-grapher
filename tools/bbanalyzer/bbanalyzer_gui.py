"""Entry point for the windowed executable (bbanalyzer.exe)."""

import multiprocessing
import sys

from bbanalyzer.gui import main

if __name__ == "__main__":
    multiprocessing.freeze_support()
    sys.exit(main(sys.argv[1:]))
