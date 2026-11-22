"""Entry point for the PyInstaller executable."""

import multiprocessing
import sys

from bbanalyzer.__main__ import main

if __name__ == "__main__":
    multiprocessing.freeze_support()
    sys.exit(main())
