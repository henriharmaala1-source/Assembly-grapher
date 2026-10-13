#!/usr/bin/env python
"""
Minimal diagnostic to verify Assembly-grapher setup.
Run this to check if OCP/OCC is installed and working.
"""
import sys
import os

print("=" * 70)
print("  ASSEMBLY-GRAPHER SETUP DIAGNOSTIC")
print("=" * 70)
print()

# Python version
print(f"Python: {sys.version}")
print(f"Executable: {sys.executable}")
print()

# OCP (cadquery-ocp)
print("1. Checking OCP (cadquery-ocp) via cadquery...")
try:
    import cadquery as cq
    print(f"   ✓ cadquery {cq.__version__} installed")
    print(f"   ✓ Can use STEP import")
except ImportError as e:
    print(f"   ✗ cadquery NOT installed")
    print(f"   Install with: pip install cadquery")
    print()

# OCC (pythonocc-core)
print("2. Checking OCC (pythonocc-core)...")
try:
    from OCC.Core.STEPControl import STEPControl_Reader
    print(f"   ✓ OCC.Core available")
    print(f"   ✓ Can use STEP import")
except ImportError as e:
    print(f"   ✗ OCC.Core NOT installed")
    print(f"   Install with: conda install -c conda-forge pythonocc-core")

# NetworkX
print("3. Checking networkx...")
try:
    import networkx as nx
    print(f"   ✓ networkx {nx.__version__} installed")
except ImportError:
    print(f"   ✗ networkx NOT installed")
    print(f"   Install with: pip install networkx")

# tkinter
print("4. Checking tkinter (GUI)...")
try:
    import tkinter as tk
    print(f"   ✓ tkinter available")
except ImportError:
    print(f"   ✗ tkinter NOT available")
    print(f"   This is required for the GUI. Try: pip install tk")

# Test STEP import capability
print()
print("5. Testing STEP import...")
try:
    from assembly_graph.importers.step_importer import import_step, _OCC_AVAILABLE
    if _OCC_AVAILABLE:
        print(f"   ✓ STEP importer ready")
    else:
        print(f"   ✗ STEP importer NOT available")
        print(f"   → Install cadquery: pip install cadquery")
except Exception as e:
    print(f"   ✗ Error: {e}")

print()
print("=" * 70)
print("  If all items are ✓, you can run: python gui_main.py")
print("  Or use the VBS launcher: AssemblyGrapher.vbs")
print("=" * 70)
