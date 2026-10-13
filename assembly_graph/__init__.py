"""
assembly_graph — subassembly detection, order generation, and optimization.

Public API
──────────
    from assembly_graph import AssemblyPlanner, LiaisonMatrix, AssemblyGraph

    planner = AssemblyPlanner(part_ids=[...])
    planner.liaison.add_contact("P001", "P002")
    plan = planner.plan()
    print(plan.summary())
"""

from .liaison_matrix       import LiaisonMatrix, Contact
from .assembly_graph       import AssemblyGraph, AssemblyNode, CycleError
from .subassembly_detector import SubassemblyDetector, DetectedSubassembly
from .planner              import AssemblyPlanner, AssemblyPlan
from .sequence.generator   import SequenceGenerator
from .sequence.optimizer   import SequenceOptimizer, ScoredSequence, OptimizationWeights

__all__ = [
    "LiaisonMatrix", "Contact",
    "AssemblyGraph", "AssemblyNode", "CycleError",
    "SubassemblyDetector", "DetectedSubassembly",
    "AssemblyPlanner", "AssemblyPlan",
    "SequenceGenerator",
    "SequenceOptimizer", "ScoredSequence", "OptimizationWeights",
]
