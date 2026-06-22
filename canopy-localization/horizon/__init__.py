"""Canopy localization — horizon (skyline) generation & matching.

Two halves that meet in *curve space* (elevation-angle vs azimuth):

  H_lidar : ray-cast from a DSM height-field  (geometry, auto-labeled w/ coords)
  H_cam   : classical sky/canopy segmentation of a camera frame (training-free)

Matching is shape-to-shape on those curves, so LiDAR-derived curves are valid
references/training data for real camera curves with no real-world data.
"""
