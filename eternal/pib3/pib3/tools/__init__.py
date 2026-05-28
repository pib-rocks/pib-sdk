"""Helper tools for pib3 package.

This module contains preprocessing and utility tools:
- proto_converter: Convert Webots .proto files to URDF format
- calibrate_joints: Interactive joint limit calibration tool

Run calibration tool:
    python -m pib3.tools.calibrate_joints --host 172.26.34.149 --group left_hand
"""

from .proto_converter import convert_proto_to_urdf

__all__ = ["convert_proto_to_urdf"]
