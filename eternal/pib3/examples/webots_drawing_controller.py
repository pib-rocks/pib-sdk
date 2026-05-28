"""Webots drawing controller - Draw an image using the PIB robot.

This controller is designed to run inside Webots simulation.
Copy this file to your Webots controller directory.

Usage:
    1. Copy to your Webots project's controllers folder
    2. Set this as the controller for your PIB robot in Webots
    3. Run the simulation

Requirements:
    pip install pib3
"""

import pib3
from pib3 import PaperConfig, TrajectoryConfig, IKConfig, Joint

print(f"pib3 version: {pib3.__version__}")

# Image to draw
img_path = "/home/amr/repositories/b3_pib_inverse_kinematic_pip_package/examples/Icon_IT_hover.png"

# Paper configuration
paper = PaperConfig(
    size=0.20,           # 20cm x 20cm drawing area
    height_z=0.65,       # 65cm table height
    drawing_scale=0.85,  # Scale drawing to 85% of paper
)
print(f"Paper config: {paper}")

with pib3.Webots() as robot:
    # Move arm to neutral starting position (0 radians for all joints)
    print("Moving to start position...")
    robot.go_home()

    # Set hand to index finger pointing pose for drawing
    # 0% = bent/closed, 100% = stretched/open
    print("Setting hand pose for drawing...")
    robot.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 0.0,
        Joint.THUMB_LEFT_STRETCH: 0.0,
        Joint.INDEX_LEFT: 100.0,         # Index extended (pointing)
        Joint.MIDDLE_LEFT: 0.0,          # Others closed
        Joint.RING_LEFT: 0.0,
        Joint.PINKY_LEFT: 0.0,
    }, async_=False)

    # Generate trajectory from image
    print("Computing trajectory from image...")
    trajectory = pib3.generate_trajectory(
        img_path,
        config=TrajectoryConfig(
            ik=IKConfig(grip_style="index_finger"),
            paper=paper,
        ),
    )
    print(f"Trajectory computed: {len(trajectory)} waypoints")

    # Save trajectory for inspection
    trajectory.to_json("output.json")
    print("Trajectory saved to output.json")

    # Execute the drawing trajectory
    print("Drawing...")
    robot.run_trajectory(trajectory)
    print("Done!")
