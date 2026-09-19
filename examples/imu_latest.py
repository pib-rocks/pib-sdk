#!/usr/bin/env python3
"""Live robot: print IMU samples from latest() with age and orientation checks."""

import argparse
import time

from pib_sdk.features.imu import IMU


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument(
        "--poll-s",
        type=float,
        default=0.2,
        help="client-side poll period; IMU publish intervals are not uniform",
    )
    args = parser.parse_args()

    with IMU(host=args.host) as imu:
        printed = 0
        while printed < args.samples:
            data = imu.latest()
            if data is None:
                print("no IMU sample yet")
            else:
                printed += 1
                acc = data.acceleration_m_s2
                gyro = data.angular_velocity_rad_s
                print(
                    f"age_s={data.age_s:.3f} "
                    f"acc=({acc.x:.3f}, {acc.y:.3f}, {acc.z:.3f}) m/s² "
                    f"gyro=({gyro.x:.4f}, {gyro.y:.4f}, {gyro.z:.4f}) rad/s "
                    f"orientation_available={data.orientation_available}"
                )
                if not data.orientation_available:
                    print("no orientation available on this hardware")
            time.sleep(args.poll_s)


if __name__ == "__main__":
    main()
