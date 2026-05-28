#!/usr/bin/env python3
"""
ROS Topic Discovery Tool for PIB Robot.

Lists all available ROS topics and their message types from the robot's
rosbridge server. Useful for debugging and exploring available data streams.

Usage:
    python -m pib3.tools.list_topics --host 172.26.34.149

    # Filter topics by name pattern:
    python -m pib3.tools.list_topics --host 172.26.34.149 --filter camera

    # Show only specific message types:
    python -m pib3.tools.list_topics --host 172.26.34.149 --type sensor_msgs
"""

import argparse
import sys


def list_topics(host: str, port: int, filter_pattern: str = None, type_filter: str = None) -> None:
    """
    Connect to robot and list all ROS topics.

    Args:
        host: Robot IP address.
        port: Rosbridge websocket port.
        filter_pattern: Optional pattern to filter topic names (case-insensitive).
        type_filter: Optional pattern to filter message types (case-insensitive).
    """
    try:
        import roslibpy
    except ImportError:
        print("Error: roslibpy is required.")
        print("Install with: pip install roslibpy")
        sys.exit(1)

    print(f"Connecting to {host}:{port}...")

    client = roslibpy.Ros(host=host, port=port)

    try:
        client.run(timeout=10)
    except Exception as e:
        print(f"Error connecting: {e}")
        sys.exit(1)

    if not client.is_connected:
        print("Failed to connect to rosbridge server.")
        print("Check that:")
        print("  1. The robot is powered on")
        print("  2. rosbridge_server is running")
        print(f"  3. The IP address {host} is correct")
        sys.exit(1)

    print("Connected!\n")

    # Get all topics
    topics = client.get_topics()

    # Build topic list with types
    topic_info = []
    for topic in sorted(topics):
        topic_type = client.get_topic_type(topic)
        topic_info.append((topic, topic_type))

    client.close()

    # Apply filters
    if filter_pattern:
        filter_lower = filter_pattern.lower()
        topic_info = [(t, ty) for t, ty in topic_info if filter_lower in t.lower()]

    if type_filter:
        type_lower = type_filter.lower()
        topic_info = [(t, ty) for t, ty in topic_info if type_lower in ty.lower()]

    # Display results
    if not topic_info:
        print("No topics found matching the filter criteria.")
        return

    # Calculate column width for alignment
    max_topic_len = max(len(t) for t, _ in topic_info)
    col_width = min(max_topic_len + 2, 50)

    print(f"{'TOPIC':<{col_width}} MESSAGE TYPE")
    print("-" * (col_width + 40))

    for topic, topic_type in topic_info:
        print(f"{topic:<{col_width}} {topic_type}")

    print(f"\nTotal: {len(topic_info)} topic(s)")


def main():
    parser = argparse.ArgumentParser(
        description="List ROS topics from PIB robot's rosbridge server.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List all topics
  python -m pib3.tools.list_topics --host 172.26.34.149

  # Filter topics containing 'camera'
  python -m pib3.tools.list_topics --host 172.26.34.149 --filter camera

  # Show only sensor_msgs types
  python -m pib3.tools.list_topics --host 172.26.34.149 --type sensor_msgs

  # Combine filters
  python -m pib3.tools.list_topics --host 172.26.34.149 --filter imu --type sensor

Common topics on PIB robot:
  /camera/image/compressed - Camera stream (sensor_msgs/msg/CompressedImage)
  /camera/ai/detections    - AI detection results (std_msgs/msg/String)
  /joint_trajectory        - Motor positions (trajectory_msgs/msg/JointTrajectory)
  /camera/imu              - IMU sensor data (sensor_msgs/msg/Imu)
        """,
    )

    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)",
    )
    parser.add_argument(
        "--filter", "-f",
        dest="filter_pattern",
        help="Filter topics by name (case-insensitive)",
    )
    parser.add_argument(
        "--type", "-t",
        dest="type_filter",
        help="Filter by message type (case-insensitive)",
    )

    args = parser.parse_args()

    list_topics(
        host=args.host,
        port=args.port,
        filter_pattern=args.filter_pattern,
        type_filter=args.type_filter,
    )


if __name__ == "__main__":
    main()

"""
TOPIC                         MESSAGE TYPE
---------------------------------------------------------------------
/audio_stream                 std_msgs/msg/Int16MultiArray
/camera/ai/available_models   std_msgs/msg/String
/camera/ai/config             std_msgs/msg/String
/camera/ai/current_model      std_msgs/msg/String
/camera/ai/detections         std_msgs/msg/String
/camera/ai/status             std_msgs/msg/String
/camera/error                 std_msgs/msg/String
/camera/image/compressed      sensor_msgs/msg/CompressedImage
/camera/imu                   sensor_msgs/msg/Imu
/camera/imu/accelerometer     geometry_msgs/msg/Vector3Stamped
/camera/imu/config            std_msgs/msg/String
/camera/imu/gyroscope         geometry_msgs/msg/Vector3Stamped
/camera/preview_size          std_msgs/msg/Int32MultiArray
/camera/quality_factor        std_msgs/msg/Int32
/camera/rgb/image             std_msgs/msg/String
/camera/timer_period          std_msgs/msg/Float64
/camera/video/config          std_msgs/msg/String
/camera_topic                 std_msgs/msg/String
/chat_is_listening            datatypes/msg/ChatIsListening
/chat_messages                datatypes/msg/ChatMessage
/client_count                 std_msgs/msg/Int32
/connected_clients            rosbridge_msgs/msg/ConnectedClients
/delete_token                 std_msgs/msg/Empty
/display_image                datatypes/msg/DisplayImage
/doa_angle                    std_msgs/msg/Int32
/joint_trajectory             trajectory_msgs/msg/JointTrajectory
/motor_current                diagnostic_msgs/msg/DiagnosticStatus
/motor_settings               datatypes/msg/MotorSettings
/parameter_events             rcl_interfaces/msg/ParameterEvent
/program_input                datatypes/msg/ProgramInput
/proxy_run_program_feedback   datatypes/msg/ProxyRunProgramFeedback
/proxy_run_program_result     datatypes/msg/ProxyRunProgramResult
/proxy_run_program_status     datatypes/msg/ProxyRunProgramStatus
/public_api_token             std_msgs/msg/String
/quality_factor_topic         std_msgs/msg/Int32
/rosout                       rcl_interfaces/msg/Log
/size_topic                   std_msgs/msg/Int32MultiArray
/solid_state_relay_state      datatypes/msg/SolidStateRelayState
/timer_period_topic           std_msgs/msg/Float64
/voice_assistant_state        datatypes/msg/VoiceAssistantState
"""
