#!/usr/bin/env python3
"""
Robot Connection Diagnostic Tool

This script diagnoses connection issues to the PIB robot by checking:
1. Network ping
2. TCP port connectivity
3. Rosbridge WebSocket protocol

Usage:
    python diagnose_connection.py --host 172.26.34.149
    python diagnose_connection.py --host 172.26.34.149 --port 9090

Note:
    This tool works with both the physical robot and Webots simulation.
    For local Webots, no IP is needed.
"""

import argparse
import sys
import socket
import logging

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy is not installed. Please install it with 'pip install roslibpy'")
    sys.exit(1)

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

def check_ping(host):
    import subprocess
    print(f"Checking ping to {host}...")
    try:
        output = subprocess.check_output(
            ["ping", "-c", "1", "-W", "2", host],
            stderr=subprocess.STDOUT,
            text=True
        )
        print(f"  [OK] Ping successful:\n{output}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  [FAIL] Ping failed:\n{e.output}")
        return False

def check_port(host, port):
    print(f"Checking TCP connection to {host}:{port}...")
    try:
        sock = socket.create_connection((host, port), timeout=2)
        sock.close()
        print(f"  [OK] TCP connection successful")
        return True
    except Exception as e:
        print(f"  [FAIL] TCP connection failed: {e}")
        return False

def check_rosbridge(host, port):
    print(f"Checking rosbridge connection to {host}:{port}...")
    
    ros = roslibpy.Ros(host=host, port=port)
    try:
        ros.run(timeout=5)
        if ros.is_connected:
            print("  [OK] Rosbridge connected successfully")
            ros.close()
            return True
        else:
            print("  [FAIL] Rosbridge run() returned but not connected")
            return False
    except Exception as e:
        print(f"  [FAIL] Rosbridge connection error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Robot Connection Diagnostic Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python diagnose_connection.py --host 172.26.34.149
  python diagnose_connection.py --host 172.26.34.149 --port 9090
        """
    )
    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)"
    )
    args = parser.parse_args()

    host = args.host
    port = args.port

    print("=== Robot Connection Diagnostic Tool ===")
    print(f"Target: {host}:{port}\n")

    ping_ok = check_ping(host)
    port_ok = check_port(host, port)
    ros_ok = False

    if port_ok:
        ros_ok = check_rosbridge(host, port)

    print("\n=== Summary ===")
    print(f"Network (Ping):   {'✅ PASS' if ping_ok else '❌ FAIL'}")
    print(f"Port {port} (TCP):  {'✅ PASS' if port_ok else '❌ FAIL'}")
    print(f"Rosbridge Proto:  {'✅ PASS' if ros_ok else '❌ FAIL'}")

    if ping_ok and port_ok and ros_ok:
        print("\nSUCCESS: Connection to robot looks good!")
        print("You can now use: Robot(host='{}', port={})".format(host, port))
    else:
        print("\nFAILURE: Some checks failed. See details above.")
        if not ping_ok:
            print("\nTroubleshooting network issues:")
            print("  1. Check robot is powered on")
            print("  2. Check both devices are on the same network")
            print(f"  3. Verify IP address is correct: {host}")
        if ping_ok and not port_ok:
            print("\nTroubleshooting port issues:")
            print("  1. Verify rosbridge_server is running on the robot")
            print("  2. Check firewall settings on the robot")
            print(f"  3. Confirm rosbridge uses port {port}")

if __name__ == "__main__":
    main()
