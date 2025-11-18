#!/usr/bin/env python3
import time
import sys
import serial
from pymavlink import mavutil
import serial.tools.list_ports

# ---------- User Settings ----------
ETH_UDP_URI = "udp:0.0.0.0:14550"
ETH_BAUD = 57600
ETH_SYSID = 1
ETH_COMPID = 1

PX4_SERIAL_COMMANDS = [
    "mavlink stop-all",
    "mavlink start -d /dev/ttyS5 -b 57600 -m 0 -f 1 -r 0",  # RS232-2
    "mavlink start -d /dev/ttyS6 -b 57600 -m 0 -f 1 -r 0",  # RS232-1
]

RS232_BAUD = 57600
HEARTBEAT_TIMEOUT = 4.0  # seconds

# ---------- Utility ----------
def print_info(msg):
    print(msg)

# ---------- Ethernet MAVLink Check ----------
def check_ethernet():
    print("\n=== Checking Ethernet MAVLink (UDP) ===")
    try:
        mav_udp = mavutil.mavlink_connection(ETH_UDP_URI, autoreconnect=True)
        mav_udp.wait_heartbeat(timeout=5)
        print(f"[OK] Heartbeat received (sys={mav_udp.target_system}, comp={mav_udp.target_component})")
        return mav_udp, True
    except Exception as e:
        print(f"[FAIL] Ethernet heartbeat failed: {e}")
        return None, False

# ---------- Enable Serial MAVLink via UDP ----------
def enable_serial_mavlink_via_udp(mav_udp):
    """
    Send PX4 shell commands over MAVLink to enable serial interfaces.
    """
    print("\n[ETHERNET] Enabling MAVLink on RS232 ports...")
    for cmd in PX4_SERIAL_COMMANDS:
        print_info(f"  -> Sending: {cmd}")

        # PX4 expects a shell wakeup
        mav_udp.mav.command_long_send(
            ETH_SYSID,
            ETH_COMPID,
            511,  # MAV_CMD_DO_SEND_SCRIPT
            0,
            1,    # shell
            0, 0, 0, 0, 0, 0
        )

        # Send command in 70-byte chunks
        cmd_bytes = [ord(c) for c in cmd + "\n"]
        while cmd_bytes:
            chunk = cmd_bytes[:70]
            chunk += [0] * (70 - len(chunk))  # pad to 70
            mav_udp.mav.serial_control_send(
                0,  # device: SERIAL_CONTROL_DEV_SHELL
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
                0, 0, 70,
                chunk
            )
            cmd_bytes = cmd_bytes[70:]

        time.sleep(0.15)
    print("[ETHERNET] Commands sent. Waiting for PX4 to start serial MAVLink...")
    time.sleep(1.2)

# ---------- RS232 Detection ----------
def detect_com_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print(f"\n=== Scanning COM ports for MAVLink ===")
    print(f"Detected COM ports: {ports}")
    return ports

# ---------- RS232 Heartbeat Check ----------
def check_rs232_ports(ports):
    status = {}
    for port in ports:
        try:
            mav_serial = mavutil.mavlink_connection(port, baud=RS232_BAUD, autoreconnect=True)
            mav_serial.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
            print(f"[OK] Heartbeat received on {port} (sys={mav_serial.target_system}, comp={mav_serial.target_component})")
            status[port] = True
            mav_serial.close()
        except Exception as e:
            print(f"[FAIL] No heartbeat on {port} within {HEARTBEAT_TIMEOUT}s")
            status[port] = False
    return status

# ---------- Main ----------
def main():
    results = {
        "Ethernet": False,
        "RS232_ports": {}
    }

    # Check Ethernet first
    mav_udp, eth_ok = check_ethernet()
    results["Ethernet"] = eth_ok

    # Enable RS232 MAVLink if Ethernet is OK
    if eth_ok:
        enable_serial_mavlink_via_udp(mav_udp)

    # Detect RS232 ports
    rs232_ports = detect_com_ports()

    # Check RS232 heartbeats
    if rs232_ports:
        rs232_status = check_rs232_ports(rs232_ports)
        results["RS232_ports"] = rs232_status

    # ---------- Summary ----------
    print("\n=== Summary of MAVLink Communication Checks ===")
    print(f"Ethernet/UDP: {'[OK]' if results['Ethernet'] else '[FAIL]'}")
    if results["RS232_ports"]:
        print("RS232 Ports:")
        for port, status in results["RS232_ports"].items():
            print(f"  {port}: {'[OK]' if status else '[FAIL]'}")
    else:
        print("RS232 Ports: None detected")
    print("\n=== Check Complete ===")

if __name__ == "__main__":
    main()
